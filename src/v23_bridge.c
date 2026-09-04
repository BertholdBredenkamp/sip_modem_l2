/*
 * v23_bridge.c (v2)
 *
 * V.23-Modem (CEPT/BTX, 1200/75 bit/s) fuer die Rolle "Gegenstelle/Zentrale",
 * nachgebaut nach dem bewaehrten "app_softmodem" fuer Asterisk (Christian
 * Groeger, 2010, GPL) - konkret dessen Antwortmodem-Zweig ("answerer"),
 * der nachweislich mit einem echten DBT03 funktioniert.
 *
 * Zentrale Erkenntnisse aus der Referenz, die hier uebernommen wurden:
 *
 *   1. FSK_FRAME_MODE_SYNC statt FSK_FRAME_MODE_ASYNC: Das Bit-Framing
 *      (Start-/Stopbit-Erkennung) wird HIER SELBST gemacht, nicht von
 *      spandsps async-Schicht. Deren Leerlaufverhalten (kontinuierliche
 *      gerahmte 0xFF-Fuellzeichen) verwirrt offenbar reale DBT03-Hardware;
 *      der echte Leerlauf ist reiner, ungerahmter Mark-Pegel (Bit=1).
 *
 *   2. Traegererkennung + NUL-Byte-Kickstart: Erst wenn vom Terminal
 *      tatsaechlich ein Traeger/Bit ankommt (SIG_STATUS_CARRIER_UP, dann
 *      ein echtes Bit), gilt die Verbindung als "aktiv". Erst DANN wird
 *      ein einzelnes NUL-Byte (0x00) gesendet - das DBT03 erwartet dieses
 *      Anstossbyte, bevor es selbst Daten schickt bzw. reagiert.
 *
 *   3. Rollenfeste Kanalzuordnung fuer die Zentrale/Antwortseite:
 *      TX = FSK_V23CH1 (1200 bit/s, Vorwaertskanal), RX = FSK_V23CH2
 *      (75 bit/s, Rueckkanal) - das ist die "answerer"-Belegung der
 *      Referenz und entspricht der klassischen Prestel/BTX-Asymmetrie.
 *
 * Anders als die erste Version dieser Datei haelt dieser Code die
 * TCP-Verbindung zum BTX-Server selbst (wie die Referenz), statt sie an
 * Python zu delegieren - das vermeidet zusaetzliche Pufferungs-/Timing-
 * Unterschiede gegenueber dem bewaehrten Vorbild.
 *
 * v3: Layer-2 (FTZ 157 D2)
 * ------------------------
 * Bisher (v2) wurden die vom Bit-Ringpuffer zusammengesetzten Bytes 1:1
 * zwischen Terminal und TCP-Socket durchgereicht - keine Blockrahmen,
 * keine BCC-Pruefung, kein ACK/NAK-Handshake. Das reicht fuer einen reinen
 * Transparenzkanal, ist aber nicht das, was ein echtes DBT03 erwartet: das
 * FTZ-157-D2-Protokoll (Blockrahmen mit CRC-16/ARC-BCC, ACK0/ACK1/NAK/WACK,
 * T.F.I.-Aushandlung, Retransmit) sitzt in der Referenz *oberhalb* der
 * reinen V.23-Bitschicht.
 *
 * Diese Fassung baut das ein, uebernommen 1:1 aus dem ESP32-Port dieses
 * Projekts (btx_l2.h, dort bereits gegen ein echtes DBT03 verifiziert):
 *
 *   - Ein vom Bit-Ringpuffer zusammengesetztes RX-Byte (vom Terminal) geht
 *     jetzt an btx_l2_rx_byte() statt direkt auf den Socket. Die L2-Engine
 *     unterscheidet Steuerzeichen (ACK/NAK/WACK/ENQ/EOT/SOH-T.F.I.) von
 *     reinen Tastaturbytes; letztere landen als BTX_L2_EV_KEY im
 *     btx_l2_bridge-Sendepuffer und werden von dort periodisch zum Server
 *     geflusht (l2_flush_keys(), analog zu btx_l2_dbt03_flush_keys() beim
 *     ESP32).
 *   - TX-Bytes (an das Terminal) kommen jetzt aus btx_l2_tx_byte() statt
 *     aus einem rohen recv() auf dem Socket. Der Server-Bytestrom wird
 *     dazu in btx_l2_bridge.rx gepuffert, in Seiten zu je BTX_L2_CHUNK
 *     Bytes zerlegt (btx_l2_bridge_poll()) und blockweise mit Framing/BCC
 *     ausgesendet; bei einem Link-Reset haelt die Bridge die Bytes vor, bis
 *     sie bestaetigt sind, und sendet sie sonst erneut (l2_pump_socket_rx(),
 *     analog zu btx_l2_dbt03_pump_socket_rx()).
 *   - Der periodische L2-Tick (Timeouts fuer Response/Flowcontrol/Control,
 *     Retransmits) wird von v23_link_generate_tx() aus getrieben: die
 *     Funktion wird ohnehin pro RTP-Haeppchen (160 Samples @ 8000 Hz =
 *     20 ms) aufgerufen, die Millisekundenzahl ergibt sich direkt aus
 *     nsamples - kein eigener Wall-Clock-Tick noetig (entspricht in der
 *     Groessenordnung genau dem BTX_L2_DBT03_LOOP_MS=20 der ESP32-Fassung).
 *   - Wichtig: Die physikalische NUL-Byte-Kickstart-Logik (Punkt 2 oben)
 *     bleibt unveraendert VOR der L2-Schicht bestehen - sie gehoert zur
 *     V.23-Traegeraushandlung auf der Telefon-/Audiostrecke und hat mit
 *     FTZ 157 D2 nichts zu tun. Die L2-Sitzung (T.F.I.-Anfrage) startet
 *     daher erst, sobald das Kickstart-NUL tatsaechlich gesendet wurde.
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <spandsp.h>

#include "btx_l2.h"

#define BITBUFFER_SIZE 16
#define DATABITS 8
#define STOPBITS 1
#define RX_DEBUG_BYTES 20

/* Gemeinsamer Zustand zwischen RX- und TX-Seite, wie in der Referenz. */
typedef struct
{
    int answertone; /* -1: kein Traeger vom Terminal, 0: Traeger da, 1: Terminal sendet aktiv Daten */
    int nulsent;     /* 0/1: wurde das Anstoss-NUL-Byte schon gesendet? */
    int finished;    /* 1: Socket zu / Verbindung soll beendet werden */
    int rx_debug_count; /* zum Testen: Anzahl bisher ausgegebener RX-Bytes (max RX_DEBUG_BYTES) */
} link_state_t;

typedef struct
{
    int bitbuffer[BITBUFFER_SIZE];
    int writepos;
    int readpos;
    int fill;
    link_state_t *state;
    btx_l2_bridge *bridge; /* L2-Engine + Server-Puffer, siehe unten */
} modem_data_t;

typedef struct
{
    fsk_rx_state_t *fsk_rx;
    fsk_tx_state_t *fsk_tx;
    modem_data_t rx;
    modem_data_t tx;
    link_state_t state;
    int sock;              /* TCP-Verbindung zum BTX-Server */
    btx_l2_bridge bridge;  /* L2-Sitzung + Chunking-Puffer (siehe btx_l2.h) */
    int l2_started;        /* 0/1: wurde btx_l2_request_tfi() schon ausgeloest? */
} v23_link_t;

/* Von spandsp fuer jedes demodulierte Bit aufgerufen (RX, vom DBT03). */
static void modem_put_bit(void *user_data, int bit)
{
    modem_data_t *rx = (modem_data_t *) user_data;
    int stop, i;

    if (rx->state->answertone <= 0)
    {
        if (bit == SIG_STATUS_CARRIER_UP)
            rx->state->answertone = 0;
        if (bit == 1 && rx->state->answertone == 0)
            rx->state->answertone = 1; /* Terminal sendet jetzt aktiv */
        return;
    }

    if (bit != 0 && bit != 1)
        return; /* sonstige SIG_STATUS_*-Codes ignorieren */

    rx->bitbuffer[rx->writepos] = bit;
    rx->writepos = (rx->writepos + 1) % BITBUFFER_SIZE;
    if (rx->fill < BITBUFFER_SIZE)
    {
        rx->fill++;
    }
    else
    {
        rx->readpos = (rx->readpos + 1) % BITBUFFER_SIZE;
    }

    while (rx->fill >= (1 + DATABITS + STOPBITS))
    {
        if (rx->bitbuffer[rx->readpos] == 0)
        {
            stop = (rx->readpos + 1 + DATABITS) % BITBUFFER_SIZE;
            if (rx->bitbuffer[stop] == 1)
            {
                uint8_t byte = 0;
                for (i = 0; i < DATABITS; i++)
                {
                    if (rx->bitbuffer[(rx->readpos + 1 + i) % BITBUFFER_SIZE])
                        byte |= (1 << i); /* LSB zuerst */
                }

                if (rx->state->rx_debug_count < RX_DEBUG_BYTES)
                {
                    fprintf(stderr, "[V23 RX #%02d] 0x%02X '%c'\n",
                            rx->state->rx_debug_count, byte,
                            isprint(byte) ? byte : '.');
                    fflush(stderr);
                    rx->state->rx_debug_count++;
                }

                /* Byte vom Terminal geht in die Layer-2-Engine statt direkt
                 * auf den Server-Socket: ACK0/ACK1/NAK/WACK/ENQ/EOT und die
                 * T.F.I.-Antwort werden dort interpretiert; reine
                 * Tastaturbytes loest btx_l2_rx_byte() als BTX_L2_EV_KEY aus
                 * und die Bridge sammelt sie in bridge.tx, von wo
                 * l2_flush_keys() sie zum Server sendet. */
                btx_l2_rx_byte(&rx->bridge->link, byte);

                rx->readpos = (rx->readpos + 1 + DATABITS + STOPBITS) % BITBUFFER_SIZE;
                rx->fill -= (1 + DATABITS + STOPBITS);
            }
            else
            {
                rx->fill--;
                rx->readpos = (rx->readpos + 1) % BITBUFFER_SIZE;
            }
        }
        else
        {
            rx->fill--;
            rx->readpos = (rx->readpos + 1) % BITBUFFER_SIZE;
        }
    }
}

/* Von spandsp aufgerufen, wenn das naechste zu sendende Bit gebraucht wird
 * (TX, an das DBT03). */
static int modem_get_bit(void *user_data)
{
    modem_data_t *tx = (modem_data_t *) user_data;
    int i;

    if (tx->writepos == tx->readpos)
    {
        if (tx->state->nulsent > 0)
        {
            uint8_t byte;

            /* Naechstes Byte kommt jetzt aus der L2-Engine (Blockrahmen,
             * ACK-Antworten, T.F.I.-Anfrage, ...) statt aus einem rohen
             * recv() auf dem Socket. btx_l2_tx_byte() liefert 0, wenn
             * gerade nichts ansteht (z.B. IDLE oder am Flowcontrol-Gate) -
             * dann faellt dieser Slot auf reinen Leerlauf (Mark) zurueck,
             * genau wie vorher beim leeren Socket. */
            if (btx_l2_tx_byte(&tx->bridge->link, &byte))
            {
                for (i = 0; i < (DATABITS + STOPBITS); i++)
                {
                    tx->bitbuffer[tx->writepos] = (i >= DATABITS) ? 1 : ((byte & (1 << i)) ? 1 : 0);
                    tx->writepos = (tx->writepos + 1) % BITBUFFER_SIZE;
                }
                return 0; /* Startbit sofort ausliefern */
            }
        }
        else if (tx->state->answertone > 0)
        {
            /* Traeger vom Terminal erkannt -> jetzt das Anstoss-NUL-Byte senden */
            for (i = 0; i < (DATABITS + STOPBITS); i++)
            {
                tx->bitbuffer[tx->writepos] = (i >= DATABITS) ? 1 : 0; /* 0x00 */
                tx->writepos = (tx->writepos + 1) % BITBUFFER_SIZE;
            }
            tx->state->nulsent = 1;
            return 0;
        }
        return 1; /* Leerlauf: reiner Mark-Pegel, kein Framing */
    }

    i = tx->bitbuffer[tx->readpos];
    tx->readpos = (tx->readpos + 1) % BITBUFFER_SIZE;
    return i;
}

static int connect_tcp(const char *host, int port)
{
    struct addrinfo hints, *res, *rp;
    char portstr[16];
    int sock = -1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    snprintf(portstr, sizeof(portstr), "%d", port);

    if (getaddrinfo(host, portstr, &hints, &res) != 0)
        return -1;

    for (rp = res; rp != NULL; rp = rp->ai_next)
    {
        sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock < 0)
            continue;
        if (connect(sock, rp->ai_addr, rp->ai_addrlen) == 0)
            break;
        close(sock);
        sock = -1;
    }
    freeaddrinfo(res);
    if (sock >= 0)
        fcntl(sock, F_SETFL, O_NONBLOCK);
    return sock;
}

/*
 * Liest, was der BTX-Server aktuell zu bieten hat, in bridge->rx - ohne
 * ueber den dort verbleibenden Platz hinaus zu lesen (dieser Platz bremst
 * den Server per TCP-Backpressure, siehe btx_l2.h). Setzt peer_closed bei
 * EOF oder einem harten Fehler, was den Linger-Countdown startet.
 *
 * Analog zu btx_l2_dbt03_pump_socket_rx() im ESP32-Port, hier aber mit
 * normalen BSD-Sockets (der Socket ist bereits O_NONBLOCK, siehe
 * connect_tcp()) statt lwIP/MSG_DONTWAIT-spezifischem Code.
 */
static void l2_pump_socket_rx(btx_l2_bridge *g, int sock)
{
    size_t space;
    ssize_t n;

    if (g->peer_closed)
        return;

    space = sizeof(g->rx) - g->rx_len;
    if (space == 0)
        return; /* Terminal hat noch nicht aufgeholt; erst mal abarbeiten lassen */

    n = recv(sock, g->rx + g->rx_len, space, 0);
    if (n < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            fprintf(stderr, "v23: recv() Fehler (errno=%d), Server-Seite beendet\n", errno);
            g->peer_closed = 1;
            g->linger_ms = BTX_L2_LINGER_MS;
        }
        return;
    }
    if (n == 0)
    {
        fprintf(stderr, "v23: Server hat die Verbindung geschlossen\n");
        g->peer_closed = 1;
        g->linger_ms = BTX_L2_LINGER_MS;
        return;
    }
    g->rx_len += (size_t) n;
}

/*
 * Sendet die von der L2-Engine gesammelten Tastaturbytes (BTX_L2_EV_KEY)
 * zum Server. Anders als die ESP32-Fassung (die einen blockierenden send()
 * auf lwIP annimmt) beruecksichtigt diese Version, dass unser Socket
 * O_NONBLOCK ist und send() daher weniger als tx_len schreiben oder mit
 * EAGAIN scheitern kann - in beiden Faellen bleibt der Rest im Puffer und
 * wird beim naechsten Tick erneut versucht.
 */
static int l2_flush_keys(btx_l2_bridge *g, int sock)
{
    ssize_t n;

    if (g->tx_len == 0)
        return 0;

    n = send(sock, g->tx, g->tx_len, 0);
    if (n < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0; /* Server gerade nicht aufnahmebereit, naechstes Mal wieder versuchen */
        fprintf(stderr, "v23: send() Fehler beim Senden der Tastatureingaben\n");
        return -1;
    }
    if ((size_t) n < g->tx_len)
        memmove(g->tx, g->tx + n, g->tx_len - (size_t) n);
    g->tx_len -= (size_t) n;
    return 0;
}

/*
 * Ein L2-Zyklus: Tastaturbytes zum Server flushen, Server-Bytes einlesen,
 * eine wartende Seite ggf. in die L2-Engine einspeisen, alle L2-Timer um
 * elapsed_ms weiterlaufen lassen. Wird pro v23_link_generate_tx()-Aufruf
 * ausgefuehrt (also einmal pro RTP-Haeppchen, ueblicherweise alle 20 ms) -
 * elapsed_ms ergibt sich direkt aus der Samplezahl, kein separater
 * Wall-Clock-Tick noetig.
 *
 * Die L2-Sitzung selbst (T.F.I.-Anfrage) startet erst, sobald das
 * physikalische Kickstart-NUL tatsaechlich raus ist - vorher wuerde die
 * Zentrale L2-Rahmen senden, bevor das Terminal ueberhaupt zuhoert.
 */
static void v23_link_tick(v23_link_t *l, unsigned elapsed_ms)
{
    if (l->state.nulsent && !l->l2_started)
    {
        l->l2_started = 1;
        btx_l2_request_tfi(&l->bridge.link);
    }

    if (l2_flush_keys(&l->bridge, l->sock) < 0)
        l->state.finished = 1;

    l2_pump_socket_rx(&l->bridge, l->sock);
    btx_l2_bridge_poll(&l->bridge);

    if (elapsed_ms > 0)
    {
        btx_l2_tick(&l->bridge.link, elapsed_ms);
        btx_l2_bridge_tick(&l->bridge, elapsed_ms);
    }

    if (btx_l2_bridge_finished(&l->bridge))
        l->state.finished = 1;
}

/* Baut die TCP-Verbindung auf und initialisiert TX (CH1, 1200bit/s) und
 * RX (CH2, 75bit/s) fest in der Zentralen-/Antwortrolle. Gibt NULL bei
 * Verbindungsfehler zurueck. */
void *v23_link_new(const char *host, int port)
{
    v23_link_t *l = (v23_link_t *) calloc(1, sizeof(*l));
    if (!l)
        return NULL;

    l->state.answertone = -1;
    l->state.nulsent = 0;
    l->state.finished = 0;
    l->state.rx_debug_count = 0;
    l->l2_started = 0;

    int sock = connect_tcp(host, port);
    if (sock < 0)
    {
        free(l);
        return NULL;
    }

    l->sock = sock;
    btx_l2_bridge_init(&l->bridge);

    l->rx.state = &l->state;
    l->rx.bridge = &l->bridge;
    l->tx.state = &l->state;
    l->tx.bridge = &l->bridge;

    l->fsk_tx = fsk_tx_init(NULL, &preset_fsk_specs[FSK_V23CH1], modem_get_bit, &l->tx);
    /* FSK_FRAME_MODE_SYNC (=1, "TRUE" in der Referenz): rohe Bits inkl.
     * SIG_STATUS_*-Codes an modem_put_bit, KEIN automatisches Framing
     * durch spandsp - das machen wir hier bewusst selbst. */
    l->fsk_rx = fsk_rx_init(NULL, &preset_fsk_specs[FSK_V23CH2], FSK_FRAME_MODE_SYNC, modem_put_bit, &l->rx);

    if (!l->fsk_tx || !l->fsk_rx)
    {
        close(sock);
        free(l->fsk_tx);
        free(l->fsk_rx);
        free(l);
        return NULL;
    }

    /* Bewusst KEINE fsk_tx_power()/fsk_rx_signal_cutoff()-Aufrufe: im
     * Test hat das Setzen von -28/-35 dBm0 (Werte aus der Referenz) die
     * Sendeamplitude so weit abgesenkt, dass selbst die tolerante
     * Empfangsschwelle keinen Traeger mehr erkannt hat. Die spandsp-
     * Standardwerte wurden im Loopback-Test verifiziert (Traeger korrekt
     * erkannt, alle Bits korrekt demoduliert) und werden daher belassen.
     * Falls sich am echten DBT03/Telefonkanal Traegererkennungsprobleme
     * zeigen, hier zuerst mit fsk_rx_signal_cutoff() experimentieren,
     * OHNE gleichzeitig fsk_tx_power() zu senken. */

    return l;
}

/* Verarbeitet vom DBT03 empfangenes PCM-Audio (16 Bit, 8000 Hz). */
int v23_link_process_rx(void *handle, const int16_t *samples, int nsamples)
{
    v23_link_t *l = (v23_link_t *) handle;
    return fsk_rx(l->fsk_rx, samples, nsamples);
}

/* Erzeugt PCM-Audio (16 Bit, 8000 Hz) zum Senden an das DBT03. Treibt
 * nebenbei den L2-Tick (siehe v23_link_tick()) - ueblicherweise einmal
 * alle 20 ms (nsamples=160) aus der RTP-Taktung heraus aufgerufen. */
int v23_link_generate_tx(void *handle, int16_t *out, int nsamples)
{
    v23_link_t *l = (v23_link_t *) handle;
    unsigned elapsed_ms = (unsigned) ((long) nsamples * 1000 / 8000);

    v23_link_tick(l, elapsed_ms);

    return fsk_tx(l->fsk_tx, out, nsamples);
}

/* 1, wenn die TCP-Verbindung beendet wurde und aufgelegt werden sollte. */
int v23_link_finished(void *handle)
{
    v23_link_t *l = (v23_link_t *) handle;
    return l->state.finished;
}

void v23_link_free(void *handle)
{
    v23_link_t *l = (v23_link_t *) handle;
    if (!l)
        return;
    fsk_tx_free(l->fsk_tx);
    fsk_rx_free(l->fsk_rx);
    if (l->sock >= 0)
        close(l->sock);
    free(l);
}
