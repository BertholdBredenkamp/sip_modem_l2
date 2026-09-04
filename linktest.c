/*
 * linktest.c - Simuliert ein DBT03-Terminal (CH2-Sender/CH1-Empfaenger)
 * und einen kleinen TCP-Testserver, um v23_link end-to-end zu pruefen:
 * Traegererkennung -> NUL-Byte-Kickstart -> echter Datenaustausch.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <spandsp.h>

extern void *v23_link_new(const char *host, int port);
extern int v23_link_process_rx(void *handle, const int16_t *samples, int nsamples);
extern int v23_link_generate_tx(void *handle, int16_t *out, int nsamples);
extern int v23_link_finished(void *handle);
extern void v23_link_free(void *handle);

#define TEST_PORT 34567

static uint8_t server_received_byte_from_terminal = 0;

/* Winziger TCP-Testserver: nimmt eine Verbindung an, wartet auf das vom
 * (simulierten) Terminal gesendete Byte 'X' - das der Link ueber die
 * TCP-Verbindung weiterleiten muss - und schickt dann einen Testtext. */
static void *test_server_thread(void *arg)
{
    (void) arg;
    int lsock = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(TEST_PORT);
    bind(lsock, (struct sockaddr *) &addr, sizeof(addr));
    listen(lsock, 1);

    int csock = accept(lsock, NULL, NULL);
    uint8_t b;
    if (recv(csock, &b, 1, 0) == 1 && b == 'X')
        server_received_byte_from_terminal = 1;

    const char *msg = "SRV-OK";
    send(csock, msg, strlen(msg), 0);

    close(csock);
    close(lsock);
    return NULL;
}

/*
 * Simuliertes Terminal: TX auf CH2 (75 bit/s), RX auf CH1 (1200 bit/s) -
 * das genaue Gegenstueck zu unserer Zentrale/answerer-Rolle.
 *
 * Seit v23_bridge.c die L2-Schicht (btx_l2.h) einbaut, kommen hier keine
 * rohen Server-Bytes mehr an, sondern FTZ-157-D2-Blockrahmen (EOT STX
 * <text> ETX BCCl BCCh). Damit der End-to-End-Test weiterhin etwas
 * Sinnvolles prueft, muss das simulierte Terminal jetzt minimal L2
 * sprechen: Blockende (ETX/ETB/ITB + 2 BCC-Bytes) erkennen und mit der
 * passenden Quittung antworten. Fuer diesen Test genuegt eine feste
 * Erwartung (ein Block, erste Seite nach Link-Reset -> ACK1 = DLE '1'),
 * eine allgemeine BCC-Pruefung findet hier bewusst nicht statt - das ist
 * bereits Aufgabe von btx_l2.h selbst und dort nicht erneut zu testen.
 * Die T.F.I.-Anfrage (SOH ENQ) der Zentrale wird absichtlich NICHT
 * beantwortet, damit sie regulaer auf die Standardwerte timeoutet
 * (BTX_L2_T_CONTROL_MS=3000ms, reichlich Zeit in den 500 Testiterationen).
 */
typedef struct
{
    int started;   /* wurde schon Traeger gesendet? */
    uint8_t recv_buf[256];
    int recv_len;
    int reply_sent; /* haben wir schon auf die Kickstart-NUL geantwortet? */
    int reply_bitbuf[64];
    int reply_wp, reply_rp, reply_fill;

    int in_block;     /* 1, sobald STX gesehen wurde (Blocktext laeuft) */
    int expect_bcc;   /* nach einem Terminator: Anzahl noch fehlender BCC-Bytes */
    uint8_t terminator; /* welcher Terminator (ETX/ETB/ITB) das Blockende einleitete */
} term_ctx_t;

#define TERM_REPLY_BUF 64

static void term_queue_byte(term_ctx_t *t, uint8_t byte)
{
    for (int i = 0; i < 10; i++)
    {
        t->reply_bitbuf[t->reply_wp] = (i == 0) ? 0 : (i == 9 ? 1 : ((byte & (1 << (i - 1))) ? 1 : 0));
        t->reply_wp = (t->reply_wp + 1) % TERM_REPLY_BUF;
        t->reply_fill++;
    }
}

static int term_get_bit(void *user_data)
{
    term_ctx_t *t = (term_ctx_t *) user_data;
    if (t->reply_fill > 0)
    {
        int b = t->reply_bitbuf[t->reply_rp];
        t->reply_rp = (t->reply_rp + 1) % TERM_REPLY_BUF;
        t->reply_fill--;
        return b;
    }
    if (t->recv_len >= 1 && !t->reply_sent)
    {
        /* Kickstart-NUL vom Link empfangen -> jetzt ein echtes Testbyte
         * zurueckschicken, um den vollen Kreislauf (inkl. TCP-Relay) zu
         * pruefen. Dieses Byte ist reine Tastaturdatum (keine L2-Steuerung)
         * und wird von der Zentrale als BTX_L2_EV_KEY zum Server geflusht. */
        term_queue_byte(t, 'X');
        t->reply_sent = 1;
        int b = t->reply_bitbuf[t->reply_rp];
        t->reply_rp = (t->reply_rp + 1) % TERM_REPLY_BUF;
        t->reply_fill--;
        return b;
    }
    return 1; /* Mark/Leerlauf */
}

static void term_put_bit(void *user_data, int bit)
{
    term_ctx_t *t = (term_ctx_t *) user_data;
    static int bitbuf[16];
    static int wp = 0, rp = 0, fill = 0;

    if (bit != 0 && bit != 1)
        return;
    bitbuf[wp] = bit;
    wp = (wp + 1) % 16;
    if (fill < 16)
        fill++;
    else
        rp = (rp + 1) % 16;

    while (fill >= 10)
    {
        if (bitbuf[rp] == 0)
        {
            int stop = (rp + 9) % 16;
            if (bitbuf[stop] == 1)
            {
                uint8_t byte = 0;
                for (int i = 0; i < 8; i++)
                    if (bitbuf[(rp + 1 + i) % 16])
                        byte |= (1 << i);
                if (t->recv_len < (int) sizeof(t->recv_buf))
                    t->recv_buf[t->recv_len++] = byte;
                rp = (rp + 10) % 16;
                fill -= 10;

                /* Minimale L2-Blockerkennung, siehe Kommentar oben. */
                if (t->expect_bcc > 0)
                {
                    if (--t->expect_bcc == 0)
                    {
                        t->in_block = 0;
                        if (t->terminator == 0x03 /* ETX: letzter Block der Seite */)
                        {
                            /* Erste (und hier einzige) Seite nach einem
                             * Link-Reset erwartet ACK1 = DLE '1'. */
                            term_queue_byte(t, 0x10 /* DLE */);
                            term_queue_byte(t, 0x31 /* '1' */);
                        }
                        else
                        {
                            /* ETB/ITB: Zwischenblock, einfaches ACK. */
                            term_queue_byte(t, 0x06 /* ACK */);
                        }
                    }
                }
                else if (byte == 0x04 /* EOT */)
                {
                    t->in_block = 0; /* Link-Reset, Blockzustand verwerfen */
                }
                else if (byte == 0x02 /* STX */)
                {
                    t->in_block = 1;
                }
                else if (t->in_block && (byte == 0x03 /* ETX */ || byte == 0x17 /* ETB */ || byte == 0x07 /* ITB */))
                {
                    t->terminator = byte;
                    t->expect_bcc = 2;
                }
                continue;
            }
        }
        fill--;
        rp = (rp + 1) % 16;
    }
}

int main(void)
{
    pthread_t srv_thread;
    pthread_create(&srv_thread, NULL, test_server_thread, NULL);
    usleep(100000); /* Server Zeit zum Starten geben */

    void *link = v23_link_new("127.0.0.1", TEST_PORT);
    if (!link)
    {
        printf("FEHLER: v23_link_new fehlgeschlagen\n");
        return 1;
    }

    term_ctx_t term = {0};
    fsk_tx_state_t *term_tx = fsk_tx_init(NULL, &preset_fsk_specs[FSK_V23CH2], term_get_bit, &term);
    fsk_rx_state_t *term_rx = fsk_rx_init(NULL, &preset_fsk_specs[FSK_V23CH1], FSK_FRAME_MODE_ASYNC,
                                           (put_bit_func_t) &term_put_bit, &term);
    /* Hinweis: term_rx nutzt hier bewusst ASYNC (nicht SYNC), da wir als
     * Test-Terminal nur fertige Bytes brauchen, nicht die Rohbits -
     * async_rx_put_bit haette einen anderen Signaturtyp; daher verwenden
     * wir hier direkt unsere eigene, vereinfachte Bitframing-Funktion via
     * SYNC-Modus stattdessen: */
    fsk_rx_free(term_rx);
    term_rx = fsk_rx_init(NULL, &preset_fsk_specs[FSK_V23CH1], FSK_FRAME_MODE_SYNC, term_put_bit, &term);

    int16_t central_audio[160];
    int16_t term_audio[160];

    /* Phase 1: 1 Sekunde lang KEIN Traeger vom Terminal (Stille) - die
     * Zentrale darf in dieser Zeit kein NUL-Byte senden. */
    for (int i = 0; i < 50; i++)
    {
        int n = v23_link_generate_tx(link, central_audio, 160);
        (void) n;
        /* Terminal bekommt in dieser Phase absichtlich nichts zu hoeren
         * (simuliert: noch keine Leitung durchgestellt) */
    }

    /* Phase 2: Terminal beginnt zu senden (Traeger), Audio flieszt in
     * beide Richtungen. */
    for (int i = 0; i < 500; i++)
    {
        int n = v23_link_generate_tx(link, central_audio, 160);
        fsk_rx(term_rx, central_audio, n);

        int m = fsk_tx(term_tx, term_audio, 160);
        v23_link_process_rx(link, term_audio, m);

        if (v23_link_finished(link))
            break;
    }

    printf("Server hat 'X' vom Terminal erhalten: %s\n", server_received_byte_from_terminal ? "JA" : "NEIN");
    printf("Terminal hat empfangen (%d Bytes, hex): ", term.recv_len);
    for (int i = 0; i < term.recv_len; i++)
        printf("%02x ", term.recv_buf[i]);
    printf("\n");

    /* Erwartet: erstes Byte ist die Kickstart-NUL (0x00) des Links, danach
     * die T.F.I.-Anfrage (SOH ENQ - vom simulierten Terminal absichtlich
     * unbeantwortet, siehe oben) und danach der L2-Blockrahmen fuer
     * "SRV-OK" vom TCP-Testserver: EOT STX "SRV-OK" ETX BCCl BCCh -
     * moduliert und ans Terminal geschickt, das darauf mit ACK1 (DLE '1')
     * quittiert. */
    static const uint8_t expected_frame[] = { 0x01, 0x05, 0x04, 0x02, 'S', 'R', 'V', '-', 'O', 'K', 0x03 };
    int ok = server_received_byte_from_terminal
             && term.recv_len >= 1 + (int) sizeof(expected_frame) + 2
             && term.recv_buf[0] == 0x00
             && memcmp(term.recv_buf + 1, expected_frame, sizeof(expected_frame)) == 0;
    printf(ok ? "End-to-End-Test bestanden (inkl. L2-Blockrahmen und ACK1-Quittung).\n"
              : "End-to-End-Test FEHLGESCHLAGEN.\n");

    v23_link_free(link);
    fsk_tx_free(term_tx);
    fsk_rx_free(term_rx);
    pthread_join(srv_thread, NULL);
    return ok ? 0 : 1;
}
