/*
 * selftest.c - Loopback-Selbsttest fuer die V.23-Modulation/Demodulation.
 *
 * Erzeugt fuer beide V.23-Kanaele (1200 bit/s Vorwaerts- und 75 bit/s
 * Rueckkanal) PCM-Audio aus einem bekannten Testtext, speist es direkt
 * wieder in den passenden Demodulator ein und vergleicht das Ergebnis.
 * Dient dazu, die spandsp-Bindung unabhaengig von SIP/Asterisk zu pruefen.
 *
 * Bauen: siehe build.sh (Ziel "selftest")
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <spandsp.h>

typedef struct
{
    const uint8_t *data;
    int len;
    int pos;
} tx_feed_t;

static int feed_get_byte(void *user_data)
{
    tx_feed_t *f = (tx_feed_t *) user_data;
    if (f->pos >= f->len)
        return -1;
    return f->data[f->pos++];
}

typedef struct
{
    uint8_t out[1024];
    int len;
} rx_sink_t;

static void sink_put_byte(void *user_data, int byte)
{
    rx_sink_t *sink = (rx_sink_t *) user_data;
    if (byte >= 0 && sink->len < (int) sizeof(sink->out))
        sink->out[sink->len++] = (uint8_t) byte;
}

static int run_channel(const char *label, int fsk_channel, const char *text)
{
    tx_feed_t feed = { (const uint8_t *) text, (int) strlen(text), 0 };
    async_tx_state_t *async_tx = async_tx_init(NULL, 8, ASYNC_PARITY_NONE, 1, 0, feed_get_byte, &feed);
    fsk_tx_state_t *txs = fsk_tx_init(NULL, &preset_fsk_specs[fsk_channel],
                                          (get_bit_func_t) async_tx_get_bit, async_tx);

    rx_sink_t sink = { .len = 0 };
    async_rx_state_t *async_rx = async_rx_init(NULL, 8, ASYNC_PARITY_NONE, 1, 0, sink_put_byte, &sink);
    fsk_rx_state_t *rxs = fsk_rx_init(NULL, &preset_fsk_specs[fsk_channel], FSK_FRAME_MODE_ASYNC,
                                          (put_bit_func_t) async_rx_put_bit, async_rx);

    /* Genug Samples fuer den Testtext plus etwas Nachlauf erzeugen, in
     * 160er-Haeppchen (= 20ms @ 8000Hz, wie spaeter ueber RTP). Nach dem
     * Text sendet async_tx im Leerlauf kontinuierlich gerahmte
     * Fuellzeichen (0xFF) statt reiner Stille/Ruhetraeger - das ist
     * authentisches V.23-Leerlaufverhalten und wird von echter
     * BTX-Software als Leitungsleerlauf erkannt, nicht als Nutzdaten. */
    int16_t chunk[160];
    int text_len = (int) strlen(text);
    int bits_per_char = 10; /* Start + 8 Datenbits + Stop */
    int baud = (fsk_channel == FSK_V23CH1) ? 1200 : 75;
    int total_samples = (text_len * bits_per_char * 8000) / baud + 800; /* + 100ms Nachlauf */
    int generated = 0;
    while (generated < total_samples)
    {
        int n = fsk_tx(txs, chunk, 160);
        fsk_rx(rxs, chunk, n);
        generated += n;
        if (n == 0)
            break;
    }

    int ok = (sink.len >= text_len) && (memcmp(sink.out, text, text_len) == 0);
    printf("[%s] gesendet=\"%s\" empfangen (erste %d Bytes)=\"%.*s\" "
           "(insgesamt %d Bytes inkl. Leerlauf-Fuellzeichen) -> %s\n",
           label, text, text_len, text_len, sink.out, sink.len, ok ? "OK" : "FEHLER");

    fsk_tx_free(txs);
    async_tx_free(async_tx);
    fsk_rx_free(rxs);
    async_rx_free(async_rx);
    return ok;
}

int main(void)
{
    int ok1 = run_channel("V23CH1 1200bit/s", FSK_V23CH1, "BTXTEST-1200");
    int ok2 = run_channel("V23CH2  75bit/s", FSK_V23CH2, "BTXTEST-75");

    if (ok1 && ok2)
    {
        printf("Selbsttest bestanden.\n");
        return 0;
    }
    printf("Selbsttest FEHLGESCHLAGEN.\n");
    return 1;
}
