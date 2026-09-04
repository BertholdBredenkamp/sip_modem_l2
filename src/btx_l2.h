/*
 * Bildschirmtext (BTX) layer-2 protocol, condensed into one header.
 *
 * Ported from the btx_modem_sip project, which implements the FTZ 157 D2
 * exchange (Btx Zentrale) side that talks to a real DBT-03 terminal. This is
 * a single-file merge of that project's btx_proto.h, crc16.{h,c},
 * btx_frame.{h,c}, btx_link.{h,c} and the chunking half of gateway.{h,c}
 * (the non-blocking-socket half of gateway.c is left out; the caller
 * already owns its own socket). gateway.c also cut chunks on
 * presentation-layer sequence boundaries, justified there by an observation
 * on a real page rather than a spec citation; simple fixed-size chunking is
 * used here instead, since nothing in FTZ 157 D2 requires the former and it
 * added a real bug surface (see git history) for a benefit that may have
 * been misattributed in the first place.
 *
 * ESP32 port notes (Berthold Bredenkamp, DBT03-ESP32-Projekt):
 *  - l2_log_ts() used clock_gettime(CLOCK_REALTIME) + localtime_r() on the
 *    host build. On the ESP32 there is normally no RTC/NTP sync, so that
 *    would just print epoch-adjacent garbage. It has been swapped for a
 *    millisecond-since-boot stamp from esp_timer_get_time(), which is what
 *    actually matters for checking response/timeout timing on real
 *    hardware. The host (non-ESP_PLATFORM) build keeps the original
 *    wall-clock version so this file still compiles standalone for testing.
 *  - Everything else (framing, CRC-16/ARC BCC, ACK/NAK/WACK state machine,
 *    T.F.I. negotiation, chunking bridge) is unchanged from the source
 *    project.
 *
 * Two layers of API:
 *
 *   btx_l2_*        the layer-2 exchange: bytes in (btx_l2_rx_byte), bytes
 *                    out (btx_l2_tx_byte), a millisecond tick, and whole
 *                    pages queued with btx_l2_send_page(). Knows nothing
 *                    about sockets or SPI.
 *
 *   btx_l2_bridge_*  cuts a raw byte stream (e.g. from a BTX-over-IP server)
 *                    into fixed-size pages and feeds them to the embedded
 *                    btx_l2_bridge.link, resending them if the link
 *                    resets before they are acknowledged. The caller is
 *                    still responsible for actually moving bytes to and from
 *                    a socket; this only manages the buffers in between.
 *
 * Also provides btx_l2_parity_encode/decode, an unrelated byte-level 7-bit +
 * parity transform for the classic BTX/CEPT convention of using bit 7 as a
 * parity check over the low 7 bits. This is independent of everything above
 * and is only ever applied on the raw passthrough path, never together with
 * layer-2 framing (overwriting bit 7 of a layer-2 control byte would corrupt
 * it, and layer 2 already always talks 8-bit clean).
 */

#ifndef BTX_LAYER2_H
#define BTX_LAYER2_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_timer.h"
#else
#include <time.h>
#endif

/*
 * Every "l2:" log line is prefixed with a timestamp, so a real hardware run
 * can be checked for how much time actually elapses between events (e.g. a
 * block going out and its ACK/NAK coming back) rather than guessing from
 * log order alone.
 *
 * Set BTX_L2_QUIET to 1 (compile flag) to drop all L2_LOG output, e.g. once
 * the protocol is known-working and the logging overhead on every block is
 * not wanted any more.
 */
#ifndef BTX_L2_QUIET
#define BTX_L2_QUIET 0
#endif

#ifdef ESP_PLATFORM
static void l2_log_ts(void)
{
	int64_t us = esp_timer_get_time();
	fprintf(stderr, "[%06lld.%03lld] ", (long long)(us / 1000000),
	        (long long)((us / 1000) % 1000));
}
#else
static void l2_log_ts(void)
{
	struct timespec ts;
	struct tm tmv;

	clock_gettime(CLOCK_REALTIME, &ts);
	localtime_r(&ts.tv_sec, &tmv);
	fprintf(stderr, "[%02d:%02d:%02d.%03ld] ", tmv.tm_hour, tmv.tm_min,
	        tmv.tm_sec, ts.tv_nsec / 1000000);
}
#endif

#if BTX_L2_QUIET
#define L2_LOG(...) do { } while (0)
#else
#define L2_LOG(...) do { l2_log_ts(); fprintf(stderr, __VA_ARGS__); } while (0)
#endif

/* ------------------------------------------------------- wire constants */

/* Single-character line control functions, from the CEPT C0 control set. */
#define BTX_L2_SOH 0x01 /* start of heading; T.F.I. and error codes, no BCC */
#define BTX_L2_STX 0x02 /* start of text; excluded from the BCC it starts */
#define BTX_L2_ETX 0x03 /* end of text; included in BCC, BCC follows */
#define BTX_L2_EOT 0x04 /* end of transmission; resets the link and ACK0/1 state */
#define BTX_L2_ENQ 0x05 /* enquiry; forward abort or "repeat your response" */
#define BTX_L2_ACK 0x06 /* acknowledges an intermediate block (ITB) only */
#define BTX_L2_ITB 0x07 /* end of intermediate block; included in BCC, BCC follows */
#define BTX_L2_DLE 0x10 /* data link escape; introduces the two-byte responses */
#define BTX_L2_NAK 0x15 /* negative acknowledgement */
#define BTX_L2_ETB 0x17 /* end of transmission block; included in BCC, BCC follows */

/* Two-character responses: DLE followed by one of these. */
#define BTX_L2_ACK0_2ND 0x30 /* DLE '0' */
#define BTX_L2_ACK1_2ND 0x31 /* DLE '1' */
#define BTX_L2_WACK_2ND 0x3B /* DLE ';' - "temporarily not ready to receive" */

/* Block/message sizes. ITB size is negotiated by the T.F.I.; 32 is default. */
#define BTX_L2_ITB_SIZE_DEFAULT 32
#define BTX_L2_ITB_SIZE_MAX 256
#define BTX_L2_MESSAGE_MAX 2048

/* Timers, in milliseconds. */
#define BTX_L2_T_RESPONSE_MS 3000 /* awaiting a response to a block or ENQ */
#define BTX_L2_T_FLOWCTRL_MS 3000 /* awaiting an outstanding ITB acknowledgement */
#define BTX_L2_T_CONTROL_MS 3000  /* awaiting anything while in control mode */

#define BTX_L2_ENQ_RETRY_LIMIT 7

/* --------------------------------------------------------------- framing */

/* Largest block this can emit: EOT STX <=256 text bytes> ETX BCCl BCCh. */
#define BTX_L2_FRAME_BUFSZ (BTX_L2_ITB_SIZE_MAX + 5)

#define BTX_L2_FRAME_LEAD_EOT 0x01u /* prefix an EOT to reset the link */
#define BTX_L2_FRAME_LEAD_STX 0x02u /* prefix an STX to start text mode */

#define BTX_L2_CRC_INIT 0x0000u

/* CRC-16/ARC (poly x^16+x^15+x^2+1, reflected 0xA001), used as the Block
 * Check Character. Register starts at zero, no final XOR. */
static uint16_t l2_crc16_update(uint16_t crc, uint8_t byte)
{
	int i;

	crc = (uint16_t)(crc ^ byte);
	for (i = 0; i < 8; i++) {
		if (crc & 1)
			crc = (uint16_t)((crc >> 1) ^ 0xA001);
		else
			crc = (uint16_t)(crc >> 1);
	}
	return crc;
}

/* Number of intermediate blocks a message of len splits into at itb_size. */
static size_t l2_frame_blocks(size_t len, size_t itb_size)
{
	if (len == 0)
		return 1; /* a zero-length message is still sent as STX ETX BCC */
	return (len + itb_size - 1) / itb_size;
}

/*
 * Serialise block `index` of msg[0..len) into out (>= BTX_L2_FRAME_BUFSZ
 * bytes). Wire format: EOT STX <text> ITB BCCl BCCh <text> ITB BCCl BCCh ...
 * <text> ETX BCCl BCCh. Each block's BCC accumulation restarts and covers
 * only that block's text plus its own terminator; STX/EOT are excluded.
 * Returns the number of bytes written, or 0 if index/cap are out of range.
 */
static size_t l2_frame_block(const uint8_t *msg, size_t len, size_t itb_size,
                              int use_itb, size_t index, unsigned flags,
                              uint8_t *out, size_t cap)
{
	size_t blocks = l2_frame_blocks(len, itb_size);
	size_t offset, text_len, i, n = 0;
	int is_last;
	uint8_t terminator;
	uint16_t bcc;

	if (index >= blocks || cap < BTX_L2_FRAME_BUFSZ)
		return 0;

	offset = index * itb_size;
	text_len = len - offset;
	if (text_len > itb_size)
		text_len = itb_size;

	is_last = (index + 1 == blocks);
	if (!is_last)
		terminator = use_itb ? BTX_L2_ITB : BTX_L2_ETB;
	else
		terminator = BTX_L2_ETX;

	if (flags & BTX_L2_FRAME_LEAD_EOT)
		out[n++] = BTX_L2_EOT;
	if (flags & BTX_L2_FRAME_LEAD_STX)
		out[n++] = BTX_L2_STX;

	bcc = BTX_L2_CRC_INIT;
	for (i = 0; i < text_len; i++) {
		out[n++] = msg[offset + i];
		bcc = l2_crc16_update(bcc, msg[offset + i]);
	}

	out[n++] = terminator;
	bcc = l2_crc16_update(bcc, terminator);

	/* Least significant byte goes onto the line first. */
	out[n++] = (uint8_t)(bcc & 0xFF);
	out[n++] = (uint8_t)(bcc >> 8);

	return n;
}

/* ------------------------------------------------------------ link layer */

typedef enum {
	BTX_L2_EV_KEY,          /* a keyboard byte arrived from the terminal */
	BTX_L2_EV_TFI,          /* the Terminal Facility Identifier was settled */
	BTX_L2_EV_PAGE_SENT,    /* the queued page was fully acknowledged */
	BTX_L2_EV_SEVERE_ERROR, /* recovery failed; the link was reset */
	BTX_L2_EV_TERMINAL_EOT  /* the terminal reset the link with EOT */
} btx_l2_event;

/* data carries the keyboard byte for BTX_L2_EV_KEY and is 0 otherwise. */
typedef void (*btx_l2_app_fn)(void *ctx, btx_l2_event ev, uint8_t data);

/* Responses the terminal can send. Anything else is keyboard data. */
typedef enum {
	BTX_L2_RESP_ACK,
	BTX_L2_RESP_ACK0,
	BTX_L2_RESP_ACK1,
	BTX_L2_RESP_NAK,
	BTX_L2_RESP_WACK,
	BTX_L2_RESP_ENQ,
	BTX_L2_RESP_EOT
} btx_l2_resp;

static const char *l2_resp_name(btx_l2_resp r)
{
	switch (r) {
	case BTX_L2_RESP_ACK: return "ACK";
	case BTX_L2_RESP_ACK0: return "ACK0";
	case BTX_L2_RESP_ACK1: return "ACK1";
	case BTX_L2_RESP_NAK: return "NAK";
	case BTX_L2_RESP_WACK: return "WACK";
	case BTX_L2_RESP_ENQ: return "ENQ";
	case BTX_L2_RESP_EOT: return "EOT";
	}
	return "?";
}

#define BTX_L2_RESP_QUEUE 8

typedef enum {
	BTX_L2_ST_IDLE,       /* nothing to send */
	BTX_L2_ST_TFI_WAIT,   /* SOH ENQ sent, waiting for the identifier */
	BTX_L2_ST_TEXT,       /* emitting the text of the current block */
	BTX_L2_ST_GATE,       /* at the gate, needs the previous block's response */
	BTX_L2_ST_TAIL,       /* emitting the terminator and its two BCC bytes */
	BTX_L2_ST_FINAL_WAIT, /* whole page sent, awaiting ACK0/ACK1 */
	BTX_L2_ST_ABORT_WAIT  /* ENQ sent, awaiting the NAK that ends a forward abort */
} btx_l2_state;

typedef struct {
	btx_l2_app_fn cb;
	void *ctx;
	btx_l2_state state;

	/* Negotiated by the T.F.I., defaults apply until it settles. */
	size_t itb_size;
	int use_itb;
	int tfi_done;

	/* The page being sent. Owned here, because retransmission needs it. */
	uint8_t msg[BTX_L2_MESSAGE_MAX];
	size_t msg_len;
	size_t nblocks;
	size_t block; /* block currently being emitted */

	/* Serialised bytes of the current block. */
	uint8_t out[BTX_L2_FRAME_BUFSZ];
	size_t out_len;
	size_t out_pos;
	size_t out_gate; /* index of the terminator, where the gate check happens */

	/* Control bytes to emit ahead of everything else (ENQ, EOT, SOH ENQ). */
	uint8_t ctl[4];
	size_t ctl_len;
	size_t ctl_pos;

	/* acked = number of leading blocks known to be acknowledged; block n's
	 * terminator may only go out once acked >= n. */
	size_t acked;
	int ack_parity; /* 0 expects ACK0 for the final block, 1 expects ACK1 */

	size_t run_first;      /* first block of the current transmission run */
	int discard_responses; /* set while the first block of a run goes out */
	unsigned retransmits;  /* retransmissions attempted for this page */
	int final_pending;     /* a WACK interrupted the wait for the final ack */

	int abort_reason; /* why an ENQ is outstanding */
	btx_l2_resp resp[BTX_L2_RESP_QUEUE];
	size_t resp_count;

	/* Receive side: reassembles DLE pairs and SOH frames. */
	int rx_dle;
	int rx_soh;
	uint8_t rx_soh_buf[8];
	size_t rx_soh_len;

	unsigned timer_ms; /* 0 when no timer is running */
	unsigned enq_retries;
} btx_l2;

/*
 * Which alternating acknowledgement the terminal is expected to send for the
 * first page after a link reset. Following the Bisync convention, the first
 * block after a reset counts as block one, so it expects ACK1.
 */
#define BTX_L2_FIRST_ACK_PARITY 1

/* Give up and report a severe error after this many retransmissions. */
#define BTX_L2_RETRANSMIT_LIMIT 3

/* Why an ENQ was sent, which decides how its responses are interpreted. */
enum {
	L2_ABORT_NONE,
	L2_ABORT_NAK,   /* a NAK arrived, so exactly one NAK is owed in return */
	L2_ABORT_TO,    /* a block response timed out mid page, two are owed */
	L2_ABORT_FINAL, /* the final block's response timed out */
	L2_ABORT_WACK   /* the terminal asked us to wait */
};

static const char *l2_abort_reason_name(int reason)
{
	switch (reason) {
	case L2_ABORT_NONE: return "none";
	case L2_ABORT_NAK: return "nak";
	case L2_ABORT_TO: return "timeout";
	case L2_ABORT_FINAL: return "final-timeout";
	case L2_ABORT_WACK: return "wack";
	}
	return "?";
}

static const char *btx_l2_state_name(const btx_l2 *l)
{
	switch (l->state) {
	case BTX_L2_ST_IDLE: return "idle";
	case BTX_L2_ST_TFI_WAIT: return "tfi-wait";
	case BTX_L2_ST_TEXT: return "text";
	case BTX_L2_ST_GATE: return "gate";
	case BTX_L2_ST_TAIL: return "tail";
	case BTX_L2_ST_FINAL_WAIT: return "final-wait";
	case BTX_L2_ST_ABORT_WAIT: return "abort-wait";
	}
	return "?";
}

static void l2_emit_ctl(btx_l2 *l, const uint8_t *bytes, size_t n)
{
	size_t i;

	l->ctl_len = 0;
	l->ctl_pos = 0;
	for (i = 0; i < n && i < sizeof(l->ctl); i++)
		l->ctl[l->ctl_len++] = bytes[i];
}

static void l2_notify(btx_l2 *l, btx_l2_event ev, uint8_t data)
{
	if (l->cb != NULL)
		l->cb(l->ctx, ev, data);
}

static void l2_go_idle(btx_l2 *l)
{
	l->state = BTX_L2_ST_IDLE;
	l->timer_ms = 0;
	l->resp_count = 0;
	l->abort_reason = L2_ABORT_NONE;
	l->msg_len = 0;
}

static void l2_severe_error(btx_l2 *l)
{
	static const uint8_t eot = BTX_L2_EOT;
	size_t i;

	L2_LOG(
	        "l2: severe error in %s (acked %zu of %zu blocks, abort_reason=%s, responses=[",
	        btx_l2_state_name(l), l->acked, l->nblocks,
	        l2_abort_reason_name(l->abort_reason));
	for (i = 0; i < l->resp_count; i++)
		fprintf(stderr, "%s%s", i ? "," : "", l2_resp_name(l->resp[i]));
	fprintf(stderr, "]), resetting link\n");
	l2_emit_ctl(l, &eot, 1);
	l2_go_idle(l);
	l2_notify(l, BTX_L2_EV_SEVERE_ERROR, 0);
}

static void l2_load_block(btx_l2 *l)
{
	unsigned flags = 0;

	if (l->block == l->run_first) {
		/* First block of a run opens text mode with STX; the EOT that
		 * resets the link goes in front only for the message's block 0. */
		flags |= BTX_L2_FRAME_LEAD_STX;
		if (l->block == 0)
			flags |= BTX_L2_FRAME_LEAD_EOT;
	}
	if (!l->use_itb)
		flags |= BTX_L2_FRAME_LEAD_STX; /* every block is its own Message Block */

	l->out_len = l2_frame_block(l->msg, l->msg_len, l->itb_size, l->use_itb,
	                             l->block, flags, l->out, sizeof(l->out));
	l->out_pos = 0;
	l->out_gate = l->out_len - 3; /* tail = terminator + 2 BCC bytes */
	l->state = BTX_L2_ST_TEXT;

	{
		size_t i;

		L2_LOG("l2: tx block %zu/%zu (%zu bytes):", l->block,
		        l->nblocks, l->out_len);
		for (i = 0; i < l->out_len; i++)
			fprintf(stderr, " %02x", l->out[i]);
		fprintf(stderr, "\n");
	}
}

static void l2_start_run(btx_l2 *l, size_t from_block)
{
	l->block = from_block;
	l->run_first = from_block;
	l->acked = from_block;
	l->resp_count = 0;
	l->abort_reason = L2_ABORT_NONE;
	l->enq_retries = 0;
	l->timer_ms = 0;

	/* While the first block of a run is going out, every response except
	 * keyboard data is discarded: the terminal cannot have anything to say
	 * about it yet. */
	l->discard_responses = 1;

	if (from_block == 0)
		l->ack_parity = BTX_L2_FIRST_ACK_PARITY;

	l2_load_block(l);
}

static void l2_retransmit(btx_l2 *l, size_t from_block)
{
	if (++l->retransmits > BTX_L2_RETRANSMIT_LIMIT) {
		L2_LOG("l2: retransmit limit of %d reached\n",
		        BTX_L2_RETRANSMIT_LIMIT);
		l2_severe_error(l);
		return;
	}
	L2_LOG("l2: retransmitting from block %zu (attempt %u/%d)\n",
	        from_block, l->retransmits, BTX_L2_RETRANSMIT_LIMIT);
	l2_start_run(l, from_block);
}

static void l2_send_enq(btx_l2 *l, int reason)
{
	static const uint8_t enq = BTX_L2_ENQ;

	L2_LOG(
	        "l2: sending ENQ, reason=%s (state=%s block=%zu acked=%zu enq_retries=%u)\n",
	        l2_abort_reason_name(reason), btx_l2_state_name(l), l->block,
	        l->acked, l->enq_retries + 1);
	if (++l->enq_retries > BTX_L2_ENQ_RETRY_LIMIT) {
		L2_LOG("l2: ENQ retry limit of %d reached\n",
		        BTX_L2_ENQ_RETRY_LIMIT);
		l2_severe_error(l);
		return;
	}
	l2_emit_ctl(l, &enq, 1);
	l->abort_reason = reason;
	l->state = BTX_L2_ST_ABORT_WAIT;
	l->resp_count = 0;
	l->timer_ms = BTX_L2_T_RESPONSE_MS;
}

static void btx_l2_init(btx_l2 *l, btx_l2_app_fn cb, void *ctx)
{
	memset(l, 0, sizeof(*l));
	l->cb = cb;
	l->ctx = ctx;
	l->state = BTX_L2_ST_IDLE;
	l->itb_size = BTX_L2_ITB_SIZE_DEFAULT;
	l->use_itb = 1;
	l->ack_parity = BTX_L2_FIRST_ACK_PARITY;
}

static int btx_l2_busy(const btx_l2 *l)
{
	return l->state != BTX_L2_ST_IDLE || l->ctl_pos < l->ctl_len;
}

/* Ask the terminal for its facilities. Optional: defaults apply until this
 * completes or times out. */
static void btx_l2_request_tfi(btx_l2 *l)
{
	static const uint8_t soh_enq[] = { BTX_L2_SOH, BTX_L2_ENQ };

	if (l->state != BTX_L2_ST_IDLE)
		return;

	l2_emit_ctl(l, soh_enq, sizeof(soh_enq));
	l->state = BTX_L2_ST_TFI_WAIT;
	l->timer_ms = BTX_L2_T_CONTROL_MS;
}

/* Queue a page. Returns 0 on success, -1 if one is already in flight or the
 * page is longer than BTX_L2_MESSAGE_MAX. */
static int btx_l2_send_page(btx_l2 *l, const uint8_t *data, size_t len)
{
	if (len > BTX_L2_MESSAGE_MAX)
		return -1;
	if (l->state != BTX_L2_ST_IDLE)
		return -1;

	memcpy(l->msg, data, len);
	l->msg_len = len;
	l->nblocks = l2_frame_blocks(l->msg_len, l->itb_size);
	l->retransmits = 0;
	l2_start_run(l, 0);
	return 0;
}

/* ----------------------------------------------------------- receive side */

/*
 * Map the T.F.I. block size parameter. '0' means the terminal does not
 * support ITB at all and wants 256-byte Message Blocks; '2' is the default.
 */
static void l2_apply_tfi_parameter(btx_l2 *l, uint8_t parameter)
{
	switch (parameter) {
	case '0': l->use_itb = 0; l->itb_size = 256; break;
	case '2': l->use_itb = 1; l->itb_size = 32; break;
	case '3': l->use_itb = 1; l->itb_size = 64; break;
	case '4': l->use_itb = 1; l->itb_size = 128; break;
	case '5': l->use_itb = 1; l->itb_size = 256; break;
	default: break; /* '1' is not supported; keep the defaults */
	}
}

static void l2_parse_tfi(btx_l2 *l)
{
	size_t i = 0;

	L2_LOG("l2: T.F.I. reply, %zu byte(s):", l->rx_soh_len);
	for (i = 0; i < l->rx_soh_len; i++)
		fprintf(stderr, " %02x", l->rx_soh_buf[i]);
	fprintf(stderr, "\n");

	i = 0;
	while (i < l->rx_soh_len) {
		uint8_t c = l->rx_soh_buf[i];

		if (c == '@' && i + 1 < l->rx_soh_len) {
			l2_apply_tfi_parameter(l, l->rx_soh_buf[i + 1]);
			i += 2;
		} else if (c == '/') {
			break; /* specific error code; not interpreted here */
		} else {
			i++; /* a decoder class byte, not acted on */
		}
	}

	L2_LOG("l2: T.F.I. applied: use_itb=%d itb_size=%zu\n",
	        l->use_itb, l->itb_size);

	l->tfi_done = 1;
	if (l->state == BTX_L2_ST_TFI_WAIT) {
		l->state = BTX_L2_ST_IDLE;
		l->timer_ms = 0;
	}
	l2_notify(l, BTX_L2_EV_TFI, 0);
}

static void l2_handle_response(btx_l2 *l, btx_l2_resp r);

/* One received byte from the terminal. */
static void btx_l2_rx_byte(btx_l2 *l, uint8_t byte)
{
	/*
	 * An SOH frame carries the T.F.I. or a specific error code and has no
	 * BCC. It is terminated by ETB, ETX or EOT.
	 */
	if (l->rx_soh) {
		if (byte == BTX_L2_ETB || byte == BTX_L2_ETX || byte == BTX_L2_EOT) {
			l->rx_soh = 0;
			l2_parse_tfi(l);
		} else if (l->rx_soh_len < sizeof(l->rx_soh_buf)) {
			l->rx_soh_buf[l->rx_soh_len++] = byte;
		}
		return;
	}

	if (l->rx_dle) {
		l->rx_dle = 0;
		switch (byte) {
		case BTX_L2_ACK0_2ND: l2_handle_response(l, BTX_L2_RESP_ACK0); return;
		case BTX_L2_ACK1_2ND: l2_handle_response(l, BTX_L2_RESP_ACK1); return;
		case BTX_L2_WACK_2ND: l2_handle_response(l, BTX_L2_RESP_WACK); return;
		case BTX_L2_EOT: l2_handle_response(l, BTX_L2_RESP_EOT); return;
		default:
			/* Not a sequence we know; treat both bytes as keyboard data. */
			L2_LOG(
			        "l2: unrecognized DLE sequence (DLE %02x), treating as keyboard data\n",
			        byte);
			l2_notify(l, BTX_L2_EV_KEY, BTX_L2_DLE);
			l2_notify(l, BTX_L2_EV_KEY, byte);
			return;
		}
	}

	switch (byte) {
	case BTX_L2_SOH: l->rx_soh = 1; l->rx_soh_len = 0; return;
	case BTX_L2_DLE: l->rx_dle = 1; return;
	case BTX_L2_ACK: l2_handle_response(l, BTX_L2_RESP_ACK); return;
	case BTX_L2_NAK: l2_handle_response(l, BTX_L2_RESP_NAK); return;
	case BTX_L2_ENQ: l2_handle_response(l, BTX_L2_RESP_ENQ); return;
	case BTX_L2_EOT: l2_handle_response(l, BTX_L2_RESP_EOT); return;
	default:
		/* Unframed keyboard data, which may arrive at any moment. */
		L2_LOG("l2: rx keyboard byte 0x%02x (state=%s)\n", byte,
		        btx_l2_state_name(l));
		l2_notify(l, BTX_L2_EV_KEY, byte);
		return;
	}
}

/* Finish the page successfully. */
static void l2_message_done(btx_l2 *l)
{
	l->ack_parity ^= 1;
	l2_go_idle(l);
	l2_notify(l, BTX_L2_EV_PAGE_SENT, 0);
}

static int l2_is_expected_final_ack(const btx_l2 *l, btx_l2_resp r)
{
	return (l->ack_parity == 0) ? (r == BTX_L2_RESP_ACK0) : (r == BTX_L2_RESP_ACK1);
}

/* Responses collected while an ENQ is outstanding. */
static void l2_handle_abort_response(btx_l2 *l, btx_l2_resp r)
{
	if (l->resp_count < BTX_L2_RESP_QUEUE)
		l->resp[l->resp_count++] = r;

	L2_LOG("l2: abort-wait resp #%zu = %s (reason=%s)\n",
	        l->resp_count, l2_resp_name(r), l2_abort_reason_name(l->abort_reason));

	switch (l->abort_reason) {
	case L2_ABORT_NAK:
		/* The only valid answer to a forward abort is one NAK. */
		if (r == BTX_L2_RESP_NAK)
			l2_retransmit(l, l->acked);
		else
			l2_severe_error(l);
		return;

	case L2_ABORT_WACK:
		/* The terminal was busy; its answer to our ENQ is the real block
		 * status, so feed it back through the normal path. */
		l->abort_reason = L2_ABORT_NONE;
		l->state = l->final_pending ? BTX_L2_ST_FINAL_WAIT : BTX_L2_ST_GATE;
		l->resp_count = 0;
		l->timer_ms = BTX_L2_T_RESPONSE_MS;
		l2_handle_response(l, r);
		return;

	case L2_ABORT_TO:
		/* Two responses are owed: the block that never answered, and the ENQ. */
		if (l->resp_count < 2) {
			if (r == BTX_L2_RESP_ACK)
				l->timer_ms = BTX_L2_T_RESPONSE_MS;
			return;
		}
		if (l->resp[0] == BTX_L2_RESP_ACK && l->resp[1] == BTX_L2_RESP_NAK) {
			l->acked++; /* the block was fine; resume with the next one */
			l2_retransmit(l, l->acked);
		} else if (l->resp[0] == BTX_L2_RESP_NAK && l->resp[1] == BTX_L2_RESP_NAK) {
			l2_retransmit(l, l->acked);
		} else {
			l2_severe_error(l);
		}
		return;

	case L2_ABORT_FINAL:
		if (l2_is_expected_final_ack(l, r) && l->resp_count == 1) {
			l2_message_done(l);
		} else if (r == BTX_L2_RESP_NAK && l->resp_count == 1) {
			return; /* wait out the timer */
		} else {
			l2_severe_error(l);
		}
		return;

	default:
		l2_severe_error(l);
		return;
	}
}

static void l2_handle_response(btx_l2 *l, btx_l2_resp r)
{
	L2_LOG(
	        "l2: rx %s (state=%s block=%zu acked=%zu discard=%d)\n",
	        l2_resp_name(r), btx_l2_state_name(l), l->block, l->acked,
	        l->discard_responses);

	if (r == BTX_L2_RESP_EOT) {
		/* The terminal reset the link; whatever we were sending is gone. */
		l2_go_idle(l);
		l2_notify(l, BTX_L2_EV_TERMINAL_EOT, 0);
		return;
	}

	if (l->state == BTX_L2_ST_IDLE || l->state == BTX_L2_ST_TFI_WAIT)
		return; /* nothing outstanding to acknowledge */

	if (l->discard_responses)
		return;

	if (l->state == BTX_L2_ST_ABORT_WAIT) {
		l2_handle_abort_response(l, r);
		return;
	}

	if (r == BTX_L2_RESP_WACK) {
		/* Positive but does not move ACK0/ACK1 on; answer with ENQ. */
		l->enq_retries = 0;
		l->final_pending = (l->state == BTX_L2_ST_FINAL_WAIT);
		l2_send_enq(l, L2_ABORT_WACK);
		return;
	}

	if (l->state == BTX_L2_ST_FINAL_WAIT) {
		if (l2_is_expected_final_ack(l, r)) {
			l2_message_done(l);
		} else if (r == BTX_L2_RESP_NAK) {
			l2_retransmit(l, l->nblocks - 1); /* resend the last block */
		} else {
			l2_severe_error(l); /* wrong parity, or an ACK where ACK0/1 was due */
		}
		return;
	}

	/* Mid page: this should be an ACK or a NAK for an intermediate block. */
	switch (r) {
	case BTX_L2_RESP_ACK:
		if (l->acked + 1 > l->block) {
			l2_severe_error(l); /* more acks than blocks sent */
			return;
		}
		l->acked++;
		l->timer_ms = 0;
		return;

	case BTX_L2_RESP_NAK:
		/* Block l->acked failed; forward abort and retransmit from there. */
		l2_send_enq(l, L2_ABORT_NAK);
		return;

	default:
		l2_severe_error(l); /* an ACK0/ACK1 mid page is a severe error */
		return;
	}
}

/* ---------------------------------------------------------- transmit side */

/* Next byte to transmit. Returns 1 with a byte in *out, 0 if nothing to send. */
static int btx_l2_tx_byte(btx_l2 *l, uint8_t *out)
{
	if (l->ctl_pos < l->ctl_len) {
		*out = l->ctl[l->ctl_pos++];
		return 1;
	}

	if (l->state == BTX_L2_ST_TEXT) {
		if (l->out_pos < l->out_gate) {
			*out = l->out[l->out_pos++];
			return 1;
		}
		l->state = BTX_L2_ST_GATE; /* last text byte is out; check the gate */
	}

	if (l->state == BTX_L2_ST_GATE) {
		/* The terminator of block n may only go out once block n-1's
		 * response is in. There is nothing to wait for on a run's first block. */
		if (l->acked < l->block) {
			if (l->timer_ms == 0)
				l->timer_ms = BTX_L2_T_FLOWCTRL_MS;
			return 0;
		}
		l->state = BTX_L2_ST_TAIL;
	}

	if (l->state == BTX_L2_ST_TAIL) {
		*out = l->out[l->out_pos++];
		if (l->out_pos < l->out_len)
			return 1;

		/* Block complete, including its Block Check Characters. */
		l->discard_responses = 0;
		if (l->block + 1 < l->nblocks) {
			l->block++;
			l2_load_block(l);
		} else {
			l->state = BTX_L2_ST_FINAL_WAIT;
			l->timer_ms = BTX_L2_T_RESPONSE_MS;
			l->enq_retries = 0;
		}
		return 1;
	}

	return 0;
}

/* Advance all timers by ms milliseconds. */
static void btx_l2_tick(btx_l2 *l, unsigned ms)
{
	if (l->timer_ms == 0)
		return;

	if (l->timer_ms > ms) {
		l->timer_ms -= ms;
		return;
	}
	l->timer_ms = 0;

	switch (l->state) {
	case BTX_L2_ST_TFI_WAIT:
		/* No answer in time: basic facilities, 32-byte ITB blocks. */
		L2_LOG(
		        "l2: T.F.I. request timed out, using defaults (use_itb=%d itb_size=%zu)\n",
		        l->use_itb, l->itb_size);
		l->state = BTX_L2_ST_IDLE;
		l->tfi_done = 1;
		l2_notify(l, BTX_L2_EV_TFI, 0);
		return;

	case BTX_L2_ST_GATE:
		l2_send_enq(l, L2_ABORT_TO); /* the previous block's response never came */
		return;

	case BTX_L2_ST_FINAL_WAIT:
		l2_send_enq(l, L2_ABORT_FINAL);
		return;

	case BTX_L2_ST_ABORT_WAIT:
		if (l->abort_reason == L2_ABORT_FINAL && l->resp_count == 1 &&
		    l->resp[0] == BTX_L2_RESP_NAK) {
			l2_retransmit(l, l->nblocks - 1); /* a lone NAK: resend the last block */
			return;
		}
		if (l->abort_reason == L2_ABORT_WACK) {
			l2_send_enq(l, L2_ABORT_WACK); /* still busy: ask again */
			return;
		}
		l2_severe_error(l); /* fewer responses than expected */
		return;

	default:
		return;
	}
}

/* ------------------------------------------------------ 7-bit parity mode */

/*
 * Classic BTX/CEPT convention: 7 data bits plus a parity bit occupying bit 7
 * of the octet. Only ever used on the raw passthrough path, never combined
 * with layer-2 framing (bit 7 of a layer-2 control byte carries no meaning
 * to overwrite, since layer 2 is always 8-bit clean).
 *
 * Encode overwrites bit 7 outright with the even-parity bit of bits 0-6; no
 * verification is done on decode (just strip bit 7) since a single bad bit
 * should not be allowed to hang the link. If a real terminal turns out to
 * want odd parity instead, flip the encode function's final XOR.
 */
static inline uint8_t btx_l2_parity_encode(uint8_t b)
{
	uint8_t v = (uint8_t)(b & 0x7f);
	uint8_t x = v;

	x ^= (uint8_t)(x >> 4);
	x ^= (uint8_t)(x >> 2);
	x ^= (uint8_t)(x >> 1);
	return (uint8_t)(v | ((x & 1u) << 7)); /* bit7 set so the total bit count is even */
}

static inline uint8_t btx_l2_parity_decode(uint8_t b)
{
	return (uint8_t)(b & 0x7f);
}

/* --------------------------------------------------------- terminal bridge */

/*
 * How much of the server's stream goes into one page. At 1200 bit/s, 256
 * bytes is about two seconds on the line.
 */
#define BTX_L2_CHUNK 256

/*
 * Only enough to hold the chunk going out and the next one being gathered.
 * Reading no further than this leaves the rest in the kernel's receive
 * window, so TCP applies the backpressure and the server simply waits,
 * rather than us accumulating data the terminal will not see for a while.
 */
#define BTX_L2_BUF (2 * BTX_L2_CHUNK)
#define BTX_L2_TXBUF 256

/* How long to keep draining to the terminal after the server goes, at most. */
#define BTX_L2_LINGER_MS 15000

/* How many times a page is offered before it is abandoned. */
#define BTX_L2_MAX_ATTEMPTS 3

typedef struct {
	btx_l2 link;

	/* From the server, waiting to go out to the terminal. */
	uint8_t rx[BTX_L2_BUF];
	size_t rx_len;

	/* Keystrokes waiting to go to the server. */
	uint8_t tx[BTX_L2_TXBUF];
	size_t tx_len;
	unsigned tx_dropped;

	/* The server has gone, but anything already read still has to reach
	 * the terminal, so the caller keeps draining for a while. */
	int peer_closed;
	unsigned linger_ms;

	/* Bytes handed to the link and not yet acknowledged. They stay at the
	 * front of rx until the page is confirmed, so a link reset can put
	 * them back on the line instead of losing them. */
	size_t inflight;
	unsigned attempts; /* retries spent on the current page so far */
	unsigned resends;  /* pages actually sent a second time */
	unsigned lost;     /* bytes finally given up on */
} btx_l2_bridge;

static void btx_l2_bridge_on_event(void *ctx, btx_l2_event ev, uint8_t data)
{
	btx_l2_bridge *g = (btx_l2_bridge *)ctx;

	switch (ev) {
	case BTX_L2_EV_KEY:
		/* Queued for the caller to flush to the server; see btx_l2_bridge_poll. */
		if (g->tx_len < sizeof(g->tx)) {
			g->tx[g->tx_len++] = data;
		} else {
			g->tx_dropped++;
			L2_LOG(
			        "l2: keystroke 0x%02x dropped, send queue full (%u lost so far)\n",
			        data, g->tx_dropped);
		}
		break;

	case BTX_L2_EV_SEVERE_ERROR:
	case BTX_L2_EV_TERMINAL_EOT:
		/*
		 * Layer 2 gave up on a page. The bytes are still in rx, so put
		 * them back rather than skipping ahead: a stream cannot be
		 * redrawn on request.
		 */
		if (g->inflight > 0) {
			if (++g->attempts >= BTX_L2_MAX_ATTEMPTS) {
				L2_LOG(
				        "l2: link reset, giving up on %zu bytes after %u tries\n",
				        g->inflight, g->attempts);
				g->lost += (unsigned)g->inflight;
				memmove(g->rx, g->rx + g->inflight, g->rx_len - g->inflight);
				g->rx_len -= g->inflight;
				g->attempts = 0;
			} else {
				g->resends++;
			}
			g->inflight = 0;
		}
		break;

	case BTX_L2_EV_PAGE_SENT:
		L2_LOG("l2: page sent OK (%zu bytes)\n", g->inflight);
		g->attempts = 0;
		if (g->inflight > 0) {
			memmove(g->rx, g->rx + g->inflight, g->rx_len - g->inflight);
			g->rx_len -= g->inflight;
			g->inflight = 0;
		}
		break;

	case BTX_L2_EV_TFI:
		break;
	}
}

static void btx_l2_bridge_init(btx_l2_bridge *g)
{
	memset(g, 0, sizeof(*g));
	btx_l2_init(&g->link, btx_l2_bridge_on_event, g);
}

/* Hand buffered server data to the link when it is free. Call every frame. */
static void btx_l2_bridge_poll(btx_l2_bridge *g)
{
	size_t chunk;

	if (g->rx_len == 0 || g->inflight > 0 || btx_l2_busy(&g->link))
		return;

	chunk = g->rx_len < BTX_L2_CHUNK ? g->rx_len : BTX_L2_CHUNK;
	if (btx_l2_send_page(&g->link, g->rx, chunk) != 0)
		return;

	/*
	 * Deliberately not consumed here: the bytes stay in rx until the
	 * terminal acknowledges the page, so a reset can send them again.
	 */
	g->inflight = chunk;
}

/* Count down the linger timer. Call once per frame. */
static void btx_l2_bridge_tick(btx_l2_bridge *g, unsigned ms)
{
	if (g->linger_ms > ms)
		g->linger_ms -= ms;
	else
		g->linger_ms = 0;
}

/*
 * 1 when the server has gone and everything it sent has reached the
 * terminal, so the caller can end the session. Also true once the linger
 * timer runs out, in case the link never drains.
 */
static int btx_l2_bridge_finished(const btx_l2_bridge *g)
{
	if (!g->peer_closed)
		return 0;
	if (g->linger_ms == 0)
		return 1;
	return g->rx_len == 0 && !btx_l2_busy(&g->link);
}

#endif /* BTX_LAYER2_H */
