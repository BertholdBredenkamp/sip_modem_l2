#!/usr/bin/env python3
"""
sip_btx_modem.py

SIP-Modem als Gegenstelle (Host/Zentrale) zum DBT03: registriert sich bei
einem Asterisk-Server, nimmt eingehende Anrufe entgegen, moduliert/
demoduliert V.23 (1200/75 bit/s, CEPT/BTX-Standard) im Audiostream und
reicht die Nutzdaten ueber eine TCP-Verbindung an den Bildschirmtext-
Server weiter.

Die eigentliche Modem-Logik (Traegererkennung, das fuer das DBT03 noetige
NUL-Byte-Kickstart, Bit-Framing und die TCP-Verbindung zum BTX-Server)
steckt komplett in v23_bridge.c / v23modem.py und ist nachgebaut nach dem
bewaehrten "app_softmodem" fuer Asterisk (Christian Groeger, GPL), das
nachweislich mit einem echten DBT03 funktioniert. Dieses Skript kuemmert
sich nur noch um SIP/RTP via PJSUA2 und reicht PCM-Audio an V23Link durch.

Ablauf pro Anruf:

    DBT03 --(Telefonnetz)--> Asterisk --(SIP/RTP)--> dieses Programm
                                                          |
                                                    V23Link (v23_bridge.c):
                                                    V.23-Demodulation (75 bit/s),
                                                    Traegererkennung, NUL-Kickstart
                                                          |
                                                          v
                                                    TCP-Verbindung (in C gehalten)
                                                          |
                                                          v
                                                    BTX-Server (--btx-host:--btx-port)
                                                          |
                                                    V.23-Modulation (1200 bit/s)
                                                          |
                                                          v
                                                    zurueck ueber SIP/RTP an DBT03

WICHTIGER HINWEIS ZU PJSUA2 / PJPROJECT
----------------------------------------
Dieses Programm braucht die Python-SWIG-Bindings von PJSUA2 (Modul
"pjsua2"), die NICHT per pip installierbar sind, sondern aus dem
pjproject-Quellcode gebaut werden muessen:

    git clone https://github.com/pjsip/pjproject.git
    cd pjproject
    ./configure --disable-video CFLAGS="-fPIC"
    make dep && make
    sudo make install
    cd pjsip-apps/src/swig
    make python
    sudo make -C python install

Kritisch: Der hier verwendete Callback AudioMediaPort.onFrameReceived()
(zum Empfang von Audio AUS dem Call, d.h. den Toenen des DBT03) existiert
in PJSUA2 erst seit PJSIP 2.14 (siehe pjsip/pjproject Aenderung #3569).
Mit aelteren Versionen fehlt diese Methode.

Vor dem produktiven Einsatz pruefen:

    python3 -c "import pjsua2 as pj; print(pj.AudioMediaPort); \\
                print(pj.AudioMediaPort.onFrameReceived)"

Wenn das fehlschlaegt: neuere pjproject-Version verwenden bzw. die SWIG-
Bindings frisch aus dem aktuellen Quellcode generieren.
"""

from __future__ import annotations

import argparse
import logging
import time

import pjsua2 as pj

from v23modem import V23Link

SAMPLE_RATE = 8000
BITS_PER_SAMPLE = 16
CHANNELS = 1
FRAME_MS = 20
SAMPLES_PER_FRAME = SAMPLE_RATE * FRAME_MS // 1000  # 160

log = logging.getLogger("sip_btx_modem")


class BtxModemPort(pj.AudioMediaPort):
    """Bindeglied zwischen dem SIP-Audiostream und dem V.23-Modem-Link.

    onFrameRequested liefert das an das DBT03 zu sendende Audio (1200 bit/s
    Vorwaertskanal). onFrameReceived nimmt das vom DBT03 empfangene Audio
    entgegen (75 bit/s Rueckkanal). Traegererkennung, der fuer das DBT03
    noetige NUL-Byte-Kickstart und die TCP-Verbindung zum BTX-Server werden
    komplett von V23Link (v23_bridge.c) erledigt - siehe dort fuer Details.
    """

    def __init__(self, btx_host: str, btx_port: int) -> None:
        super().__init__()
        self.link = V23Link(btx_host, btx_port)

        fmt = pj.MediaFormatAudio()
        fmt.type = pj.PJMEDIA_TYPE_AUDIO
        fmt.clockRate = SAMPLE_RATE
        fmt.channelCount = CHANNELS
        fmt.bitsPerSample = BITS_PER_SAMPLE
        fmt.frameTimeUsec = FRAME_MS * 1000
        self.createPort("btx_modem", fmt)

    # --- Von PJSUA2 aufgerufen, wenn Audio Richtung DBT03 gebraucht wird ---
    def onFrameRequested(self, frame: "pj.MediaFrame") -> None:
        pcm = self.link.generate_tx(SAMPLES_PER_FRAME)
        frame.type = pj.PJMEDIA_FRAME_TYPE_AUDIO
        frame.buf = pj.ByteVector(pcm)
        frame.size = len(pcm)

    # --- Von PJSUA2 aufgerufen, wenn Audio vom DBT03 eingetroffen ist ---
    def onFrameReceived(self, frame: "pj.MediaFrame") -> None:
        if frame.type != pj.PJMEDIA_FRAME_TYPE_AUDIO or frame.size == 0:
            return
        pcm = bytes(frame.buf[: frame.size])
        self.link.process_rx(pcm)

    def finished(self) -> bool:
        return self.link.finished()

    def close(self) -> None:
        self.link.close()


class BtxCall(pj.Call):
    def __init__(self, acc: "SipAccount", call_id: int, btx_host: str, btx_port: int) -> None:
        super().__init__(acc, call_id)
        self.acc = acc
        self.call_id = call_id
        self.btx_host = btx_host
        self.btx_port = btx_port
        self.modem_port: BtxModemPort | None = None

    def onCallState(self, prm: "pj.OnCallStateParam") -> None:
        ci = self.getInfo()
        log.info("Anrufstatus: %s", ci.stateText)
        if ci.state == pj.PJSIP_INV_STATE_DISCONNECTED:
            if self.modem_port is not None:
                self.modem_port.close()
            # Referenz freigeben, damit das Objekt nach dem Callback vom
            # Garbage Collector eingesammelt werden kann.
            self.acc.forget_call(self.call_id)

    def onCallMediaState(self, prm: "pj.OnCallMediaStateParam") -> None:
        ci = self.getInfo()
        for mi in ci.media:
            if mi.type == pj.PJMEDIA_TYPE_AUDIO and mi.status == pj.PJSUA_CALL_MEDIA_ACTIVE:
                call_media = self.getAudioMedia(mi.index)

                try:
                    self.modem_port = BtxModemPort(self.btx_host, self.btx_port)
                except RuntimeError as exc:
                    log.error("Konnte BTX-Modem-Link nicht aufbauen: %s", exc)
                    hangup_prm = pj.CallOpParam()
                    hangup_prm.statusCode = pj.PJSIP_SC_SERVICE_UNAVAILABLE
                    self.hangup(hangup_prm)
                    return

                # Unser moduliertes Audio (1200 bit/s) -> zum DBT03
                self.modem_port.startTransmit(call_media)
                # Das Audio vom DBT03 (75 bit/s) -> in unseren Demodulator
                call_media.startTransmit(self.modem_port)
                log.info("Medienpfad zwischen SIP-Call und V.23-Modem hergestellt.")


class SipAccount(pj.Account):
    def __init__(self, btx_host: str, btx_port: int) -> None:
        super().__init__()
        self.btx_host = btx_host
        self.btx_port = btx_port
        # Muss aktive Call-Objekte referenzieren: PJSUA2 legt den Anruf
        # sofort auf, wenn das Python-Wrapper-Objekt (hier: BtxCall) vom
        # Garbage Collector eingesammelt wird, weil dessen C++-Destruktor
        # den Call beendet. Ohne diese Liste wuerde jeder eingehende Anruf
        # sofort nach onIncomingCall() wieder aufgelegt (sichtbar im Log als
        # "hanging up: code=0" direkt nach der 200 OK-Antwort).
        self._active_calls: dict[int, "BtxCall"] = {}

    def onRegState(self, prm: "pj.OnRegStateParam") -> None:
        ai = self.getInfo()
        log.info("Registrierung bei Asterisk: %s (Code %d)", ai.regStatusText, ai.regStatus)

    def onIncomingCall(self, prm: "pj.OnIncomingCallParam") -> None:
        log.info("Eingehender Anruf, nehme an ...")
        call = BtxCall(self, prm.callId, self.btx_host, self.btx_port)
        self._active_calls[prm.callId] = call
        call_prm = pj.CallOpParam()
        call_prm.statusCode = pj.PJSIP_SC_OK
        call.answer(call_prm)

    def forget_call(self, call_id: int) -> None:
        self._active_calls.pop(call_id, None)

    def check_finished_calls(self) -> None:
        """Legt Calls auf, deren BTX-TCP-Verbindung sich beendet hat
        (Server hat geschlossen o.ae.) - mirror von session.finished in
        der Asterisk-Referenz."""
        for call in list(self._active_calls.values()):
            if call.modem_port is not None and call.modem_port.finished():
                log.info("BTX-Verbindung beendet, lege auf.")
                hangup_prm = pj.CallOpParam()
                hangup_prm.statusCode = pj.PJSIP_SC_OK
                try:
                    call.hangup(hangup_prm)
                except Exception:  # noqa: BLE001 - Call koennte schon weg sein
                    pass


def build_endpoint() -> pj.Endpoint:
    ep = pj.Endpoint()
    ep.libCreate()

    ep_cfg = pj.EpConfig()
    ep_cfg.uaConfig.threadCnt = 2
    ep_cfg.medConfig.clockRate = SAMPLE_RATE
    ep_cfg.medConfig.audioFramePtime = FRAME_MS
    # Wichtig fuer Modemtoene: VAD/Silence Suppression schneidet normalerweise
    # Passagen weg, die nicht wie Sprache klingen - ein kontinuierlicher
    # V.23-Traeger faellt genau darunter und wuerde sonst teilweise verstuemmelt
    # oder ganz unterdrueckt (sichtbar im Log als "!VAD re-enabled").
    ep_cfg.medConfig.noVad = True
    ep.libInit(ep_cfg)

    sip_tp_cfg = pj.TransportConfig()
    sip_tp_cfg.port = 0  # freien lokalen Port waehlen
    ep.transportCreate(pj.PJSIP_TRANSPORT_UDP, sip_tp_cfg)

    ep.libStart()

    # Ohne diese Zeile verbindet PJSUA2 aktive Calls automatisch mit einem
    # lokalen Sound-Device (Mikrofon/Lautsprecher) - auf einem Server ohne
    # echte Soundkarte entsteht dabei trotzdem ein "virtuelles" Geraet, an
    # das der Call geroutet wird, statt an unseren BtxModemPort. Sichtbares
    # Symptom im Log: "Opening sound device (speaker + mic) ..." und
    # "Conf connect: 2 --> 1" direkt nach dem Medienpfad-Aufbau, gefolgt
    # von "playdbuf !Underflow" - der Modem-Port haengt dann im Leeren.
    # setNullDev() unterbindet dieses automatische Sound-Device-Routing.
    ep.audDevManager().setNullDev()

    return ep


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sip-server", required=True, help="Asterisk-Server (Host oder Host:Port)")
    parser.add_argument("--sip-user", required=True, help="SIP-Benutzername/Extension")
    parser.add_argument("--sip-password", required=True, help="SIP-Passwort")
    parser.add_argument("--sip-domain", default=None, help="SIP-Domain, falls abweichend von --sip-server")
    parser.add_argument("--btx-host", required=True, help="Hostname/IP des BTX-Servers")
    parser.add_argument("--btx-port", required=True, type=int, help="TCP-Port des BTX-Servers")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    ep = build_endpoint()

    domain = args.sip_domain or args.sip_server
    acc_cfg = pj.AccountConfig()
    acc_cfg.idUri = f"sip:{args.sip_user}@{domain}"
    acc_cfg.regConfig.registrarUri = f"sip:{args.sip_server}"
    cred = pj.AuthCredInfo("digest", "*", args.sip_user, 0, args.sip_password)
    acc_cfg.sipConfig.authCreds.append(cred)

    account = SipAccount(args.btx_host, args.btx_port)
    account.create(acc_cfg)

    log.info("SIP-BTX-Modem laeuft. Warte auf eingehende Anrufe ... (Strg+C zum Beenden)")
    try:
        while True:
            ep.libHandleEvents(100)
            account.check_finished_calls()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        account.shutdown()
        ep.libDestroy()


if __name__ == "__main__":
    main()
