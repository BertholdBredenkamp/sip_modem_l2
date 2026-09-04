# SIP-BTX-Modem

Linux-Programm, das als V.23-Modem-Gegenstelle (Host/Zentrale) zu einem
DBT03-BTX-Terminal fungiert: Es nimmt den Anruf des DBT03 über SIP/Asterisk
entgegen, moduliert/demoduliert V.23 (1200/75 bit/s, CEPT/BTX-Standard) und
reicht die Nutzdaten über eine TCP-Verbindung an einen Bildschirmtext-
Server weiter.

```
DBT03 --(Telefonnetz)--> Asterisk --(SIP/RTP)--> sip_btx_modem.py
                                                       |
                                                  V23Link (v23_bridge.c):
                                                  Trägererkennung, NUL-Byte-
                                                  Kickstart, V.23-Demod. (75 bit/s)
                                                       |
                                                       v
                                                  Layer 2, FTZ 157 D2 (btx_l2.h):
                                                  Blockrahmen, CRC-16/ARC-BCC,
                                                  ACK0/ACK1/NAK/WACK, T.F.I.
                                                       |
                                                       v
                                                  TCP-Verbindung (von C gehalten)
                                                       |
                                                       v
                                                  BTX-Server (--btx-host:--btx-port)
                                                       |
                                                  Layer 2 (btx_l2.h) + V.23-Modulation
                                                  (1200 bit/s)
                                                       |
                                                       v
                                                  zurück über SIP/RTP
```

## Komponenten

| Datei                | Zweck                                                                 |
|-----------------------|------------------------------------------------------------------------|
| `v23_bridge.c`        | V.23-Modem (spandsp, manuelles Bit-Framing) + Trägererkennung + NUL-Byte-Kickstart + Layer-2-Protokoll (`btx_l2.h`) + TCP-Verbindung zum BTX-Server, kompiliert zu `libv23bridge.so` |
| `btx_l2.h`            | FTZ-157-D2-Layer-2-Protokoll (Blockrahmen, CRC-16/ARC-BCC, ACK0/ACK1/NAK/WACK, T.F.I.-Aushandlung, Retransmit) - 1:1 übernommen aus dem ESP32-Port dieses Projekts, dort bereits gegen ein echtes DBT03 verifiziert |
| `v23modem.py`         | ctypes-Anbindung von `libv23bridge.so` (Klasse `V23Link`) - API unverändert durch die L2-Integration |
| `sip_btx_modem.py`    | Hauptprogramm: SIP-Registrierung (PJSUA2), Medienpfad                 |
| `selftest.c`          | Reiner Loopback-Test der V.23-Modulation/-Demodulation (ohne Handshake) |
| `linktest.c`          | End-to-End-Test: simuliertes Terminal + TCP-Testserver, prüft Trägererkennung, NUL-Kickstart und Datenfluss in beide Richtungen |
| `build.sh`            | Baut `libv23bridge.so`, `selftest` und `linktest`                     |

Die Rollenaufteilung ist fest verdrahtet, wie es der klassischen
Prestel/BTX-Asymmetrie entspricht: **Wir senden mit 1200 bit/s** (Kanal 1,
Vorwärtskanal, Zentrale → Terminal) und **empfangen mit 75 bit/s** (Kanal 2,
Rückkanal, Terminal → Zentrale).

## Entstehungsgeschichte / warum das Design so aussieht

Die erste Version dieses Projekts nutzte spandsps automatische Async-Schicht
(`FSK_FRAME_MODE_ASYNC`) und reichte Bytes 1:1 zwischen V.23-Audio und
TCP-Server durch. Damit blieb der Bildschirm des echten DBT03 schwarz, ohne
jede Reaktion. Die Fehlersuche ergab mehrere Ursachen, teils auf SIP/Audio-
Ebene, teils auf Modem-Protokollebene:

1. **PJSUA2 verband Anrufe automatisch mit einem (nicht vorhandenen)
   lokalen Sound-Device** statt mit unserem eigenen `AudioMediaPort` →
   behoben mit `Endpoint.audDevManager().setNullDev()`.
2. **VAD/Silence Suppression** schnitt den kontinuierlichen V.23-Träger
   teilweise weg → behoben mit `EpConfig.medConfig.noVad = True`.
3. **Fehlendes Verbindungs-Handshake**: Ein echtes DBT03 erwartet, dass die
   Zentrale nach erkanntem Trägersignal des Terminals aktiv ein einzelnes
   **NUL-Byte (0x00)** sendet, bevor irgendetwas passiert. Außerdem
   verwendet spandsps Async-Schicht im Leerlauf kontinuierliche gerahmte
   `0xFF`-Füllzeichen statt echtem, ungerahmtem Mark-Pegel — das
   verwirrt reale DBT03-Hardware offenbar zusätzlich.

Punkt 3 wurde durch einen Vergleich mit zwei Referenzimplementierungen
gelöst, die nachweislich mit echten BTX-Terminals funktionieren:
[`Casandro/btx_modem`](https://github.com/Casandro/btx_modem) (natives
Asterisk-Dialplan-Modul) und vor allem **`app_softmodem.c`** (Christian
Groeger, 2010, ein generisches Softmodem für Asterisk mit spezieller
BTX/DBT03-Unterstützung), auf der die aktuelle `v23_bridge.c` direkt basiert:

- **`FSK_FRAME_MODE_SYNC`** statt `ASYNC`: Das Start-/Stop-Bit-Framing wird
  in `v23_bridge.c` selbst gemacht (`modem_get_bit`/`modem_put_bit`,
  Bit-Ringpuffer), nicht von spandsps Async-Schicht. Der Leerlauf ist
  dadurch reiner Mark-Pegel (Bit=1), keine gerahmten Füllzeichen.
- **Trägererkennung + NUL-Byte-Kickstart**: Erst wenn vom Terminal
  tatsächlich Trägersignal (`SIG_STATUS_CARRIER_UP`, dann ein echtes Bit)
  ankommt, gilt die Verbindung als aktiv — erst dann wird das
  Anstoß-NUL-Byte gesendet.
- **Keine expliziten `fsk_tx_power()`/`fsk_rx_signal_cutoff()`-Aufrufe**:
  Im Test hat das Setzen der Referenzwerte (-28/-35 dBm0) die
  Sendeamplitude so weit abgesenkt, dass selbst die eigene, tolerante
  Empfangsschwelle keinen Träger mehr erkannt hat. Die spandsp-
  Standardwerte wurden im Loopback-Test (`linktest.c`) verifiziert
  (Träger korrekt erkannt, alle Bits korrekt demoduliert) und daher
  belassen. Falls sich am echten Telefonkanal Trägererkennungsprobleme
  zeigen: zuerst nur mit `fsk_rx_signal_cutoff()` experimentieren, ohne
  gleichzeitig `fsk_tx_power()` abzusenken.

## Layer 2 (FTZ 157 D2)

Seit dieser Version reicht `v23_bridge.c` Bytes nicht mehr 1:1 zwischen
V.23-Bitschicht und TCP-Server durch, sondern über `btx_l2.h` (Blockrahmen,
BCC-Prüfung, ACK0/ACK1/NAK/WACK, T.F.I.-Aushandlung, Retransmit bei
Fehlern) — dieselbe Engine, die im ESP32-Port dieses Projekts bereits gegen
ein echtes DBT03 verifiziert wurde. Die physikalische NUL-Byte-Kickstart-
Logik (siehe Punkt 3 oben) bleibt davon unberührt bestehen; die L2-Sitzung
(T.F.I.-Anfrage) startet erst, sobald das Kickstart-NUL tatsächlich
gesendet wurde. Der L2-Tick (Timeouts, Retransmits) läuft aus
`v23_link_generate_tx()` heraus, getaktet über die ohnehin pro RTP-Paket
(20 ms) durchgereichte Samplezahl — kein eigener Wall-Clock-Tick nötig.

## Verifizierter Stand (in dieser Sandbox getestet)

- `v23_bridge.c` kompiliert sauber gegen `libspandsp-dev 0.0.6`.
- `linktest.c`: kompletter End-to-End-Durchlauf bestanden — Trägererkennung,
  NUL-Byte-Kickstart, T.F.I.-Timeout auf Standardwerte, L2-Blockrahmen
  (EOT STX "SRV-OK" ETX BCC) Server→Terminal, ACK1-Quittung Terminal→
  Zentrale und Terminal→Server-Weiterleitung eines Tastaturbytes wurden
  alle korrekt durchlaufen (simuliertes Terminal + lokaler TCP-Testserver,
  kein echtes DBT03).
- `v23modem.py`: ctypes-Anbindung (`V23Link`) per Smoke-Test verifiziert;
  die öffentliche API ist durch die L2-Integration unverändert.
- `sip_btx_modem.py`: SIP-Registrierung, Medienpfad und Audio-Routing wurden
  bereits gegen einen echten Asterisk-Server und ein echtes DBT03 getestet
  (siehe Punkte 1+2 oben) — der physikalische Handshake (Punkt 3) und das
  L2-Protokoll selbst stehen beim echten Terminal noch aus.

## Bekannter Stolperstein: PJSUA2-Version

Der Empfangspfad (`onFrameReceived`, um die Töne des DBT03 aus dem Call
abzugreifen) existiert in PJSUA2 **erst seit PJSIP 2.14**. PJSUA2 ist nicht
per `pip install` verfügbar, sondern muss aus dem pjproject-Quellcode
gebaut werden:

```bash
git clone https://github.com/pjsip/pjproject.git
cd pjproject
./configure --disable-video CFLAGS="-fPIC"
make dep && make
sudo make install
cd pjsip-apps/src/swig
make python              # swig muss installiert sein (apt-get install swig python3-dev)
sudo make -C python install
```

Prüfen:

```bash
python3 -c "import pjsua2 as pj; print(pj.AudioMediaPort); print(pj.AudioMediaPort.onFrameReceived)"
```

## Weitere bekannte PJSUA2-Fallstricke (bereits im Code behoben)

- **Call-Objekt wird vom Garbage Collector eingesammelt**: Ohne eine
  gehaltene Referenz auf das `BtxCall`-Python-Objekt legt PJSUA2 den Anruf
  sofort nach `onIncomingCall()` wieder auf (`SipAccount._active_calls`
  hält die Referenz).
- **Automatisches Sound-Device-Routing** und **VAD** — siehe oben.

## Setup

```bash
sudo apt-get install libspandsp-dev gcc pkg-config
./build.sh
./selftest    # optional: reine Modem-Modulation/-Demodulation
./linktest    # optional: End-to-End inkl. simuliertem Terminal

# PJSUA2 wie oben beschrieben bauen/installieren, dann:
python3 sip_btx_modem.py \
    --sip-server pbx.example.org \
    --sip-user 1234 \
    --sip-password '...' \
    --btx-host btx.example.org \
    --btx-port 20000 \
    -v
```

## Offene Punkte / nächste Schritte

- Test mit dem echten DBT03: prüfen, ob Trägererkennung, NUL-Byte-
  Kickstart UND das L2-Protokoll (T.F.I.-Antwort, ACK0/ACK1-Verhalten) in
  der Praxis wie erwartet greifen.
- Falls der Träger auf der echten Telefon-/VoIP-Strecke nicht zuverlässig
  erkannt wird: `fsk_rx_signal_cutoff()` in `v23_link_new()` vorsichtig
  anpassen (siehe Hinweis oben).
- Sauberes Verhalten bei Verbindungsabbruch weiter absichern (aktuell prüft
  `SipAccount.check_finished_calls()` periodisch `V23Link.finished()` und
  legt dann auf).
- `BTX_L2_QUIET` (in `btx_l2.h`) ist standardmäßig 0, jedes Blockereignis
  wird also mit Zeitstempel nach stderr geloggt — für einen laufenden
  Betrieb ggf. auf 1 setzen, sobald das L2-Verhalten am echten Terminal
  bestätigt ist.
