# Layer 2 (FTZ 157 D2) im SIP-BTX-Modem

Dieses Dokument erklärt, was die Layer-2-Schicht (`btx_l2.h`) macht, warum
sie nötig ist und wie sie in `v23_bridge.c` eingebunden ist. Es ergänzt die
Kurzfassung in der Haupt-`README.md`.

## Warum überhaupt eine Layer 2?

Ohne Layer 2 (Stand v2 dieses Projekts) wurden Bytes 1:1 zwischen der
V.23-Bitschicht (Modem-Modulation/-Demodulation) und dem TCP-Socket zum
BTX-Server durchgereicht: ein Byte vom Terminal ging direkt auf den
Socket, ein Byte vom Socket direkt aufs Terminal. Das ist ein reiner
Transparenzkanal - funktional wie ein Nullmodem-Kabel, nur über V.23 statt
über eine serielle Leitung.

Ein echtes BTX-Terminal wie das DBT03 erwartet aber mehr: Die
Bildschirmtext-Norm **FTZ 157 D2** definiert eine eigene Sicherungsschicht
oberhalb der reinen Modemverbindung, mit:

- **Blockrahmen** statt freiem Bytestrom: Daten werden in Blöcke verpackt,
  die klar markiert sind (Start, Ende, Prüfsumme).
- **Prüfsumme (BCC)** pro Block, damit Übertragungsfehler erkannt werden.
- **Quittungen (ACK/NAK/WACK)**, damit der Sender weiß, ob ein Block
  angekommen ist, und fehlerhafte Blöcke erneut schicken kann.
- **Terminal Facility Identifier (T.F.I.)**, eine kurze Aushandlung zu
  Beginn der Sitzung, mit der das Terminal seine Fähigkeiten mitteilt
  (z. B. welche Blockgröße es kann).

Ohne das reagiert ein echtes Terminal im Zweifel gar nicht, weil es auf
diese Rahmung wartet, bevor es die Nutzdaten überhaupt als BTX-Inhalt
interpretiert.

## Die Bausteine von `btx_l2.h`

`btx_l2.h` ist eine Ein-Datei-Zusammenfassung des Layer-2-Protokolls,
ursprünglich aus dem `btx_modem_sip`-Projekt und für den ESP32-Port dieses
Projekts übernommen (dort bereits gegen ein echtes DBT03 verifiziert).
Zwei Ebenen:

### 1. `btx_l2_*` - die eigentliche Sicherungsschicht

Kennt nur Bytes, keine Sockets. Kernfunktionen:

| Funktion | Zweck |
|---|---|
| `btx_l2_init()` | Initialisiert eine Sitzung, Zustand `IDLE` |
| `btx_l2_request_tfi()` | Schickt `SOH ENQ`, fragt das Terminal nach seinen Fähigkeiten |
| `btx_l2_send_page(daten, länge)` | Stellt eine Seite Nutzdaten zum Versand ein (max. `BTX_L2_MESSAGE_MAX` = 2048 Bytes) |
| `btx_l2_tx_byte(&byte)` | Liefert das nächste zu sendende Byte, 0 wenn gerade nichts ansteht |
| `btx_l2_rx_byte(byte)` | Füttert ein empfangenes Byte in die Zustandsmaschine |
| `btx_l2_tick(ms)` | Lässt alle Timeouts (Antwortzeit, Flusskontrolle, Retransmit) um `ms` weiterlaufen |
| `btx_l2_busy()` | 1, solange noch etwas zu senden/quittieren ist |

Ereignisse (`btx_l2_app_fn`-Callback) informieren die Anwendung:

- `BTX_L2_EV_KEY` - ein echtes Tastaturbyte vom Terminal (keine
  Protokoll-Steuerung)
- `BTX_L2_EV_TFI` - die T.F.I.-Aushandlung ist abgeschlossen (Antwort oder
  Timeout auf Standardwerte)
- `BTX_L2_EV_PAGE_SENT` - die aktuelle Seite wurde vollständig quittiert
- `BTX_L2_EV_SEVERE_ERROR` - Wiederholungen ausgeschöpft, Link wurde
  zurückgesetzt (`EOT`)
- `BTX_L2_EV_TERMINAL_EOT` - das Terminal selbst hat mit `EOT` zurückgesetzt

### 2. `btx_l2_bridge_*` - die Anbindung an einen Bytestrom

Zerlegt einen rohen Bytestrom (z. B. vom BTX-Server) in Seiten fester
Größe (`BTX_L2_CHUNK` = 256 Byte) und reicht sie nacheinander an
`btx_l2_send_page()` weiter. Bytes bleiben im Puffer, bis sie bestätigt
sind - erst dann werden sie freigegeben; scheitert eine Seite, bleiben sie
liegen und werden erneut versucht (bis `BTX_L2_MAX_ATTEMPTS`). Tastaturbytes
(`BTX_L2_EV_KEY`) landen automatisch in `bridge.tx`, bereit zum Versand an
den Server.

## Wie ein Block auf der Leitung aussieht

```
EOT STX <bis zu 256 Byte Text> ETX BCC_lo BCC_hi
```

- `EOT` (0x04) nur vor dem allerersten Block einer Sitzung (setzt den Link
  zurück).
- `STX` (0x02) eröffnet den Textmodus.
- Bei mehreren Blöcken pro Seite (Text länger als die verhandelte
  ITB-Größe) trennt `ITB` (0x07) bzw. `ETB` (0x17) die Zwischenblöcke,
  jeweils mit eigener BCC; nur der letzte Block endet mit `ETX` (0x03).
- `BCC` ist CRC-16/ARC über den Blocktext plus Terminator, LSB zuerst auf
  der Leitung.

Das Terminal antwortet auf jeden Block mit `ACK` (0x06, Zwischenblock)
bzw. am Seitenende mit `DLE '0'` oder `DLE '1'` (abwechselnd, damit ein
verlorenes ACK erkennbar bleibt), im Fehlerfall mit `NAK` (0x15) oder - bei
kurzzeitiger Überlastung - `DLE ';'` (WACK, "bitte warten").

## Einbindung in `v23_bridge.c`

Die V.23-Bitschicht bleibt unverändert (Trägererkennung, NUL-Byte-
Kickstart, Bit-Ringpuffer für Start-/Stopbit-Framing - siehe Haupt-
`README.md`). Layer 2 sitzt eine Ebene darüber, an genau zwei Stellen:

```
Terminal --Bits--> modem_put_bit() --Byte--> btx_l2_rx_byte()
                                                    |
                                          (ACK/NAK/WACK/T.F.I.
                                           interpretiert, oder
                                           BTX_L2_EV_KEY -> bridge.tx)
                                                    |
                                                    v
                                          l2_flush_keys() --> Server-Socket

Server-Socket --> l2_pump_socket_rx() --> bridge.rx --> btx_l2_bridge_poll()
                                                              |
                                                    btx_l2_send_page()
                                                              |
                                                              v
modem_get_bit() <--Byte-- btx_l2_tx_byte() <----------- (Blockrahmen)
      |
    Bits --> Terminal
```

Konkret:

- **`modem_put_bit()`** (RX vom Terminal, bit- und byteweise wie zuvor)
  ruft nach dem Zusammensetzen eines Bytes jetzt `btx_l2_rx_byte()` statt
  `send()` auf. Die L2-Engine entscheidet selbst, ob es sich um eine
  Quittung, eine T.F.I.-Antwort oder ein Tastaturbyte handelt.
- **`modem_get_bit()`** (TX ans Terminal) holt das nächste Byte über
  `btx_l2_tx_byte()` statt über ein rohes `recv()`. Kommt gerade nichts
  (Zustand `IDLE` oder am Flusskontroll-Gate wartend), fällt der Slot auf
  reinen Leerlauf (Mark-Pegel) zurück - wie zuvor bei leerem Socket.
- **`l2_pump_socket_rx()`** und **`l2_flush_keys()`** sind die
  Linux-Entsprechungen zu `btx_l2_dbt03_pump_socket_rx()` /
  `btx_l2_dbt03_flush_keys()` aus dem ESP32-Port, hier mit normalen
  nicht-blockierenden BSD-Sockets statt lwIP.
- **`v23_link_tick()`** bündelt einen L2-Zyklus (Keys flushen, Server
  lesen, Seite ggf. einspeisen, Timer weiterlaufen lassen) und wird aus
  **`v23_link_generate_tx()`** heraus aufgerufen - die Funktion läuft
  ohnehin einmal pro RTP-Häppchen (160 Samples @ 8000 Hz = 20 ms), die
  Millisekundenzahl für `btx_l2_tick()` ergibt sich direkt aus der
  Samplezahl. Kein zusätzlicher Wall-Clock-Tick nötig.
- Die **NUL-Byte-Kickstart-Logik bleibt unverändert vor Layer 2 bestehen**:
  Sie gehört zur physikalischen V.23-Trägeraushandlung, nicht zu FTZ 157 D2.
  Die L2-Sitzung (`btx_l2_request_tfi()`) startet daher erst, sobald das
  Kickstart-NUL tatsächlich gesendet wurde (`l2_started`-Flag in
  `v23_link_tick()`).

## Was sich für Python (`v23modem.py`, `sip_btx_modem.py`) ändert

Nichts an der Schnittstelle: `V23Link` bietet weiterhin nur
`process_rx()`, `generate_tx()`, `finished()`, `close()`. Die gesamte
L2-Logik ist intern in der C-Bibliothek gekapselt.

## Verifiziert vs. offen

- Verifiziert (Simulation, `linktest.c`): Trägererkennung, NUL-Kickstart,
  T.F.I.-Timeout auf Standardwerte, Blockrahmen inkl. BCC, ACK1-Quittung,
  Weiterleitung eines Tastaturbytes zum Server.
- Noch offen: Verhalten am echten DBT03 - insbesondere ob es tatsächlich
  mit `DLE '0'`/`DLE '1'` quittiert wie erwartet, und wie es auf die
  T.F.I.-Anfrage antwortet (die Simulation lässt sie bewusst timeouten).
