"""
v23modem.py

Duenner ctypes-Wrapper um libv23bridge.so (v2, siehe v23_bridge.c), das ein
V.23-Modem (CEPT/BTX, 1200/75 bit/s) fuer die Rolle "Gegenstelle/Zentrale"
implementiert - nachgebaut nach dem bewaehrten "app_softmodem" fuer
Asterisk (Christian Groeger, GPL), das nachweislich mit einem echten DBT03
funktioniert.

Anders als in einer frueheren Version dieser Datei haelt die C-Bibliothek
die TCP-Verbindung zum BTX-Server jetzt SELBST (wie die Referenz) - inkl.
der Traegererkennung und des NUL-Byte-Kickstarts, die fuer das DBT03 noetig
sind. Python reicht nur noch PCM-Audio in beide Richtungen durch.

Audio ist durchgehend 16-Bit PCM @ 8000 Hz (Telefonie-Standard, passt
direkt zum PJSUA2-AudioMediaPort).
"""

from __future__ import annotations

import ctypes
import os

_libpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libv23bridge.so")
_lib = ctypes.CDLL(_libpath)

_lib.v23_link_new.argtypes = [ctypes.c_char_p, ctypes.c_int]
_lib.v23_link_new.restype = ctypes.c_void_p
_lib.v23_link_process_rx.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int16), ctypes.c_int]
_lib.v23_link_process_rx.restype = ctypes.c_int
_lib.v23_link_generate_tx.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int16), ctypes.c_int]
_lib.v23_link_generate_tx.restype = ctypes.c_int
_lib.v23_link_finished.argtypes = [ctypes.c_void_p]
_lib.v23_link_finished.restype = ctypes.c_int
_lib.v23_link_free.argtypes = [ctypes.c_void_p]


class V23Link:
    """Ein V.23-Modem-Link zwischen einem SIP-Audiostream (DBT03) und
    einem BTX-Server (TCP). Haelt Traegererkennung, NUL-Byte-Kickstart und
    die TCP-Verbindung intern in C - siehe v23_bridge.c fuer Details."""

    def __init__(self, host: str, port: int) -> None:
        self._handle = _lib.v23_link_new(host.encode("utf-8"), port)
        if not self._handle:
            raise RuntimeError(f"v23_link_new() fehlgeschlagen (Verbindung zu {host}:{port}?)")

    def process_rx(self, pcm: bytes) -> None:
        """Vom DBT03 empfangenes Audio verarbeiten. pcm: little-endian
        16-bit PCM @ 8000 Hz."""
        n = len(pcm) // 2
        buf = (ctypes.c_int16 * n).from_buffer_copy(pcm)
        _lib.v23_link_process_rx(self._handle, buf, n)

    def generate_tx(self, nsamples: int) -> bytes:
        """Erzeugt nsamples PCM-Samples (16 Bit, 8000 Hz) zum Senden an das
        DBT03 - inkl. Traeger/Leerlauf, solange keine echten Daten
        anstehen. Fuer eine gleichmaessige RTP-Taktung in 20ms-Haeppchen
        aufrufen (= 160 Samples @ 8000 Hz)."""
        buf = (ctypes.c_int16 * nsamples)()
        n = _lib.v23_link_generate_tx(self._handle, buf, nsamples)
        return bytes(buf)[: n * 2]

    def finished(self) -> bool:
        """True, wenn die TCP-Verbindung zum BTX-Server beendet wurde und
        der SIP-Call entsprechend aufgelegt werden sollte."""
        return bool(_lib.v23_link_finished(self._handle))

    def close(self) -> None:
        if self._handle:
            _lib.v23_link_free(self._handle)
            self._handle = None

    def __del__(self) -> None:
        self.close()
