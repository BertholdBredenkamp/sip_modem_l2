#!/bin/sh
# Baut libv23bridge.so (fuer v23modem.py) sowie die optionalen Tests
# selftest (reine V.23-Modulation/Demodulation) und linktest (End-to-End
# inkl. simuliertem Terminal und TCP-Testserver).
#
# Voraussetzung (Debian/Ubuntu): sudo apt-get install libspandsp-dev gcc pkg-config
set -e

cd "$(dirname "$0")"

echo "==> libv23bridge.so"
gcc -O2 -Wall -Wextra -fPIC -shared -o libv23bridge.so v23_bridge.c \
    $(pkg-config --cflags --libs spandsp) -lpthread

echo "==> selftest (optional, prueft reine V.23-Modulation/-Demodulation)"
gcc -O2 -Wall -Wextra -o selftest selftest.c $(pkg-config --cflags --libs spandsp)

echo "==> linktest (optional, End-to-End: simuliertes Terminal + TCP-Testserver)"
gcc -O2 -Wall -Wextra -o linktest linktest.c v23_bridge.c $(pkg-config --cflags --libs spandsp) -lpthread

echo "Fertig. Tests ausfuehren mit: ./selftest && ./linktest"

