#!/bin/sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

SOURCE="$ROOT/web/bernal-home"
FIRMWARE="$ROOT/openwrt/files/www/bernal-home"

echo "=== Bernal Home web sync ==="
echo "SOURCE   : $SOURCE"
echo "FIRMWARE : $FIRMWARE"
echo

[ -f "$SOURCE/index.html" ] || {
    echo "ERROR: falta $SOURCE/index.html"
    exit 1
}

mkdir -p "$FIRMWARE"

# Frontend principal
cp "$SOURCE/index.html" \
   "$FIRMWARE/index.html"

# Copiar también los assets existentes que forman parte
# del frontend, si están presentes.
for file in \
    service-worker.js \
    manifest.json
do
    if [ -f "$SOURCE/$file" ]; then
        cp "$SOURCE/$file" \
           "$FIRMWARE/$file"
        echo "SYNC: $file"
    fi
done

echo "SYNC: index.html"

echo
echo "=== CHECKSUM INDEX ==="

SRC_SHA="$(sha256sum "$SOURCE/index.html" | awk '{print $1}')"
DST_SHA="$(sha256sum "$FIRMWARE/index.html" | awk '{print $1}')"

echo "SOURCE   : $SRC_SHA"
echo "FIRMWARE : $DST_SHA"

if [ "$SRC_SHA" != "$DST_SHA" ]; then
    echo "ERROR: index.html no coincide"
    exit 1
fi

echo
echo "OK: frontend sincronizado"
