#!/bin/bash

BASE=/opt/bernal-home-server

echo "[BA HOME] Preparando servidor..."

mkdir -p "$BASE"
mkdir -p "$BASE/keys"

if [ ! -d "$BASE/venv" ]; then
    python3 -m venv "$BASE/venv"
fi

"$BASE/venv/bin/pip" install \
    Flask requests pywebpush cryptography

cp server.py "$BASE/server.py"
cp monitor.py "$BASE/monitor.py"

cp systemd/bernal-home.service \
   /etc/systemd/system/bernal-home.service

cp systemd/bernal-home-monitor.service \
   /etc/systemd/system/bernal-home-monitor.service

systemctl daemon-reload

systemctl enable apache2
systemctl enable bernal-home
systemctl enable bernal-home-monitor

systemctl restart bernal-home
systemctl restart bernal-home-monitor

echo "[BA HOME] Instalación terminada"
echo "[BA HOME] Recuerda instalar la clave VAPID privada y configurar Apache."
