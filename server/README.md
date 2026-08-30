# Bernal Home Server

Backend de BA Home para:

- Web Push
- Suscripciones de móviles
- Monitorización de puertas
- Alertas por puerta abierta
- Historial persistente

## Rutas principales

- `/api/health`
- `/api/subscribe`
- `/api/subscriptions`
- `/api/push/test`
- `/api/history`

## Instalación base

Ruta prevista:

    /opt/bernal-home-server

Crear entorno virtual:

    python3 -m venv /opt/bernal-home-server/venv

Instalar dependencias:

    /opt/bernal-home-server/venv/bin/pip install -r requirements.txt

## Datos no incluidos en Git

Nunca subir:

- claves VAPID privadas
- subscriptions.json
- history.jsonl
- claves privadas TLS o CA

La clave VAPID privada esperada por la instalación actual es:

    /opt/bernal-home-server/keys/vapid-private.pem

## Servicios systemd

Los ejemplos están en:

    server/systemd/

Después de instalarlos:

    systemctl daemon-reload
    systemctl enable --now bernal-home
    systemctl enable --now bernal-home-monitor

