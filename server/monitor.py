import json
import time
from datetime import datetime
from pathlib import Path

import requests
from pywebpush import webpush, WebPushException


STATE_URL = "https://192.168.1.4/bernal-home/state.json"

BASE = Path("/opt/bernal-home-server")
SUBSCRIPTIONS_FILE = BASE / "subscriptions.json"
HISTORY_FILE = BASE / "history.jsonl"
VAPID_PRIVATE_KEY = BASE / "keys" / "vapid-private.pem"

POLL_SECONDS = 2
OPEN_WARNING_SECONDS = 120

VAPID_CLAIMS = {
    "sub": "mailto:admin@familiabernalalcaraz.com"
}

DOORS = {
    "4": "Puerta principal",
    "3": "Puerta pérgola",
}

door_state = {
    node: {
        "state": None,
        "opened_at": None,
        "alert_sent": False,
    }
    for node in DOORS
}


def now_text():
    return datetime.now().astimezone().isoformat(
        timespec="seconds"
    )


def load_subscriptions():
    if not SUBSCRIPTIONS_FILE.exists():
        return []

    try:
        with SUBSCRIPTIONS_FILE.open(
            "r", encoding="utf-8"
        ) as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except Exception as e:
        print(
            f"[BA MONITOR] subscriptions: {e}",
            flush=True
        )
        return []


def save_subscriptions(items):
    with SUBSCRIPTIONS_FILE.open(
        "w", encoding="utf-8"
    ) as f:
        json.dump(
            items,
            f,
            ensure_ascii=False,
            indent=2
        )


def append_history(node, state):
    event = {
        "time": now_text(),
        "node": int(node),
        "name": DOORS[node],
        "state": state,
    }

    with HISTORY_FILE.open(
        "a", encoding="utf-8"
    ) as f:
        f.write(
            json.dumps(
                event,
                ensure_ascii=False
            ) + "\n"
        )

    print(
        f"[BA MONITOR] {DOORS[node]} -> {state}",
        flush=True
    )


def send_push(title, body):
    subscriptions = load_subscriptions()

    payload = json.dumps({
        "title": title,
        "body": body,
        "url": "https://192.168.1.4/bernal-home/"
    })

    valid = []

    for subscription in subscriptions:
        try:
            webpush(
                subscription_info=subscription,
                data=payload,
                vapid_private_key=str(
                    VAPID_PRIVATE_KEY
                ),
                vapid_claims=VAPID_CLAIMS
            )

            valid.append(subscription)

        except WebPushException as e:
            status = getattr(
                getattr(e, "response", None),
                "status_code",
                None
            )

            print(
                f"[BA MONITOR] push error "
                f"HTTP={status}: {e}",
                flush=True
            )

            if status not in (404, 410):
                valid.append(subscription)

    if len(valid) != len(subscriptions):
        save_subscriptions(valid)


def process_door(node, current):
    info = door_state[node]
    previous = info["state"]

    if current not in ("open", "closed"):
        return

    # Primera lectura: conocemos el estado pero no
    # inventamos un evento histórico.
    if previous is None:
        info["state"] = current

        if current == "open":
            info["opened_at"] = time.monotonic()
            info["alert_sent"] = False

        return

    # Cambio real de estado.
    if current != previous:
        append_history(node, current)

        info["state"] = current

        if current == "open":
            info["opened_at"] = time.monotonic()
            info["alert_sent"] = False

        else:
            info["opened_at"] = None
            info["alert_sent"] = False

        return

    # Sigue abierta: comprobar temporizador.
    if (
        current == "open"
        and info["opened_at"] is not None
        and not info["alert_sent"]
    ):
        elapsed = (
            time.monotonic() - info["opened_at"]
        )

        if elapsed >= OPEN_WARNING_SECONDS:
            minutes = int(elapsed // 60)

            send_push(
                "⚠️ BA Home",
                f"{DOORS[node]} lleva abierta "
                f"{minutes} minutos"
            )

            info["alert_sent"] = True

            print(
                f"[BA MONITOR] ALERTA: "
                f"{DOORS[node]} abierta "
                f"{minutes} minutos",
                flush=True
            )


def read_state():
    response = requests.get(
        STATE_URL,
        timeout=5,
        verify="/etc/ssl/certs/ca-certificates.crt"
    )

    response.raise_for_status()
    return response.json()


def main():
    print(
        "[BA MONITOR] Bernal Home Monitor iniciado",
        flush=True
    )

    last_error = None

    while True:
        try:
            data = read_state()
            nodes = data.get("nodes", {})

            for node in DOORS:
                state = nodes.get(node) or {}
                process_door(
                    node,
                    state.get("contact")
                )

            if last_error is not None:
                print(
                    "[BA MONITOR] DCH-G020 recuperado",
                    flush=True
                )
                last_error = None

        except Exception as e:
            message = str(e)

            if message != last_error:
                print(
                    f"[BA MONITOR] DCH-G020: "
                    f"{message}",
                    flush=True
                )
                last_error = message

        time.sleep(POLL_SECONDS)


if __name__ == "__main__":
    main()
