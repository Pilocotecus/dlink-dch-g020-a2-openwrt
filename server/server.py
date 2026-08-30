from flask import Flask, jsonify, request
from datetime import datetime
from pathlib import Path
from pywebpush import webpush, WebPushException
import json

app = Flask(__name__)

@app.after_request
def add_cors_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "https://192.168.1.4"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    return response


BASE = Path("/opt/bernal-home-server")
SUBSCRIPTIONS_FILE = BASE / "subscriptions.json"
VAPID_PRIVATE_KEY = BASE / "keys" / "vapid-private.pem"

VAPID_CLAIMS = {
    "sub": "mailto:admin@familiabernalalcaraz.com"
}


def load_subscriptions():
    if not SUBSCRIPTIONS_FILE.exists():
        return []

    try:
        with SUBSCRIPTIONS_FILE.open("r", encoding="utf-8") as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except Exception:
        return []


def save_subscriptions(items):
    with SUBSCRIPTIONS_FILE.open("w", encoding="utf-8") as f:
        json.dump(items, f, ensure_ascii=False, indent=2)


@app.get("/api/health")
def health():
    return jsonify({
        "service": "Bernal Home Server",
        "status": "ok",
        "subscriptions": len(load_subscriptions()),
        "time": datetime.now().astimezone().isoformat(timespec="seconds")
    })


@app.post("/api/subscribe")
def subscribe():
    subscription = request.get_json(silent=True)

    if not subscription or not subscription.get("endpoint"):
        return jsonify({"ok": False, "error": "invalid subscription"}), 400

    subscriptions = load_subscriptions()

    endpoint = subscription["endpoint"]

    if not any(s.get("endpoint") == endpoint for s in subscriptions):
        subscriptions.append(subscription)
        save_subscriptions(subscriptions)

    return jsonify({
        "ok": True,
        "subscriptions": len(subscriptions)
    })



@app.get("/api/history")
def history():
    history_file = BASE / "history.jsonl"
    events = []

    if history_file.exists():
        with history_file.open("r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()

                if not line:
                    continue

                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    continue

    return jsonify({
        "count": len(events),
        "events": events[-200:]
    })


@app.get("/api/subscriptions")
def subscriptions():
    items = load_subscriptions()

    return jsonify({
        "count": len(items)
    })


@app.post("/api/push/test")
def push_test():
    subscriptions = load_subscriptions()

    payload = json.dumps({
        "title": "🔔 BA Home",
        "body": "Notificación de prueba recibida correctamente.",
        "url": "https://192.168.1.4/bernal-home/"
    })

    sent = 0
    failed = 0
    valid = []

    for subscription in subscriptions:
        try:
            webpush(
                subscription_info=subscription,
                data=payload,
                vapid_private_key=str(VAPID_PRIVATE_KEY),
                vapid_claims=VAPID_CLAIMS
            )

            sent += 1
            valid.append(subscription)

        except WebPushException as e:
            failed += 1
            print(f"[BA HOME] PUSH ERROR: {e}", flush=True)

            status = getattr(getattr(e, "response", None), "status_code", None)

            if status not in (404, 410):
                valid.append(subscription)

    if len(valid) != len(subscriptions):
        save_subscriptions(valid)

    return jsonify({
        "ok": True,
        "sent": sent,
        "failed": failed,
        "subscriptions": len(valid)
    })


@app.get("/")
def root():
    return jsonify({
        "name": "Bernal Home Server",
        "status": "running"
    })


if __name__ == "__main__":
    app.run(
        host="127.0.0.1",
        port=8765,
        debug=False
    )
