const CACHE = "bernal-home-v6";

const STATIC_FILES = [
  "/bernal-home/",
  "/bernal-home/index.html",
  "/bernal-home/manifest.json"
];

self.addEventListener("install", event => {
  event.waitUntil(
    caches.open(CACHE).then(
      cache => cache.addAll(STATIC_FILES)
    )
  );

  self.skipWaiting();
});

self.addEventListener("activate", event => {
  event.waitUntil(
    Promise.all([
      self.clients.claim(),
      caches.keys().then(keys =>
        Promise.all(
          keys
            .filter(key => key !== CACHE)
            .map(key => caches.delete(key))
        )
      )
    ])
  );
});

self.addEventListener("fetch", event => {
  const url = new URL(event.request.url);

  if (
    url.pathname.endsWith("/state.json") ||
    url.pathname.endsWith("/history.jsonl")
  ) {
    return;
  }

  event.respondWith(
    fetch(event.request)
      .catch(() => caches.match(event.request))
  );
});

self.addEventListener("push", event => {
  let data = {
    title: "BA Home",
    body: "Nueva notificación",
    url: "/bernal-home/"
  };

  if (event.data) {
    try {
      data = {
        ...data,
        ...event.data.json()
      };
    }
    catch {
      data.body = event.data.text();
    }
  }

  event.waitUntil(
    self.registration.showNotification(
      data.title,
      {
        body: data.body,
        icon: "/bernal-home/icons/icon-192.png",
        badge: "/bernal-home/icons/icon-192.png",
        tag: "ba-home",
        renotify: true,
        data: {
          url: data.url
        }
      }
    )
  );
});

self.addEventListener(
  "notificationclick",
  event => {
    event.notification.close();

    const target =
      event.notification.data?.url ||
      "/bernal-home/";

    event.waitUntil(
      clients.matchAll({
        type: "window",
        includeUncontrolled: true
      }).then(windowClients => {
        for (const client of windowClients) {
          if ("focus" in client) {
            client.navigate(target);
            return client.focus();
          }
        }

        return clients.openWindow(target);
      })
    );
  }
);
