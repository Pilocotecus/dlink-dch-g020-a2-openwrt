const CACHE = "bernal-home-v5";

const STATIC_FILES = [
  "/bernal-home/",
  "/bernal-home/index.html",
  "/bernal-home/manifest.json"
];

self.addEventListener("install", event => {
  event.waitUntil(
    caches.open(CACHE).then(cache => cache.addAll(STATIC_FILES))
  );
  self.skipWaiting();
});

self.addEventListener("activate", event => {
  event.waitUntil(self.clients.claim());
});

self.addEventListener("fetch", event => {
  const url = new URL(event.request.url);

  if (url.pathname.endsWith("/state.json")) {
    return;
  }

  event.respondWith(
    fetch(event.request).catch(() => caches.match(event.request))
  );
});
