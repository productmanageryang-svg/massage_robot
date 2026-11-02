self.addEventListener("install", function (event) {
  self.skipWaiting();
});

self.addEventListener("activate", function (event) {
  event.waitUntil(self.clients.claim());
});

self.addEventListener("fetch", function (event) {
  const url = new URL(event.request.url);
  if (url.pathname.startsWith("/api/") || url.pathname.startsWith("/voice/")) {
    return;
  }
  event.respondWith(
    fetch(event.request).catch(function () {
      return caches.match(event.request).then(function (hit) {
        return hit || fetch(event.request);
      });
    })
  );
});
