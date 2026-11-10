// Progressive Web App (PWA) Service Worker for Smart Starter Dashboard
const CACHE_NAME = 'smart-starter-v3';
const ASSETS_TO_CACHE = [
    '/',
    '/index.html',
    '/manifest.json',
    '/icon.svg',
    'https://cdn.tailwindcss.com',
    'https://unpkg.com/mqtt/dist/mqtt.min.js'
];

// Install Event: Cache shell and external CDN libraries for fast offline/mobile loading
self.addEventListener('install', (event) => {
    event.waitUntil(
        caches.open(CACHE_NAME).then((cache) => {
            console.log('[SW] Pre-caching dashboard shell and CDN libraries v3...');
            return cache.addAll(ASSETS_TO_CACHE).catch(err => {
                console.warn('[SW] Caching non-critical asset failed:', err);
            });
        })
    );
    self.skipWaiting();
});

// Activate Event: Immediately purge old cache versions (v1, v2)
self.addEventListener('activate', (event) => {
    event.waitUntil(
        caches.keys().then((cacheNames) => {
            return Promise.all(
                cacheNames.map((cache) => {
                    if (cache !== CACHE_NAME) {
                        console.log('[SW] Purging old cache version:', cache);
                        return caches.delete(cache);
                    }
                })
            );
        })
    );
    self.clients.claim();
});

// Fetch Event: Network first for index.html to guarantee latest UI, falling back to cache
self.addEventListener('fetch', (event) => {
    if (event.request.url.includes('index.html') || event.request.url === self.location.origin + '/') {
        event.respondWith(
            fetch(event.request).then((networkResponse) => {
                if (networkResponse.status === 200) {
                    const responseClone = networkResponse.clone();
                    caches.open(CACHE_NAME).then((cache) => {
                        cache.put(event.request, responseClone);
                    });
                }
                return networkResponse;
            }).catch(() => caches.match(event.request))
        );
        return;
    }

    event.respondWith(
        caches.match(event.request).then((cachedResponse) => {
            if (cachedResponse) {
                return cachedResponse;
            }
            return fetch(event.request).then((networkResponse) => {
                if (event.request.method === 'GET' && networkResponse.status === 200) {
                    const responseClone = networkResponse.clone();
                    caches.open(CACHE_NAME).then((cache) => {
                        cache.put(event.request, responseClone);
                    });
                }
                return networkResponse;
            });
        })
    );
});

// Mobile Push Notification Event Handler
self.addEventListener('notificationclick', (event) => {
    event.notification.close();
    event.waitUntil(
        clients.matchAll({ type: 'window', includeUncontrolled: true }).then((clientList) => {
            if (clientList.length > 0) {
                return clientList[0].focus();
            }
            return clients.openWindow('/');
        })
    );
});
