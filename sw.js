// Progressive Web App (PWA) Service Worker for Smart Starter Dashboard
const CACHE_NAME = 'smart-starter-v2';
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
            console.log('[SW] Pre-caching dashboard shell and CDN libraries...');
            return cache.addAll(ASSETS_TO_CACHE).catch(err => {
                console.warn('[SW] Caching non-critical asset failed:', err);
            });
        })
    );
    self.skipWaiting();
});

// Activate Event: Clean up old cache versions
self.addEventListener('activate', (event) => {
    event.waitUntil(
        caches.keys().then((cacheNames) => {
            return Promise.all(
                cacheNames.map((cache) => {
                    if (cache !== CACHE_NAME) {
                        console.log('[SW] Clearing old cache:', cache);
                        return caches.delete(cache);
                    }
                })
            );
        })
    );
    self.clients.claim();
});

// Fetch Event: Cache First for fast mobile load, falling back to network
self.addEventListener('fetch', (event) => {
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
