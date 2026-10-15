import { defineConfig } from 'vite';
import { VitePWA } from 'vite-plugin-pwa';

export default defineConfig({
  // Relative base so the site works under https://<user>.github.io/<repo>/
  base: './',
  build: { target: 'es2022', chunkSizeWarningLimit: 1200 },
  plugins: [
    VitePWA({
      registerType: 'autoUpdate',
      injectRegister: false,
      includeAssets: ['icons/*.png', 'models/*.glb'],
      manifest: {
        name: 'Chuck Norris · Team 3950',
        short_name: 'Chuck Norris',
        description: 'Interactive 3D showcase of FRC Team 3950’s robot.',
        display: 'fullscreen',
        orientation: 'any',
        background_color: '#060708',
        theme_color: '#060708',
        start_url: './',
        scope: './',
        icons: [
          { src: 'icons/icon-192.png', sizes: '192x192', type: 'image/png' },
          { src: 'icons/icon-512.png', sizes: '512x512', type: 'image/png' },
          { src: 'icons/icon-512.png', sizes: '512x512', type: 'image/png', purpose: 'maskable' },
        ],
      },
      workbox: {
        // Precache everything, including the model, so it works with zero Wi-Fi after the first open.
        globPatterns: ['**/*.{js,css,html,png,svg,glb,webmanifest}'],
        maximumFileSizeToCacheInBytes: 30 * 1024 * 1024,
        cleanupOutdatedCaches: true,
      },
    }),
  ],
});
