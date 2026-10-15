#!/usr/bin/env node
// Renders the Home Screen / PWA icons into public/icons/ (run once; the PNGs are committed).
import { chromium } from 'playwright';
import fs from 'node:fs';

const svg = (size) => `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}" viewBox="0 0 512 512">
  <defs>
    <radialGradient id="bg" cx="50%" cy="42%" r="75%">
      <stop offset="0" stop-color="#23262c"/><stop offset="0.55" stop-color="#101114"/><stop offset="1" stop-color="#060708"/>
    </radialGradient>
    <linearGradient id="m" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#c22d55"/><stop offset="1" stop-color="#7d1731"/></linearGradient>
  </defs>
  <rect width="512" height="512" fill="url(#bg)"/>
  <g fill="none" stroke="#f08ca3" stroke-opacity="0.18" stroke-width="2">
    <ellipse cx="256" cy="360" rx="190" ry="54"/><ellipse cx="256" cy="360" rx="120" ry="34"/>
  </g>
  <g transform="translate(256 262)">
    <path d="M-120 40 L0 100 L120 40 L0 -20 Z" fill="#3a3e46"/>
    <path d="M-120 40 L0 100 L0 128 L-120 68 Z" fill="#23262c"/>
    <path d="M120 40 L0 100 L0 128 L120 68 Z" fill="#2c3037"/>
    <path d="M-62 -6 L0 26 L62 -6 L0 -38 Z" fill="url(#m)"/>
    <path d="M-62 -6 L0 26 L0 76 L-62 44 Z" fill="#6b1229"/>
    <path d="M62 -6 L0 26 L0 76 L62 44 Z" fill="#8f1c3a"/>
    <circle cx="-34" cy="-70" r="30" fill="#d7dbe2"/><circle cx="34" cy="-70" r="30" fill="#d7dbe2"/>
    <rect x="-46" y="-70" width="24" height="70" fill="#d7dbe2"/><rect x="22" y="-70" width="24" height="70" fill="#d7dbe2"/>
    <circle cx="-34" cy="-70" r="13" fill="#1a1c20"/><circle cx="34" cy="-70" r="13" fill="#1a1c20"/>
  </g>
  <text x="256" y="462" text-anchor="middle" font-family="-apple-system, Helvetica, Arial, sans-serif" font-size="54" font-weight="700" fill="#ffffff" letter-spacing="2">3950</text>
</svg>`;

fs.mkdirSync('public/icons', { recursive: true });
fs.writeFileSync('public/icons/icon.svg', svg(512));
const browser = await chromium.launch();
const page = await browser.newPage();
for (const [name, size] of [['apple-touch-icon.png', 180], ['icon-192.png', 192], ['icon-512.png', 512]]) {
  await page.setViewportSize({ width: size, height: size });
  await page.setContent(`<html><body style="margin:0;background:#060708">${svg(size)}</body></html>`);
  await page.screenshot({ path: `public/icons/${name}`, omitBackground: false });
}
await browser.close();
console.log('Icons written to public/icons/');
