// Lucide icons (ISC licence), inlined at build time so nothing loads at runtime.
import move from 'lucide-static/icons/move.svg?raw';
import arrowDownToLine from 'lucide-static/icons/arrow-down-to-line.svg?raw';
import layers from 'lucide-static/icons/layers.svg?raw';
import crosshair from 'lucide-static/icons/crosshair.svg?raw';
import zap from 'lucide-static/icons/zap.svg?raw';
import box from 'lucide-static/icons/box.svg?raw';
import house from 'lucide-static/icons/house.svg?raw';
import squarePlay from 'lucide-static/icons/square-play.svg?raw';
import boxes from 'lucide-static/icons/boxes.svg?raw';
import maximize from 'lucide-static/icons/maximize.svg?raw';
import minimize from 'lucide-static/icons/minimize.svg?raw';
import chevronLeft from 'lucide-static/icons/chevron-left.svg?raw';
import chevronRight from 'lucide-static/icons/chevron-right.svg?raw';
import pointer from 'lucide-static/icons/pointer.svg?raw';
import circleCheck from 'lucide-static/icons/circle-check.svg?raw';
import cog from 'lucide-static/icons/cog.svg?raw';
import cpu from 'lucide-static/icons/cpu.svg?raw';
import gauge from 'lucide-static/icons/gauge.svg?raw';
import target from 'lucide-static/icons/target.svg?raw';
import wrench from 'lucide-static/icons/wrench.svg?raw';
import camera from 'lucide-static/icons/camera.svg?raw';
import battery from 'lucide-static/icons/battery.svg?raw';
import x from 'lucide-static/icons/x.svg?raw';

const ICONS: Record<string, string> = {
  move, 'arrow-down-to-line': arrowDownToLine, layers, crosshair, zap, box, house,
  'square-play': squarePlay, boxes, maximize, minimize, 'chevron-left': chevronLeft,
  'chevron-right': chevronRight, pointer, 'circle-check': circleCheck, cog, cpu, gauge,
  target, wrench, camera, battery, x,
};

/** Returns inline SVG markup for an icon name; unknown names fall back to a box. */
export function icon(name: string): string {
  const svg = (ICONS[name] ?? ICONS.box).replace(/<!--.*?-->/s, '').replace('stroke-width="2"', 'stroke-width="1.75"');
  return `<span class="icon" aria-hidden="true">${svg}</span>`;
}

export const ICON_NAMES = Object.keys(ICONS);
