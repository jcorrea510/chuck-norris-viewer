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
import i_settings from 'lucide-static/icons/settings.svg?raw';
import cpu from 'lucide-static/icons/cpu.svg?raw';
import gauge from 'lucide-static/icons/gauge.svg?raw';
import target from 'lucide-static/icons/target.svg?raw';
import wrench from 'lucide-static/icons/wrench.svg?raw';
import camera from 'lucide-static/icons/camera.svg?raw';
import battery from 'lucide-static/icons/battery.svg?raw';
import x from 'lucide-static/icons/x.svg?raw';
import i_search from 'lucide-static/icons/search.svg?raw';
import i_rotate_3d from 'lucide-static/icons/rotate-3d.svg?raw';
import i_play from 'lucide-static/icons/play.svg?raw';
import i_scan_eye from 'lucide-static/icons/scan-eye.svg?raw';
import i_rotate_ccw from 'lucide-static/icons/rotate-ccw.svg?raw';
import i_eye_off from 'lucide-static/icons/eye-off.svg?raw';
import i_focus from 'lucide-static/icons/focus.svg?raw';
import i_arrow_up_from_line from 'lucide-static/icons/arrow-up-from-line.svg?raw';
import i_eye from 'lucide-static/icons/eye.svg?raw';
import i_gamepad from 'lucide-static/icons/gamepad-2.svg?raw';
import i_keyboard from 'lucide-static/icons/keyboard.svg?raw';
import i_bot from 'lucide-static/icons/bot.svg?raw';
import i_smartphone from 'lucide-static/icons/smartphone.svg?raw';
import i_abd from 'lucide-static/icons/arrow-big-down.svg?raw';
import i_abu from 'lucide-static/icons/arrow-big-up.svg?raw';
import i_cdot from 'lucide-static/icons/circle-dot.svg?raw';
import i_undo from 'lucide-static/icons/undo-2.svg?raw';
import i_hand from 'lucide-static/icons/hand.svg?raw';
import i_trophy from 'lucide-static/icons/trophy.svg?raw';
import i_video from 'lucide-static/icons/video.svg?raw';
import i_volume from 'lucide-static/icons/volume-2.svg?raw';

const ICONS: Record<string, string> = {
  move, 'arrow-down-to-line': arrowDownToLine, layers, crosshair, zap, box, house,
  'square-play': squarePlay, boxes, maximize, minimize, 'chevron-left': chevronLeft,
  'chevron-right': chevronRight, pointer, 'circle-check': circleCheck, cog, settings: i_settings, cpu, gauge,
  target, wrench, camera, battery, x,
  'search': i_search,
  'rotate-3d': i_rotate_3d,
  'play': i_play,
  'scan-eye': i_scan_eye,
  'rotate-ccw': i_rotate_ccw,
  'eye-off': i_eye_off,
  'focus': i_focus,
  'arrow-up-from-line': i_arrow_up_from_line,
  'eye': i_eye,
  'gamepad-2': i_gamepad,
  'keyboard': i_keyboard,
  'bot': i_bot,
  'smartphone': i_smartphone,
  'arrow-big-down': i_abd,
  'arrow-big-up': i_abu,
  'circle-dot': i_cdot,
  'undo-2': i_undo,
  'hand': i_hand,
  'trophy': i_trophy,
  'video': i_video,
  'volume-2': i_volume,
};

/** Returns inline SVG markup for an icon name; unknown names fall back to a box. */
export function icon(name: string): string {
  const svg = (ICONS[name] ?? ICONS.box).replace(/<!--.*?-->/s, '').replace('stroke-width="2"', 'stroke-width="1.75"');
  return `<span class="icon" aria-hidden="true">${svg}</span>`;
}

export const ICON_NAMES = Object.keys(ICONS);
