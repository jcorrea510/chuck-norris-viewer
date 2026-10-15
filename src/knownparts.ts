// Published specs for common FRC components, shown when a judge taps one.
// Matched by name; first match wins.
import type { Spec } from './config';

interface Known { match: RegExp; summary: string; specs: Spec[] }

const KNOWN: Known[] = [
  { match: /kraken x60/i, summary: 'Brushless motor with an integrated TalonFX controller.', specs: [
    { label: 'Free Speed', value: '6,000 RPM' }, { label: 'Stall Torque', value: '7.09 N·m' }, { label: 'Peak Power', value: '1,108 W' }, { label: 'Maker', value: 'WCP' }] },
  { match: /kraken x44/i, summary: 'Compact brushless motor with an integrated TalonFX controller.', specs: [
    { label: 'Free Speed', value: '7,758 RPM' }, { label: 'Stall Torque', value: '4.11 N·m' }, { label: 'Peak Power', value: '835 W' }, { label: 'Maker', value: 'WCP' }] },
  { match: /neo vortex|vortex/i, summary: 'REV brushless motor with a swappable controller.', specs: [
    { label: 'Free Speed', value: '6,784 RPM' }, { label: 'Stall Torque', value: '3.6 N·m' }] },
  { match: /neo 550/i, summary: 'Small REV brushless motor.', specs: [{ label: 'Free Speed', value: '11,000 RPM' }, { label: 'Stall Torque', value: '0.97 N·m' }] },
  { match: /\bneo\b/i, summary: 'REV brushless motor.', specs: [{ label: 'Free Speed', value: '5,676 RPM' }, { label: 'Stall Torque', value: '2.6 N·m' }] },
  { match: /sds mk4i/i, summary: 'Swerve Drive Specialties inverted swerve module.', specs: [
    { label: 'Wheel', value: '4" × 1.5"' }, { label: 'Steering', value: '150:7 ratio' }, { label: 'Encoder', value: 'CTRE CANcoder' }] },
  { match: /cancoder/i, summary: 'Absolute magnetic encoder on the CAN bus.', specs: [{ label: 'Resolution', value: '4,096 counts/rev' }, { label: 'Maker', value: 'CTRE' }] },
  { match: /limelight/i, summary: 'Smart camera for AprilTag localization and targeting.', specs: [{ label: 'Use', value: 'Pose estimation' }, { label: 'Maker', value: 'Limelight' }] },
  { match: /vh-109|robot radio/i, summary: 'Official FRC robot radio.', specs: [{ label: 'Model', value: 'Vivid-Hosting VH-109' }, { label: 'Bands', value: '6 GHz field link' }] },
  { match: /pdh|power distribution/i, summary: 'Distributes battery power to every circuit, with CAN telemetry.', specs: [{ label: 'Channels', value: '24 (20 high-current)' }, { label: 'Maker', value: 'REV' }] },
  { match: /120a|main breaker/i, summary: 'The robot’s main power switch and circuit breaker.', specs: [{ label: 'Rating', value: '120 A' }] },
  { match: /\bmpm\b|mini power/i, summary: 'Mini Power Module for low-current devices.', specs: [{ label: 'Maker', value: 'REV' }] },
  { match: /rsl|signal light/i, summary: 'Robot Signal Light: flashes when the robot is enabled.', specs: [{ label: 'Required', value: 'FRC rule' }] },
  { match: /maxplanetary/i, summary: 'Modular REV planetary gearbox.', specs: [{ label: 'Maker', value: 'REV' }] },
  { match: /versaplanetary/i, summary: 'Modular VEX planetary gearbox.', specs: [{ label: 'Maker', value: 'VEX / WCP' }] },
  { match: /200t 20dp turntable|turntable gear/i, summary: 'The large ring gear each turret rotates on.', specs: [{ label: 'Teeth', value: '200T, 20 DP' }, { label: 'Maker', value: 'AndyMark' }] },
  { match: /brass flywheel/i, summary: 'Heavy flywheel that stores energy so every shot leaves at the same speed.', specs: [{ label: 'Diameter', value: '4"' }, { label: 'Material', value: 'Brass' }] },
  { match: /colson/i, summary: 'Grippy, durable wheel.', specs: [{ label: 'Diameter', value: '4"' }] },
  { match: /flex wheel/i, summary: 'Soft compliant wheel that grips game pieces.', specs: [{ label: 'Durometer', value: '30A' }] },
  { match: /mecanum roller/i, summary: 'Angled rollers push fuel sideways and forward at once.', specs: [] },
  { match: /smart robot servo/i, summary: 'Programmable servo that sets the shooter hood angle.', specs: [{ label: 'Maker', value: 'REV' }] },
  { match: /brain pan polycarb/i, summary: 'Hinged polycarbonate cover protecting the electronics underneath.', specs: [{ label: 'Hinges', value: '4× McMaster 1609A53' }] },
  { match: /turret wiring side panel/i, summary: 'Side panel that guides the turret wiring.', specs: [] },
];

export function knownFor(name: string): Known | null {
  return KNOWN.find((k) => k.match.test(name)) ?? null;
}
