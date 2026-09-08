# sprayplot laser beacon tracker — firmware

Two ESP32 sketches implementing an active-beacon localization system: a
base station that steers a laser via two mirror servos, and a robot-side
receiver that demodulates a photodiode signal and reports it back over
ESP-NOW.

Copy the tree into your Arduino sketchbook and keep the folder layout —
each `.ino` `#include`s `../shared/protocol.h`.

## Files

```
sprayplot_firmware/
  shared/protocol.h          packet structs + constants, included by both sketches
  base_station/base_station.ino
  robot_receiver/robot_receiver.ino
MOTION.md                   rail trolley + working line + safety line + face frame
```

## Hardware

**Base station**
- ESP32 dev board
- 2x servo (azimuth mirror, elevation mirror) — pins 18, 19
- Laser diode module, driven via LEDC PWM — pin 21

**Robot**
- ESP32 dev board (can be the same board driving motion/spray, or a dedicated one)
- Photodiode + transimpedance amp feeding an ADC1 pin — pin 34
- Face motion: free-rolling wheels + rail trolley hoist + independent safety line
  (see `MOTION.md`). Encoder tick scales start at 0 until the drums exist.

## Before flashing

Get each board's MAC address (`WiFi.macAddress()` in a throwaway sketch)
and fill in:
- `ROBOT_MAC` in `base_station.ino`
- `BASE_MAC` in `robot_receiver.ino`

Install the `ESP32Servo` library via the Arduino Library Manager.

## First run: boundary mapping

1. Flash and power both boards. The base station boots straight into
   `BOUNDARY_MAP` mode and prints a live status line over serial:
   ```
   az=90.00  el=90.00  step=1.00  amp=0.12          pts=0
   ```
2. Walk the beam to each corner of the intended work area using the
   serial monitor:

   | key | action |
   |-----|--------|
   | `w` / `s` | jog elevation up / down |
   | `a` / `d` | jog azimuth left / right |
   | `+` / `-` | increase / decrease jog step size |
   | `c` | confirm current point as a boundary vertex |
   | `u` | undo the last confirmed point |
   | `f` | finish mapping — computes and saves the az/el envelope |
   | `g` | start tracking (requires the envelope to be finished first) |
   | `r` | return to boundary mapping from any other state |

   Watch the `amp=` value — it climbs and the line gets an
   `<-- ALIGNED` suffix once the beam is actually centered on the
   photodiode. Confirm points there, not just by eye.
3. Press `f` once you have at least 3 points. The envelope is saved to
   flash (NVS), so it survives a power cycle — on the next boot you'll
   see `loaded saved envelope: ...` and can skip straight to `g`.
4. Press `g` to start `SEARCH -> ACQUIRE -> TRACK`.

## Tuning

These constants live at the top of `base_station.ino` and are meant to be
adjusted on the bench, not guessed in advance:

| constant | what it does | symptom if wrong |
|---|---|---|
| `SERVO_MIN_DEG` / `SERVO_MAX_DEG` | usable servo travel | mirror hits a mechanical stop |
| `LOCK_THRESHOLD` | amplitude considered "beam is hitting the diode" | false locks (too low) or never acquiring (too high) |
| `DITHER_DEG` | dither circle size during TRACK | too small: loses lock on fast motion; too large: visible beam jitter |
| `TRACK_GAIN` | proportional gain in the dither loop | too high: oscillation; too low: sluggish tracking |
| `ACQUIRE_HOLD_MS` | how long a hit must persist before trusting it | shorter = faster lock but more false positives |
| `LOST_TIMEOUT_MS` | how long a dropout is tolerated before declaring LOST | too short: drops lock on brief occlusions (e.g. paint mist) |

Start with the defaults, run `TRACK`, and increase `TRACK_GAIN` until you
see oscillation, then back off by roughly a third.

## Known gaps

- Encoder inputs (`rail_ticks`, `winch_ticks`, `wheel_ticks`) are declared
  but not attached to hardware. Set `MM_PER_*_TICK` once the drums and
  wheels exist; until then the fuse rides a stationary prior plus the
  locked laser residual.
- `MM_PER_DEG_AZ` / `MM_PER_DEG_EL` are a 3 m standoff guess. Replace with
  a real base-to-wall extrinsic after the first mapped envelope.
- The Goertzel normalization constant (`GOERTZEL_N * 2048.0f`) assumes a
  12-bit ADC with a photodiode signal roughly centered mid-scale; rescale
  after checking real signal levels on your photodiode/amp combination.
- No CMYK / multi-channel handling — this covers localization only.
