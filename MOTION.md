# sprayplot motion — rail hoist + free-rolling face

Wheels own the paint path. The rail owns lift and column index.

```
==== modular rail =====================================
         trolley + hoist motor
              |  working line (powered)
              |  safety line  (passive, separate anchor)
              v
         [paintbot]  wheels loaded on the face
                     nozzles offset to one side of the contact patches
```

## Roles

| piece | does | does not |
|---|---|---|
| wall wheels | roll the swath, hold standoff, stay off wet film | lift the mass |
| rail trolley + motor | pay out / haul the working line, step one swath sideways | steer the spray path |
| safety line | independent backup through a fall-arrest / overspeed lock | carry running load |

Two lines, two anchors. One motor is enough if the safety line is passive.

## Cycle

1. Trolley at column *n*, bot at the top (`l ≈ 0`).
2. Winch pays out; wheels roll down; nozzles spray the offset strip.
3. Bottom of swath: spray off.
4. Winch hauls back to the top.
5. Trolley steps by swath width minus overlap.
6. Repeat.

## Pose

Face frame: origin at the first rail-section end, +X along the rail, +Y down.

| symbol | source |
|---|---|
| `s_mm` | trolley encoder |
| `l_mm` | winch-drum encoder (working line only) |
| `wheel_*` | face-wheel odometry |
| laser az/el | complementary correction while `STATE_TRACK` |

`fuseWithOdometry()` treats `(s, l)` as the kinematic prior and the locked bearing as a residual on cable stretch, rail sag, and wheel slip. Encoder scale factors (`MM_PER_RAIL_TICK`, `MM_PER_WINCH_TICK`, `MM_PER_WHEEL_TICK`) stay bench knobs until the drums and wheels exist.

Safety-line payout is not in the pose. If the overspeed lock trips, `POSE_SAFETY_LOCKED` is set and the fuse stops trusting `l_mm`.
