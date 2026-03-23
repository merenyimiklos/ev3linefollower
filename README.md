# ev3linefollower

Competition-style 3-sensor line follower for LEGO Mindstorms EV3.

---

## Control Strategy

The robot uses three colour sensors in reflected-light-intensity mode.

| Sensor | Port | Role |
|--------|------|------|
| Left   | 1    | Edge guard / left-side recovery |
| Middle | 2    | Primary PD tracking sensor |
| Right  | 3    | Edge guard / right-side recovery |

### Normal tracking (PD control)
The middle sensor rides the **edge** of the black line.
Its raw reading is normalised to a –1 … +1 error signal (0 = perfectly on the edge).
A **PD controller** (proportional + derivative) converts that error into a
differential speed correction applied to the two large motors (ports C and B).
The derivative term damps oscillation on straight sections.

### Side-sensor overrides
When a side sensor detects black *without* the middle sensor:
- **Left sensor only** → robot drifted too far right; apply a sharp left correction.
- **Right sensor only** → robot drifted too far left; apply a sharp right correction.
- **Both side sensors** → robot is centred on a wide mark; continue with PD.

These overrides give fast, reliable recovery on tight curves and cross-junctions.

### Line-lost recovery
When **all three sensors** see white the line is considered lost.
The robot pivots toward the side of the **last known error** at `RECOVERY_SPEED`
until any sensor finds the line again.

---

## File

| File | Description |
|------|-------------|
| `line_follower.py` | Full line-follower program (Python 3, ev3dev2) |

---

## Running on the robot

```bash
python3 line_follower.py
```

Press the **center button** on the EV3 brick to stop cleanly.

---

## Tuning Guide

All key parameters live at the top of `line_follower.py`.

### Robot zig-zags or oscillates
- **Decrease `KP`** (e.g. 0.8 → 0.5) to reduce the sharpness of each correction.
- **Increase `KD`** (e.g. 1.2 → 1.8) to add more damping.
- **Decrease `CORRECTION_SCALE`** to limit the maximum steering authority.

### Robot is too slow
- **Increase `BASE_SPEED`** (e.g. 40 → 55).
- If it starts to oscillate at higher speed, increase `KD` proportionally.

### Robot loses the line too often
- **Decrease `BASE_SPEED`** so the robot reacts before flying off the line.
- **Increase `SIDE_OVERRIDE`** so side-sensor corrections are more aggressive.
- **Decrease `RECOVERY_SPEED`** so the search pivot is more controlled.
- Check the `BLACK` and `WHITE` calibration constants — poor calibration is
  the most common cause of unreliable detection.
  Run the robot over both surfaces and print `sensor.reflected_light_intensity`
  to find the real min/max values, then update `BLACK` and `WHITE` accordingly.
