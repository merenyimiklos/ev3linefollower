# ev3linefollower

Competition-ready 3-sensor PD line follower for **LEGO Mindstorms EV3** running the **EV3RT (TOPPERS)** real-time operating system.

## Hardware Setup

| Device | Port |
|---|---|
| Left color sensor | Port 1 |
| Middle color sensor | Port 2 |
| Right color sensor | Port 3 |
| Left large motor | Port C |
| Right large motor | Port B |

The three sensors are arranged in a triangular formation: the middle sensor leads in front, while the left and right sensors sit slightly behind.

## Control Strategy

The algorithm combines a **PD (Proportional-Derivative) controller** on the middle sensor with side-sensor assistance:

1. **Primary PD control** – The middle sensor reading is compared against a calibrated threshold (midpoint of black/white values). The error feeds a proportional + derivative controller to produce a base steering correction.
2. **Side-sensor fine-tuning** – The differential between the left and right sensor readings adds a secondary correction term, improving responsiveness to curves and intersections.
3. **Line-lost recovery** – When all three sensors read white (line completely lost), the robot steers aggressively in the last known direction until the line is reacquired.

Motor output is:

```
left_motor  = base_speed + correction
right_motor = base_speed - correction
```

Both values are clamped to [-100, 100].

## Building

Place this directory inside the EV3RT SDK workspace, then build with:

```sh
make app=ev3linefollower
```

Transfer the resulting binary to the EV3 brick via USB or Bluetooth.

## Tuning Guide

### Calibration

Before tuning control parameters, measure the actual reflected-light values on your surface:

1. Place each sensor over the **black line** and note the reading → set `BLACK_VALUE`.
2. Place each sensor over the **white surface** and note the reading → set `WHITE_VALUE`.
3. `THRESHOLD` is computed automatically as the midpoint.

### Adjusting KP (Proportional Gain)

| Change | Effect |
|---|---|
| Increase KP | Faster response to deviations, but may cause oscillation |
| Decrease KP | Smoother but slower corrections; robot may drift off the line |

Start with `KP = 0.5` and increase by 0.1 until the robot follows the line without drifting. If it oscillates (wiggles side to side), reduce KP or increase KD.

### Adjusting KD (Derivative Gain)

| Change | Effect |
|---|---|
| Increase KD | Reduces oscillation by damping rapid changes |
| Decrease KD | Less damping; faster but potentially unstable |

Start with `KD = 0.2` and increase by 0.05. A good KD makes the robot approach the line smoothly without overshooting.

### Increasing Speed

1. Increase `BASE_SPEED` in small steps (e.g. 50 → 55 → 60).
2. After each increase, re-tune KP and KD to maintain stability.
3. If the robot loses the line on tight curves, increase `RECOVERY_GAIN` or `SIDE_KP`.

### Reducing Oscillation

- **Increase KD** – the most effective way to damp oscillation.
- **Decrease KP** – reduce the aggressiveness of corrections.
- **Increase `LOOP_DELAY_MS`** – slowing the loop can smooth behavior at the cost of responsiveness.

### Improving Recovery

- **Increase `RECOVERY_GAIN`** – makes the robot turn more sharply when the line is lost.
- **Ensure `last_direction` is accurate** – verify that calibration constants are correct so that the line-lost condition triggers reliably.