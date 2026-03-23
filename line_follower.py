#!/usr/bin/env python3
"""
EV3 Competition-Style 3-Sensor Line Follower
=============================================
Control Strategy:
  - The middle sensor (port 2) is the primary tracking sensor.
    Its normalised error (0 = on line, positive = drifted right,
    negative = drifted left) feeds a PD controller that steers
    the robot smoothly along the line edge.

  - The left (port 1) and right (port 3) sensors act as guards:
      * If only the left sensor sees black  → sharp right recovery.
      * If only the right sensor sees black → sharp left recovery.
      * If both side sensors see black      → robot is centred on a
        wide mark; continue straight.
      * If all three sensors see white      → line is lost; spin
        toward the direction of the last known error until a sensor
        detects black again.

  - A PD controller (proportional + derivative) on the middle
    sensor's error provides smooth, oscillation-free steering under
    normal conditions, while the side-sensor overrides give fast
    recovery on tight curves and cross-junctions.

Hardware:
  - Left motor  : port C
  - Right motor : port B
  - Left sensor : port 1  (Color sensor – Reflected Light Intensity)
  - Mid sensor  : port 2  (Color sensor – Reflected Light Intensity)
  - Right sensor: port 3  (Color sensor – Reflected Light Intensity)
"""

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.button import Button

# ─────────────────────────────────────────────────────────────────────────────
# CALIBRATION CONSTANTS
# Measure these on your actual surface and line before a run.
# Black: lowest reflected-light reading (dark line).
# White: highest reflected-light reading (bright mat).
# ─────────────────────────────────────────────────────────────────────────────
BLACK = 5      # Reflected light intensity for black (0–100)
WHITE = 90     # Reflected light intensity for white (0–100)

# Threshold below which a sensor is considered "on the black line"
# Placed at the midpoint between black and white.
THRESHOLD = (BLACK + WHITE) // 2   # ≈ 47

# ─────────────────────────────────────────────────────────────────────────────
# TUNING PARAMETERS
# Adjust these to match your robot's mechanical behaviour.
# ─────────────────────────────────────────────────────────────────────────────
BASE_SPEED       = 40   # Normal forward speed (% of motor max).  ↑ = faster.
KP               = 0.8  # Proportional gain.  ↑ = stronger mid-curve correction.
KD               = 1.2  # Derivative gain.    ↑ = dampens oscillation.
CORRECTION_SCALE = 30   # Max correction applied by the PD term (% speed units).
RECOVERY_SPEED   = 25   # Speed used when searching for a lost line.  ↓ = safer.
SIDE_OVERRIDE    = 35   # Fixed correction added when a side sensor fires.

# ─────────────────────────────────────────────────────────────────────────────
# HARDWARE INITIALISATION
# ─────────────────────────────────────────────────────────────────────────────
motor_left  = LargeMotor(OUTPUT_C)
motor_right = LargeMotor(OUTPUT_B)

sensor_left  = ColorSensor(INPUT_1)
sensor_mid   = ColorSensor(INPUT_2)
sensor_right = ColorSensor(INPUT_3)

# Switch all sensors to reflected-light-intensity mode
for sensor in (sensor_left, sensor_mid, sensor_right):
    sensor.mode = ColorSensor.MODE_COL_REFLECT

btn = Button()   # Used to stop the program cleanly with the EV3 center button


# ─────────────────────────────────────────────────────────────────────────────
# HELPER FUNCTIONS
# ─────────────────────────────────────────────────────────────────────────────

def read_sensors():
    """Return (left, mid, right) reflected-light values as integers 0–100."""
    return (
        sensor_left.reflected_light_intensity,
        sensor_mid.reflected_light_intensity,
        sensor_right.reflected_light_intensity,
    )


def is_black(value):
    """Return True when a sensor reads below the black/white threshold."""
    return value < THRESHOLD


def normalise(value):
    """
    Map a raw sensor reading to a –1 … +1 error signal.
      –1 = fully on black (line)
      +1 = fully on white (off line)
    The middle sensor tracks the *edge* of the line, so its target
    is THRESHOLD (≈0 on this scale).
    """
    return (value - THRESHOLD) / (WHITE - BLACK) * 2.0


def set_motors(left_pct, right_pct):
    """Drive both motors at the given speed percentages (clamped ±100)."""
    left_pct  = max(-100, min(100, left_pct))
    right_pct = max(-100, min(100, right_pct))
    motor_left.run_forever(speed_sp=SpeedPercent(left_pct))
    motor_right.run_forever(speed_sp=SpeedPercent(right_pct))


def stop_motors():
    """Brake both motors."""
    motor_left.stop(stop_action='brake')
    motor_right.stop(stop_action='brake')


# ─────────────────────────────────────────────────────────────────────────────
# MAIN CONTROL LOOP
# ─────────────────────────────────────────────────────────────────────────────

def main():
    last_error = 0.0        # Previous PD error (for derivative term)
    last_direction = 1      # +1 = last drift was right, –1 = left
                            # Used when all sensors see white (line lost)

    print("EV3 Line Follower started. Press center button to stop.")

    while not btn.any():
        left_val, mid_val, right_val = read_sensors()

        on_left  = is_black(left_val)
        on_mid   = is_black(mid_val)
        on_right = is_black(right_val)

        # ── Case 1: Line completely lost (all sensors see white) ────────────
        if not on_left and not on_mid and not on_right:
            # Spin slowly toward the side where the line was last seen.
            if last_direction > 0:
                # Line was to the right → pivot right
                set_motors(RECOVERY_SPEED, -RECOVERY_SPEED)
            else:
                # Line was to the left → pivot left
                set_motors(-RECOVERY_SPEED, RECOVERY_SPEED)
            # Keep last_error unchanged so the derivative is not corrupted.
            continue

        # ── Case 2: Side-sensor override ────────────────────────────────────
        # A side sensor sees the line but the middle sensor does not.
        # Apply a sharp fixed correction to bring the robot back on track.
        if on_left and not on_mid and not on_right:
            # Robot drifted too far right; left sensor found the line.
            last_direction = -1
            set_motors(BASE_SPEED - SIDE_OVERRIDE, BASE_SPEED + SIDE_OVERRIDE)
            last_error = -1.0   # Reset derivative reference
            continue

        if on_right and not on_mid and not on_left:
            # Robot drifted too far left; right sensor found the line.
            last_direction = 1
            set_motors(BASE_SPEED + SIDE_OVERRIDE, BASE_SPEED - SIDE_OVERRIDE)
            last_error = 1.0    # Reset derivative reference
            continue

        # ── Case 3: Normal PD control (mid sensor active) ───────────────────
        # Compute normalised error from the middle sensor.
        # Positive error → robot is drifting left (mid sensor moving toward white on right side).
        # Negative error → robot is drifting right.
        error = normalise(mid_val)

        # Update last known drift direction
        if error > 0:
            last_direction = 1
        elif error < 0:
            last_direction = -1

        # Proportional term
        p_term = KP * error

        # Derivative term (rate of change of error)
        d_term = KD * (error - last_error)
        last_error = error

        # Combined correction (scaled to speed units)
        correction = (p_term + d_term) * CORRECTION_SCALE

        # Side sensors damp the correction to prevent overshoot.
        # If the same-side sensor also sees black, the robot is deep
        # on the line; reduce aggression slightly.
        if on_left and error < 0:
            correction *= 0.6   # Already going left – soften the push
        if on_right and error > 0:
            correction *= 0.6   # Already going right – soften the push

        # Apply correction: positive correction → turn right
        #   left motor speeds up, right motor slows down.
        left_speed  = BASE_SPEED + correction
        right_speed = BASE_SPEED - correction

        set_motors(left_speed, right_speed)

    # ── Clean shutdown ───────────────────────────────────────────────────────
    stop_motors()
    print("Stopped.")


if __name__ == '__main__':
    main()
