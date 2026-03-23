/**
 * @file app.c
 * @brief Competition-ready 3-sensor PD line follower for EV3RT (TOPPERS).
 *
 * Control Strategy
 * ================
 * The robot uses three color sensors arranged in a triangular formation
 * (middle sensor in front, left and right sensors slightly behind) to follow
 * a black line on a white surface.
 *
 * - The middle sensor provides the primary PD control signal.  Its reflected-
 *   light value is compared against a threshold (midpoint between calibrated
 *   black and white values) to compute a proportional error.  A derivative
 *   term damps oscillation.
 *
 * - The left and right sensors supply supplementary correction.  When the
 *   middle sensor is near the threshold (line centered), the side sensors
 *   fine-tune steering.  When the middle sensor loses the line entirely,
 *   the side sensors determine which direction the line has gone so the
 *   robot can steer back.
 *
 * - If all three sensors read white (line completely lost), the robot falls
 *   back to the last known error direction and steers aggressively until
 *   the line is reacquired.
 *
 * Motor output is computed as:
 *     left_motor  = base_speed + correction
 *     right_motor = base_speed - correction
 * and clamped to the valid EV3 power range [-100, 100].
 */

#include "app.h"

#include <stdlib.h>  /* abs() */

/* ===================================================================
 * Hardware port assignments
 * =================================================================== */

/** @name Sensor ports */
/** @{ */
#define LEFT_SENSOR   EV3_PORT_1   /**< Left color sensor   */
#define MIDDLE_SENSOR EV3_PORT_2   /**< Middle color sensor  */
#define RIGHT_SENSOR  EV3_PORT_3   /**< Right color sensor   */
/** @} */

/** @name Motor ports */
/** @{ */
#define LEFT_MOTOR    EV3_PORT_C   /**< Left large motor     */
#define RIGHT_MOTOR   EV3_PORT_B   /**< Right large motor    */
/** @} */

/* ===================================================================
 * Calibration constants
 * Measure on your surface and adjust these values accordingly.
 * =================================================================== */

#define BLACK_VALUE   10   /**< Reflected-light reading on the black line  */
#define WHITE_VALUE   70   /**< Reflected-light reading on the white surface */
#define THRESHOLD     ((BLACK_VALUE + WHITE_VALUE) / 2) /**< Line/no-line boundary */

/* ===================================================================
 * PD control tuning constants
 * =================================================================== */

#define KP            0.65  /**< Proportional gain                          */
#define KD            0.30  /**< Derivative gain (damps oscillation)        */

/* ===================================================================
 * Speed and correction parameters
 * =================================================================== */

#define BASE_SPEED    50    /**< Forward speed when line is centered        */
#define SIDE_KP       0.35  /**< Proportional gain for side-sensor assist   */
#define RECOVERY_GAIN 60    /**< Aggressive correction when line is lost    */

/* ===================================================================
 * Timing
 * =================================================================== */

#define LOOP_DELAY_MS 5     /**< Control-loop period in milliseconds        */

/* ===================================================================
 * Motor power limits
 * =================================================================== */

#define MOTOR_MIN    (-100) /**< Minimum motor power */
#define MOTOR_MAX      100  /**< Maximum motor power */

/* ===================================================================
 * Helper: clamp a value to the valid motor power range.
 * =================================================================== */

/**
 * Clamp an integer to the range [MOTOR_MIN, MOTOR_MAX].
 *
 * @param value  Raw motor power value.
 * @return       Clamped value within [-100, 100].
 */
static int clamp(int value)
{
    if (value > MOTOR_MAX) return MOTOR_MAX;
    if (value < MOTOR_MIN) return MOTOR_MIN;
    return value;
}

/* ===================================================================
 * Sensor reading
 * =================================================================== */

/** Holds the three reflected-light readings for one cycle. */
typedef struct {
    int left;    /**< Left sensor reading   */
    int middle;  /**< Middle sensor reading  */
    int right;   /**< Right sensor reading   */
} sensor_values_t;

/**
 * Read all three color sensors in reflected-light mode.
 *
 * @param[out] sv  Pointer to a sensor_values_t that receives the readings.
 */
static void read_sensors(sensor_values_t *sv)
{
    sv->left   = ev3_color_sensor_get_reflect(LEFT_SENSOR);
    sv->middle = ev3_color_sensor_get_reflect(MIDDLE_SENSOR);
    sv->right  = ev3_color_sensor_get_reflect(RIGHT_SENSOR);
}

/* ===================================================================
 * Error computation
 * =================================================================== */

/**
 * Compute the steering correction from sensor readings.
 *
 * The function combines three inputs:
 *   1. PD control on the middle sensor (primary).
 *   2. Side-sensor differential for fine-tuning.
 *   3. Recovery steering when the line is lost.
 *
 * @param sv              Current sensor readings.
 * @param prev_error      Error from the previous cycle (for derivative).
 * @param[out] error_out  Receives the current cycle's error value.
 * @param last_direction  Last known steering direction (negative = line
 *                        was to the left, positive = to the right).
 * @return                Combined correction value to apply to the motors.
 */
static int compute_correction(const sensor_values_t *sv,
                              int  prev_error,
                              int *error_out,
                              int  last_direction)
{
    int correction;

    /* --- Primary PD control on the middle sensor ---------------------- */
    int error = sv->middle - THRESHOLD;  /* negative → on black / too far left
                                            positive → on white / too far right */

    /* Proportional + derivative terms */
    double pd = KP * error + KD * (error - prev_error);

    /* --- Side-sensor differential for fine-tuning --------------------- */
    int side_diff = sv->left - sv->right;  /* positive → right sensor darker
                                              (line to the right) */

    /* --- Recovery: all sensors on white (line completely lost) --------- */
    bool line_lost = (sv->left  > THRESHOLD) &&
                     (sv->middle > THRESHOLD) &&
                     (sv->right > THRESHOLD);

    if (line_lost) {
        /*
         * Steer aggressively in the last known direction.
         * last_direction carries the sign: negative means the line was
         * last seen to the left, positive to the right.
         */
        int dir = (last_direction >= 0) ? 1 : -1;
        correction = dir * RECOVERY_GAIN;
    } else {
        /* Normal operation: PD + side-sensor assist */
        correction = (int)(pd + SIDE_KP * side_diff);
    }

    *error_out = error;
    return correction;
}

/* ===================================================================
 * Motor output
 * =================================================================== */

/**
 * Apply a correction value to both drive motors.
 *
 * left_motor  = base_speed + correction
 * right_motor = base_speed - correction
 *
 * Both values are clamped to [-100, 100].
 *
 * @param correction  Steering correction (positive → turn right).
 */
static void apply_motors(int correction)
{
    int left_power  = clamp(BASE_SPEED + correction);
    int right_power = clamp(BASE_SPEED - correction);

    ev3_motor_set_power(LEFT_MOTOR,  left_power);
    ev3_motor_set_power(RIGHT_MOTOR, right_power);
}

/* ===================================================================
 * Sensor / motor initialization
 * =================================================================== */

/**
 * Configure all sensors and motors used by the application.
 */
static void init_devices(void)
{
    /* --- Color sensors (reflected-light mode) --- */
    ev3_sensor_config(LEFT_SENSOR,   COLOR_SENSOR);
    ev3_sensor_config(MIDDLE_SENSOR, COLOR_SENSOR);
    ev3_sensor_config(RIGHT_SENSOR,  COLOR_SENSOR);

    /* --- Large motors --- */
    ev3_motor_config(LEFT_MOTOR,  LARGE_MOTOR);
    ev3_motor_config(RIGHT_MOTOR, LARGE_MOTOR);
}

/* ===================================================================
 * Main task
 * =================================================================== */

/**
 * Main control task – entry point called by the EV3RT kernel.
 *
 * Runs the PD line-following control loop indefinitely.
 *
 * @param exinf  Extra information (unused).
 */
void main_task(intptr_t exinf)
{
    (void)exinf;  /* Suppress unused-parameter warning */

    /* ---- Initialize hardware ---- */
    init_devices();

    /* Allow sensors to settle after configuration */
    tslp_tsk(500U * 1000U);  /* 500 ms in microseconds */

    /* ---- Control-loop state variables ---- */
    int prev_error     = 0;   /* Previous cycle error (for derivative)       */
    int last_direction = 0;   /* Last known line direction for recovery      */

    sensor_values_t sv;

    /* ---- Main control loop ---- */
    while (1) {
        /* 1. Read all sensors */
        read_sensors(&sv);

        /* 2. Compute correction */
        int error;
        int correction = compute_correction(&sv, prev_error,
                                            &error, last_direction);

        /* 3. Update state for next cycle */
        prev_error = error;

        /*
         * Track the direction the line was last seen.
         * A non-zero error from the middle sensor, or a clear side-sensor
         * signal, updates last_direction.
         */
        if (error != 0) {
            last_direction = error;
        } else if (sv.left < THRESHOLD) {
            last_direction = -1;  /* Line is to the left */
        } else if (sv.right < THRESHOLD) {
            last_direction = 1;   /* Line is to the right */
        }

        /* 4. Drive motors */
        apply_motors(correction);

        /* 5. Wait to maintain a consistent loop period */
        tslp_tsk(LOOP_DELAY_MS * 1000U);  /* Convert ms → µs */
    }
}
