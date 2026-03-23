/**
 * @file app.c
 * @brief Competition-ready 3-sensor PD line follower for EV3RT (TOPPERS).
 *
 * Common EV3RT Compile Errors (and how they are avoided here)
 * ===========================================================
 *  1. Wrong headers -- e.g. including "ev3.h" or "pybricks.h" instead of
 *     "ev3api.h".  We include only "app.h" which pulls in "ev3api.h".
 *  2. Wrong API names -- e.g. motor_start(), set_sensor_mode(), nxt_motor().
 *     We use ev3_motor_set_power(), ev3_color_sensor_get_reflect(), etc.
 *  3. Wrong type constants -- e.g. EV3_LARGE_MOTOR, NXT_SENSOR.  Correct
 *     EV3RT constants are LARGE_MOTOR and COLOR_SENSOR.
 *  4. Wrong port constants -- e.g. PORT_1 or INPUT_1.  Correct EV3RT
 *     constants are EV3_PORT_1 .. EV3_PORT_4 (sensors) and
 *     EV3_PORT_A .. EV3_PORT_D (motors).
 *  5. Using bool without <stdbool.h> -- ev3api.h does not guarantee bool.
 *     We use plain int for boolean-like flags to be safe.
 *  6. C99 mixed declarations -- declaring variables after statements inside
 *     a block.  Some cross-compiler configurations default to C89/C90.
 *     We declare all local variables at the top of each block.
 *  7. Missing main_task prototype -- the TOPPERS configurator (app.cfg)
 *     references main_task; it must be declared extern in app.h.
 *  8. Forgetting ev3_sensor_config / ev3_motor_config -- sensors and motors
 *     must be configured before first use or reads return garbage / motors
 *     do not respond.
 *  9. Wrong tslp_tsk units -- EV3RT (TOPPERS/HRP3) measures RELTIM in
 *     microseconds, not milliseconds.  tslp_tsk(1000) = 1 ms.
 * 10. Floating-point on ARM -- compiles fine with the EV3RT GCC toolchain
 *     but double-precision may be slow.  We keep expressions simple.
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
 *
 * Compilation Checklist
 * =====================
 * - Required headers:
 *     app.h  (must #include "ev3api.h")
 * - Motor configuration:
 *     ev3_motor_config(port, LARGE_MOTOR) for EV3_PORT_B and EV3_PORT_C
 * - Sensor configuration:
 *     ev3_sensor_config(port, COLOR_SENSOR) for EV3_PORT_1/2/3
 * - app.cfg must contain:
 *     INCLUDE("app_common.cfg");
 *     CRE_TSK for main_task
 * - app.h must:
 *     #include "ev3api.h"
 *     declare: extern void main_task(intptr_t exinf);
 * - Makefile.inc must list:
 *     APPL_COBJS += app
 */

#include "app.h"

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

#define BLACK         10   /**< Reflected-light reading on the black line  */
#define WHITE         70   /**< Reflected-light reading on the white surface */
#define THRESHOLD     ((BLACK + WHITE) / 2) /**< Line/no-line boundary */

/* ===================================================================
 * PD control tuning constants
 * =================================================================== */

#define KP            0.65  /**< Proportional gain                          */
#define KD            0.30  /**< Derivative gain (damps oscillation)        */

/* ===================================================================
 * Speed and correction parameters
 * =================================================================== */

#define BASE_SPEED      50    /**< Forward speed when line is centered      */
#define SIDE_KP         0.35  /**< Proportional gain for side-sensor assist */
#define RECOVERY_SPEED  60    /**< Aggressive correction when line is lost  */

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
 * ev3_color_sensor_get_reflect() returns uint8_t (0..100).
 * Storing in int is safe and avoids sign-conversion warnings later.
 *
 * @param[out] sv  Pointer to a sensor_values_t that receives the readings.
 */
static void read_sensors(sensor_values_t *sv)
{
    sv->left   = (int)ev3_color_sensor_get_reflect(LEFT_SENSOR);
    sv->middle = (int)ev3_color_sensor_get_reflect(MIDDLE_SENSOR);
    sv->right  = (int)ev3_color_sensor_get_reflect(RIGHT_SENSOR);
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
    /* All declarations at the top of the block for C89 safety. */
    int error;
    double pd;
    int side_diff;
    int line_lost;  /* 1 = line completely lost, 0 = at least one sensor sees it */
    int dir;
    int correction;

    /* --- Primary PD control on the middle sensor ---------------------- */
    error = sv->middle - THRESHOLD;  /* negative = on black / too far left
                                        positive = on white / too far right */

    /* Proportional + derivative terms */
    pd = KP * error + KD * (error - prev_error);

    /* --- Side-sensor differential for fine-tuning --------------------- */
    side_diff = sv->left - sv->right;  /* positive = right sensor darker
                                          (line to the right) */

    /* --- Recovery: all sensors on white (line completely lost) --------- */
    line_lost = (sv->left   > THRESHOLD) &&
                (sv->middle > THRESHOLD) &&
                (sv->right  > THRESHOLD);

    if (line_lost) {
        /*
         * Steer aggressively in the last known direction.
         * last_direction carries the sign: negative means the line was
         * last seen to the left, positive to the right.
         */
        dir = (last_direction >= 0) ? 1 : -1;
        correction = dir * RECOVERY_SPEED;
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
 * @param correction  Steering correction (positive = turn right).
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
 *
 * Must be called before any sensor read or motor write.
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
 * Main control task -- entry point called by the EV3RT kernel.
 *
 * Runs the PD line-following control loop indefinitely.
 *
 * @param exinf  Extra information (unused).
 */
void main_task(intptr_t exinf)
{
    /* All declarations at the top of the block for C89 safety. */
    int prev_error;
    int last_direction;
    sensor_values_t sv;

    (void)exinf;  /* Suppress unused-parameter warning */

    /* ---- Initialize hardware ---- */
    init_devices();

    /*
     * Allow sensors to settle after configuration.
     * tslp_tsk() takes microseconds in EV3RT (TOPPERS/HRP3).
     * 500 ms = 500 * 1000 us = 500000 us.
     */
    tslp_tsk(500U * 1000U);

    /* ---- Control-loop state variables ---- */
    prev_error     = 0;   /* Previous cycle error (for derivative)       */
    last_direction = 0;   /* Last known line direction for recovery      */

    /* ---- Main control loop ---- */
    while (1) {
        int error;
        int correction;

        /* 1. Read all sensors */
        read_sensors(&sv);

        /* 2. Compute correction */
        correction = compute_correction(&sv, prev_error,
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

        /* 5. Wait to maintain a consistent loop period.
         *    LOOP_DELAY_MS * 1000U converts milliseconds to microseconds. */
        tslp_tsk(LOOP_DELAY_MS * 1000U);
    }
}
