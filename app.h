/**
 * @file app.h
 * @brief EV3RT application header for 3-sensor line follower.
 *
 * Declares the main task entry point required by the EV3RT (TOPPERS) kernel.
 */

#ifndef APP_H
#define APP_H

#include "ev3api.h"

/**
 * Main task entry point.
 * Called by the EV3RT kernel after initialization.
 *
 * @param exinf Extra information (unused).
 */
extern void main_task(intptr_t exinf);

#endif /* APP_H */
