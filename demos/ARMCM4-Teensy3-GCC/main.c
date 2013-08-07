/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

/*
 * Working area for the LED blinker thread.
 */
static WORKING_AREA(waThread1, 64);

/*
 * LED blinker thread.
 */
__attribute__((noreturn))
static void Thread1(void *arg) {
    (void)arg;

    chRegSetThreadName("LEDBlinker");
    while (TRUE) {
        palTogglePad(IOPORT3, 5);
        chThdSleepMilliseconds(500);
    }
}

/*
 * Application entry point.
 */
int main(void) {
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    /*
     * Creates the blinker thread.
     */
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO,
                      (tfunc_t)Thread1, NULL);

    /*
     * Halt if the thread dies for any reason.
     */
    chSysHalt();

    return 0;
}
