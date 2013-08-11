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

/**
 * @file    templates/hal_lld.c
 * @brief   HAL Driver subsystem low level driver source template.
 *
 * @addtogroup HAL
 * @{
 */

#include "ch.h"
#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 * @todo    Use a macro to define the system clock frequency.
 *
 * @notapi
 */
void hal_lld_init(void) {

    /* SysTick initialization using the system clock. */
    nvicSetSystemHandlerPriority(HANDLER_SYSTICK, CORTEX_PRIORITY_SYSTICK);
    SysTick->LOAD = 48000000 / CH_FREQUENCY - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_ENABLE_Msk |
                    SysTick_CTRL_TICKINT_Msk;
}

/**
 * @brief   MK20D5 clock initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function is meant to be invoked early during the system
 *          initialization, it is usually invoked from the file
 *          @p board.c.
 * @todo    This function needs to be more generic.
 *
 * @special
 */
void mk20d5_clock_init(void) {

    /* Unlock the watchdog and set to allow updates */
    WDOG->UNLOCK = 0xC520;
    WDOG->UNLOCK = 0xD928;
    WDOG->STCTRLH = WDOG_STCTRLH_ALLOWUPDATE_MASK;

    /* Enable clocks for always used peripherals */
    SIM->SCGC6 = SIM_SCGC6_FTFL_MASK |
                 SIM_SCGC6_DMAMUX_MASK |
                 SIM_SCGC6_SPI0_MASK |
                 SIM_SCGC6_I2S_MASK |
                 SIM_SCGC6_CRC_MASK |
                 SIM_SCGC6_USBDCD_MASK |
                 SIM_SCGC6_PDB_MASK |
                 SIM_SCGC6_PIT_MASK |
                 SIM_SCGC6_FTM0_MASK |
                 SIM_SCGC6_FTM1_MASK |
                 SIM_SCGC6_ADC0_MASK |
                 SIM_SCGC6_RTC_MASK;

    /* If the RTC oscillator is not enabled, get it started now */
    if (!(RTC->CR & RTC_CR_OSCE_MASK)) {
        RTC->SR = 0;
        RTC->CR = RTC_CR_SC16P_MASK | RTC_CR_SC4P_MASK | RTC_CR_OSCE_MASK;
    }

    /* Release I/O pins hold, if just woke up from VLLS mode */
    if (PMC->REGSC & PMC_REGSC_ACKISO_MASK) PMC->REGSC |= PMC_REGSC_ACKISO_MASK;

    /*
     * Start in FEI mode
     */

    /* Enable capacitors for crystal */
    OSC->CR = OSC_CR_SC8P_MASK | OSC_CR_SC2P_MASK;

    /* Enable OSC, 8-32 MHz range, low power mode */
    MCG->C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS0_MASK;

    /* Switch to crystal as clock source, FLL input of 16 MHz / 512 */
    MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);

    /* Wait for crystal oscillator to begin */
    while ((MCG->S & MCG_S_OSCINIT0_MASK) == 0);

    /* Wait for the FLL to use the oscillator */
    while ((MCG->S & MCG_S_IREFST_MASK) != 0);

    /* Wait for the MCGOUTCLK to use the oscillator */
    while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));

    /*
     * Now in FBE mode
     */

    /* Config PLL input for 4 MHz (16 MHz crystal / 4) */
    MCG->C5 = MCG_C5_PRDIV0(3);

    /* Config PLL for 96 MHz output */
    MCG->C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0);

    /* Wait for PLL to start using crystal as its input */
    while (!(MCG->S & MCG_S_PLLST_MASK));

    /* Wait for PLL to lock */
    while (!(MCG->S & MCG_S_LOCK0_MASK));

    /*
     * Now in PBE mode
     */

    /* Clock config divisors: 48 MHz core, 48 MHz bus, 24 MHz flash */
    SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) |
                   SIM_CLKDIV1_OUTDIV2(1) |
                   SIM_CLKDIV1_OUTDIV4(3);

    /* Switch to PLL as clock source, FLL input of 16 MHz / 512 */
    MCG->C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(4);

    /* Wait for PLL clock to be used */
    while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3));

    /*
     * Now in PEE mode
     */

    /* Configure USB for a 48 MHz clock (96 MHz PLL / 2) */
    SIM->CLKDIV2 = SIM_CLKDIV2_USBDIV(1);

    /* USB uses PLL clock, trace is CPU clock, CLKOUT = OSCERCLK0 */
    SIM->SOPT2 = SIM_SOPT2_USBSRC_MASK |
                 SIM_SOPT2_PLLFLLSEL_MASK |
                 SIM_SOPT2_TRACECLKSEL_MASK |
                 SIM_SOPT2_CLKOUTSEL(6);
}

/** @} */
