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
 * @file    MK20D5/hal_lld.h
 * @brief   MK20D5 HAL subsystem low level driver header template.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - MK20D5 (optional)
 *          - MK20D5_SYSCLK
 *          .
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _HAL_LLD_H_
#define _HAL_LLD_H_

#include "MK20D5.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Defines the support for realtime counters in the HAL.
 */
#define HAL_IMPLEMENTS_COUNTERS TRUE

/**
 * @name    Platform identification
 * @{
 */
#define PLATFORM_NAME           "MK20D5 Kinetis K20 50 MHz"
/** @} */

/**
 * @name    Absolute maximum ratings
 * @{
 */
/**
 * @brief   Maximum core clock frequency.
 */
#define MK20D5_CORECLK_MAX      50000000

/**
 * @brief   Minimum core clock frequency.
 */
#define MK20D5_CORECLK_MIN      4000000

/**
 * @brief   Maximum system clock frequency.
 * @note    Always the same as the core clock maximum.
 */
#define MK20D5_SYSCLK_MAX       MK20D5_CORECLK_MAX

/**
 * @brief   Minimum system clock frequency.
 * @note    Always the same as the core clock minimum.
 */
#define MK20D5_SYSCLK_MIN       MK20D5_CORECLK_MIN

/**
 * @brief   Maximum bus clock frequency.
 */
#define MK20D5_BUSCLK_MAX       50000000

/**
 * @brief   Minimum bus clock frequency.
 */
#define MK20D5_BUSCLK_MIN       4000000

/**
 * @brief   Maximum flash clock frequency.
 */
#define MK20D5_FLASHCLK_MAX     25000000

/**
 * @brief   Minimum flash clock frequency.
 */
#define MK20D5_FLASHCLK_MIN     1000000
/** @} */

/*===========================================================================*/
/* Platform specific friendly IRQ names.                                     */
/*===========================================================================*/

/**
 * @name    IRQ vector names
 * @{
 */
#define DMA_CH0_IRQHandler      Vector40	/**< DMA channel 0               */
#define DMA_CH1_IRQHandler      Vector44    /**< DMA channel 1               */
#define DMA_CH2_IRQHandler      Vector48    /**< DMA channel 2               */
#define DMA_CH3_IRQHandler      Vector4C    /**< DMA channel 3               */
#define DMA_ERROR_IRQHandler    Vector50    /**< DMA error                   */
#define DMA_IRQHandler          Vector54    /**< DMA event                   */
#define FLASH_CMD_IRQHandler    Vector58    /**< Flash command               */
#define FLASH_ERROR_IRQHandler  Vector5C    /**< Flash error                 */
#define LOW_VOLTAGE_IRQHandler  Vector60    /**< Low voltage                 */
#define LLWU_IRQHandler         Vector64    /**< Low leakage wakeup          */
#define WDOG_IRQHandler         Vector68    /**< Watchdog event              */
#define I2C0_IRQHandler         Vector6C    /**< I2C0 event                  */
#define SPI0_IRQHandler         Vector70    /**< SPI0 event                  */
#define I2S0_TX_IRQHandler      Vector74    /**< I2S0 transmit               */
#define I2S1_RX_IRQHandler      Vector78    /**< I2S1 receive                */
#define UART0_LON_IRQHandler    Vector7C    /**< UART0 CEA709.1-B (LON)
                                                 status                      */
#define UART0_STATUS_IRQHandler Vector80    /**< UART0 status                */
#define UART0_ERROR_IRQHandler  Vector84    /**< UART0 error                 */
#define UART1_STATUS_IRQHandler Vector88    /**< UART1 status                */
#define UART1_ERROR_IRQHandler  Vector8C    /**< UART1 error                 */
#define UART2_STATUS_IRQHandler Vector90    /**< UART2 status                */
#define UART2_ERROR_IRQHandler  Vector94    /**< UART2 error                 */
#define ADC0_IRQHandler         Vector98    /**< ADC0 event                  */
#define CMP0_IRQHandler         Vector9C    /**< CMP0 event                  */
#define CMP1_IRQHandler         VectorA0    /**< CMP1 event                  */
#define FTM0_IRQHandler         VectorA4    /**< FTM0 event                  */
#define FTM1_IRQHandler         VectorA8    /**< FTM1 event                  */
#define CMT_IRQHandler          VectorAC    /**< CMT event                   */
#define RTC_ALARM_IRQHandler    VectorB0    /**< RTC alarm                   */
#define RTC_SECONDS_IRQHandler  VectorB4    /**< RTC seconds                 */
#define PIT_CH0_IRQHandler      VectorB8    /**< PIT channel 0               */
#define PIT_CH1_IRQHandler      VectorBC    /**< PIT channel 1               */
#define PIT_CH2_IRQHandler      VectorC0    /**< PIT channel 2               */
#define PIT_CH3_IRQHandler      VectorC4	/**< PIT channel 3               */
#define PDB_IRQHandler          VectorC8    /**< PDB event                   */
#define USB_OTG_IRQHandler      VectorCC    /**< USB OTG event               */
#define USB_CHARGE_IRQHandler   VectorD0    /**< USB charger detected        */
#define TSI_IRQHandler          VectorD4    /**< TSI event                   */
#define MCG_IRQHandler          VectorD8    /**< MCG event                   */
#define LPTMR_IRQHandler        VectorDC    /**< LPTMR event                 */
#define PORTA_IRQHandler        VectorE0    /**< Pin detect (Port A)         */
#define PORTB_IRQHandler        VectorE4    /**< Pin detect (Port B)         */
#define PORTC_IRQHandler        VectorE8    /**< Pin detect (Port C)         */
#define PORTD_IRQHandler        VectorEC    /**< Pin detect (Port D)         */
#define PORTE_IRQHandler        VectorF0    /**< Pin detect (Port E)         */
#define SOFTWARE_IRQHandler     VectorF4    /**< Software interrupt          */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(MK20D5_MCUCONF)
#error "Using a wrong mcuconf.h file, MK20D5_MCUCONF not defined"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type representing a system clock frequency.
 */
typedef uint32_t halclock_t;

/**
 * @brief   Type of the realtime free counter value.
 */
typedef uint32_t halrtcnt_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Returns the current value of the system free running counter.
 * @note    This service is implemented by returning the content of the
 *          DWT_CYCCNT register.
 *
 * @return              The value of the system free running counter of
 *                      type halrtcnt_t.
 *
 * @notapi
 */
#define hal_lld_get_counter_value()         DWT_CYCCNT

/**
 * @brief   Realtime counter frequency.
 * @note    The DWT_CYCCNT register is incremented directly by the system
 *          clock so this function returns STM32_HCLK.
 *
 * @return              The realtime counter frequency of type halclock_t.
 *
 * @notapi
 */
#define hal_lld_get_counter_frequency()     0

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void mk20d5_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* _HAL_LLD_H_ */

/** @} */
