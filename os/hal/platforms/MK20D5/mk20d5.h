/******************************************************************************
 * @file    mk20d5.h
 * @author  Jacob McGladdery
 * @version 1.0.0
 * @date    5-August-2013
 * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer Header File.
 *          This file contains all the peripheral register's definitions, bits
 *          definitions and memory mapping for MK20D5 devices.
 *
 ******************************************************************************
 * @attention
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *****************************************************************************/

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup mk20d5
  * @{
  */

#ifndef _MK20D5_H_
#define _MK20D5_H_

#ifdef __cplusplus
  extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV                 0x0001    /**< Core revision r0p1                           */
#define __MPU_PRESENT             0         /**< MK20D5 does not provide an MPU               */
#define __NVIC_PRIO_BITS          4         /**< MK20D5 uses 4 Bits for the Priority Levels   */
#define __Vendor_SysTickConfig    0         /**< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             0         /**< FPU not present                              */

/**
 * @brief MK20D5 Interrupt Number Definition
 */
typedef enum IRQn
{
    /**** Cortex-M4 Processor Exceptions Numbers ****************************************/
    Reset_IRQn               = -15,    /**< 1  Reset Interrupt                          */
    NonMaskableInt_IRQn      = -14,    /**< 2  Non Maskable Interrupt                   */
    Hardfault_IRQn           = -13,    /**< 3  Hardfault Interrupt                      */
    MemoryManagement_IRQn    = -12,    /**< 4  Memory Management Interrupt              */
    SVCall_IRQn              = -5,     /**< 11 Supervisor Call Interrupt                */
    DebugMonitor_IRQn        = -4,     /**< 12 Debug Monitor Interrupt                  */
    PendSV_IRQn              = -2,     /**< 14 Pendable Service Interrupt               */
    SysTick_IRQn             = -1,     /**< 15 System Tick Interrupt                    */

    /**** Kinetis K20 specific Interrupt Numbers ****************************************/
    DMA0_IRQn                = 0,      /**< DMA Channel 0 Transfer Complete Interrupt   */
    DMA1_IRQn                = 1,      /**< DMA Channel 1 Transfer Complete Interrupt   */
    DMA2_IRQn                = 2,      /**< DMA Channel 2 Transfer Complete Interrupt   */
    DMA3_IRQn                = 3,      /**< DMA Channel 3 Transfer Complete Interrupt   */
    DMA_Error_IRQn           = 4,      /**< DMA Error Interrupt                         */
    Reserved0_IRQn           = 5,      /**< Reserved Interrupt 0                        */
    FTFL_IRQn                = 6,      /**< FTFL Interrupt                              */
    Read_Collision_IRQn      = 7,      /**< Read collision Interrupt                    */
    LVD_LVW_IRQn             = 8,      /**< Low Voltage Detect, Low Voltage Warning     */
    LLW_IRQn                 = 9,      /**< Low Leakage Wakeup                          */
    Watchdog_IRQn            = 10,     /**< WDOG Interrupt                              */
    I2C0_IRQn                = 11,     /**< I2C0 Interrupt                              */
    SPI0_IRQn                = 12,     /**< SPI0 Interrupt                              */
    I2S0_Tx_IRQn             = 13,     /**< I2S0 Transmit Interrupt                     */
    I2S0_Rx_IRQn             = 14,     /**< I2S0 Receive Interrupt                      */
    UART0_LON_IRQn           = 15,     /**< UART0 LON Interrupt                         */
    UART0_RX_TX_IRQn         = 16,     /**< UART0 Receive/Transmit Interrupt            */
    UART0_ERR_IRQn           = 17,     /**< UART0 Error Interrupt                       */
    UART1_RX_TX_IRQn         = 18,     /**< UART1 Receive/Transmit Interrupt            */
    UART1_ERR_IRQn           = 19,     /**< UART1 Error Interrupt                       */
    UART2_RX_TX_IRQn         = 20,     /**< UART2 Receive/Transmit Interrupt            */
    UART2_ERR_IRQn           = 21,     /**< UART2 Error Interrupt                       */
    ADC0_IRQn                = 22,     /**< ADC0 Interrupt                              */
    CMP0_IRQn                = 23,     /**< CMP0 Interrupt                              */
    CMP1_IRQn                = 24,     /**< CMP1 Interrupt                              */
    FTM0_IRQn                = 25,     /**< FTM0 Fault, Overflow and Channels Interrupt */
    FTM1_IRQn                = 26,     /**< FTM1 Fault, Overflow and Channels Interrupt */
    CMT_IRQn                 = 27,     /**< CMT Interrupt                               */
    RTC_IRQn                 = 28,     /**< RTC Interrupt                               */
    RTC_Seconds_IRQn         = 29,     /**< RTC Seconds Interrupt                       */
    PIT0_IRQn                = 30,     /**< PIT Timer Channel 0 Interrupt               */
    PIT1_IRQn                = 31,     /**< PIT Timer Channel 1 Interrupt               */
    PIT2_IRQn                = 32,     /**< PIT Timer Channel 2 Interrupt               */
    PIT3_IRQn                = 33,     /**< PIT Timer Channel 3 Interrupt               */
    PDB0_IRQn                = 34,     /**< PDB0 Interrupt                              */
    USB0_IRQn                = 35,     /**< USB0 Interrupt                              */
    USBDCD_IRQn              = 36,     /**< USBDCD Interrupt                            */
    TSI0_IRQn                = 37,     /**< TSI0 Interrupt                              */
    MCG_IRQn                 = 38,     /**< MCG Interrupt                               */
    LPTimer_IRQn             = 39,     /**< LPTimer Interrupt                           */
    PORTA_IRQn               = 40,     /**< Port A Interrupt                            */
    PORTB_IRQn               = 41,     /**< Port B Interrupt                            */
    PORTC_IRQn               = 42,     /**< Port C Interrupt                            */
    PORTD_IRQn               = 43,     /**< Port D Interrupt                            */
    PORTE_IRQn               = 44,     /**< Port E Interrupt                            */
    SWI_IRQn                 = 45      /**< Software Interrupt                          */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"   /* Cortex-M4 processor and core peripherals */
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/*
** Start of section using anonymous unions
*/
#if defined(__ARMCC_VERSION)
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/**
  * @brief ADC - Analog to Digital Converter
  */
typedef struct
{
    __IO uint32_t SC1[2];               /**< ADC status and control registers 1, array offset: 0x0, array step: 0x4 */
    __IO uint32_t CFG1;                 /**< ADC configuration register 1, offset: 0x8 */
    __IO uint32_t CFG2;                 /**< Configuration register 2, offset: 0xC */
    __I  uint32_t R[2];                 /**< ADC data result register, array offset: 0x10, array step: 0x4 */
    __IO uint32_t CV1;                  /**< Compare value registers, offset: 0x18 */
    __IO uint32_t CV2;                  /**< Compare value registers, offset: 0x1C */
    __IO uint32_t SC2;                  /**< Status and control register 2, offset: 0x20 */
    __IO uint32_t SC3;                  /**< Status and control register 3, offset: 0x24 */
    __IO uint32_t OFS;                  /**< ADC offset correction register, offset: 0x28 */
    __IO uint32_t PG;                   /**< ADC plus-side gain register, offset: 0x2C */
    __IO uint32_t MG;                   /**< ADC minus-side gain register, offset: 0x30 */
    __IO uint32_t CLPD;                 /**< ADC plus-side general calibration value register, offset: 0x34 */
    __IO uint32_t CLPS;                 /**< ADC plus-side general calibration value register, offset: 0x38 */
    __IO uint32_t CLP4;                 /**< ADC plus-side general calibration value register, offset: 0x3C */
    __IO uint32_t CLP3;                 /**< ADC plus-side general calibration value register, offset: 0x40 */
    __IO uint32_t CLP2;                 /**< ADC plus-side general calibration value register, offset: 0x44 */
    __IO uint32_t CLP1;                 /**< ADC plus-side general calibration value register, offset: 0x48 */
    __IO uint32_t CLP0;                 /**< ADC plus-side general calibration value register, offset: 0x4C */
         uint8_t  RESERVED_0[4];
    __IO uint32_t CLMD;                 /**< ADC minus-side general calibration value register, offset: 0x54 */
    __IO uint32_t CLMS;                 /**< ADC minus-side general calibration value register, offset: 0x58 */
    __IO uint32_t CLM4;                 /**< ADC minus-side general calibration value register, offset: 0x5C */
    __IO uint32_t CLM3;                 /**< ADC minus-side general calibration value register, offset: 0x60 */
    __IO uint32_t CLM2;                 /**< ADC minus-side general calibration value register, offset: 0x64 */
    __IO uint32_t CLM1;                 /**< ADC minus-side general calibration value register, offset: 0x68 */
    __IO uint32_t CLM0;                 /**< ADC minus-side general calibration value register, offset: 0x6C */
} ADC_TypeDef;

/**
  * @brief CMP - Comparator
  */
typedef struct
{
    __IO uint8_t CR0;                   /**< CMP Control Register 0, offset: 0x0 */
    __IO uint8_t CR1;                   /**< CMP Control Register 1, offset: 0x1 */
    __IO uint8_t FPR;                   /**< CMP Filter Period Register, offset: 0x2 */
    __IO uint8_t SCR;                   /**< CMP Status and Control Register, offset: 0x3 */
    __IO uint8_t DACCR;                 /**< DAC Control Register, offset: 0x4 */
    __IO uint8_t MUXCR;                 /**< MUX Control Register, offset: 0x5 */
} CMP_TypeDef;

/**
 * @brief CMT - Carrier Modulator Transmitter
 */
typedef struct
{
    __IO uint8_t CGH1;                  /**< CMT Carrier Generator High Data Register 1, offset: 0x0 */
    __IO uint8_t CGL1;                  /**< CMT Carrier Generator Low Data Register 1, offset: 0x1 */
    __IO uint8_t CGH2;                  /**< CMT Carrier Generator High Data Register 2, offset: 0x2 */
    __IO uint8_t CGL2;                  /**< CMT Carrier Generator Low Data Register 2, offset: 0x3 */
    __IO uint8_t OC;                    /**< CMT Output Control Register, offset: 0x4 */
    __IO uint8_t MSC;                   /**< CMT Modulator Status and Control Register, offset: 0x5 */
    __IO uint8_t CMD1;                  /**< CMT Modulator Data Register Mark High, offset: 0x6 */
    __IO uint8_t CMD2;                  /**< CMT Modulator Data Register Mark Low, offset: 0x7 */
    __IO uint8_t CMD3;                  /**< CMT Modulator Data Register Space High, offset: 0x8 */
    __IO uint8_t CMD4;                  /**< CMT Modulator Data Register Space Low, offset: 0x9 */
    __IO uint8_t PPS;                   /**< CMT Primary Prescaler Register, offset: 0xA */
    __IO uint8_t DMA;                   /**< CMT Direct Memory Access, offset: 0xB */
} CMT_TypeDef;

/**
 * @brief CRC - Cyclic Redundancy Check
 */
typedef struct
{
    union {                             /* offset: 0x0 */
        struct {                            /* offset: 0x0 */
            __IO uint16_t CRCL;                 /**< CRC_CRCL register., offset: 0x0 */
            __IO uint16_t CRCH;                 /**< CRC_CRCH register., offset: 0x2 */
        } ACCESS16BIT;
        __IO uint32_t CRC;                  /**< CRC Data Register, offset: 0x0 */
        struct {                            /* offset: 0x0 */
            __IO uint8_t CRCLL;                 /**< CRC_CRCLL register., offset: 0x0 */
            __IO uint8_t CRCLU;                 /**< CRC_CRCLU register., offset: 0x1 */
            __IO uint8_t CRCHL;                 /**< CRC_CRCHL register., offset: 0x2 */
            __IO uint8_t CRCHU;                 /**< CRC_CRCHU register., offset: 0x3 */
        } ACCESS8BIT;
    };
    union {                             /* offset: 0x4 */
        struct {                            /* offset: 0x4 */
            __IO uint16_t GPOLYL;               /**< CRC_GPOLYL register., offset: 0x4 */
            __IO uint16_t GPOLYH;               /**< CRC_GPOLYH register., offset: 0x6 */
        } GPOLY_ACCESS16BIT;
        __IO uint32_t GPOLY;                /**< CRC Polynomial Register, offset: 0x4 */
        struct {                            /* offset: 0x4 */
            __IO uint8_t GPOLYLL;               /**< CRC_GPOLYLL register., offset: 0x4 */
            __IO uint8_t GPOLYLU;               /**< CRC_GPOLYLU register., offset: 0x5 */
            __IO uint8_t GPOLYHL;               /**< CRC_GPOLYHL register., offset: 0x6 */
            __IO uint8_t GPOLYHU;               /**< CRC_GPOLYHU register., offset: 0x7 */
        } GPOLY_ACCESS8BIT;
    };
    union {                             /* offset: 0x8 */
        __IO uint32_t CTRL;                 /**< CRC Control Register, offset: 0x8 */
        struct {                            /* offset: 0x8 */
            __IO uint8_t RESERVED_0[3];
            __IO uint8_t CTRLHU;                /**< CRC_CTRLHU register., offset: 0xB */
        } CTRL_ACCESS8BIT;
    };
} CRC_TypeDef;

/**
 * @brief eDMA - Direct Memory Access
 */
typedef struct
{
    __IO uint32_t CR;                   /**< Control Register, offset: 0x0 */
    __I  uint32_t ES;                   /**< Error Status Register, offset: 0x4 */
         uint8_t  RESERVED_0[4];
    __IO uint32_t ERQ;                  /**< Enable Request Register, offset: 0xC */
         uint8_t  RESERVED_1[4];
    __IO uint32_t EEI;                  /**< Enable Error Interrupt Register, offset: 0x14 */
    __O  uint8_t CEEI;                  /**< Clear Enable Error Interrupt Register, offset: 0x18 */
    __O  uint8_t SEEI;                  /**< Set Enable Error Interrupt Register, offset: 0x19 */
    __O  uint8_t CERQ;                  /**< Clear Enable Request Register, offset: 0x1A */
    __O  uint8_t SERQ;                  /**< Set Enable Request Register, offset: 0x1B */
    __O  uint8_t CDNE;                  /**< Clear DONE Status Bit Register, offset: 0x1C */
    __O  uint8_t SSRT;                  /**< Set START Bit Register, offset: 0x1D */
    __O  uint8_t CERR;                  /**< Clear Error Register, offset: 0x1E */
    __O  uint8_t CINT;                  /**< Clear Interrupt Request Register, offset: 0x1F */
         uint8_t  RESERVED_2[4];
    __IO uint32_t INT;                  /**< Interrupt Request Register, offset: 0x24 */
         uint8_t  RESERVED_3[4];
    __IO uint32_t ERR;                  /**< Error Register, offset: 0x2C */
         uint8_t  RESERVED_4[4];
    __IO uint32_t HRS;                  /**< Hardware Request Status Register, offset: 0x34 */
         uint8_t  RESERVED_5[200];
    __IO uint8_t DCHPRI3;               /**< Channel n Priority Register, offset: 0x100 */
    __IO uint8_t DCHPRI2;               /**< Channel n Priority Register, offset: 0x101 */
    __IO uint8_t DCHPRI1;               /**< Channel n Priority Register, offset: 0x102 */
    __IO uint8_t DCHPRI0;               /**< Channel n Priority Register, offset: 0x103 */
         uint8_t RESERVED_6[3836];
    struct {                            /* offset: 0x1000, array step: 0x20 */
        __IO uint32_t SADDR;                /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
        __IO uint16_t SOFF;                 /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
        __IO uint16_t ATTR;                 /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
        union {                             /* offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLNO;          /**< TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLOFFNO;       /**< TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
            __IO uint32_t NBYTES_MLOFFYES;      /**< TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20 */
        };
        __IO uint32_t SLAST;                /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
        __IO uint32_t DADDR;                /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
        __IO uint16_t DOFF;                 /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
        union {                             /* offset: 0x1016, array step: 0x20 */
            __IO uint16_t CITER_ELINKNO;        /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
            __IO uint16_t CITER_ELINKYES;       /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
        };
        __IO uint32_t DLAST_SGA;            /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
        __IO uint16_t CSR;                  /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
        union {                             /* offset: 0x101E, array step: 0x20 */
            __IO uint16_t BITER_ELINKNO;        /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
            __IO uint16_t BITER_ELINKYES;       /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
        };
    } TCD[4];
} DMA_TypeDef;

/**
 * @brief DMAMUX - Direct Memory Access Multiplexer
 */
typedef struct
{
    __IO uint8_t CHCFG[16];             /**< Channel Configuration Register, array offset: 0x0, array step: 0x1 */
} DMAMUX_TypeDef;

/**
 * @brief External Watchdog Monitor
 */
typedef struct
{
    __IO uint8_t CTRL;                  /**< Control Register, offset: 0x0 */
    __O  uint8_t SERV;                  /**< Service Register, offset: 0x1 */
    __IO uint8_t CMPL;                  /**< Compare Low Register, offset: 0x2 */
    __IO uint8_t CMPH;                  /**< Compare High Register, offset: 0x3 */
} EWM_TypeDef;

/**
 * @brief FTFL - Flash Memory Module
 */
typedef struct
{
    __IO uint8_t FSTAT;                 /**< Flash Status Register, offset: 0x0 */
    __IO uint8_t FCNFG;                 /**< Flash Configuration Register, offset: 0x1 */
    __I  uint8_t FSEC;                  /**< Flash Security Register, offset: 0x2 */
    __I  uint8_t FOPT;                  /**< Flash Option Register, offset: 0x3 */
    __IO uint8_t FCCOB3;                /**< Flash Common Command Object Registers, offset: 0x4 */
    __IO uint8_t FCCOB2;                /**< Flash Common Command Object Registers, offset: 0x5 */
    __IO uint8_t FCCOB1;                /**< Flash Common Command Object Registers, offset: 0x6 */
    __IO uint8_t FCCOB0;                /**< Flash Common Command Object Registers, offset: 0x7 */
    __IO uint8_t FCCOB7;                /**< Flash Common Command Object Registers, offset: 0x8 */
    __IO uint8_t FCCOB6;                /**< Flash Common Command Object Registers, offset: 0x9 */
    __IO uint8_t FCCOB5;                /**< Flash Common Command Object Registers, offset: 0xA */
    __IO uint8_t FCCOB4;                /**< Flash Common Command Object Registers, offset: 0xB */
    __IO uint8_t FCCOBB;                /**< Flash Common Command Object Registers, offset: 0xC */
    __IO uint8_t FCCOBA;                /**< Flash Common Command Object Registers, offset: 0xD */
    __IO uint8_t FCCOB9;                /**< Flash Common Command Object Registers, offset: 0xE */
    __IO uint8_t FCCOB8;                /**< Flash Common Command Object Registers, offset: 0xF */
    __IO uint8_t FPROT3;                /**< Program Flash Protection Registers, offset: 0x10 */
    __IO uint8_t FPROT2;                /**< Program Flash Protection Registers, offset: 0x11 */
    __IO uint8_t FPROT1;                /**< Program Flash Protection Registers, offset: 0x12 */
    __IO uint8_t FPROT0;                /**< Program Flash Protection Registers, offset: 0x13 */
         uint8_t RESERVED_0[2];
    __IO uint8_t FEPROT;                /**< EEPROM Protection Register, offset: 0x16 */
    __IO uint8_t FDPROT;                /**< Data Flash Protection Register, offset: 0x17 */
} FTFL_TypeDef;

/**
 * @brief FTM - FlexTimer Module
 */
typedef struct
{
    __IO uint32_t SC;                   /**< Status and Control, offset: 0x0 */
    __IO uint32_t CNT;                  /**< Counter, offset: 0x4 */
    __IO uint32_t MOD;                  /**< Modulo, offset: 0x8 */
    struct {                            /* offset: 0xC, array step: 0x8 */
        __IO uint32_t CnSC;                 /**< Channel (n) Status and Control, array offset: 0xC, array step: 0x8 */
        __IO uint32_t CnV;                  /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
    } CONTROLS[8];
    __IO uint32_t CNTIN;                /**< Counter Initial Value, offset: 0x4C */
    __IO uint32_t STATUS;               /**< Capture and Compare Status, offset: 0x50 */
    __IO uint32_t MODE;                 /**< Features Mode Selection, offset: 0x54 */
    __IO uint32_t SYNC;                 /**< Synchronization, offset: 0x58 */
    __IO uint32_t OUTINIT;              /**< Initial State for Channels Output, offset: 0x5C */
    __IO uint32_t OUTMASK;              /**< Output Mask, offset: 0x60 */
    __IO uint32_t COMBINE;              /**< Function for Linked Channels, offset: 0x64 */
    __IO uint32_t DEADTIME;             /**< Deadtime Insertion Control, offset: 0x68 */
    __IO uint32_t EXTTRIG;              /**< FTM External Trigger, offset: 0x6C */
    __IO uint32_t POL;                  /**< Channels Polarity, offset: 0x70 */
    __IO uint32_t FMS;                  /**< Fault Mode Status, offset: 0x74 */
    __IO uint32_t FILTER;               /**< Input Capture Filter Control, offset: 0x78 */
    __IO uint32_t FLTCTRL;              /**< Fault Control, offset: 0x7C */
    __IO uint32_t QDCTRL;               /**< Quadrature Decoder Control and Status, offset: 0x80 */
    __IO uint32_t CONF;                 /**< Configuration, offset: 0x84 */
    __IO uint32_t FLTPOL;               /**< FTM Fault Input Polarity, offset: 0x88 */
    __IO uint32_t SYNCONF;              /**< Synchronization Configuration, offset: 0x8C */
    __IO uint32_t INVCTRL;              /**< FTM Inverting Control, offset: 0x90 */
    __IO uint32_t SWOCTRL;              /**< FTM Software Output Control, offset: 0x94 */
    __IO uint32_t PWMLOAD;              /**< FTM PWM Load, offset: 0x98 */
} FTM_TypeDef;

/**
 * @brief GPIO - General Purpose Input Output
 */
typedef struct
{
    __IO uint32_t PDOR;                 /**< Port Data Output Register, offset: 0x0 */
    __O  uint32_t PSOR;                 /**< Port Set Output Register, offset: 0x4 */
    __O  uint32_t PCOR;                 /**< Port Clear Output Register, offset: 0x8 */
    __O  uint32_t PTOR;                 /**< Port Toggle Output Register, offset: 0xC */
    __I  uint32_t PDIR;                 /**< Port Data Input Register, offset: 0x10 */
    __IO uint32_t PDDR;                 /**< Port Data Direction Register, offset: 0x14 */
} GPIO_TypeDef;

/**
 * @brief I2C - Inter-Integrated Circuit
 */
typedef struct
{
    __IO uint8_t A1;                    /**< I2C Address Register 1, offset: 0x0 */
    __IO uint8_t F;                     /**< I2C Frequency Divider register, offset: 0x1 */
    __IO uint8_t C1;                    /**< I2C Control Register 1, offset: 0x2 */
    __IO uint8_t S;                     /**< I2C Status Register, offset: 0x3 */
    __IO uint8_t D;                     /**< I2C Data I/O register, offset: 0x4 */
    __IO uint8_t C2;                    /**< I2C Control Register 2, offset: 0x5 */
    __IO uint8_t FLT;                   /**< I2C Programmable Input Glitch Filter register, offset: 0x6 */
    __IO uint8_t RA;                    /**< I2C Range Address register, offset: 0x7 */
    __IO uint8_t SMB;                   /**< I2C SMBus Control and Status register, offset: 0x8 */
    __IO uint8_t A2;                    /**< I2C Address Register 2, offset: 0x9 */
    __IO uint8_t SLTH;                  /**< I2C SCL Low Timeout Register High, offset: 0xA */
    __IO uint8_t SLTL;                  /**< I2C SCL Low Timeout Register Low, offset: 0xB */
} I2C_TypeDef;

/**
 * @brief I2S - Synchronous Audio Interface
 */
typedef struct
{
    __IO uint32_t TCSR;                 /**< SAI Transmit Control Register, offset: 0x0 */
    __IO uint32_t TCR1;                 /**< SAI Transmit Configuration 1 Register, offset: 0x4 */
    __IO uint32_t TCR2;                 /**< SAI Transmit Configuration 2 Register, offset: 0x8 */
    __IO uint32_t TCR3;                 /**< SAI Transmit Configuration 3 Register, offset: 0xC */
    __IO uint32_t TCR4;                 /**< SAI Transmit Configuration 4 Register, offset: 0x10 */
    __IO uint32_t TCR5;                 /**< SAI Transmit Configuration 5 Register, offset: 0x14 */
         uint8_t  RESERVED_0[8];
    __O  uint32_t TDR[2];               /**< SAI Transmit Data Register, array offset: 0x20, array step: 0x4 */
         uint8_t  RESERVED_1[24];
    __I  uint32_t TFR[2];               /**< SAI Transmit FIFO Register, array offset: 0x40, array step: 0x4 */
         uint8_t  RESERVED_2[24];
    __IO uint32_t TMR;                  /**< SAI Transmit Mask Register, offset: 0x60 */
         uint8_t  RESERVED_3[28];
    __IO uint32_t RCSR;                 /**< SAI Receive Control Register, offset: 0x80 */
    __IO uint32_t RCR1;                 /**< SAI Receive Configuration 1 Register, offset: 0x84 */
    __IO uint32_t RCR2;                 /**< SAI Receive Configuration 2 Register, offset: 0x88 */
    __IO uint32_t RCR3;                 /**< SAI Receive Configuration 3 Register, offset: 0x8C */
    __IO uint32_t RCR4;                 /**< SAI Receive Configuration 4 Register, offset: 0x90 */
    __IO uint32_t RCR5;                 /**< SAI Receive Configuration 5 Register, offset: 0x94 */
         uint8_t  RESERVED_4[8];
    __I  uint32_t RDR[2];               /**< SAI Receive Data Register, array offset: 0xA0, array step: 0x4 */
         uint8_t  RESERVED_5[24];
    __I  uint32_t RFR[2];               /**< SAI Receive FIFO Register, array offset: 0xC0, array step: 0x4 */
         uint8_t  RESERVED_6[24];
    __IO uint32_t RMR;                  /**< SAI Receive Mask Register, offset: 0xE0 */
         uint8_t  RESERVED_7[28];
    __IO uint32_t MCR;                  /**< SAI MCLK Control Register, offset: 0x100 */
    __IO uint32_t MDR;                  /**< MCLK Divide Register, offset: 0x104 */
} I2S_TypeDef;

/**
 * @brief LLWU - Low-Leakage Wakeup Unit
 */
typedef struct
{
    __IO uint8_t PE1;                   /**< LLWU Pin Enable 1 Register, offset: 0x0 */
    __IO uint8_t PE2;                   /**< LLWU Pin Enable 2 Register, offset: 0x1 */
    __IO uint8_t PE3;                   /**< LLWU Pin Enable 3 Register, offset: 0x2 */
    __IO uint8_t PE4;                   /**< LLWU Pin Enable 4 Register, offset: 0x3 */
    __IO uint8_t ME;                    /**< LLWU Module Enable Register, offset: 0x4 */
    __IO uint8_t F1;                    /**< LLWU Flag 1 Register, offset: 0x5 */
    __IO uint8_t F2;                    /**< LLWU Flag 2 Register, offset: 0x6 */
    __IO uint8_t F3;                    /**< LLWU Flag 3 Register, offset: 0x7 */
    __IO uint8_t FILT1;                 /**< LLWU Pin Filter 1 Register, offset: 0x8 */
    __IO uint8_t FILT2;                 /**< LLWU Pin Filter 2 Register, offset: 0x9 */
    __IO uint8_t RST;                   /**< LLWU Reset Enable Register, offset: 0xA */
} LLWU_TypeDef;

/**
 * @brief LPTMR - Low-Power Timer
 */
typedef struct
{
    __IO uint32_t CSR;                  /**< Low Power Timer Control Status Register, offset: 0x0 */
    __IO uint32_t PSR;                  /**< Low Power Timer Prescale Register, offset: 0x4 */
    __IO uint32_t CMR;                  /**< Low Power Timer Compare Register, offset: 0x8 */
    __I  uint32_t CNR;                  /**< Low Power Timer Counter Register, offset: 0xC */
} LPTMR_TypeDef;

/**
 * @brief MCG - Multipurpose Clock Generator
 */
typedef struct
{
    __IO uint8_t C1;                    /**< MCG Control 1 Register, offset: 0x0 */
    __IO uint8_t C2;                    /**< MCG Control 2 Register, offset: 0x1 */
    __IO uint8_t C3;                    /**< MCG Control 3 Register, offset: 0x2 */
    __IO uint8_t C4;                    /**< MCG Control 4 Register, offset: 0x3 */
    __IO uint8_t C5;                    /**< MCG Control 5 Register, offset: 0x4 */
    __IO uint8_t C6;                    /**< MCG Control 6 Register, offset: 0x5 */
    __I  uint8_t S;                     /**< MCG Status Register, offset: 0x6 */
         uint8_t RESERVED_0[1];
    __IO uint8_t SC;                    /**< MCG Status and Control Register, offset: 0x8 */
         uint8_t RESERVED_1[1];
    __IO uint8_t ATCVH;                 /**< MCG Auto Trim Compare Value High Register, offset: 0xA */
    __IO uint8_t ATCVL;                 /**< MCG Auto Trim Compare Value Low Register, offset: 0xB */
    __IO uint8_t C7;                    /**< MCG Control 7 Register, offset: 0xC */
    __IO uint8_t C8;                    /**< MCG Control 8 Register, offset: 0xD */
} MCG_TypeDef;

/**
 * @brief OSC - Oscillator
 */
typedef struct
{
    __IO uint8_t CR;                    /**< OSC Control Register, offset: 0x0 */
} OSC_TypeDef;

/**
 * @brief PDB - Programmable Delay Block
 */
typedef struct
{
    __IO uint32_t SC;                   /**< Status and Control Register, offset: 0x0 */
    __IO uint32_t MOD;                  /**< Modulus Register, offset: 0x4 */
    __I  uint32_t CNT;                  /**< Counter Register, offset: 0x8 */
    __IO uint32_t IDLY;                 /**< Interrupt Delay Register, offset: 0xC */
    struct {                            /* offset: 0x10, array step: 0x10 */
        __IO uint32_t C1;                   /**< Channel n Control Register 1, array offset: 0x10, array step: 0x10 */
        __O  uint32_t S;                    /**< Channel n Status Register, array offset: 0x14, array step: 0x10 */
        __IO uint32_t DLY[2];               /**< Channel n Delay 0 Register..Channel n Delay 1 Register, array offset: 0x18, array step: index*0x10, index2*0x4 */
    } CH[1];
         uint8_t  RESERVED_0[368];
    __IO uint32_t POEN;                 /**< Pulse-Out n Enable Register, offset: 0x190 */
    __IO uint32_t PODLY[2];             /**< Pulse-Out n Delay Register, array offset: 0x194, array step: 0x4 */
} PDB_TypeDef;

/**
 * @brief PIT - Periodic Interrupt Timer
 */
typedef struct
{
    __IO uint32_t MCR;                  /**< PIT Module Control Register, offset: 0x0 */
         uint8_t  RESERVED_0[252];
    struct {                            /* offset: 0x100, array step: 0x10 */
        __IO uint32_t LDVAL;                /**< Timer Load Value Register, array offset: 0x100, array step: 0x10 */
        __IO uint32_t CVAL;                 /**< Current Timer Value Register, array offset: 0x104, array step: 0x10 */
        __IO uint32_t TCTRL;                /**< Timer Control Register, array offset: 0x108, array step: 0x10 */
        __IO uint32_t TFLG;                 /**< Timer Flag Register, array offset: 0x10C, array step: 0x10 */
    } CHANNEL[4];
} PIT_TypeDef;

/**
 * @brief PMC - Power Management Controller
 */
typedef struct
{
    __IO uint8_t LVDSC1;                /**< Low Voltage Detect Status and Control 1 Register, offset: 0x0 */
    __IO uint8_t LVDSC2;                /**< Low Voltage Detect Status and Control 2 Register, offset: 0x1 */
    __IO uint8_t REGSC;                 /**< Regulator Status and Control Register, offset: 0x2 */
} PMC_TypeDef;

/**
 * @brief PORT - Port Control and Interrupts
 */
typedef struct
{
    __IO uint32_t PCR[32];              /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
    __O  uint32_t GPCLR;                /**< Global Pin Control Low Register, offset: 0x80 */
    __O  uint32_t GPCHR;                /**< Global Pin Control High Register, offset: 0x84 */
         uint8_t  RESERVED_0[24];
    __O  uint32_t ISFR;                 /**< Interrupt Status Flag Register, offset: 0xA0 */
         uint8_t  RESERVED_1[28];
    __IO uint32_t DFER;                 /**< Digital Filter Enable Register, offset: 0xC0 */
    __IO uint32_t DFCR;                 /**< Digital Filter Clock Register, offset: 0xC4 */
    __IO uint32_t DFWR;                 /**< Digital Filter Width Register, offset: 0xC8 */
} PORT_TypeDef;

/**
 * @brief RCM - Reset Control Module
 */
typedef struct
{
    __I  uint8_t SRS0;                  /**< System Reset Status Register 0, offset: 0x0 */
    __I  uint8_t SRS1;                  /**< System Reset Status Register 1, offset: 0x1 */
         uint8_t RESERVED_0[2];
    __IO uint8_t RPFC;                  /**< Reset Pin Filter Control Register, offset: 0x4 */
    __IO uint8_t RPFW;                  /**< Reset Pin Filter Width Register, offset: 0x5 */
         uint8_t RESERVED_1[1];
    __I  uint8_t MR;                    /**< Mode Register, offset: 0x7 */
} RCM_TypeDef;

/**
 * @brief RTC - Real Time Clock
 */
typedef struct
{
    __IO uint32_t TSR;                  /**< RTC Time Seconds Register, offset: 0x0 */
    __IO uint32_t TPR;                  /**< RTC Time Prescaler Register, offset: 0x4 */
    __IO uint32_t TAR;                  /**< RTC Time Alarm Register, offset: 0x8 */
    __IO uint32_t TCR;                  /**< RTC Time Compensation Register, offset: 0xC */
    __IO uint32_t CR;                   /**< RTC Control Register, offset: 0x10 */
    __IO uint32_t SR;                   /**< RTC Status Register, offset: 0x14 */
    __IO uint32_t LR;                   /**< RTC Lock Register, offset: 0x18 */
    __IO uint32_t IER;                  /**< RTC Interrupt Enable Register, offset: 0x1C */
         uint8_t  RESERVED_0[2016];
    __IO uint32_t WAR;                  /**< RTC Write Access Register, offset: 0x800 */
    __IO uint32_t RAR;                  /**< RTC Read Access Register, offset: 0x804 */
} RTC_TypeDef;

/**
 * @brief SIM - System Integration Module
 */
typedef struct
{
    __IO uint32_t SOPT1;                /**< System Options Register 1, offset: 0x0 */
    __IO uint32_t SOPT1CFG;             /**< SOPT1 Configuration Register, offset: 0x4 */
         uint8_t  RESERVED_0[4092];
    __IO uint32_t SOPT2;                /**< System Options Register 2, offset: 0x1004 */
         uint8_t  RESERVED_1[4];
    __IO uint32_t SOPT4;                /**< System Options Register 4, offset: 0x100C */
    __IO uint32_t SOPT5;                /**< System Options Register 5, offset: 0x1010 */
         uint8_t  RESERVED_2[4];
    __IO uint32_t SOPT7;                /**< System Options Register 7, offset: 0x1018 */
         uint8_t  RESERVED_3[8];
    __I  uint32_t SDID;                 /**< System Device Identification Register, offset: 0x1024 */
         uint8_t  RESERVED_4[12];
    __IO uint32_t SCGC4;                /**< System Clock Gating Control Register 4, offset: 0x1034 */
    __IO uint32_t SCGC5;                /**< System Clock Gating Control Register 5, offset: 0x1038 */
    __IO uint32_t SCGC6;                /**< System Clock Gating Control Register 6, offset: 0x103C */
    __IO uint32_t SCGC7;                /**< System Clock Gating Control Register 7, offset: 0x1040 */
    __IO uint32_t CLKDIV1;              /**< System Clock Divider Register 1, offset: 0x1044 */
    __IO uint32_t CLKDIV2;              /**< System Clock Divider Register 2, offset: 0x1048 */
    __I  uint32_t FCFG1;                /**< Flash Configuration Register 1, offset: 0x104C */
    __I  uint32_t FCFG2;                /**< Flash Configuration Register 2, offset: 0x1050 */
    __I  uint32_t UIDH;                 /**< Unique Identification Register High, offset: 0x1054 */
    __I  uint32_t UIDMH;                /**< Unique Identification Register Mid-High, offset: 0x1058 */
    __I  uint32_t UIDML;                /**< Unique Identification Register Mid Low, offset: 0x105C */
    __I  uint32_t UIDL;                 /**< Unique Identification Register Low, offset: 0x1060 */
} SIM_TypeDef;

/**
 * @brief SMC - System Mode Controller
 */
typedef struct
{
    __IO uint8_t PMPROT;                /**< Power Mode Protection Register, offset: 0x0 */
    __IO uint8_t PMCTRL;                /**< Power Mode Control Register, offset: 0x1 */
    __IO uint8_t VLLSCTRL;              /**< VLLS Control Register, offset: 0x2 */
    __I  uint8_t PMSTAT;                /**< Power Mode Status Register, offset: 0x3 */
} SMC_TypeDef;

/**
 * @brief SPI - Serial Peripheral Interface
 */
typedef struct
{
    __IO uint32_t MCR;                  /**< DSPI Module Configuration Register, offset: 0x0 */
         uint8_t  RESERVED_0[4];
    __IO uint32_t TCR;                  /**< DSPI Transfer Count Register, offset: 0x8 */
    union {                             /* offset: 0xC */
        __IO uint32_t CTAR[2];              /**< DSPI Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
        __IO uint32_t CTAR_SLAVE[1];        /**< DSPI Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
    };
         uint8_t  RESERVED_1[24];
    __IO uint32_t SR;                   /**< DSPI Status Register, offset: 0x2C */
    __IO uint32_t RSER;                 /**< DSPI DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
    union {                             /* offset: 0x34 */
        __IO uint32_t PUSHR;                /**< DSPI PUSH TX FIFO Register In Master Mode, offset: 0x34 */
        __IO uint32_t PUSHR_SLAVE;          /**< DSPI PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
    };
    __I  uint32_t POPR;                 /**< DSPI POP RX FIFO Register, offset: 0x38 */
    __I  uint32_t TXFR0;                /**< DSPI Transmit FIFO Registers, offset: 0x3C */
    __I  uint32_t TXFR1;                /**< DSPI Transmit FIFO Registers, offset: 0x40 */
    __I  uint32_t TXFR2;                /**< DSPI Transmit FIFO Registers, offset: 0x44 */
    __I  uint32_t TXFR3;                /**< DSPI Transmit FIFO Registers, offset: 0x48 */
         uint8_t  RESERVED_2[48];
    __I  uint32_t RXFR0;                /**< DSPI Receive FIFO Registers, offset: 0x7C */
    __I  uint32_t RXFR1;                /**< DSPI Receive FIFO Registers, offset: 0x80 */
    __I  uint32_t RXFR2;                /**< DSPI Receive FIFO Registers, offset: 0x84 */
    __I  uint32_t RXFR3;                /**< DSPI Receive FIFO Registers, offset: 0x88 */
} SPI_TypeDef;

/**
 * @brief TSI - Touch Sense Input
 */
typedef struct
{
    __IO uint32_t GENCS;                /**< General Control and Status Register, offset: 0x0 */
    __IO uint32_t SCANC;                /**< SCAN Control Register, offset: 0x4 */
    __IO uint32_t PEN;                  /**< Pin Enable Register, offset: 0x8 */
    __IO uint32_t WUCNTR;               /**< Wake-Up Channel Counter Register, offset: 0xC */
         uint8_t  RESERVED_0[240];
    __I  uint32_t CNTR1;                /**< Counter Register, offset: 0x100 */
    __I  uint32_t CNTR3;                /**< Counter Register, offset: 0x104 */
    __I  uint32_t CNTR5;                /**< Counter Register, offset: 0x108 */
    __I  uint32_t CNTR7;                /**< Counter Register, offset: 0x10C */
    __I  uint32_t CNTR9;                /**< Counter Register, offset: 0x110 */
    __I  uint32_t CNTR11;               /**< Counter Register, offset: 0x114 */
    __I  uint32_t CNTR13;               /**< Counter Register, offset: 0x118 */
    __I  uint32_t CNTR15;               /**< Counter Register, offset: 0x11C */
    __IO uint32_t THRESHOLD;            /**< Low Power Channel Threshold Register, offset: 0x120 */
} TSI_TypeDef;

/**
 * @brief UART - Universal Asynchronous Receiver/Transmitter
 */
typedef struct
{
    __IO uint8_t BDH;                   /**< UART Baud Rate Registers:High, offset: 0x0 */
    __IO uint8_t BDL;                   /**< UART Baud Rate Registers: Low, offset: 0x1 */
    __IO uint8_t C1;                    /**< UART Control Register 1, offset: 0x2 */
    __IO uint8_t C2;                    /**< UART Control Register 2, offset: 0x3 */
    __I  uint8_t S1;                    /**< UART Status Register 1, offset: 0x4 */
    __IO uint8_t S2;                    /**< UART Status Register 2, offset: 0x5 */
    __IO uint8_t C3;                    /**< UART Control Register 3, offset: 0x6 */
    __IO uint8_t D;                     /**< UART Data Register, offset: 0x7 */
    __IO uint8_t MA1;                   /**< UART Match Address Registers 1, offset: 0x8 */
    __IO uint8_t MA2;                   /**< UART Match Address Registers 2, offset: 0x9 */
    __IO uint8_t C4;                    /**< UART Control Register 4, offset: 0xA */
    __IO uint8_t C5;                    /**< UART Control Register 5, offset: 0xB */
    __I  uint8_t ED;                    /**< UART Extended Data Register, offset: 0xC */
    __IO uint8_t MODEM;                 /**< UART Modem Register, offset: 0xD */
    __IO uint8_t IR;                    /**< UART Infrared Register, offset: 0xE */
         uint8_t RESERVED_0[1];
    __IO uint8_t PFIFO;                 /**< UART FIFO Parameters, offset: 0x10 */
    __IO uint8_t CFIFO;                 /**< UART FIFO Control Register, offset: 0x11 */
    __IO uint8_t SFIFO;                 /**< UART FIFO Status Register, offset: 0x12 */
    __IO uint8_t TWFIFO;                /**< UART FIFO Transmit Watermark, offset: 0x13 */
    __I  uint8_t TCFIFO;                /**< UART FIFO Transmit Count, offset: 0x14 */
    __IO uint8_t RWFIFO;                /**< UART FIFO Receive Watermark, offset: 0x15 */
    __I  uint8_t RCFIFO;                /**< UART FIFO Receive Count, offset: 0x16 */
         uint8_t RESERVED_1[1];
    __IO uint8_t C7816;                 /**< UART 7816 Control Register, offset: 0x18 */
    __IO uint8_t IE7816;                /**< UART 7816 Interrupt Enable Register, offset: 0x19 */
    __IO uint8_t IS7816;                /**< UART 7816 Interrupt Status Register, offset: 0x1A */
    union {                             /* offset: 0x1B */
        __IO uint8_t WP7816_T_TYPE0;        /**< UART 7816 Wait Parameter Register, offset: 0x1B */
        __IO uint8_t WP7816_T_TYPE1;        /**< UART 7816 Wait Parameter Register, offset: 0x1B */
    };
    __IO uint8_t WN7816;                /**< UART 7816 Wait N Register, offset: 0x1C */
    __IO uint8_t WF7816;                /**< UART 7816 Wait FD Register, offset: 0x1D */
    __IO uint8_t ET7816;                /**< UART 7816 Error Threshold Register, offset: 0x1E */
    __IO uint8_t TL7816;                /**< UART 7816 Transmit Length Register, offset: 0x1F */
         uint8_t RESERVED_2[1];
    __IO uint8_t C6;                    /**< UART CEA709.1-B Control Register 6, offset: 0x21 */
    __IO uint8_t PCTH;                  /**< UART CEA709.1-B Packet Cycle Time Counter High, offset: 0x22 */
    __IO uint8_t PCTL;                  /**< UART CEA709.1-B Packet Cycle Time Counter Low, offset: 0x23 */
    __IO uint8_t B1T;                   /**< UART CEA709.1-B Beta1 Timer, offset: 0x24 */
    __IO uint8_t SDTH;                  /**< UART CEA709.1-B Secondary Delay Timer High, offset: 0x25 */
    __IO uint8_t SDTL;                  /**< UART CEA709.1-B Secondary Delay Timer Low, offset: 0x26 */
    __IO uint8_t PRE;                   /**< UART CEA709.1-B Preamble, offset: 0x27 */
    __IO uint8_t TPL;                   /**< UART CEA709.1-B Transmit Packet Length, offset: 0x28 */
    __IO uint8_t IE;                    /**< UART CEA709.1-B Interrupt Enable Register, offset: 0x29 */
    __IO uint8_t WB;                    /**< UART CEA709.1-B WBASE, offset: 0x2A */
    __IO uint8_t S3;                    /**< UART CEA709.1-B Status Register, offset: 0x2B */
    __IO uint8_t S4;                    /**< UART CEA709.1-B Status Register, offset: 0x2C */
    __I  uint8_t RPL;                   /**< UART CEA709.1-B Received Packet Length, offset: 0x2D */
    __I  uint8_t RPREL;                 /**< UART CEA709.1-B Received Preamble Length, offset: 0x2E */
    __IO uint8_t CPW;                   /**< UART CEA709.1-B Collision Pulse Width, offset: 0x2F */
    __IO uint8_t RIDT;                  /**< UART CEA709.1-B Receive Indeterminate Time, offset: 0x30 */
    __IO uint8_t TIDT;                  /**< UART CEA709.1-B Transmit Indeterminate Time, offset: 0x31 */
} UART_TypeDef;

/**
 * @brief USB - Universal Serial Bus OTG Controller
 */
typedef struct
{
    __I  uint8_t PERID;                 /**< Peripheral ID Register, offset: 0x0 */
         uint8_t RESERVED_0[3];
    __I  uint8_t IDCOMP;                /**< Peripheral ID Complement Register, offset: 0x4 */
         uint8_t RESERVED_1[3];
    __I  uint8_t REV;                   /**< Peripheral Revision Register, offset: 0x8 */
         uint8_t RESERVED_2[3];
    __I  uint8_t ADDINFO;               /**< Peripheral Additional Info Register, offset: 0xC */
         uint8_t RESERVED_3[3];
    __IO uint8_t OTGISTAT;              /**< OTG Interrupt Status Register, offset: 0x10 */
         uint8_t RESERVED_4[3];
    __IO uint8_t OTGICR;                /**< OTG Interrupt Control Register, offset: 0x14 */
         uint8_t RESERVED_5[3];
    __IO uint8_t OTGSTAT;               /**< OTG Status Register, offset: 0x18 */
         uint8_t RESERVED_6[3];
    __IO uint8_t OTGCTL;                /**< OTG Control Register, offset: 0x1C */
         uint8_t RESERVED_7[99];
    __IO uint8_t ISTAT;                 /**< Interrupt Status Register, offset: 0x80 */
         uint8_t RESERVED_8[3];
    __IO uint8_t INTEN;                 /**< Interrupt Enable Register, offset: 0x84 */
         uint8_t RESERVED_9[3];
    __IO uint8_t ERRSTAT;               /**< Error Interrupt Status Register, offset: 0x88 */
         uint8_t RESERVED_10[3];
    __IO uint8_t ERREN;                 /**< Error Interrupt Enable Register, offset: 0x8C */
         uint8_t RESERVED_11[3];
    __I  uint8_t STAT;                  /**< Status Register, offset: 0x90 */
         uint8_t RESERVED_12[3];
    __IO uint8_t CTL;                   /**< Control Register, offset: 0x94 */
         uint8_t RESERVED_13[3];
    __IO uint8_t ADDR;                  /**< Address Register, offset: 0x98 */
         uint8_t RESERVED_14[3];
    __IO uint8_t BDTPAGE1;              /**< BDT Page Register 1, offset: 0x9C */
         uint8_t RESERVED_15[3];
    __IO uint8_t FRMNUML;               /**< Frame Number Register Low, offset: 0xA0 */
         uint8_t RESERVED_16[3];
    __IO uint8_t FRMNUMH;               /**< Frame Number Register High, offset: 0xA4 */
         uint8_t RESERVED_17[3];
    __IO uint8_t TOKEN;                 /**< Token Register, offset: 0xA8 */
         uint8_t RESERVED_18[3];
    __IO uint8_t SOFTHLD;               /**< SOF Threshold Register, offset: 0xAC */
         uint8_t RESERVED_19[3];
    __IO uint8_t BDTPAGE2;              /**< BDT Page Register 2, offset: 0xB0 */
         uint8_t RESERVED_20[3];
    __IO uint8_t BDTPAGE3;              /**< BDT Page Register 3, offset: 0xB4 */
         uint8_t RESERVED_21[11];
    struct {                            /* offset: 0xC0, array step: 0x4 */
        __IO uint8_t ENDPT;                 /**< Endpoint Control Register, array offset: 0xC0, array step: 0x4 */
             uint8_t RESERVED_0[3];
    } ENDPOINT[16];
    __IO uint8_t USBCTRL;               /**< USB Control Register, offset: 0x100 */
         uint8_t RESERVED_22[3];
    __I  uint8_t OBSERVE;               /**< USB OTG Observe Register, offset: 0x104 */
         uint8_t RESERVED_23[3];
    __IO uint8_t CONTROL;               /**< USB OTG Control Register, offset: 0x108 */
         uint8_t RESERVED_24[3];
    __IO uint8_t USBTRC0;               /**< USB Transceiver Control Register 0, offset: 0x10C */
         uint8_t RESERVED_25[7];
    __IO uint8_t USBFRMADJUST;          /**< Frame Adjust Register, offset: 0x114 */
} USB_TypeDef;

/**
 * @brief USBCD - USB Device Charger Detection Module
 */
typedef struct
{
    __IO uint32_t CONTROL;                                /**< Control Register, offset: 0x0 */
    __IO uint32_t CLOCK;                                  /**< Clock Register, offset: 0x4 */
    __I  uint32_t STATUS;                                 /**< Status Register, offset: 0x8 */
         uint8_t  RESERVED_0[4];
    __IO uint32_t TIMER0;                                 /**< TIMER0 Register, offset: 0x10 */
    __IO uint32_t TIMER1;                                 /**< , offset: 0x14 */
    __IO uint32_t TIMER2;                                 /**< , offset: 0x18 */
} USBCD_TypeDef;

/**
 * @brief VREF - Voltage Reference
 */
typedef struct
{
    __IO uint8_t TRM;                   /**< VREF Trim Register, offset: 0x0 */
    __IO uint8_t SC;                    /**< VREF Status and Control Register, offset: 0x1 */
} VREF_TypeDef;

/**
 * @brief WDOG - Watchdog Timer
 */
typedef struct
{
    __IO uint16_t STCTRLH;              /**< Watchdog Status and Control Register High, offset: 0x0 */
    __IO uint16_t STCTRLL;              /**< Watchdog Status and Control Register Low, offset: 0x2 */
    __IO uint16_t TOVALH;               /**< Watchdog Time-out Value Register High, offset: 0x4 */
    __IO uint16_t TOVALL;               /**< Watchdog Time-out Value Register Low, offset: 0x6 */
    __IO uint16_t WINH;                 /**< Watchdog Window Register High, offset: 0x8 */
    __IO uint16_t WINL;                 /**< Watchdog Window Register Low, offset: 0xA */
    __IO uint16_t REFRESH;              /**< Watchdog Refresh Register, offset: 0xC */
    __IO uint16_t UNLOCK;               /**< Watchdog Unlock Register, offset: 0xE */
    __IO uint16_t TMROUTH;              /**< Watchdog Timer Output Register High, offset: 0x10 */
    __IO uint16_t TMROUTL;              /**< Watchdog Timer Output Register Low, offset: 0x12 */
    __IO uint16_t RSTCNT;               /**< Watchdog Reset Count Register, offset: 0x14 */
    __IO uint16_t PRESC;                /**< Watchdog Prescaler Register, offset: 0x16 */
} WDOG_TypeDef;

/*
** End of section using anonymous unions
*/
#if defined(__ARMCC_VERSION)
  #pragma pop
#elif defined(__CWCC__)
  #pragma pop
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/**
  * @}
  */ /* Peripheral_registers_structures */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define ADC_BASE            ((uint32_t)0x4003B000)
#define CMP_BASE
#define CMT_BASE
#define CRC_BASE
#define DMA_BASE
#define DMAMUX_BASE
#define FTFL_BASE
#define FTM_BASE
#define GPIO_BASE
#define I2C_BASE
#define I2S_BASE
#define LLWU_BASE
#define LPTMR_BASE
#define MCG_BASE
#define OSC_BASE
#define PDB_BASE
#define PIT_BASE
#define PMC_BASE
#define PORT_BASE
#define RCM_BASE
#define RTC_BASE
#define SIM_BASE
#define SMC_BASE
#define SPI_BASE
#define TSI_BASE
#define UART_BASE
#define USB_BASE
#define USBCD_BASE
#define VREF_BASE
#define WDOG_BASE

/*
 * @}
 */ /* Peripheral_memory_map */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MK20D5_H_ */

/**
  * @}
  */ /* mk20d5 */

/**
  * @}
  */ /* CMSIS */
