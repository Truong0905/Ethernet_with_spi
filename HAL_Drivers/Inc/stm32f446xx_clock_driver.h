/*
 * stm32f446xx_clock_driver.h
 *
 *  Created on: Sep 16, 2023
 *      Author: Truong
 */

#ifndef INC_STM32F446XX_CLOCK_DRIVER_H_
#define INC_STM32F446XX_CLOCK_DRIVER_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RCC_HSE_FREQUENCY   (8000000U) /* 8Mhz */

#define RCC_HSI_FREQUENCY   (16000000U) /* 16Mhz */

#define RCC_ONE_MHZ  (1000000u)
 /********************************** SHIFL-MASK-MACRO  ********************************************/

/********* RCC clock control register (RCC_CR)***********************/

#define RCC_CR_HSION_MASK  (1u)
#define RCC_CR_HSION_SHIFT  (0u)
/*! HSION: Internal high-speed clock enable
 * 0: HSI oscillator OFF
 * 1: HSI oscillator ON
**/
#define RCC_CR_HSION(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_HSION_SHIFT)) & RCC_CR_HSION_MASK)

/**************/

#define RCC_CR_HSIRDY_MASK  (2u)
#define RCC_CR_HSIRDY_SHIFT  (1u)
/*! HSIRDY: Internal high-speed clock ready flag
 * 0: HSI oscillator not ready
 * 1: HSI oscillator ready
**/
#define RCC_CR_HSIRDY(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_HSIRDY_SHIFT)) & RCC_CR_HSIRDY_MASK)


/**************/

#define RCC_CR_HSEON_MASK  (0x010000)
#define RCC_CR_HSEON_SHIFT  (16U)
/*! HSEON: HSE clock enable
 * 0: HSE oscillator OFF
 * 1: HSE oscillator ON
**/
#define RCC_CR_HSEON(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_HSEON_SHIFT)) & RCC_CR_HSION_MASK)


/**************/

#define RCC_CR_HSERDY_MASK  (0x020000)
#define RCC_CR_HSERDY_SHIFT  (17u)
/*!  HSERDY: HSE clock ready flag
 * 0: HSE oscillator not ready
 * 1: HSE oscillator ready
**/
#define RCC_CR_HSERDY(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_HSERDY_SHIFT)) & RCC_CR_HSERDY_MASK)

/**************/

#define RCC_CR_PLLON_MASK  (0x01000000)
#define RCC_CR_PLLON_SHIFT  (24u)
/*! PLLON: Main PLL (PLL) enable
 * 0:  PLL OFF
 * 1:  PLL ON
**/
#define RCC_CR_PLLON(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_PLLON_SHIFT)) & RCC_CR_PLLON_MASK)

/**************/

#define RCC_CR_PLLRDY_MASK  (0x01000000)
#define RCC_CR_PLLRDY_SHIFT  (24u)
/*! PLLRDY: Main PLL (PLL) clock ready flag
 * 0:  PLL unlocked
 * 1:  PLL locked
**/

/**************/

#define RCC_CR_PLLI2SON_MASK  (0x04000000)
#define RCC_CR_PLLI2SON_SHIFT  (26u)
/*!  PLLI2S enable
 * 0:  : PLLI2S OFF
 * 1:  : PLLI2S ON
**/
#define RCC_CR_PLLI2SON(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_PLLI2SON_SHIFT)) & RCC_CR_PLLI2SON_MASK)

/**************/

#define RCC_CR_PLLI2SRDY_MASK  (0x08000000)
#define RCC_CR_PLLI2SRDY_SHIFT  (27u)
/*! Bit 27 PLLI2SRDY: PLLI2S clock ready flag
    Set by hardware to indicate that the PLLI2S is locked.
    0: PLLI2S unlocked
    1: PLLI2S locked
**/

/**************/

#define RCC_CR_PLLSAION_MASK  (0x10000000)
#define RCC_CR_PLLSAION_SHIFT  (28u)
/*! PLLSAION: PLLSAI enable
Set and cleared by software to enable PLLSAI.
Cleared by hardware when entering Stop or Standby mode.
0: PLLSAI OFF
1: PLLSAI ON
**/
#define RCC_CR_PLLSAION(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_PLLSAION_SHIFT)) & RCC_CR_PLLSAION_MASK)

/**************/

#define RCC_CR_PLLSAIRDY_MASK  (0x20000000)
#define RCC_CR_PLLSAIRDY_SHIFT  (29u)
/*! LLSAIRDY: PLLSAI clock ready flag
Set by hardware to indicate that the PLLSAI is locked.
0: PLLSAI unlocked
1: PLLSAI locked
**/

/**************/


/********* RCC PLL configuration register (RCC_PLLCFGR) ***********************/
#define RCC_PLLCFGR_LLSRC_MASK  (0x400000)
#define RCC_PLLCFGR_LLSRC_SHIFT  (22u)
/*!  LLSRC: Main PLL(PLL) and audio PLL (PLLI2S) entry clock source
    Set and cleared by software to select PLL and PLLI2S clock source. This bit can be written
    only when PLL and PLLI2S are disabled.
    0: HSI clock selected as PLL and PLLI2S clock entry
    1: HSE oscillator clock selected as PLL and PLLI2S clock entry
**/
#define RCC_PLLCFGR_LLSRC(x)  (((uint32_t)(((uint32_t)x)<< RCC_PLLCFGR_LLSRC_SHIFT)) & RCC_PLLCFGR_LLSRC_MASK)

#define RCC_PLLCFGR_PLLSRC_HSE RCC_PLLCFGR_LLSRC_MASK
#define RCC_PLLCFGR_PLLSRC_HSI (0u)

#define RCC_PLLCFGR_PLLM_MASK  (0x3F)
#define RCC_PLLCFGR_PLLM_SHIFT  (0u)
/*!  PLLM[5:0]: Division factor for the main PLL (PLL) input clock
    Caution: The software has to set these bits correctly to ensure that the VCO input frequency
    ranges from 1 to 2 MHz. It is recommended to select a frequency of 2 MHz to limit
    PLL jitter.
    VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    000000: PLLM = 0, wrong configuration
    000001: PLLM = 1, wrong configuration
    000010: PLLM = 2
    000011: PLLM = 3
    000100: PLLM = 4
    ...
    111110: PLLM = 62
    111111: PLLM = 63
**/
#define RCC_PLLCFGR_PLLM(x)  (((uint32_t)(((uint32_t)x)<< RCC_PLLCFGR_PLLM_SHIFT)) & RCC_PLLCFGR_PLLM_MASK)

/**************/

#define RCC_PLLCFGR_PLLN_MASK  (0x7FC0)
#define RCC_PLLCFGR_PLLN_SHIFT  (6u)
/*!  PLLN[8:0]: Main PLL (PLL) multiplication factor for VCO
    Set and cleared by software to control the multiplication factor of the VCO. These bits can
    be written only when PLL is disabled. Only half-word and word accesses are allowed to
    write these bits.
    Caution: The software has to set these bits correctly to ensure that the VCO output
    frequency is between 100 and 432 MHz.
    VCO output frequency = VCO input frequency × PLLN with 50 ≤ PLLN ≤ 432
    000000000: PLLN = 0, wrong configuration
    000000001: PLLN = 1, wrong configuration ...
    000110010: PLLN = 50
    ...
    001100011: PLLN = 99
    001100100: PLLN = 100
    ...
    110110000: PLLN = 432
    110110001: PLLN = 433, wrong configuration ...
    111111111: PLLN = 511, wrong configuration
**/
#define RCC_PLLCFGR_PLLN(x)  (((uint32_t)(((uint32_t)x)<< RCC_PLLCFGR_PLLN_SHIFT)) & RCC_PLLCFGR_PLLN_MASK)

/**************/

#define RCC_PLLCFGR_PLLP_MASK  (0x030000)
#define RCC_PLLCFGR_PLLP_SHIFT  (16u)
/*! Bits 17:16 PLLP[1:0]: Main PLL (PLL) division factor for main system clock
Set and cleared by software to control the frequency of the general PLL output clock. These
bits can be written only if PLL is disabled.
Caution: The software has to set these bits correctly not to exceed 180 MHz on this domain.
PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8
00: PLLP = 2
01: PLLP = 4
10: PLLP = 6
11: PLLP = 8
**/
#define RCC_PLLCFGR_PLLP(x)  (((uint32_t)(((uint32_t)x)<< RCC_PLLCFGR_PLLP_SHIFT)) & RCC_PLLCFGR_PLLP_MASK)

/**************/

#define RCC_PLLCFGR_PLLQ_MASK  (0x0F000000)
#define RCC_PLLCFGR_PLLQ_SHIFT  (24u)
/*! Bits 27:24 PLLQ[3:0]: Main PLL (PLL) division factor for USB OTG FS, SDIOclocks
    Set and cleared by software to control the frequency of USB OTG FS clock and the
    SDIOclock. These bits should be written only if PLL is disabled.
    Caution: The USB OTG FS requires a 48 MHz clock to work correctly. The SDIOneeds a
    frequency lower than or equal to 48 MHz to work correctly.
    USB OTG FS clock frequency = VCO frequency / PLLQ with 2 ≤ PLLQ ≤ 15
    0000: PLLQ = 0, wrong configuration
    0001: PLLQ = 1, wrong configuration
    0010: PLLQ = 2
    0011: PLLQ = 3
    0100: PLLQ = 4
    ...
    1111: PLLQ = 15
**/
#define RCC_PLLCFGR_PLLQ(x)  (((uint32_t)(((uint32_t)x)<< RCC_PLLCFGR_PLLQ_SHIFT)) & RCC_PLLCFGR_PLLQ_MASK)

/**************/

#define RCC_PLLCFGR_PLLR_MASK  (0x70000000)
#define RCC_PLLCFGR_PLLR_SHIFT  (28u)
/*! Bits 30:28 PLLR[2:0]: Main PLL division factor for I2Ss, SAIs, SYSTEM and SPDIF-Rx clocks
    Set and cleared by software to control the frequency of the clock. These bits should be
    written only if PLL is disabled.
    Clock frequency = VCO frequency / PLLR with 2 ≤ PLLR ≤ 7
    000: PLLR = 0, wrong configuration
    001: PLLR = 1, wrong configuration
    010: PLLR = 2
    011: PLLR = 3
    ...
    111: PLLR = 7
**/
#define RCC_PLLCFGR_PLLR(x)  (((uint32_t)(((uint32_t)x)<< RCC_PLLCFGR_PLLR_SHIFT)) & RCC_PLLCFGR_PLLR_MASK)

/********* RCC clock configuration register (RCC_CFGR)***********************/

#define RCC_CFGR_SWS_MASK  (0x0C)
#define RCC_CFGR_SWS_SHIFT  (2u)
/*!  SWS[1:0]: System clock switch status
    Set and cleared by hardware to indicate which clock source is used as the system clock.
    00: HSI oscillator used as the system clock
    01: HSE oscillator used as the system clock
    10: PLL used as the system clock
    11: PLL_R used as the system clock
**/
#define RCC_CFGR_SWS(x)  (((uint32_t)(((uint32_t)x)<< RCC_CFGR_SWS_SHIFT)) & RCC_CFGR_SWS_MASK)

#define RCC_CFGR_SWS_HSI                   0x00000000U                         /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                   0x00000004U                         /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock */

/**************/



 /********************************** END  ********************************************/

 /********************************** GET STATUS  ********************************************/

#define RCC_GET_SYSCLK_SOURCE (RCC->CFGR & RCC_CFGR_SWS_MASK)

#define RCC_GET_SYSCLK_SOURCE (RCC->CFGR & RCC_CFGR_SWS_MASK)


/** @defgroup RCC_PLLP_Clock_Divider PLLP Clock Divider
  * @{
  */
#define RCC_PLLP_DIV2                  ((uint32_t)0x00000002U)
#define RCC_PLLP_DIV4                  ((uint32_t)0x00000004U)
#define RCC_PLLP_DIV6                  ((uint32_t)0x00000006U)
#define RCC_PLLP_DIV8                  ((uint32_t)0x00000008U)

#define RCC_PLLP_DIV_LEN   (4u)
#define RCC_PLLP_DIV_ARR   { \
RCC_PLLP_DIV2, \
RCC_PLLP_DIV4, \
RCC_PLLP_DIV6, \
RCC_PLLP_DIV8, \
}
 #define RCC_PLLCFGR_PLL_MASK  (uint32_t)((RCC_PLLCFGR_PLLM_MASK | RCC_PLLCFGR_PLLN_MASK | RCC_PLLCFGR_PLLQ_MASK | RCC_PLLCFGR_PLLP_MASK | RCC_PLLCFGR_PLLR_MASK))

#define __RCC_PLL_CONFIG(__PLLM, __PLLN, __PLLP, __PLLQ, __PLLR )               \
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLL_MASK ) |            \
    (((__PLLM) << RCC_PLLCFGR_PLLM_SHIFT) & RCC_PLLCFGR_PLLM_MASK ) |   \
    (((__PLLN) << RCC_PLLCFGR_PLLN_SHIFT) & RCC_PLLCFGR_PLLN_MASK ) |   \
    (((__PLLP) << RCC_PLLCFGR_PLLP_SHIFT) & RCC_PLLCFGR_PLLP_MASK ) |   \
    (((__PLLQ) << RCC_PLLCFGR_PLLQ_SHIFT) & RCC_PLLCFGR_PLLQ_MASK ) |   \
    (((__PLLR) << RCC_PLLCFGR_PLLR_SHIFT) & RCC_PLLCFGR_PLLR_MASK )

/*******************************************************************************
 * Struct, enum, union
 ******************************************************************************/

/**
 *  @brief The enumination to determine what type of clock source to provide for system clock
 *
 * */
typedef enum
{
    CLOCK_HSI = 0u,  /* Internal clock source only */
    CLOCK_HSE = 1u, /* External clock source only */
}Clock_Source_t;

typedef enum
{
    CLOCK_PLL_ON = 0u,
    CLOCK_PLL_OFF = 11u,
}Clock_PLL_State_t;

typedef enum
{
    CLOCK_HSE_ON = 0u,
    CLOCK_HSE_OF = 1u,
}Clock_HSE_State_t;

/**
  * @brief  RCC PLL configuration structure definition
  */
typedef struct
{

  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 63    */

  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 50 and Max_Data = 432  */

  uint32_t PLLP;       /*!< PLLP: Division factor for main system clock (SYSCLK).
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */

  uint32_t PLLQ;       /*!< PLLQ: Division factor for OTG FS, SDMMC and RNG clocks.
                            This parameter must be a number between Min_Data = 2 and Max_Data = 15    */
  uint32_t PLLR;
}RCC_PLLInit_type;

typedef enum
{
    RCC_SELECT_SystemClock_180_MHZ = 180u,
    RCC_SELECT_SystemClock_48_MHZ = 48u,
    RCC_SELECT_SystemClock_72_MHZ = 72u,
}Clock_Set_SystemClock_t;

typedef struct
{
    Clock_Source_t Source;

    Clock_PLL_State_t PLL_state;

    Clock_Set_SystemClock_t SystemClock;

    Clock_HSE_State_t HSE_state;
}Clock_SelectSource_type;

/*******************************************************************************
 * Gloabal variables
 ******************************************************************************/

/*******************************************************************************
 * Local variables
 ******************************************************************************/

/*******************************************************************************
 * Static Function Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Global Funtions Prototypes
 ******************************************************************************/
LT_status_t CLOCK_SourceCfg(Clock_SelectSource_type *sourceClock);

/*******************************************************************************
 * Static Functions
 ******************************************************************************/

/*******************************************************************************
 * Global Funtions
 ******************************************************************************/


#endif /* INC_STM32F446XX_CLOCK_DRIVER_H_ */