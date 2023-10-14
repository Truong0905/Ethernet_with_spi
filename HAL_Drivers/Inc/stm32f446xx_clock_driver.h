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
#define RCC_CR_PLLRDY(x)  (((uint32_t)(((uint32_t)x)<< RCC_CR_PLLRDY_SHIFT)) & RCC_CR_PLLRDY_MASK)

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

typedef struct
{
    Clock_Source_t Source;
    Clock_PLL_State_t PLL_state;

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