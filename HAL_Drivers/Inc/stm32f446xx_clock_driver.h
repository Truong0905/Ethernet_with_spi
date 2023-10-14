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
    CLOCK_PLL_HSE =2u, /* External clock source with PLL */
    CLOCK_PLL_HSI =3u, /* Internal clock source with PLL */
}Clock_SelectSource_t;

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

/*******************************************************************************
 * Static Functions
 ******************************************************************************/

/*******************************************************************************
 * Global Funtions
 ******************************************************************************/


#endif /* INC_STM32F446XX_CLOCK_DRIVER_H_ */