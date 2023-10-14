/*
 * stm32f446xx_clock_driver.c
 *
 *  Created on: Sep 16, 2023
 *      Author: Truong
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "stm32f446xx.h"

#include "stm32f446xx_clock_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Gloabal variables
 ******************************************************************************/

/*******************************************************************************
 * Local variables
 ******************************************************************************/

/*******************************************************************************
 * Static Function Prototypes
 ******************************************************************************/
static HAL_status_t CLOCK_SelectSource(Clock_SelectSource_t sourceClock);

/*******************************************************************************
 * Global Funtions Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Static Functions
 ******************************************************************************/
static HAL_status_t CLOCK_SelectSource(Clock_SelectSource_t sourceClock)
{
    HAL_status_t retVal = E_OK;
    uint8_t pll_enable = FALSE;

    switch (sourceClock)
    {

    case CLOCK_PLL_HSI:
        pll_enable = TRUE;
    case CLOCK_HSI:
        {
            if ((RCC->CR & RCC_CR_HSIRDY_MASK) == 0)
            {
                if (RCC->CR & RCC_CR_HSERDY_MASK != 0)
                {
                    RCC->CR &= ~RCC_CR_HSEON(0);

                    while (RCC->CR & RCC_CR_HSERDY_MASK != 0);
                }

                RCC->CR |= RCC_CR_HSION(1);
            }
        }
        break;

    case CLOCK_PLL_HSE:
        pll_enable = TRUE;
    case CLOCK_HSE:
        {
            if ((RCC->CR & RCC_CR_HSERDY_MASK) == 0)
            {
                if (RCC->CR & RCC_CR_HSIRDY_MASK != 0)
                {
                    RCC->CR &= ~RCC_CR_HSION(0);

                    while (RCC->CR & RCC_CR_HSIRDY_MASK != 0);
                }

                RCC->CR |= RCC_CR_HSEON(1);
            }
        }
        break;

    default:
        retVal = E_INVALID_PARAMETER;
        break;
    }

    if ((TRUE == pll_enable) && (E_OK == retVal))
    {
    }

    return retVal;
}
/*******************************************************************************
 * Global Funtions
 ******************************************************************************/
