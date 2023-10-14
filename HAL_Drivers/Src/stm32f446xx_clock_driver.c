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
static inline LT_status_t CLOCK_Hse_Config(Clock_HSE_State_t enableHSE);

/*******************************************************************************
 * Global Funtions Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Static Functions
 ******************************************************************************/
static inline LT_status_t CLOCK_Hse_Config(Clock_HSE_State_t enableHSE)
{
    LT_status_t retVal = E_OK;
    uint8_t ticks = 7u;

    switch (enableHSE)
    {
    case CLOCK_HSE_ON:
    {
        RCC->CR |= RCC_CR_HSEON(1);
        while (((RCC->CR & RCC_CR_HSERDY_MASK) == 0u ) && (ticks !=0u ))
        {
            ticks--;
        }

        if (0 == ticks )
        {
            retVal = E_TIMEOUT;
        }
    }
    break;

    case CLOCK_HSE_OF:
    {
        RCC->CR &= ~RCC_CR_HSEON_MASK;
        while (((RCC->CR & RCC_CR_HSERDY_MASK) != 0u ) && (ticks !=0u ))
        {
            ticks--;
        }

        if (0 == ticks )
        {
            retVal = E_TIMEOUT;
        }

    }
    break;

    default:
        retVal = E_INVALID_PARAMETER;
        break;
    }

    return retVal;
}


/*******************************************************************************
 * Global Funtions
 ******************************************************************************/

LT_status_t CLOCK_SourceCfg(Clock_SelectSource_type *sourceClock)
{
    LT_status_t retVal = E_OK;
    uint8_t hsiTicks = 7u;

    switch (sourceClock->Source)
    {
    case CLOCK_HSI:
    {
        if ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_HSI)
        || ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_PLL) && (RCC_PLLCFGR_PLLSRC_HSI == (RCC->PLLCFGR & RCC_PLLCFGR_LLSRC_MASK))))
        {
            if ((RCC->CR & RCC_CR_HSIRDY_MASK) == 0)
            {
                while (1)
                {
                    /* ERROR*/
                }
            }
        }
        else
        {
            RCC->CR |= RCC_CR_HSION(1);

            while (((RCC->CR & RCC_CR_HSIRDY_MASK) == 0u ) && (hsiTicks !=0u ))
            {
                hsiTicks--;
            }

            if (0 == hsiTicks )
            {
                retVal = E_TIMEOUT;
            }
        }
    }
    break;

    case CLOCK_HSE:
    {
        if ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_HSE)
        || ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_PLL) && (RCC_PLLCFGR_PLLSRC_HSE == (RCC->PLLCFGR & RCC_PLLCFGR_LLSRC_MASK))))
        {
            if ((RCC->CR & RCC_CR_HSERDY_MASK) == 0)
            {
                while (1)
                {
                    /* ERROR*/
                }
            }
        }
        else
        {
            if(E_OK != CLOCK_Hse_Config(sourceClock->HSE_state))
            {
                while (1)
                {
                    /* ERROR*/
                }
            }
        }
    }
    break;

    default:
        retVal = E_INVALID_PARAMETER;
        break;
    }

    if ((E_OK != retVal) && (sourceClock->PLL_state == CLOCK_PLL_ON))
    {
        /* Disable the main PLL. */
        /* Wait till PLL is ready */
        /* Configure the main PLL clock source, multiplication and division factors. */
        /* Enable the main PLL. */
        /* Wait till PLL is ready */

    }

    return retVal;
}
/*******************************************************************************
 * Global Funtions
 ******************************************************************************/
