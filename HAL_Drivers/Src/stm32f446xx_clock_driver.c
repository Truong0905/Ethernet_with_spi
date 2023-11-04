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
static  Clock_Set_SystemClock_t s_SystemClock = RCC_SELECT_SystemClock_16_MHZ;
static uint32_t s_SystemClockStatus = RCC_CFGR_SW_HSI;

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
static LT_status_t CLOCK_Hse_Config(const Clock_HSE_State_t enableHSE)
{
    LT_status_t retStatus = E_OK;
    uint8_t ticks = 100u;

    switch (enableHSE)
    {
    case CLOCK_HSE_ON:
    {
        SET_BIT(RCC->CR, RCC_CR_HSEON_MASK);
        while ((READ_BIT(RCC->CR, RCC_CR_HSERDY_MASK) == 0u) && (ticks != 0u))
        {
            ticks--;
        }

        if (0 == ticks)
        {
            retStatus = E_TIMEOUT;
        }
    }
    break;

    case CLOCK_HSE_OF:
    {
        CLEAR_BIT(RCC->CR, RCC_CR_HSEON_MASK);

        while ((READ_BIT(RCC->CR, RCC_CR_HSERDY_MASK) != 0u) && (ticks != 0u))
        {
            ticks--;
        }

        if (0 == ticks)
        {
            retStatus = E_TIMEOUT;
        }
    }
    break;

    default:
        retStatus = E_INVALID_PARAMETER;
        break;
    }

    return retStatus;
}

static LT_status_t CLOCK_PLL_EnableMainPLL(const bool enable)
{
    LT_status_t retStatus = E_OK;
    uint8_t ticks = 100u;

    if (true == enable)
    {
        SET_BIT(RCC->CR, RCC_CR_PLLON_MASK);
        while ((READ_BIT(RCC->CR, RCC_CR_PLLRDY_MASK) == 0u) && (ticks != 0u))
        {
            ticks--;
        }
    }
    else
    {
        CLEAR_BIT(RCC->CR, RCC_CR_PLLON_MASK);\
        while ((READ_BIT(RCC->CR, RCC_CR_PLLRDY_MASK) != 0u) && (ticks != 0u))
        {
            ticks--;
        }
    }


    if (ticks == 0u)
    {
        retStatus = E_TIMEOUT;

        while (1)
        {
            /* code */
        }
    }

    return retStatus;
}

static LT_status_t CLOCK_PLL_EnablePLLI2S(const bool enable)
{
    LT_status_t retStatus = E_OK;
    uint8_t ticks = 100u;

    if (true == enable)
    {
        SET_BIT(RCC->CR, RCC_CR_PLLI2SON_MASK);

        while ((READ_BIT(RCC->CR, RCC_CR_PLLI2SRDY_MASK) == 0u)  && (ticks != 0u))
        {
            ticks--;
        }
    }
    else
    {
        CLEAR_BIT(RCC->CR, RCC_CR_PLLI2SON_MASK);

        while ((READ_BIT(RCC->CR, RCC_CR_PLLI2SRDY_MASK) != 0u)  && (ticks != 0u))
        {
            ticks--;
        }
    }


    if (ticks == 0u)
    {
        retStatus = E_TIMEOUT;
        while (1)
        {
            /* code */
        }
    }

    return retStatus;
}

static LT_status_t CLOCK_PLL_EnablePLLSAI(const bool enable)
{
    LT_status_t retStatus = E_OK;
    uint8_t ticks = 100u;

    if (true == enable)
    {
        SET_BIT(RCC->CR, RCC_CR_PLLSAION_MASK);
    }
    else
    {
        CLEAR_BIT(RCC->CR, RCC_CR_PLLSAION_MASK);
    }

    while (((READ_BIT(RCC->CR, RCC_CR_PLLSAIRDY_MASK) == 0u) == enable) && (ticks != 0u))
    {
        ticks--;
    }

    if (ticks == 0u)
    {
        retStatus = E_TIMEOUT;
    }

    return retStatus;
}

static LT_status_t CLOCK_PLL_Configuration(const Clock_Source_t source, Clock_Set_SystemClock_t frequencySet)
{
    LT_status_t retStatus = E_OK;
    RCC_PLLInit_type selectCfg;
    uint32_t RCC_PLLP_arr[RCC_PLLP_DIV_LEN] = RCC_PLLP_DIV_ARR;
    bool succes = false;
    uint32_t upperLimit = 0;
    uint32_t lowerLimit = 0;
    uint32_t upperLimit_main = 0;
    uint32_t lowerLimit_main = 0;
    uint32_t frequencySelected_PLLP = 0;
    uint32_t frequencySelected_PLLM = 0;     /* mhz */
    uint32_t frequencySelected_PLLN = 0;     /* mhz */

    uint32_t frequencySelected_main = 0;     /* mhz */

  //  uint32_t frequencyForUSB_Work = 48; /* mhz */
    /* Indicates what clock source was selected */
    switch (source)
    {
    case CLOCK_HSI:
        CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_LLSRC_MASK);
        upperLimit_main = 16;
        lowerLimit_main = 8;
        frequencySelected_main = (uint32_t)((uint32_t)RCC_HSI_FREQUENCY / (uint32_t)RCC_ONE_MHZ); /* Mhz */
        break;

    case CLOCK_HSE:
        SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_LLSRC_MASK);
        upperLimit_main = 8;
        lowerLimit_main = 4;
        frequencySelected_main = (uint32_t)((uint32_t)RCC_HSE_FREQUENCY / (uint32_t)RCC_ONE_MHZ); /* Mhz */

        break;
    default:
        retStatus = E_INVALID_PARAMETER;
        break;
    }

    retStatus = E_NOT_FOUND;
    selectCfg.PLLQ = 2;/* Currentlly, we don't use USB. Therefore, I set PLLQ = 2 */
    selectCfg.PLLR = 2;

    for (selectCfg.PLLM = 2; selectCfg.PLLM < 63; selectCfg.PLLM++)
    {
        upperLimit = upperLimit_main;
        lowerLimit = lowerLimit_main;
        frequencySelected_PLLM = frequencySelected_main;

        if (((selectCfg.PLLM > upperLimit) || (selectCfg.PLLM < lowerLimit)))
        {
            continue;
        }
        else
        {
            frequencySelected_PLLM = frequencySelected_PLLM / selectCfg.PLLM;
            lowerLimit = 100 / frequencySelected_PLLM;
            upperLimit = 432 / frequencySelected_PLLM;
        }

        for (selectCfg.PLLN = 50; selectCfg.PLLN < 432; selectCfg.PLLN++)
        {
            frequencySelected_PLLN = frequencySelected_PLLM;
            if (((selectCfg.PLLN > upperLimit) || (selectCfg.PLLN < lowerLimit)))
            {
                continue;
            }
            else
            {
                frequencySelected_PLLN = frequencySelected_PLLN * selectCfg.PLLN;
                lowerLimit = 180 / frequencySelected_PLLN;
            }

            for (uint8_t i = 0; i < RCC_PLLP_DIV_LEN; i++)
            {
                selectCfg.PLLP = RCC_PLLP_arr[i];

                if ((selectCfg.PLLP < lowerLimit))
                {
                    break;
                }
                else
                {
                    frequencySelected_PLLP = frequencySelected_PLLN / selectCfg.PLLP;

                    if (frequencySelected_PLLP == frequencySet)
                    {
                        succes = true;
                        retStatus = E_OK;
                        s_SystemClock = frequencySet;
                        s_SystemClockStatus = RCC_CFGR_SW_PLL_P;
                        break;
                    }
                }
            } /* selectCfg.PLLP */

            if (true == succes)
            {
                break;
            }
        } /* selectCfg.PLLN */

        if (true == succes)
        {
            break;
        }
    } /* selectCfg.PLLM  */
    if (E_OK == retStatus)
    {
        __RCC_PLL_CONFIG(selectCfg.PLLM, selectCfg.PLLN, selectCfg.PLLP, selectCfg.PLLQ, selectCfg.PLLR);
    }

    return retStatus;
}

/*******************************************************************************
 * Global Funtions
 ******************************************************************************/

LT_status_t CLOCK_SourceSelectionCfg(const Clock_SelectSource_type * const sourceClock)
{
    LT_status_t retStatus = E_OK;
    uint8_t hsiTicks = 100u;

    switch (sourceClock->Source)
    {
    case CLOCK_HSI:
    {
        if ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_HSI) || ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_PLL) && (RCC_PLLCFGR_PLLSRC_HSI == READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_LLSRC_MASK))))
        {

            if ((READ_BIT(RCC->CR, RCC_CR_HSIRDY_MASK)) == 0U)
            {
                while (1)
                {
                    /* ERROR*/
                }
            }
            else
            {
                s_SystemClockStatus = RCC_CFGR_SW_HSI;
                s_SystemClock = RCC_SELECT_SystemClock_16_MHZ;
            }
        }
        else
        {
            RCC->CR |= RCC_CR_HSION(1);

            while ((READ_BIT(RCC->CR, RCC_CR_HSIRDY_MASK) == 0u) && (hsiTicks != 0u))
            {
                hsiTicks--;
            }

            if (0 == hsiTicks)
            {
                retStatus = E_TIMEOUT;
            }
            else
            {
                s_SystemClockStatus = RCC_CFGR_SW_HSI;
                s_SystemClock = RCC_SELECT_SystemClock_16_MHZ;
            }
        }
    }
    break;

    case CLOCK_HSE:
    {
        if ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_HSE) || ((RCC_GET_SYSCLK_SOURCE == RCC_CFGR_SWS_PLL) && (RCC_PLLCFGR_PLLSRC_HSE == READ_BIT(RCC->PLLCFGR, RCC_PLLCFGR_LLSRC_MASK))))
        {
            if (READ_BIT(RCC->CR, RCC_CR_HSERDY_MASK) == 0)
            {
                while (1)
                {
                    /* ERROR*/
                }
            }
            else
            {
                s_SystemClockStatus = RCC_CFGR_SW_HSE;
                s_SystemClock = RCC_SELECT_SystemClock_8_MHZ;
            }
        }
        else
        {
            if (E_OK != CLOCK_Hse_Config(sourceClock->HSE_state))
            {
                while (1)
                {
                    /* ERROR*/
                }
            }
            else
            {
                s_SystemClockStatus = RCC_CFGR_SW_HSE;
                s_SystemClock = RCC_SELECT_SystemClock_8_MHZ;
            }
        }
    }
    break;

    default:
        retStatus = E_INVALID_PARAMETER;
        break;
    }

    if ((E_OK == retStatus) && (sourceClock->PLL_state == CLOCK_PLL_ON))
    {
        retStatus = CLOCK_PLL_EnablePLLI2S(false);

        retStatus = CLOCK_PLL_EnablePLLSAI(false);

        /* Disable the main PLL. */
        retStatus = CLOCK_PLL_EnableMainPLL(false);
        if (E_OK == retStatus)
        {
            /* Configure the main PLL clock source, multiplication and division factors. */
            retStatus = CLOCK_PLL_Configuration(sourceClock->Source, sourceClock->SystemClock);

            /* Enable the main PLL. */
            if (E_OK == retStatus)
                CLOCK_PLL_EnableMainPLL(true);
        }
    }

    return retStatus;
}


LT_status_t CLOCK_SettingClock(const CLOCK_Setting_type * const pClockSetting)
{
    LT_status_t retStatus = E_OK;
    uint32_t setMask = 0;
    uint32_t valAHB = 0;
    uint32_t valAPB1 = 0;
    uint32_t valAPB2 = 0;
    uint32_t ticks = 100u;

    if (Sys_HSE_DIV64 == pClockSetting->AHBCLKDivider)
    {
        valAHB = (s_SystemClock *1000) / 64;
    }
    else if (Sys_HSE_DIV1 == pClockSetting->AHBCLKDivider)
    {
        valAHB = (s_SystemClock *1000);
    }
    else
    {
        valAHB = (s_SystemClock *1000)  / ( 2 << (pClockSetting->AHBCLKDivider + 1));
    }
    if (PPRE_DIV1 == pClockSetting->APB1CLKDivider)\
    {
        valAPB1 = valAHB;
    }
    else
    {
        valAPB1 = valAHB / (2 << pClockSetting->APB1CLKDivider );
    }


    if (PPRE_DIV1 == pClockSetting->APB2CLKDivider)\
    {
        valAPB2 = valAHB;
    }
    else
    {
        valAPB2 = valAHB / (2 << pClockSetting->APB2CLKDivider );
    }

    if ((45000 < valAPB1) || (90000 < valAPB2))
    {
        retStatus = E_OUT_OF_RANGE;
    }


    if (E_OK == retStatus)
    {
        /*-------------------------- HCLK Configuration --------------------------*/
        /* Set the highest APBx dividers in order to ensure that we do not go through
        a non-spec phase whatever we decrease or increase HCLK. */

            MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_MASK,  RCC_HCLK_DIV16);

            MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_MASK, (RCC_HCLK_DIV16 <<3));

            if (Sys_HSE_DIV1 != pClockSetting->AHBCLKDivider)
            {
                setMask = RCC_CFGR_HPRE (0x08|pClockSetting->AHBCLKDivider);
            }
            else
            {
                setMask = 0;
            }

            MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_MASK, setMask);

        /*-------------------------- HCLK Configuration --------------------------*/

            MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_MASK, s_SystemClockStatus);

            while ((RCC_GET_SYSCLK_SOURCE != (s_SystemClockStatus << 2)) && (ticks != 0u))
            {
                ticks--;
            }

            if (0 == ticks)
            {
                retStatus = E_TIMEOUT;
            }
    }

    if (E_OK == retStatus)
    {
        /*-------------------------- PCLK1 Configuration ---------------------------*/
        if (PPRE_DIV1 == pClockSetting->APB1CLKDivider)
        {
            setMask = 0;
        }
        else
        {
            setMask = RCC_CFGR_PPRE1 (0x04|pClockSetting->APB1CLKDivider);
        }

         MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_MASK, setMask);

        /*-------------------------- PCLK2 Configuration ---------------------------*/

        if (PPRE_DIV1 == pClockSetting->APB2CLKDivider)
        {
            setMask = 0;
        }
        else
        {
            setMask = RCC_CFGR_PPRE2 (0x04|pClockSetting->APB2CLKDivider);
        }

        MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_MASK, setMask);

    }

    return retStatus;
}
/*******************************************************************************
 * Global Funtions
 ******************************************************************************/
