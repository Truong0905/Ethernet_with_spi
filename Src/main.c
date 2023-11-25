/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <string.h>

#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"

#include "stm32f446xx_clock_driver.h"

#include "stm32f446xx_spi_driver.h"

static SPI_Handle_t s_hpsi2, s_hspi3;

static volatile uint8_t s_data[13] = {0};

static volatile bool s_flag = FALSE;

static LT_status_t InitClockSource(void);

static LT_status_t InitSPI(void);

static void InitGPIO(void);

static void App_ReceivedData(uint8_t AppEv);

void delay (void)
{
    for (uint32_t i = 0; i < 1000000; i ++);
}

int main(void)
{

    volatile  LT_status_t status = E_OK;
    uint8_t data[] = "Le Van Truong";
    uint32_t len = strlen(data);
    int check = 0;
    status = InitClockSource();
    if (E_OK == status)
    {
        (void) InitGPIO();
        status = InitSPI();
    }
    else
    {
        for(;;);
    }

    if (E_OK != status)
        for(;;);

    status =  SPi_ReciveDataIT(&s_hspi3, s_data, 13);
    (void) GPIO_IRQInterruptConfig(IRQ_NO_SPI3, ENABLE);
    /* Loop forever */
	while (1)
    {
         delay();

        GPIO_WriteToOutputPin (GPIOA, GPIO_PIN_NO_14, GPIO_PIN_RESET);
        (void) SPI_PeripheralControl(SPI2, ENABLE);

         status = SPI_SendData(&s_hpsi2, data,  len);

        (void) SPI_PeripheralControl(SPI2, DISABLE);
        GPIO_WriteToOutputPin (GPIOA, GPIO_PIN_NO_14, GPIO_PIN_SET);
        while ((s_flag == FALSE));

        s_flag = FALSE;
       check = memcmp(s_data, data, 13);
        if (0 == check)
        {
        	memset(s_data, 0, 13);
            status =  SPi_ReciveDataIT(&s_hspi3, s_data, 13);
        }
        else
        {
            while (1)
            {
                /* code */
            }
            
        }
    }

}




static LT_status_t InitClockSource(void)
{
    Clock_SelectSource_type cfg;
    CLOCK_Setting_type clockSetting;
    LT_status_t status = E_OK;

    cfg.Source = CLOCK_HSE;
    cfg.HSE_state = CLOCK_HSE_ON;
    cfg.PLL_state = CLOCK_PLL_ON;
    cfg.SystemClock = RCC_SELECT_SystemClock_72_MHZ;

    clockSetting.AHBCLKDivider = Sys_HSE_DIV1;
    clockSetting.APB1CLKDivider = PPRE_DIV16;
    clockSetting.APB2CLKDivider = PPRE_DIV2;

   status =  CLOCK_SourceSelectionCfg(&cfg);

    if (E_OK == status)
    {
       status = CLOCK_SettingClock( &clockSetting);
    }

    return status;
}

static LT_status_t InitSPI(void)
{
    LT_status_t retVal = E_OK;

    s_hpsi2.pSPIx = SPI2;
    s_hpsi2.SPI_Config.SPI_DeviceMode = SPI_MASTER_MODE;
    s_hpsi2.SPI_Config.SPI_BusConfig = SPI_Full_Duplex_MODE;
    s_hpsi2.SPI_Config.SPI_FirstBit = SPI_FIRSTBIT_MSB;
    s_hpsi2.SPI_Config.SPI_CPHA = SPI_CPHA_HIGH;
    s_hpsi2.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
    s_hpsi2.SPI_Config.SPI_DataSize = SPI_DS_8BITS;
    s_hpsi2.SPI_Config.SPI_SclkSpeed = SPI_DIV_4;
    s_hpsi2.pRxCallBackFunction = NULL ;
    s_hpsi2.pTxCallBackFunction = NULL ;
    s_hpsi2.select = SPI_2;

    s_hspi3.pSPIx = SPI3;
    s_hspi3.SPI_Config.SPI_DeviceMode = SPI_SLAVE_MODE;
    s_hspi3.SPI_Config.SPI_BusConfig = SPI_Full_Duplex_MODE;
    s_hspi3.SPI_Config.SPI_FirstBit = SPI_FIRSTBIT_MSB;
    s_hspi3.SPI_Config.SPI_CPHA = SPI_CPHA_HIGH;
    s_hspi3.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
    s_hspi3.SPI_Config.SPI_DataSize = SPI_DS_8BITS;
    s_hspi3.SPI_Config.SPI_SclkSpeed = SPI_DIV_4;
    s_hspi3.pRxCallBackFunction = App_ReceivedData ;
    s_hspi3.pTxCallBackFunction = NULL ;
    s_hspi3.select = SPI_2;

    retVal = SPI_Init(&s_hpsi2);
    retVal = SPI_Init(&s_hspi3);

    return retVal;
}

static void InitGPIO(void)
{
    // GPIO_Handle_t GPIO_miso_spi2;
    GPIO_Handle_t GPIO_mosi_spi2;
    GPIO_Handle_t GPIO_sclk_spi2;
    GPIO_Handle_t GPIO_nss_spi2;


    // GPIO_Handle_t GPIO_miso_spi3;
    GPIO_Handle_t GPIO_mosi_spi3;
    GPIO_Handle_t GPIO_sclk_spi3;
    GPIO_Handle_t GPIO_nss_spi3;


/************SPI2****************************************************/
    GPIO_sclk_spi2.pGPIOx = GPIOB;
    GPIO_sclk_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_sclk_spi2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_sclk_spi2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_sclk_spi2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_sclk_spi2.GPIO_PinConfig.GPIO_PinAltFunMode = 5u ;


    // GPIO_miso_spi2.pGPIOx = GPIOC;
    // GPIO_miso_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    // GPIO_miso_spi2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    // GPIO_miso_spi2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    // GPIO_miso_spi2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    // GPIO_miso_spi2.GPIO_PinConfig.GPIO_PinAltFunMode = 5u ;

    GPIO_mosi_spi2.pGPIOx = GPIOC;
    GPIO_mosi_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_mosi_spi2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_mosi_spi2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_mosi_spi2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_mosi_spi2.GPIO_PinConfig.GPIO_PinAltFunMode = 7u ;

    GPIO_nss_spi2.pGPIOx = GPIOC;
    GPIO_nss_spi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    GPIO_nss_spi2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_nss_spi2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
    GPIO_nss_spi2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_nss_spi2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP ;
    GPIO_nss_spi2.GPIO_PinConfig.GPIO_PinOPStateInit =  GPIO_OP_STATE_ON;

    GPIO_Init(&GPIO_sclk_spi2);
    // GPIO_Init(&GPIO_miso_spi2);
    GPIO_Init(&GPIO_mosi_spi2);
    GPIO_Init(&GPIO_nss_spi2);



/************SPI3****************************************************/

    GPIO_sclk_spi3.pGPIOx = GPIOC;
    GPIO_sclk_spi3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_sclk_spi3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_sclk_spi3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_sclk_spi3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_sclk_spi3.GPIO_PinConfig.GPIO_PinAltFunMode = 6u ;


    // GPIO_miso_spi3.pGPIOx = GPIOC;
    // GPIO_miso_spi3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    // GPIO_miso_spi3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    // GPIO_miso_spi3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    // GPIO_miso_spi3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    // GPIO_miso_spi3.GPIO_PinConfig.GPIO_PinAltFunMode = 6u ;

    GPIO_mosi_spi3.pGPIOx = GPIOB;
    GPIO_mosi_spi3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_mosi_spi3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_mosi_spi3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_mosi_spi3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_mosi_spi3.GPIO_PinConfig.GPIO_PinAltFunMode = 7u ;

    GPIO_nss_spi3.pGPIOx = GPIOA;
    GPIO_nss_spi3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_nss_spi3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_nss_spi3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_nss_spi3.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_nss_spi3.GPIO_PinConfig.GPIO_PinAltFunMode = 6u ;

    GPIO_Init(&GPIO_sclk_spi3);
    // GPIO_Init(&GPIO_miso_spi3);
    GPIO_Init(&GPIO_mosi_spi3);
    GPIO_Init(&GPIO_nss_spi3);
}


void SPI3_IRQHandler(void)
{
    (void) SPI_IRQHandling (&s_hspi3);
}

static void App_ReceivedData(uint8_t AppEv)
{
    if ( SPI_EVENT_RX_CMPLT == AppEv)
    {
        s_flag = TRUE;
    }
}
