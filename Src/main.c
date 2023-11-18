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

#include "stm32f446xx.h"

#include "stm32f446xx_gpio_driver.h"

#include "stm32f446xx_clock_driver.h"

#include "stm32f446xx_spi_driver.h"

static SPI_Handle_t xSPI;


static LT_status_t InitClockSource(void);

static LT_status_t InitSPI(void);

static void InitGPIO(void);

void delay (void)
{
    for (uint32_t i = 0; i < 1000000; i ++);
}

int main(void)
{

    volatile  LT_status_t status = E_OK;
    uint32_t len = 6;
    uint8_t data[] = "Hello";

    status = InitClockSource();
    (void) InitGPIO();
    status = InitSPI();


    /* Loop forever */
	while (1)
    {
         delay();

        GPIO_WriteToOutputPin (GPIOA, GPIO_PIN_NO_15, GPIO_PIN_RESET);

         status = SPI_SendData(xSPI.pSPIx, data,  len);

        GPIO_WriteToOutputPin (GPIOA, GPIO_PIN_NO_15, GPIO_PIN_SET);

    }

}




static LT_status_t InitClockSource(void)
{
    Clock_SelectSource_type cfg;
    CLOCK_Setting_type clockSetting;
    LT_status_t status = E_OK;

    cfg.Source = CLOCK_HSI;
    cfg.HSE_state = CLOCK_HSE_OF;
    cfg.PLL_state = CLOCK_PLL_OFF;
    cfg.SystemClock = RCC_SELECT_SystemClock_8_MHZ;

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

    xSPI.pSPIx = SPI2;
    xSPI.SPI_Config.SPI_DeviceMode = SPI_MASTER_MODE;
    xSPI.SPI_Config.SPI_BusConfig = SPI_Full_Duplex_MODE;
    xSPI.SPI_Config.SPI_FirstBit = SPI_FIRSTBIT_MSB;
    xSPI.SPI_Config.SPI_CPHA = SPI_CPHA_HIGH;
    xSPI.SPI_Config.SPI_CPOL = SPI_CPOL_HIGH;
    xSPI.SPI_Config.SPI_DataSize = SPI_DS_8BITS;
    xSPI.SPI_Config.SPI_SclkSpeed = SPI_DIV_256;

    retVal = SPI_Init(&xSPI);

    return retVal;
}

static void InitGPIO(void)
{
    GPIO_Handle_t GPIO_miso;
    GPIO_Handle_t GPIO_mosi;
    GPIO_Handle_t GPIO_sclk;
    GPIO_Handle_t GPIO_nss;

    GPIO_sclk.pGPIOx = GPIOB;
    GPIO_sclk.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_sclk.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_sclk.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_sclk.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_sclk.GPIO_PinConfig.GPIO_PinAltFunMode = 5u ;


    GPIO_miso.pGPIOx = GPIOC;
    GPIO_miso.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_miso.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_miso.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_miso.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_miso.GPIO_PinConfig.GPIO_PinAltFunMode = 5u ;

    GPIO_mosi.pGPIOx = GPIOC;
    GPIO_mosi.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_mosi.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIO_mosi.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_HIGH;
    GPIO_mosi.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_mosi.GPIO_PinConfig.GPIO_PinAltFunMode = 5u ;

    GPIO_nss.pGPIOx = GPIOA;
    GPIO_nss.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_nss.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_nss.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_MEDIUM;
    GPIO_nss.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUD ;
    GPIO_nss.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP ;
    GPIO_nss.GPIO_PinConfig.GPIO_PinOPStateInit =  GPIO_OP_STATE_ON;

    GPIO_Init(&GPIO_sclk);
    GPIO_Init(&GPIO_miso);
    GPIO_Init(&GPIO_mosi);
    GPIO_Init(&GPIO_nss);
}
