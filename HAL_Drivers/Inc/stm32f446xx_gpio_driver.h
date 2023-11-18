/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Mar 22, 2022
 *      Author: Truong
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/





/*******************************************************************************
 * Struct, enum, union
 ******************************************************************************/

typedef enum
{

 GPIO_PIN_NO_0 = 0u,
 GPIO_PIN_NO_1 = 1u,
 GPIO_PIN_NO_2 = 2u,
 GPIO_PIN_NO_3 = 3u,
 GPIO_PIN_NO_4 = 4u,
 GPIO_PIN_NO_5 = 5u,
 GPIO_PIN_NO_6 = 6u,
 GPIO_PIN_NO_7 = 7u,
 GPIO_PIN_NO_8 = 8u,
 GPIO_PIN_NO_9 = 9u,
 GPIO_PIN_NO_10  = 10u,
 GPIO_PIN_NO_11  = 11u,
 GPIO_PIN_NO_12  = 12u,
 GPIO_PIN_NO_13  = 13u,
 GPIO_PIN_NO_14  = 14u,
 GPIO_PIN_NO_15  = 15u,

}GPIO_PinNumber_t;

typedef enum
{
 GPIO_MODE_IN  = 0U,
 GPIO_MODE_OUT = 1U,
 GPIO_MODE_ALTFN = 2U,
 GPIO_MODE_ANALOG  = 3U,
/* Interrupt mode */
 GPIO_MODE_IT_FT  = 4U,  //  input falling edge
 GPIO_MODE_IT_RT  = 5U,  //  input rising edge
 GPIO_MODE_IT_RFT = 6U ,// IT : input rising edge, falling edge trigger

}GPIO_PinMode_t;

typedef enum
{
    GPIO_OP_SPEED_LOW = 0u,
    GPIO_OP_SPEED_MEDIUM = 1u,
    GPIO_OP_SPEED_FAST = 2u,
    GPIO_OP_SPEED_HIGH = 3u,

}GPIO_PinSpeed_t;

typedef enum
{
 GPIO_NO_PUD = 0u,
 GPIO_PIN_PU = 1u,
 GPIO_PIN_PD = 2u,
}GPIO_PinPuPdControl_t;

typedef enum
{
    GPIO_OP_TYPE_PP = 0u,
    GPIO_OP_TYPE_OD = 1u,
}GPIO_PinOPType_t;

typedef enum
{
    GPIO_OP_STATE_ON = 1u,
    GPIO_OP_STATE_OFF = 0u,
}GPIO_PinOPStateInit_t;
/**
 * @brief  Tạo cấu trúc chọn chức năng của GPIO
 *
 */
typedef struct
{
    GPIO_PinNumber_t GPIO_PinNumber;
    GPIO_PinMode_t GPIO_PinMode;
    GPIO_PinSpeed_t GPIO_PinSpeed;
    GPIO_PinPuPdControl_t GPIO_PinPuPdControl;
    GPIO_PinOPType_t GPIO_PinOPType;
    GPIO_PinOPStateInit_t GPIO_PinOPStateInit;

    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/**
 * @brief Tạo 1 cấu trúc quản lý GPIO
 *
 */
typedef struct
{
    /* data */
    GPIO_RegDef_t *pGPIOx;           // Chứa địa chỉ của GPIOx port
    GPIO_PinConfig_t GPIO_PinConfig; // Chứa GPIO pin config settings
} GPIO_Handle_t;

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
/*
 * Peripharal Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
// Cấn 1 tham chiếu  đến địa chỉ  GPIO cần dùng , 1 tham trị En or Di để bật/tắt xung clock

/***
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOhandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

/***
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
// Cần biến con trỏ dể trỏ đến địa chỉ của Port GPIO caanf dùng, 1 biến để xác định Pin cần dùng và đây là hàm có trả về 1 hoặc 0 để chỉ trạng thái pin
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
// Vì 1 port có 16pin nên dùng hàm uint16_t và ta cũng cần 1 tham chiếu đến địa chỉ Port GPIO cần dùng
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
// Tuoơng tự read pin nhưng có thêm đối số đển biết đc pin đang set hay reset
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/***
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);      // khởi tạo ngắt
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority); // Xét mức ưu tiên ngắt
void GPIO_IRQHandling(uint8_t PinNumber);                             // Trình xử lý ngắt


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
