/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*
 *   @SPI_SSM
 * chọn slave theo phần mềm hoặc không
 */
#define SPI_SSM_EN 1
#define SPI_SSM_DI 0

/*
 * LSBFIRST
 */
#define SPI_LSBFIRST_LSB 1
#define SPI_LSBFIRST_MSB 0

// Các macros dùng trong ngắt của SPI
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/**
 * @brief possible SPI application events
 *
 */
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR 3
#define SPI_EVENT_CRC__ERR 4

// Các cờ thông báo trong SPI
#define SPI_TXE_FLAG (1 << SPI_SR_TXE)
#define SPI_RXE_FLAG (1 << SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG (1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG (1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG (1 << SPI_SR_CRCERR)
#define SPI_SR_MODF_FLAG (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG (1 << SPI_SR_OVR)
#define SPI_BSY_FLAG (1 << SPI_SR_BSY)
#define SPI_FRE_FLAG (1 << SPI_SR_FRE)

 /********************************** SHIFL-MASK-MACRO  ********************************************/


/********* SPI control register 1 (SPI_CR1) ***********************/

#define SPI_CR1_CPHA_MASK  (0x00000001)
#define SPI_CR1_CPHA_SHIFT  (0u)
/*!Bit 0 CPHA: Clock phase
    0: The first clock transition is the first data capture edge
    1: The second clock transition is the first data capture edge
    Note: This bit should not be changed when communication is ongoing.
    It is not used in I2S mode and SPI TI mode except the case when CRC is applied
    at TI mode.
**/
#define SPI_CR1_CPHA(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_CPHA_SHIFT)) & SPI_CR1_CPHA_MASK)

/**************/

#define SPI_CR1_CPOL_MASK  (0x00000002)
#define SPI_CR1_CPOL_SHIFT  (1u)
/*!Bit1 CPOL: Clock polarity
    0: CK to 0 when idle
    1: CK to 1 when idle
    Note: This bit should not be changed when communication is ongoing.
    It is not used in I2S mode and SPI TI mode except the case when CRC is applied
    at TI mode.
**/
#define SPI_CR1_CPOL(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_CPOL_SHIFT)) & SPI_CR1_CPOL_MASK)

/**************/

#define SPI_CR1_MSTR_MASK  (0x00000004)
#define SPI_CR1_MSTR_SHIFT  (2u)
/*!Bit 2 MSTR: Master selection
    0: Slave configuration
    1: Master configuration
    Note: This bit should not be changed when communication is ongoing.
    It is not used in I2S mode.
**/
#define SPI_CR1_MSTR(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_MSTR_SHIFT)) & SPI_CR1_MSTR_MASK)

/**************/

#define SPI_CR1_BR_MASK  (0x00000038)
#define SPI_CR1_BR_SHIFT  (3u)
/*! Bits 5:3 BR[2:0]: Baud rate control
    000: fPCLK/2
    001: fPCLK/4
    010: fPCLK/8
    011: fPCLK/16
    100: fPCLK/32
    101: fPCLK/64
    110: fPCLK/128
    111: fPCLK/256
    Note: These bits should not be changed when communication is ongoing.
    They are not used in I2S mode.
**/
#define SPI_CR1_BR(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_BR_SHIFT)) & SPI_CR1_BR_MASK)

/**************/
#define SPI_CR1_SPE_MASK  (0x00000040)
#define SPI_CR1_SPE_SHIFT  (6u)
/*! Bit 6 SPE: SPI enable
    0: Peripheral disabled
    1: Peripheral enabled
    Note: This bit is not used in I2S mode.
**/
#define SPI_CR1_SPE(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_SPE_SHIFT)) & SPI_CR1_SPE_MASK)

/**************/

#define SPI_CR1_LSBFIRST_MASK  (0x00000080)
#define SPI_CR1_LSBFIRST_SHIFT  (7u)
/*! Bit 7 LSBFIRST: Frame format
    0: MSB transmitted first
    1: LSB transmitted first
    Note: This bit should not be changed when communication is ongoing.
**/
#define SPI_CR1_LSBFIRST(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_LSBFIRST_SHIFT)) & SPI_CR1_LSBFIRST_MASK)

/**************/

#define SPI_CR1_SSI_MASK  (0x00000100)
#define SPI_CR1_SSI_SHIFT  (8u)
/*! Bit 8 SSI: Internal slave select
    This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the
    NSS pin and the IO value of the NSS pin is ignored
**/
#define SPI_CR1_SSI(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_SSI_SHIFT)) & SPI_CR1_SSI_MASK)

/**************/

#define SPI_CR1_SSM_MASK  (0x00000200)
#define SPI_CR1_SSM_SHIFT  (9u)
/*! Bit 9 SSM: Software slave management
    When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.
    0: Software slave management disabled
    1: Software slave management enabled
**/
#define SPI_CR1_SSM(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_SSM_SHIFT)) & SPI_CR1_SSM_MASK)

/**************/

#define SPI_CR1_RXONLY_MASK  (0x0000000A)
#define SPI_CR1_RXONLY_SHIFT  (10u)
/*! Bit 10 RXONLY: Receive only mode enable
    This bit enables simplex communication using a single unidirectional line to receive data
    exclusively. Keep BIDIMODE bit clear when receive only mode is active.
    This bit is also useful in a multislave system in which this particular slave is not accessed, the
    output from the accessed slave is not corrupted.
    0: full-duplex (Transmit and receive)
    1: Output disabled (Receive-only mode)
**/
#define SPI_CR1_RXONLY(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_RXONLY_SHIFT)) & SPI_CR1_RXONLY_MASK)

/**************/
#define SPI_CR1_DFF_MASK  (0x00000800)
#define SPI_CR1_DFF_SHIFT  (11u)
/*! Bit 11 DFF: Data frame format
    0: 8-bit data frame format is selected for transmission/reception
    1: 16-bit data frame format is selected for transmission/reception
**/
#define SPI_CR1_DFF(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_DFF_SHIFT)) & SPI_CR1_DFF_MASK)

/**************/
#define SPI_CR1_BIDIOE_MASK  (0x00004000)
#define SPI_CR1_BIDIOE_SHIFT  (14u)
/*! Bit 14 BIDIOE: Output enable in bidirectional mode
    This bit combined with the BIDIMODE bit selects the direction of transfer in bidirectional
    mode
    0: Output disabled (receive-only mode)
    1: Output enabled (transmit-only mode)
    Note: This bit is not used in I2S mode.
    In master mode, the MOSI pin is used while the MISO pin is used in slave mode.
**/
#define SPI_CR1_BIDIOE(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_BIDIOE_SHIFT)) & SPI_CR1_BIDIOE_MASK)

/**************/

#define SPI_CR1_BIDIMODE_MASK  (0x00008000)
#define SPI_CR1_BIDIMODE_SHIFT  (15u)
/*! Bit 15 BIDIMODE: Bidirectional data mode enable
    This bit enables half-duplex communication using common single bidirectional data line.
    Keep RXONLY bit clear when bidirectional mode is active.
    0: 2-line unidirectional data mode selected
    1: 1-line bidirectional data mode selected
**/
#define SPI_CR1_BIDIMODE(x)  (((uint32_t)(((uint32_t)(x))<< SPI_CR1_BIDIMODE_SHIFT)) & SPI_CR1_BIDIMODE_MASK)

/**************/

/*******************************************************************************
 * Struct, enum, union
 ******************************************************************************/

typedef enum
{
    SPI_SLAVE_MODE = 0u,
    SPI_MASTER_MODE = 1u,
}SPI_DeviceMode_t;

typedef enum
{
    SPI_Full_Duplex_MODE = 0u,
    SPI_Simplex_Tx_MODE= 1u,
    SPI_Simplex_Rx_MODE= 2u,
}SPI_BusConfig_t;

typedef enum
{
    SPI_DIV_2 = 0u,
    SPI_DIV_4 = 0x08,
    SPI_DIV_8 = 0x10,
    SPI_DIV_16 = 0x18,
    SPI_DIV_32 = 0x20,
    SPI_DIV_64 = 0x28,
    SPI_DIV_128 = 0x30,
    SPI_DIV_256 = 0x38,
}SPI_SclkSpeed_t;



typedef enum
{
    SPI_CPHA_LOW = 0u,
    SPI_CPHA_HIGH = 1u,
}SPI_Clock_phase_t;


typedef enum
{
    SPI_CPOL_LOW = 0u,
    SPI_CPOL_HIGH = 1u,
}SPI_Clock_polarity_t;

typedef enum
{
    SPI_FIRSTBIT_MSB = 0u,
    SPI_FIRSTBIT_LSB = 1u,
}SPI_FirstBit_t;

/*
 *   @SPI_DS
 * Buffer là 4 bit -> 16 bit
 * CR2 -> DS
 */
typedef enum
{
    SPI_DS_8BITS = 0u,
    SPI_DS_16BITS = 1u,
    SPI_DS_NONE = 2u,
}SPI_DataSize_t;

typedef enum
{
    SPI_SSM_DISABLE = 0U,
    SPI_SSM_ENABLE = 1U,
}SPI_SSM_t;
/**
 * @brief Tạo 1 cấu trúc để lựa chọn các chức năng cho SPI
 *
 */
typedef struct
{
    SPI_DeviceMode_t SPI_DeviceMode;
    SPI_BusConfig_t SPI_BusConfig;
    SPI_SclkSpeed_t SPI_SclkSpeed;
    SPI_Clock_phase_t SPI_CPHA;
    SPI_Clock_polarity_t SPI_CPOL;
    SPI_FirstBit_t SPI_FirstBit;
    SPI_DataSize_t SPI_DataSize;
    // SPI_SSM_t SPI_SSM; always = 1

} SPI_Config_type;




typedef struct
{
    /* data */
    SPI_RegDef_t *pSPIx;     // Chứa địa chỉ của SPIx
    SPI_Config_type SPI_Config; // Chứa config settings cuar SPI

    // Các biến dùng trong ngắt SPI
    uint8_t *pTxBuffer; // Dùng lưu địa chỉ của  Tx buffer < Địa chỉ nơi lưu trữ dữ liệu muốn truyền đi >
    uint8_t *pRxBuffer; // Dùng lưu địa chỉ của Rx buffer < Địa chỉ nơi lưu trữ dữ liệu muốn nhận  về >
    uint32_t TxLen;     // Độ dài TxBuffer
    uint32_t RxLen;     // Độ dài  Rx buffer
    uint8_t TxState;
    uint8_t RxState;
} SPI_Handle_t;

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


LT_status_t SPI_Init(SPI_Handle_t *pSPIhandle);


void SPI_Deinit(SPI_RegDef_t *pSPIx);


LT_status_t SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
// void SPi_ReciveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len); // Tương tự API gửi

// uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t len);
// uint8_t SPi_ReciveDataIT(SPI_Handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t len);



// void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);      // khởi tạo ngắt
// void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority); // Xét mức ưu tiên ngắt
// void SPI_IRQHandling(SPI_Handle_t *pSPIhandle);                      // Trình xử lý ngắt


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_SSIControlBit(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/** NOTE
 * In this project, Because I set SSM = 1, this function isn't used yet
*/
void SPI_SSOEControlBit(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

// void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

// void SPI_CloseTransmission(SPI_Handle_t *pSPIhandle);

// void SPI_CloseReception(SPI_Handle_t *pSPIhandle);

// /**
//  * @brief Application callback
//  *
//  */
// void SPI_ApplicationCallback(SPI_Handle_t *pSPIhandle, uint8_t AppEv);

/*******************************************************************************
 * Static Functions
 ******************************************************************************/

/*******************************************************************************
 * Global Funtions
 ******************************************************************************/

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
