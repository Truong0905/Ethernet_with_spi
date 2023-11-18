/*
 * stm32f446xx_spi _driver.c
 *
 *  Created on: Apr 23, 2022
 *      Author: Truong
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "stm32f446xx.h"
#include "stm32f446xx_spi_driver.h"
/*******************************************************************************
 * Gloabal variables
 ******************************************************************************/
 static SPI_Config_type  s_SPI_Config = {.SPI_DataSize = SPI_DS_NONE};

/*******************************************************************************
 * Local variables
 ******************************************************************************/

/*******************************************************************************
 * Static Function Prototypes
 ******************************************************************************/
// Các hàm chỉ dùng trong mục này , ko cần gọi ra chương trình chính , để phucj vụ các trình xử lý ngắt
// static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIhandle);
// static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIhandle);
// static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIhandle);
static void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*******************************************************************************
 * Global Funtions Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Static Functions
 ******************************************************************************/
static void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

     if (EnorDi == ENABLE)
     {
          if (pSPIx == SPI1)
          {
               SPI1_PCLK_EN();
          }
          else if (pSPIx == SPI2)
          {
               SPI2_PCLK_EN();
          }
          else if (pSPIx == SPI3)
          {
               SPI3_PCLK_EN();
          }
          else if (pSPIx == SPI4)
          {
               SPI4_PCLK_EN();
          }
     }
     else
     {
          if (pSPIx == SPI1)
          {
               SPI1_PCLK_DI();
          }
          else if (pSPIx == SPI2)
          {
               SPI2_PCLK_DI();
          }
          else if (pSPIx == SPI3)
          {
               SPI3_PCLK_DI();
          }
          else if (pSPIx == SPI4)
          {
               SPI4_PCLK_DI();
          }
     }
}


// static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIhandle)
// {
//      uint16_t tempreg = 0;
//      tempreg = pSPIhandle->pSPIx->CR1;
//      tempreg = tempreg >> SPI_CR1_DFF;
//      tempreg &= 0x1;
//      if (tempreg == SPI_DS_8BITS)
//      // 8 bit
//      {
//           pSPIhandle->pSPIx->DR = *pSPIhandle->pTxBuffer;
//           pSPIhandle->TxLen--;
//           pSPIhandle->pTxBuffer++;
//      }
//      else if (SPI_DS_16BITS == tempreg)
//      {
//           pSPIhandle->pSPIx->DR = *((uint16_t *)pSPIhandle->pTxBuffer);
//           pSPIhandle->TxLen = pSPIhandle->TxLen - 2;
//           (uint16_t *)pSPIhandle->pTxBuffer++;
//      }
//      if (!pSPIhandle->TxLen)
//      {
//           // Nếu Txlen bằng 0 , đã truyền xong và đóng lại
//           SPI_CloseTransmission(pSPIhandle);
//           SPI_ApplicationCallback(pSPIhandle, SPI_EVENT_TX_CMPLT);
//      }
// }

// static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIhandle)
// {
//      uint16_t tempreg = 0;
//      tempreg = pSPIhandle->pSPIx->CR1;
//      tempreg = tempreg >> SPI_CR1_DFF;
//      tempreg &= 0x1;
//      if (tempreg == SPI_DS_8BITS)
//      {
//           *pSPIhandle->pRxBuffer = pSPIhandle->pSPIx->DR;
//           pSPIhandle->RxLen--;
//           pSPIhandle->pRxBuffer++;
//      }
//      else if (tempreg == SPI_DS_16BITS)
//      {
//           *((uint16_t *)pSPIhandle->pRxBuffer) = pSPIhandle->pSPIx->DR;
//           pSPIhandle->RxLen = pSPIhandle->RxLen - 2;
//           (uint16_t *)pSPIhandle->pRxBuffer++;
//      }

//      if (!pSPIhandle->RxLen)
//      {
//           // Nếu Rxlen bằng 0 , đã nhận xong và đóng lại
//           SPI_CloseReception(pSPIhandle);
//           SPI_ApplicationCallback(pSPIhandle, SPI_EVENT_RX_CMPLT);
//      }
// }

// static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIhandle)
// {
//      uint8_t temp; // Chỉ dùng để lúc debug thấy lỗi
//      // 1. Clear ovr flag
//      if (pSPIhandle->TxState != SPI_BUSY_IN_TX)
//      {
//           SPI_ClearOVRFlag(pSPIhandle->pSPIx);
//      }
//      (void)temp;
//      SPI_ApplicationCallback(pSPIhandle, SPI_EVENT_OVR_ERR);
// }

/*******************************************************************************
 * Global Funtions
 ******************************************************************************/
LT_status_t SPI_Init(SPI_Handle_t *pSPIhandle)
{
     LT_status_t retVal = E_OK;

     (void) SPI_PeriClockControl(pSPIhandle->pSPIx, ENABLE);
     (void) SPI_PeripheralControl(pSPIhandle->pSPIx, DISABLE);

     /* 1. Baud rate control */
     MODIFY_REG(pSPIhandle->pSPIx->CR1,SPI_CR1_BR_MASK,  pSPIhandle->SPI_Config.SPI_SclkSpeed);

     /* 2. SPI_CPOL */
     switch (pSPIhandle->SPI_Config.SPI_CPOL)
     {
     case SPI_CPOL_LOW:
          CLEAR_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_CPOL_MASK);
          break;
     case SPI_CPOL_HIGH:
          SET_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_CPOL_MASK);
          break;
     default:
          return E_INVALID_PARAMETER;
          break;
     }

     /* 3. SPI_CPHA */
     switch (pSPIhandle->SPI_Config.SPI_CPHA)
     {
     case SPI_CPHA_LOW:
          CLEAR_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_CPHA_MASK);
          break;
     case SPI_CPHA_HIGH:
          SET_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_CPHA_MASK);
          break;
     default:
          return E_INVALID_PARAMETER;
          break;
     }

     /* 4. Select mode */
     switch (pSPIhandle->SPI_Config.SPI_DeviceMode)
     {
     case SPI_SLAVE_MODE:
          CLEAR_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_MSTR_MASK);
          break;
     case SPI_MASTER_MODE:
          SET_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_MSTR_MASK);
          /* 6.  Software slave management */
          // tempreg |= SPI_CR1_SSM(0x01 & pSPIhandle->SPI_Config.SPI_SSM);
          pSPIhandle->pSPIx->CR1 |= SPI_CR1_SSM_MASK;
          pSPIhandle->pSPIx->CR1 |= SPI_CR1_SSI_MASK;
          break;
     default:
          return E_INVALID_PARAMETER;
     }

     /* 5. transmission mode*/
     switch (pSPIhandle->SPI_Config.SPI_BusConfig)
     {
          case SPI_Full_Duplex_MODE:
          case SPI_Simplex_Tx_MODE:
               CLEAR_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_BIDIMODE_MASK);
               break;
          case SPI_Simplex_Rx_MODE:
               SET_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_RXONLY_MASK);
               break;
          default:
               return E_INVALID_PARAMETER;
     }
     s_SPI_Config.SPI_BusConfig =pSPIhandle->SPI_Config.SPI_BusConfig;

     /* 7. FirstBit */
     switch (pSPIhandle->SPI_Config.SPI_FirstBit)
     {
     case SPI_FIRSTBIT_MSB:
          CLEAR_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_LSBFIRST_MASK);
          break;
     case SPI_FIRSTBIT_LSB:
          SET_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_LSBFIRST_MASK);
          break;
     default:
          return E_INVALID_PARAMETER;
          break;
     }
     s_SPI_Config.SPI_FirstBit =pSPIhandle->SPI_Config.SPI_FirstBit;

     /* 8 . Data frame format */
     switch (pSPIhandle->SPI_Config.SPI_DataSize)
     {
     case SPI_DS_8BITS:
          CLEAR_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_DFF_MASK);
          break;
     case SPI_DS_16BITS:
          SET_BIT(pSPIhandle->pSPIx->CR1,SPI_CR1_DFF_MASK);
          break;
     default:
          return E_INVALID_PARAMETER;
          break;
     }
     s_SPI_Config.SPI_DataSize =pSPIhandle->SPI_Config.SPI_DataSize;

     return retVal;
}

void SPI_Deinit(SPI_RegDef_t *pSPIx)
{
     if (pSPIx == SPI1)
     {
          SPI1_REG_RESET();
     }
     else if (pSPIx == SPI2)
     {
          SPI2_REG_RESET();
     }
     else if (pSPIx == SPI3)
     {
          SPI3_REG_RESET();
     }
     else if (pSPIx == SPI4)
     {
          SPI4_REG_RESET();
     }
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
     if (EnOrDi == ENABLE)
     {
          SET_BIT(pSPIx->CR1, SPI_CR1_SPE_MASK);
     }
     else
     {
          CLEAR_BIT(pSPIx->CR1, SPI_CR1_SPE_MASK);
     }
}

void SPI_SSIControlBit(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
     if (EnOrDi == ENABLE)
     {
          pSPIx->CR1 |= SPI_CR1_SSI_MASK;
     }
     else
     {
          pSPIx->CR1 &= ~SPI_CR1_SSI_MASK;
     }
}

/** NOTE
 * In this project, Because I set SSM = 1, this function isn't used yet
*/
void SPI_SSOEControlBit(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
     if (EnOrDi == ENABLE)
     {
          pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
     }
     else
     {
          pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
     }
}


/**
 * @brief  Kiểm tra xem Flag trong SPI_SR đã bật chưa
 *
 * @param pSPIx địa chỉ SPIx
 * @param FlagName Flag cần kiểm tra
 * @return uint8_t trả về 0 nếu chưa bật và 1 nếu đã bật
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
     if (pSPIx->SR & FlagName)
     {
          return FLAG_SET;
     }
     return FLAG_RESET;
}

/**
 * @brief  Truyền dữ liệu SPI
 *
 * @param pSPIx Địa chỉ SPIx
 * @param pTxBuffer con trỏ Data
 * @param len Kích thước dữ liệu truyền đi
 * @note  Blocking API bởi vì hàm sẽ đợi cho đến khi tất cả các byte được truyền đi ( có 2 vòng while )
 */
LT_status_t SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
     LT_status_t retVal = E_OK;
     uint8_t check = 1;

     if (!(READ_BIT(pSPIx->CR1,SPI_CR1_SPE_MASK)))
     {
          (void) SPI_PeripheralControl(pSPIx, ENABLE);
     }


     while ((len > 0) && (1 == check))
     {
          // 1. Đợi cho tới khi cờ TXE = 1 có nghĩa là Tx buffer đã trống
          while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

          switch (s_SPI_Config.SPI_DataSize)
          {
          case SPI_DS_8BITS:
          {
               pSPIx->DR = *pTxBuffer;
               len--;
               pTxBuffer++;
               break;
          }
          case SPI_DS_16BITS:
          {
               pSPIx->DR = *((uint16_t *)pTxBuffer);
               len = len - 2;
               (uint16_t *)pTxBuffer++;
               break;
          }
          default:
               retVal = E_NOT_INITIALIZED;
               check = 0;
               break;
          }
     }
     while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

     (void) SPI_PeripheralControl(pSPIx, DISABLE);

     return retVal;
}

// void SPi_ReciveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
// {
//      while (len > 0)
//      {
//           // 1. Đợi cho tới khi cờ RXE = 1 có nghĩa là Rx buffer đã đầy
//           while (SPI_GetFlagStatus(pSPIx, SPI_RXE_FLAG) == FLAG_RESET);

//           uint16_t tempreg = 0;
//           tempreg = pSPIx->CR1;
//           tempreg = tempreg >> SPI_CR1_DFF;
//           tempreg &= 0x1;
//           if (tempreg == SPI_DS_8BITS)
//           {
//                *pRxBuffer = pSPIx->DR;
//                len--;
//                pRxBuffer++;
//           }
//           else if (SPI_DS_16BITS == tempreg)
//           {
//                *((uint16_t *)pRxBuffer) = pSPIx->DR;
//                len--;
//                len--;
//                (uint16_t *)pRxBuffer++;
//           }
//      }
// }

// void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) // khởi tạo ngắt
// {
//      // config NVIC register

//      if (EnorDi == ENABLE) // Các thanh ghi này chỉ có thể enable ko thể disable
//      {
//           if (IRQNumber <= 31) // 0-31
//           {
//                // program ISER0 register
//                *NVIC_ISER0 |= (1 << IRQNumber);
//           }
//           else if (IRQNumber > 31 && IRQNumber < 64) // 32-63
//           {
//                // program ISER1 register
//                *NVIC_ISER1 |= (1 << IRQNumber % 32); // Giả sử  45 %32 = 13 => vị trí thứ 13 trong thanh ghi  ( bắt daauf từ 0)
//           }
//           else if (IRQNumber >= 64 && IRQNumber < 96) // 64 -95
//           {
//                // program ISER2 register
//                *NVIC_ISER2 |= (1 << IRQNumber % 64); // Giả sử  70 %64 = 6 => vị trí thứ 6 trong thanh ghi  ( Bắt đầu từ 0)
//           }
//      }
//      else // Các thanh ghi này chỉ có thể disable  ko thể enable
//      {
//           if (IRQNumber <= 31) // 0-31
//           {
//                // program ICER0 register
//                *NVIC_ICER0 |= (1 << IRQNumber);
//           }
//           else if (IRQNumber > 31 && IRQNumber < 64) // 32-63
//           {
//                // program ICER1 register
//                *NVIC_ICER1 |= (1 << IRQNumber % 32);
//           }
//           else if (IRQNumber >= 64 && IRQNumber < 96) // 64 -95
//           {
//                // program ICER2 register
//                *NVIC_ICER2 |= (1 << IRQNumber % 64);
//           }
//      }
// }

// void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) // Xét mức ưu tiên ngắt
// {

//      // 1. Tìm IPR resister (Interrupt Priority Registers)
//      uint8_t iprx = IRQNumber / 4;         // XÁc định xem ngắt nằm ở thanh ghi nào từ 0 - 59
//      uint8_t iprx_section = IRQNumber % 4; // Xác định xme ngắt nằm ở section nào ( mỗi thanh ghi có 4 section , 8 bit cho 1 section => 1 thanh ghi kiểm soát mức ưu tiên cho 4 ngắt)
//      uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
//      *(NVIC_PR_BASE_ADDR + iprx) = (IRQPriority << shift_amount);

//      // Giả sử ngắt nằm ở thanh ghi thứ 1 => địa chỉ là Pr_base_Addr + 1*4 = pr_base_addr 0x04   (Do đó nhân 4 vì mỗi lần ofset là 0x04)
//      // giả sử nằm ở section 1 => bắt đầu từ bit thứ 8 => lùi sang trái 8 lần . do đó phải nhân với 8
//      // Nhuwng do chỉ có 4 bit cao của mỗi section mới có tác dụng nên ta phải lùi sang trái 4 bit . vD mức ưu tiên 0000_0001 => 0001_0000
// }

// /**
//  * @brief Dùng lưu trữ địa chỉ  TxBuffer và thông tin Len và cho phép kích hoạt ngắt ( thông qua TXTIE) . Ko dùng để send
//  *
//  * @param pSPIhandle
//  * @param pTxBuffer
//  * @param len
//  * @return uint8_t  Thông báo trạng thái SPI_trans có bận không
//  */
// uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIhandle, uint8_t *pTxBuffer, uint32_t len)
// {
//      uint8_t state = pSPIhandle->TxState;
//      if (state != SPI_BUSY_IN_TX)
//      {
//           // 1. Save TxBuffer address and len information in som global variables
//           pSPIhandle->pTxBuffer = pTxBuffer;
//           pSPIhandle->TxLen = len;
//           // 2. Thông báo SPI này đang bận truyền và ko có code nào khác được tác động cho đến khi truyền xong
//           pSPIhandle->TxState = SPI_BUSY_IN_TX;
//           // 3. Bật bit TXTEIE trong SPI->CR2 là bit cho phép xảy ra ngắt khi cờ TXE ( SPI-> SR) được set có nghĩa Txbuffer đang trống vã sẵn sàng nhận dữ liệu để truyền đi
//           pSPIhandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
//      }

//      return state;
// }

// uint8_t SPi_ReciveDataIT(SPI_Handle_t *pSPIhandle, uint8_t *pRxBuffer, uint32_t len)
// {
//      uint8_t state = pSPIhandle->RxState;
//      if (state != SPI_BUSY_IN_RX)
//      {
//           // 1. Save RxBuffer address and len information in som global variables
//           pSPIhandle->pRxBuffer = pRxBuffer;
//           pSPIhandle->RxLen = len;
//           // 2. Thông báo SPI này đang bận truyền và ko có code nào khác được tác động cho đến khi nhận xong
//           pSPIhandle->RxState = SPI_BUSY_IN_RX;
//           // 3. Bật bit RXNEIE trong SPI->CR2 là bit cho phép xảy ra ngắt khi cờ RXE ( SPI-> SR) được set có nghĩa Rxbuffer đã đầy  vã sẵn sàng truyền dữ liệu đi lưu trũ
//           pSPIhandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
//      }

//      return state;
// }

// void SPI_IRQHandling(SPI_Handle_t *pSPIhandle)
// {
//      uint8_t temp1, temp2;
//      // 1 . Check TXE
//      temp1 = pSPIhandle->pSPIx->SR & (1 << SPI_SR_TXE);
//      temp2 = pSPIhandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
//      if (temp1 && temp2)
//      {
//           // handler TXE
//           spi_txe_interrupt_handle(pSPIhandle);
//      }

//      // 2 . Check RXNE
//      temp1 = pSPIhandle->pSPIx->SR & (1 << SPI_SR_RXNE);
//      temp2 = pSPIhandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
//      if (temp1 && temp2)
//      {
//           // handler RXNE
//           spi_rxne_interrupt_handle(pSPIhandle);
//      }
//      // 3 . Check CRC
//      // 4 . Check TI farme
//      // 5. Overrun erorr - Xảy ra khi cờ RXNE được set nhưng vẫn có data từ ngoài truyền vào => loại bỏ dữ liệu mới đến
//      //  Phải gửi lại data vì dât kia đã bị loại bỏ chưa kịp đọc
//      temp1 = pSPIhandle->pSPIx->SR & (1 << SPI_SR_OVR);
//      temp2 = pSPIhandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
//      if (temp1 && temp2)
//      {
//           // handler ovr erorr
//           spi_ovr_err_interrupt_handle(pSPIhandle);
//      }
// }

// void SPI_CloseTransmission(SPI_Handle_t *pSPIhandle)
// {
//      // Nếu Txlen bằng 0 , đã truyền xong và đóng lại
//      pSPIhandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); // nhằm ngăn ngừa  interrupt từ txe flag
//      pSPIhandle->pTxBuffer = NULL;
//      pSPIhandle->TxLen = 0;
//      pSPIhandle->TxState = SPI_READY;
//      SPI_ApplicationCallback(pSPIhandle, SPI_EVENT_TX_CMPLT);
// }

// void SPI_CloseReception(SPI_Handle_t *pSPIhandle)
// {
//      // Nếu Rxlen bằng 0 , đã nhận xong và đóng lại
//      pSPIhandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); // nhằm ngăn ngừa  interrupt từ rxe flag
//      pSPIhandle->pRxBuffer = NULL;
//      pSPIhandle->RxLen = 0;
//      pSPIhandle->RxState = SPI_READY;
//      SPI_ApplicationCallback(pSPIhandle, SPI_EVENT_RX_CMPLT);
// }

// void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
// {
//      uint8_t temp;
//      temp = pSPIx->DR;
//      temp = pSPIx->SR;
//      (void)temp;
// }

// __weak void SPI_ApplicationCallback(SPI_Handle_t *pSPIhandle, uint8_t AppEv)
// {
//      while (1)
//      {
//           /* code */
//      }

// }
