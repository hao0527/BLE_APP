/**************************************************************************//**
 * @file     spi.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/23 15:33 $
 * @brief    PN102 series SPI driver header file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* SPI_FIFO constants definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
/*Transmit FIFO Threshold. Controls the level of entries (or below) at which the
transmit FIFO controller triggers an interrupt*/
/*Receive FIFO Threshold. Controls the level of entries (or above) at
which the receive FIFO controller triggers an interrupt*/
enum SPI_RxFifoThreshold
{
    Spi_fifo_rx_thre_1_chars = 0x0,
    Spi_fifo_rx_thre_2_chars = 0x1,
    Spi_fifo_rx_thre_3_chars = 0x2,
    Spi_fifo_rx_thre_4_chars = 0x3,
    Spi_fifo_rx_thre_5_chars = 0x4,
    Spi_fifo_rx_thre_6_chars = 0x5,
    Spi_fifo_rx_thre_7_chars = 0x6,
    Spi_fifo_rx_thre_8_chars = 0x7,
    Spi_fifo_rx_thre_9_chars = 0x8,
    Spi_fifo_rx_thre_10_chars = 0x9,
    Spi_fifo_rx_thre_11_chars = 0xa,
    Spi_fifo_rx_thre_12_chars = 0xb,
    Spi_fifo_rx_thre_13_chars = 0xc,
    Spi_fifo_rx_thre_14_chars = 0xd,
    Spi_fifo_rx_thre_15_chars = 0xe,
    Spi_fifo_rx_thre_16_chars = 0xf,

};

enum SPI_TxFifoThreshold
{
    Spi_fifo_tx_thre_0_chars = 0x0,
    Spi_fifo_tx_thre_1_chars = 0x1,
    Spi_fifo_tx_thre_2_chars = 0x2,
    Spi_fifo_tx_thre_3_chars = 0x3,
    Spi_fifo_tx_thre_4_chars = 0x4,
    Spi_fifo_tx_thre_5_chars = 0x5,
    Spi_fifo_tx_thre_6_chars = 0x6,
    Spi_fifo_tx_thre_7_chars = 0x7,
    Spi_fifo_tx_thre_8_chars = 0x8,
    Spi_fifo_tx_thre_9_chars = 0x9,
    Spi_fifo_tx_thre_10_chars = 0xa,
    Spi_fifo_tx_thre_11_chars = 0xb,
    Spi_fifo_tx_thre_12_chars = 0xc,
    Spi_fifo_tx_thre_13_chars = 0xd,
    Spi_fifo_tx_thre_14_chars = 0xe,
    Spi_fifo_tx_thre_15_chars = 0xf,

};

typedef enum SPI_Irq
{
    Spi_irq_tx_empty         = 0x01,         //Transmit FIFO Empty Interrupt
    Spi_irq_tx_over          = 0x02,         //Transmit FIFO Overflow Interrupt
    Spi_irq_rx_under         = 0x04,         //Receive FIFO Underflow Interrupt
    Spi_irq_rx_over          = 0x08,         //Receive FIFO Overflow Interrupt
    Spi_irq_rx_full          = 0x10,         //Receive FIFO Full Interrupt
    Spi_irq_multi_mst        = 0x20,         //Multi-Master Contention Interrupt
    Spi_irq_all              = 0x3f          //All interrupts
}SPI_IrqDef;


typedef enum SPI_Role
{
    Spi_role_master = 0x0,
    Spi_role_slave = 0x1

}SPI_RoleDef;

/*Transfer Mode. Selects the mode of transfer for serial communication. This field
does not affect the transfer duplicity. Only indicates whether the receive or transmit
data are valid.*/
typedef enum SPI_TransferMode
{
    Spi_dir_full_duplex = 0x0,
    Spi_dir_tx_only = 0x1,
    Spi_dir_rx_only = 0x2,
    Spi_dir_read_eeprom = 0x3

}SPI_TransferModeDef;

/*Data Frame Size in 32-bit mode*/
typedef enum SPI_DataFrameSize
{
    Spi_data_frame_8b = 0x07,
    Spi_data_frame_16b = 0x0f,

    Spi_data_frame_32b = 0x1f
}SPI_DataFrameSizeDef;


typedef enum SPI_BaudRateDiv
{
    Spi_baudrate_div2 = 0x02,
    Spi_baudrate_div4 = 0x04,
    Spi_baudrate_div6 = 0x06,
    Spi_baudrate_div8 = 0x08,
    Spi_baudrate_div16 = 0x10,

    Spi_baudrate_div32 = 0x20,
    Spi_baudrate_div48 = 0x30,
    Spi_baudrate_div96 = 0x60

}SPI_BaudRateDivDef;


/*Serial Clock Polarity, Used to select the polarity of the inactive serial clock, which is held inactive when
the DW_apb_ssi master is not actively transferring data on the serial bus.*/
typedef enum SPI_ClockPol
{
    Spi_pol_low = 0x0,
    Spi_pol_high = 0x1
}SPI_ClockPolDef;

typedef enum SPI_ClockPhase
{
    Spi_clock_phase_1edge = 0x0,
    Spi_clock_phase_2edge = 0x1

}SPI_ClockPhaseDef;
/**
  * @brief  SPI Init Structure definition
  */
typedef struct
{
  SPI_RoleDef SPI_role;                             /*!< Specifies the SPI operating mode */
  SPI_TransferModeDef SPI_transferMode;              /*!< Specifies the SPI unidirectional or bidirectional data mode. */

  SPI_DataFrameSizeDef SPI_dataFrameSize;           /*!< Specifies the SPI data size */

  SPI_ClockPolDef SPI_CPOL;                         /*!< Specifies the serial clock steady state */
  SPI_ClockPhaseDef SPI_CPHA;                       /*!< Specifies the clock active edge for the bit capture */

  SPI_BaudRateDivDef SPI_baudRate;                  /*!< Specifies the Baud Rate used to configure the transmit and receive SCK clock. */
}SPI_InitTypeDef;




/* Control functions ********************************************************/

void SPI_EnableSpi(SPI_T* SPIx);
void SPI_DisableSpi(SPI_T* SPIx);
bool SPI_IsSpiEnabled(SPI_T* SPIx);

void SPI_Init(SPI_T* SPIx, SPI_InitTypeDef* SPI_InitStruct);

/* FIFO functions ********************************************************/
uint8_t SPI_GetTxFifoLevel(SPI_T* SPIx);
uint8_t SPI_GetRxFifoLevel(SPI_T* SPIx);

void SPI_SetTxTrigger(SPI_T *SPIx, uint32_t level);
uint32_t SPI_GetTxTrigger(SPI_T *SPIx);

void SPI_SetRxTrigger(SPI_T *SPIx, uint32_t level);
uint32_t SPI_GetRxTrigger(SPI_T *SPIx);

/* Tx/Rx data functions ********************************************************/
void SPI_SendData(SPI_T* SPIx, uint32_t Data);
uint32_t SPI_ReceiveData(SPI_T* SPIx);

/* DMA functions ********************************************************/
void SPI_SetDmaTxLevel(SPI_T* SPIx, uint32_t level);
uint32_t SPI_GetDmaTxLevel(SPI_T* SPIx);

void SPI_SetDmaRxLevel(SPI_T* SPIx, uint32_t level);
uint32_t SPI_GetDmaRxLevel(SPI_T* SPIx);

void SPI_EnableDmaTx(SPI_T* SPIx);
void SPI_DisableDmaTx(SPI_T* SPIx);
bool SPI_IsDmaTxEnabled(SPI_T* SPIx);

void SPI_EnableDmaRx(SPI_T* SPIx);
void SPI_DisableDmaRx(SPI_T* SPIx);
bool SPI_IsDmaRxEnabled(SPI_T* SPIx);

/* Interrupts and flags management functions **********************************/

//Irq Mask
void SPI_EnableIrq(SPI_T* SPIx, SPI_IrqDef irq);
void SPI_DisableIrq(SPI_T* SPIx, SPI_IrqDef irq);
bool SPI_IsIrqEnabled(SPI_T* SPIx, SPI_IrqDef irq);

uint8_t SPI_GetIrqMasked(SPI_T * SPIx);

void SPI_ClearIrq(SPI_T * SPIx, SPI_IrqDef irq);
bool SPI_IsIrqActive(SPI_T * SPIx, SPI_IrqDef irq);

bool SPI_IsBusy(SPI_T* SPIx);

//Fifo Status
bool SPI_IsTxFifoEmpty(SPI_T* SPIx);
bool SPI_IsTxFifoFull(SPI_T* SPIx);
bool SPI_IsRxFifoEmpty(SPI_T* SPIx);
bool SPI_IsRxFifoFull(SPI_T* SPIx);


#ifdef __cplusplus
}
#endif

#endif //__SPI_H__

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
