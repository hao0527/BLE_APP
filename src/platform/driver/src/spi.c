/**************************************************************************//**
 * @file     SPIx.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/02/23  1:47p $
 * @brief    PN102 series SPI driver source file
 *
 * @note
 * Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/
#include "PN102Series.h"
/** @addtogroup PN102_Device_Driver PN102 Device Driver
  @{
*/

/** @addtogroup PN102_SPI_Driver SPI Driver
  @{
*/

void SPI_EnableSpi(SPI_T* SPIx)
{
    uint32_t tmpreg = 0;

    tmpreg = SPIx->SPIER;
    tmpreg |= SPI_SPIEN_EN_Msk;

    SPIx->SPIER = tmpreg;


}
void SPI_DisableSpi(SPI_T* SPIx)
{
    uint32_t tmpreg = 0;

    tmpreg = SPIx->SPIER;
    tmpreg &= ~SPI_SPIEN_EN_Msk;

    SPIx->SPIER = tmpreg;

}

bool SPI_IsSpiEnabled(SPI_T* SPIx)
{
    uint32_t tmpreg = 0;

    tmpreg = SPIx->SPIER;

    if(tmpreg & SPI_SPIEN_EN_Msk)
        return true;
    else
        return false;
}

void SPI_Init(SPI_T* SPIx, SPI_InitTypeDef* SPI_InitStruct)
{
    uint32_t tmpreg = 0;

    /*If the DW_apb_ssi is enabled, disable it by writing 0 to the SSI Enable register*/
    if(SPI_IsSpiEnabled(SPIx))
        SPI_DisableSpi(SPIx);

    //Set up the SPI control registers for the transfer; these registers can be set in any order.
    /*Write Control Register 0 (CTRLR0).*/
    tmpreg = SPIx->CTRL0;

    tmpreg &= ~(SPI_CTL0_SCPHA_Msk | SPI_CTL0_SCPOL_Msk | SPI_CTL0_TMOD_Msk | SPI_CTL0_DFS32_Msk);
    tmpreg |= (
               (SPI_InitStruct->SPI_CPHA          << SPI_CTL0_SCPHA_Pos) & SPI_CTL0_SCPHA_Msk  |    \
               (SPI_InitStruct->SPI_CPOL          << SPI_CTL0_SCPOL_Pos) & SPI_CTL0_SCPOL_Msk  |    \
               (SPI_InitStruct->SPI_transferMode   << SPI_CTL0_TMOD_Pos ) & SPI_CTL0_TMOD_Msk  |    \
               (SPI_InitStruct->SPI_dataFrameSize  << SPI_CTL0_DFS32_Pos) & SPI_CTL0_DFS32_Msk      \
              );
    SPIx->CTRL0 = tmpreg;

    /*Write the Baud Rate Select Register (BAUDR) to set the baud rate for the transfer.*/
    tmpreg = SPIx->BAUDR;
    tmpreg = (SPI_InitStruct->SPI_baudRate << SPI_BAUDR_SCKDV_Pos) & SPI_BAUDR_SCKDV_Msk;
    SPIx->BAUDR = tmpreg;

    if(SPI_InitStruct->SPI_role == Spi_role_master)
    {
        SPIx->SER |= SPI_SER_EN_Msk;
    }
}

/* FIFO functions ********************************************************/
uint8_t SPI_GetTxFifoLevel(SPI_T* SPIx)
{
    return (uint8_t)((SPIx->TXFLR & SPI_TXFLR_TFL_Msk)>>SPI_TXFLR_TFL_Pos);
}

uint8_t SPI_GetRxFifoLevel(SPI_T* SPIx)
{
    return (uint8_t)((SPIx->RXFLR & SPI_RXFLR_RFL_Msk)>>SPI_RXFLR_RFL_Pos);
}

void SPI_SetTxTrigger(SPI_T *SPIx, uint32_t level)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->TXFTLR;

    tmpreg = (level << SPI_TXFTLR_TFT_Pos ) & SPI_TXFTLR_TFT_Msk;

    SPIx->TXFTLR = tmpreg;

}
uint32_t SPI_GetTxTrigger(SPI_T *SPIx)
{
    uint32_t tmpreg,level = 0x00;

    tmpreg = SPIx->TXFTLR;
    level = (tmpreg & SPI_TXFTLR_TFT_Msk) >> SPI_TXFTLR_TFT_Pos;

    return (uint8_t)level;

}

void SPI_SetRxTrigger(SPI_T *SPIx, uint32_t level)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->RXFTLR;

    tmpreg = (level << SPI_RXFTLR_RFT_Pos) & SPI_RXFTLR_RFT_Msk ;

    SPIx->RXFTLR = tmpreg;


}
uint32_t SPI_GetRxTrigger(SPI_T *SPIx)
{
    uint32_t tmpreg,level = 0x00;

    tmpreg = SPIx->RXFTLR;
    level = (tmpreg & SPI_RXFTLR_RFT_Msk) >> SPI_RXFTLR_RFT_Pos;

    return level;

}

void SPI_SetDmaTxLevel(SPI_T* SPIx, uint32_t level)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMATDLR;

    tmpreg = (level << SPI_DMATDLR_DMATDL_Pos) & SPI_DMATDLR_DMATDL_Msk ;

    SPIx->DMATDLR = tmpreg;

}
uint32_t SPI_GetDmaTxLevel(SPI_T* SPIx)
{
    uint32_t tmpreg ,level;

    tmpreg = SPIx->DMATDLR;

    level = (tmpreg & SPI_DMATDLR_DMATDL_Msk) >> SPI_DMATDLR_DMATDL_Pos;

    return level;

}

void SPI_SetDmaRxLevel(SPI_T* SPIx, uint32_t level)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMARDLR;

    tmpreg = (level << SPI_DMARDLR_DMARDL_Pos) & SPI_DMARDLR_DMARDL_Msk ;

    SPIx->DMARDLR = tmpreg;

}
uint32_t SPI_GetDmaRxLevel(SPI_T* SPIx)
{
    uint32_t tmpreg ,level;

    tmpreg = SPIx->DMARDLR;

    level = (tmpreg & SPI_DMARDLR_DMARDL_Msk) >> SPI_DMARDLR_DMARDL_Pos;

    return level;

}

void SPI_EnableDmaTx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;

    tmpreg |= SPI_DMACR_TDMAE_Msk;

    SPIx->DMACR = tmpreg;

}
void SPI_DisableDmaTx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;

    tmpreg &= ~SPI_DMACR_TDMAE_Msk;

    SPIx->DMACR = tmpreg;

}
bool SPI_IsDmaTxEnabled(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;

    if(tmpreg & SPI_DMACR_TDMAE_Msk)
        return true;
    else
        return false;
}

void SPI_EnableDmaRx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;

    tmpreg |= SPI_DMACR_RDMAE_Msk;

    SPIx->DMACR = tmpreg;

}
void SPI_DisableDmaRx(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;

    tmpreg &= ~SPI_DMACR_RDMAE_Msk;

    SPIx->DMACR = tmpreg;

}
bool SPI_IsDmaRxEnabled(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->DMACR;

    if(tmpreg & SPI_DMACR_RDMAE_Msk)
        return true;
    else
        return false;

}

/* Tx/Rx data functions ********************************************************/
void SPI_SendData(SPI_T* SPIx, uint32_t Data)
{
    SPIx->DR = Data;
}

uint32_t SPI_ReceiveData(SPI_T* SPIx)
{
    return SPIx->DR;
}


/* Interrupts and flags management functions **********************************/

//Irq Mask
void SPI_EnableIrq(SPI_T* SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->IMR;
    tmpreg |= ((irq << SPI_IMR_ALL_INT_Pos ) & SPI_IMR_ALL_INT_Msk);

    SPIx->IMR = tmpreg;

}
void SPI_DisableIrq(SPI_T* SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->IMR;
    tmpreg &= ~((irq << SPI_IMR_ALL_INT_Pos ) & SPI_IMR_ALL_INT_Msk );

    SPIx->IMR = tmpreg;

}
bool SPI_IsIrqEnabled(SPI_T* SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg,intmsk;

    tmpreg = SPIx->IMR;
    intmsk = (irq << SPI_IMR_ALL_INT_Pos ) & SPI_IMR_ALL_INT_Msk ;

    if(tmpreg & intmsk)
        return true;
    else
        return false;
}

uint8_t SPI_GetIrqMasked(SPI_T * SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->IMR;

    return (uint8_t)( (tmpreg & SPI_IMR_ALL_INT_Msk) >> SPI_IMR_ALL_INT_Pos);
}
/*
 *  Returns whether an I2C interrupt is active or not, after the masking
 *  stage.
*/
bool SPI_IsIrqActive(SPI_T * SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->ISR;

    if(tmpreg & irq)
        return true;
    else
        return false;
}
/*
 *  Returns whether an I2C interrupt is active or not.
*/

bool SPI_IsRawIrqActive(SPI_T * SPIx, SPI_IrqDef irq)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->RISR;

    if(tmpreg & irq)
        return true;
    else
        return false;
}

void SPI_ClearIrq(SPI_T * SPIx, SPI_IrqDef irq)
{
    if(irq == Spi_irq_all)
        (void)(SPIx->ICR);
    else
    {
        if((irq & Spi_irq_rx_under) != 0)
            (void)(SPIx->RXUICR);
        if((irq & Spi_irq_rx_over) != 0)
            (void)(SPIx->RXOICR);
        if((irq & Spi_irq_tx_over) != 0)
            (void)(SPIx->TXOICR);
        if((irq & Spi_irq_multi_mst) != 0)
            (void)(SPIx->MSTICR);
    }
}


//Fifo Status
bool SPI_IsTxFifoEmpty(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_TFE_Msk)
        return true;
    else
        return false;
}

bool SPI_IsTxFifoFull(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_TFNF_Msk)
        return false;
    else
        return true;

}

bool SPI_IsRxFifoEmpty(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_RFNE_Msk)
        return false;
    else
        return true;

}

bool SPI_IsRxFifoFull(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_RFF_Msk)
        return true;
    else
        return false;

}

bool SPI_IsBusy(SPI_T* SPIx)
{
    uint32_t tmpreg = 0x00;

    tmpreg = SPIx->SR;

    if(tmpreg & SPI_SR_BUSY_Msk)
        return true;
    else
        return false;

}


/** @addtogroup PN102_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/*@}*/ /* end of group PN102_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group PN102_SPI_Driver */

/*@}*/ /* end of group PN102_Device_Driver */

/*** (C) COPYRIGHT 2016 Panchip Technology Corp. ***/
