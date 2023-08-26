/**************************************************************************//**
* @file     dma.c
* @version  V1.00
* $Revision: 2 $
* $Date: 16/02/25 14:25 $
* @brief    PN102 series DMA driver source file
*
* @note
* Copyright (C) 2016 Panchip Technology Corp. All rights reserved.
*****************************************************************************/

//#include <stdio.h>
#include "PN102Series.h"

/*
 * DESCRIPTION
 *  This function is used to initialize the DMA controller. All
 *  interrupts are cleared and disabled; DMA channels are disabled; and
 *  the device instance structure is reset.
 */

// enable DMA CLK
void DMA_EnableClock(void)
{
    CLK->AHBCLK |= (uint32_t)0x1;
}

// disable DMA CLK
void DMA_DisableClock(void)
{
    CLK->AHBCLK &= ~((uint32_t)0x1);
}

int DMA_Init(DMA_T *DMA_Def)
{
    int errorCode;

    // Disable the DMA controller
    errorCode = DMA_Disable(DMA_Def);
    if(errorCode == 0)
    {
        // Disable all DMA channels
        errorCode = DMA_DisableChannel(DMA_Def, Dmac_all_channels);
    }
    if(errorCode == 0)
    {
        // Disable all channel interrupts
        errorCode = DMA_DisableChannelIrq(DMA_Def, Dmac_all_channels);
    }
    if(errorCode == 0)
    {
        // Mask all channel interrupts
        errorCode = DMA_MaskIrq(DMA_Def, Dmac_all_channels, Dmac_irq_all);
    }
    if(errorCode == 0)
    {
        // Clear any pending interrupts
        errorCode = DMA_ClearIrq(DMA_Def, Dmac_all_channels, Dmac_irq_all);
    }

    return errorCode;
}


/****f* dmac.functions/dw_dmac_enable
 * DESCRIPTION
 *  Function will enable the DMA controller
 */
void DMA_Enable(DMA_T *DMA_Def)
{
    uint32_t tmpreg;

    tmpreg = DMA_Def->DMA_CFG_REG_L;
    tmpreg |= DMAC_DMACFGREG_L_DMA_EN_Msk;

    DMA_Def->DMA_CFG_REG_L = tmpreg;
}

/****f* dmac.functions/dw_dmac_disable
 * DESCRIPTION
 *  Function will disable the dma controller
 */
int DMA_Disable(DMA_T *DMA_Def)
{
    int errorCode = -1;
    uint32_t tmpreg;

    errorCode = 0;
    tmpreg = DMA_Def->DMA_CFG_REG_L;
    // Check first to see if DMA is already disabled
    if(tmpreg & DMAC_DMACFGREG_L_DMA_EN_Msk)
    {
        tmpreg &= ~DMAC_DMACFGREG_L_DMA_EN_Msk;
        DMA_Def->DMA_CFG_REG_L = tmpreg;

        tmpreg = DMA_Def->DMA_CFG_REG_L;
        // Ensure that the DMA was disabled
        // May not disable due to split response on one
        // of the DMA channels
        if(tmpreg & DMAC_DMACFGREG_L_DMA_EN_Msk)
        {
            errorCode = -1;
        }
    }
    return errorCode;

}

/****f* dmac.functions/dw_dmac_isEnabled
 * DESCRIPTION
 *  This function returns when the DMA controller is enabled.
 */
bool DMA_IsEnabled(DMA_T *DMA_Def)
{
    bool ret = false;
    uint32_t tmpreg;

    tmpreg = DMA_Def->DMA_CFG_REG_L;
    if(tmpreg & DMAC_DMACFGREG_L_DMA_EN_Msk)
        ret = true;
    else
        ret = false;

    return ret;

}
/****if* dmac.api/dw_dmac_checkChannelBusy
 * DESCRIPTION
 *  This function checks if the specified DMA channel is Busy (enabled)
 *  or not. Also checks if the specified channel is in range.
 */
bool DMA_CheckChannelBusy(DMA_T * DMA_Def, DMA_ChannelDef Ch)
{
    bool invalid = false;
    uint32_t tmpreg;

    /*checks if the specified channel is in range*/
    if( (DMAC_CH_MASK & Ch) || (Ch == Dmac_no_channel))
    {
        invalid = true;
    }
    else
    {
        tmpreg = DMA_Def->CH_EN_REG_L;
        /*checks if the specified DMA channel is Busy*/
        if(tmpreg & Ch)
        {
            invalid = true;
        }
    }
    return invalid;

}
/*****/

/****if* dmac.api/dw_dmac_checkChannelRange
 * DESCRIPTION
 *  This function checks if the specified DMA channel is in range.
 */
bool DMA_CheckChannelRange(DMA_T * DMA_Def, DMA_ChannelDef Ch)
{
    bool inRange = true;

    if( (DMAC_CH_MASK & Ch) || (Ch == Dmac_no_channel))
    {
        inRange = false;
    }
    return inRange;

}
/*****/


/****f* dmac.functions/dw_dmac_enableChannel
 * DESCRIPTION
 *  This function enables the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_EnableChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    int errorCode = 0;
    uint32_t tmpreg;

    // Limit all channels to the number of channels on this
    // configuration of the DMA controller.
    if(Ch == Dmac_all_channels)
    {
        Ch &= DMAC_CH_ALL_MASK;
    }

    if(DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = -1;
    }
    else
    {
        tmpreg = Ch;
        DMA_Def->CH_EN_REG_L = tmpreg;
    }

    return errorCode;

}

/****f* dmac.functions/dw_dmac_disableChannel
 * DESCRIPTION
 *  This function disables the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_DisableChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    int errorCode = 0;
    uint32_t reg;
    uint32_t enabledCh;

    // Limit all channels to the number of channels on this
    // configuration of the DMA controller.
    if(Ch == Dmac_all_channels)
    {
        Ch &= DMAC_CH_ALL_MASK;
    }

    // Check for valid channel number
    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        // Check first if the channel was already disabled
        reg = DMA_Def->CH_EN_REG_L;
        enabledCh = reg & Ch;

        if(enabledCh != 0x0)
        {
            //Write Enable bit
            reg = Ch & (DMAC_MAX_CH_MASK << DMAC_MAX_CHANNELS);
            DMA_Def->CH_EN_REG_L = reg;

            // Ensure that the channel(s) was disabled.
            // Channel may not disable due to a split response.
            if(DMA_Def->CH_EN_REG_L & Ch)
            {
                errorCode = -1;
            }
        }
    }
    return errorCode;
}


/****f* dmac.functions/dw_dmac_isChannelEnabled
 * DESCRIPTION
 *  This function returns whether the specified DMA channel is
 *  enabled. Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
bool DMA_IsChannelEnabled(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    uint8_t chIndex;
    uint32_t reg;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        reg = DMA_Def->CH_EN_REG_L;

        if(reg & DMAC_CHENREG_L_CH_EN_Msk(chIndex))
            retval = true;
        else
            retval = false;
    }
    return retval;

}
/****f* dmac.functions/dw_dmac_getChannelEnableReg
 * DESCRIPTION
 *  This function returns the lower byte of the channel enable register
 *  (ChEnReg).
 */
uint8_t DMA_GetChannelEnableReg(DMA_T *DMA_Def)
{
    uint32_t reg;

    reg = DMA_Def->CH_EN_REG_L;

    return (uint8_t)reg;
}

/****f* dmac.functions/dw_dmac_enableChannelIrq
 * DESCRIPTION
 *  This function enables interrupts for the selected channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_EnableChannelIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    int errorCode = 0, x = 0;
    uint32_t tmpreg;

    // Limit all channels to the number of channels on this
    // configuration of the DMA controller.
    if(Ch == Dmac_all_channels)
    {
        Ch &= DMAC_CH_ALL_MASK;
    }

    if(DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = -1;
    }
    else
    {
        // Loop through each channel in turn and enable
        // the channel Irq for the selected channels.
        for(x = 0; x < NUM_CHANNELS ; x++)
        {
            if(Ch & (1 << x))
            {
                tmpreg = DMA_Def->CH[x].CTL_L;

                if((tmpreg & DMAC_CTL_L_INT_EN_Msk) != 0x1)
                {
                    tmpreg |= DMAC_CTL_L_INT_EN_Msk;
                    DMA_Def->CH[x].CTL_L = tmpreg;
                }
            }
        }

    }
    return errorCode;

}

/****f* dmac.functions/dw_dmac_disableChannelIrq
 * DESCRIPTION
 *  This function disables interrupts for the selected channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_DisableChannelIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    int errorCode = -1, x = 0;
    uint32_t tmpreg;

    // Limit all channels to the number of channels on this
    // configuration of the DMA controller.
    if(Ch == Dmac_all_channels)
    {
        Ch &= DMAC_CH_ALL_MASK;
    }

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        // Loop through each channel in turn and enable
        // the channel Irq for the selected channels.
        for(x = 0; x < NUM_CHANNELS ; x++)
        {
            if(Ch & (1 << x))
            {
                tmpreg = DMA_Def->CH[x].CTL_L;
                if((tmpreg & DMAC_CTL_L_INT_EN_Msk) != 0x0)
                {
                    tmpreg &= ~DMAC_CTL_L_INT_EN_Msk;
                    DMA_Def->CH[x].CTL_L = tmpreg;
                }
            }
        }
        errorCode = 0;
    }

    return errorCode;
}


/****f* dmac.functions/dw_dmac_isChannelIrqEnabled
 * DESCRIPTION
 *  This function returns whether interrupts are enabled for the
 *  specified DMA channel. Only ONE DMA channel can be specified for
 *  the DMA_ChannelDef argument.
 */
bool DMA_IsChannelIrqEnabled(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    uint8_t chIndex;
    uint32_t reg;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        reg = DMA_Def->CH[chIndex].CTL_L;

        if(reg & DMAC_CTL_L_INT_EN_Msk)
            retval = true;
        else
            retval = false;
    }

    return retval;

}

/*
 * DESCRIPTION
 *  This function returns a DMA channel number (enumerated) that is
 *  disabled. The function starts at channel 0 and increments up
 *  through the channels until a free channel is found.
*/
DMA_ChannelDef DMA_GetFreeChannel(DMA_T *DMA_Def)
{
    int x,y;
    uint32_t reg;
    DMA_ChannelDef retval;

    retval = Dmac_no_channel;

    // read the channel enable register
    reg = DMA_Def->CH_EN_REG_L;

    reg = reg & DMAC_CHENREG_L_CH_EN_ALL_Msk;

    // Check each channel in turn until we find one
    // that is NOT enabled.  Loop checks channels
    // starting at channel 0 and works up incrementally.
    for(x = 0; x < NUM_CHANNELS; x++)
    {
        y = (1 << x);
        if(!(reg & y))
        {
            retval = (DMA_ChannelDef)((y << DMAC_MAX_CHANNELS) | y);
            break;
        }
    }
    return retval;

}
/*****/

/****f* dmac.functions/dw_dmac_clearIrq
 * DESCRIPTION
 *  This function clears the specified interrupt(s) on the specified
 *  DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Multiple interrupt types can
 *  be specified for the dw_dmac_irq argument.
 */
int DMA_ClearIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq)
{
    int x;
    int errorCode = -1;
    uint32_t reg;

    // Limit all channels to the number of channels on this
    // configuration of the DMA controller.
    if(Ch == Dmac_all_channels)
    {
        Ch &= DMAC_CH_ALL_MASK;
    }

    // Check for valid channel number
    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        // Loop through and clear the selected channel Irq
        // for the targeted channels.
        reg = Ch & DMAC_MAX_CH_MASK;
        for(x = 0; x < DMAC_MAX_INTERRUPTS; x++)
        {
            if(Irq & (1 << x))
            {
                switch(x)
                {
                    case 0 :
                        DMA_Def->CLEAR_TFR_L = reg;
                        break;
                    case 1 :
                        DMA_Def->CLEAR_BLOCK_L = reg;
                        break;
                    case 2 :
                        DMA_Def->CLEAR_SRCTRAN_L = reg;
                        break;
                    case 3 :
                        DMA_Def->CLEAR_DSTTRAN_L = reg;
                        break;
                    case 4 :
                        DMA_Def->CLEAR_ERR_L = reg;
                        break;
                }
            }
        }
        errorCode = 0;
    }
    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_maskIrq
 * DESCRIPTION
 *  This function masks the specified interrupt(s) on the specified
 *  channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Multiple interrupt types can
 *  be specified for the dw_dmac_irq argument.
 */
int DMA_MaskIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq)
{
    int x;
    int errorCode = -1;
    uint32_t reg;

    // Limit all channels to the number of channels on this
    // configuration of the DMA controller.
    if(Ch == Dmac_all_channels)
    {
        Ch &= DMAC_CH_ALL_MASK;
    }

    // Check for valid channel number
    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        // Loop through and clear the selected channel Irq
        // for the targeted channels.
        reg = Ch & (DMAC_MAX_CH_MASK << DMAC_MAX_CHANNELS);
        for(x = 0; x < DMAC_MAX_INTERRUPTS; x++)
        {
            if(Irq & (1 << x))
            {
                switch(x)
                {
                    case 0 :
                        DMA_Def->MSK_TFR_L = reg;
                        break;
                    case 1 :
                        DMA_Def->MSK_BLOCK_L = reg;
                        break;
                    case 2 :
                        DMA_Def->MSK_SRCTRAN_L = reg;
                        break;
                    case 3 :
                        DMA_Def->MSK_DSTTRAN_L = reg;
                        break;
                    case 4 :
                        DMA_Def->MSK_ERR_L = reg;
                        break;
                }
            }
        }
        errorCode = 0;
    }
    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_unmaskIrq
 * DESCRIPTION
 *  This function unmasks the specified interrupt(s)
 *  on the specified channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Multiple interrupt types can
 *  be specified for the dw_dmac_irq argument.
 */
int DMA_UnmaskIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq)
{
    int x;
    int errorCode = -1;
    uint32_t reg;

    // Limit all channels to the number of channels on this
    // configuration of the DMA controller.
    if(Ch == Dmac_all_channels)
    {
        Ch &= DMAC_CH_ALL_MASK;
    }

    // Check for valid channel number
    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        // Loop through and clear the selected channel Irq
        // for the targeted channels.
        reg = Ch;
        for(x = 0; x < DMAC_MAX_INTERRUPTS; x++)
        {
            if(Irq & (1 << x))
            {
                switch(x)
                {
                    case 0 :
                        DMA_Def->MSK_TFR_L = reg;
                        break;
                    case 1 :
                        DMA_Def->MSK_BLOCK_L = reg;
                        break;
                    case 2 :
                        DMA_Def->MSK_SRCTRAN_L = reg;
                        break;
                    case 3 :
                        DMA_Def->MSK_DSTTRAN_L = reg;
                        break;
                    case 4 :
                        DMA_Def->MSK_ERR_L = reg;
                        break;
                }
            }
        }
        errorCode = 0;
    }
    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_isIrqMasked
 * DESCRIPTION
 *  This function returns whether the specified interrupt on the
 *  specified channel is masked.
 *  Only 1 DMA channel can be specified for the DMA_ChannelDef
 *  argument. Only 1 interrupt type can be specified for the
 *  dw_dmac_irq argument.
 */
bool DMA_IsIrqMasked(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq)
{
    uint8_t chIndex;
    uint32_t reg = 0;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        switch(Irq)
        {
            case Dmac_irq_tfr:
                reg = DMA_Def->MSK_TFR_L;
                break;
            case Dmac_irq_block:
                reg = DMA_Def->MSK_BLOCK_L;
                break;
            case Dmac_irq_srctran:
                reg = DMA_Def->MSK_SRCTRAN_L;
                break;
            case Dmac_irq_dsttran:
                reg = DMA_Def->MSK_DSTTRAN_L;
                break;
            case Dmac_irq_err:
                reg = DMA_Def->MSK_ERR_L;
                break;
        }
        // Masked will read 0 : Unmasked will read 1
        if((reg & DMAC_INT_MASK_L_Msk(chIndex)))
        {
            retval = true;
        }
        else
        {
            retval = false;
        }

    }

    return retval;

}
/*****/

/****f* dmac.functions/dw_dmac_isRawIrqActive
 * DESCRIPTION
 *  This function returns whether the specified raw interrupt on the
 *  specified channel is active.
 *  Only ONE DMA channel can be specified for the DMA_ChannelDef
 *  argument. Only 1 interrupt type can be specified for the
 *  dw_dmac_irq argument.
 */
bool DMA_IsRawIrqActive(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq)
{
    uint8_t chIndex;
    uint32_t reg = 0;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        switch(Irq)
        {
            case Dmac_irq_tfr:
                reg = DMA_Def->RAW_TFR_L;
                break;
            case Dmac_irq_block:
                reg = DMA_Def->RAW_BLOCK_L;
                break;
            case Dmac_irq_srctran:
                reg = DMA_Def->RAW_SRCTRAN_L;
                break;
            case Dmac_irq_dsttran:
                reg = DMA_Def->RAW_DSTTRAN_L;
                break;
            case Dmac_irq_err:
                reg = DMA_Def->RAW_ERR_L;
                break;
        }
        // Masked will read 0 : Unmasked will read 1
        if((reg & DMAC_INT_RAW_STAT_CLR_Msk(chIndex)))
        {
            retval = true;
        }
        else
        {
            retval = false;
        }

    }

    return retval;

}
/*****/

/****f* dmac.functions/dw_dmac_isIrqActive
 * DESCRIPTION
 *  This function returns whether the specified interrupt on the
 *  specified channel is active after masking.
 *  All DMA channels OR only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1 interrupt type can be
 *  specified for the dw_dmac_irq argument.
 */
bool DMA_IsIrqActive(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq)
{
    uint8_t chIndex;
    uint32_t reg = 0;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        switch(Irq)
        {
            case Dmac_irq_tfr:
                reg = DMA_Def->STATUS_TFR_L;
                break;
            case Dmac_irq_block:
                reg = DMA_Def->STATUS_BLOCK_L;
                break;
            case Dmac_irq_srctran:
                reg = DMA_Def->STATUS_SRCTRAN_L;
                break;
            case Dmac_irq_dsttran:
                reg = DMA_Def->STATUS_DSTTRAN_L;
                break;
            case Dmac_irq_err:
                reg = DMA_Def->STATUS_ERR_L;
                break;
        }
        // Masked will read 0 : Unmasked will read 1
        if((reg & DMAC_INT_RAW_STAT_CLR_Msk(chIndex)))
        {
            retval = true;
        }
        else
        {
            retval = false;
        }

    }

    return retval;

}
/*****/
bool DMA_IsCombinedIrqActive(DMA_T *DMA_Def, DMA_IrqDef Irq)
{
    uint32_t reg = 0;
    bool retval = false;

    reg = DMA_Def->STATUS_INT_L;
    switch(Irq)
    {
        case Dmac_irq_tfr:
            if(reg & DMAC_STATUSINT_L_TFR_Msk)
                retval = true;
            break;
        case Dmac_irq_block:
            if(reg & DMAC_STATUSINT_L_BLOCK_Msk)
                retval = true;
            break;
        case Dmac_irq_srctran:
            if(reg & DMAC_STATUSINT_L_SRCTRAN_Msk)
                retval = true;
            break;
        case Dmac_irq_dsttran:
            if(reg & DMAC_STATUSINT_L_DSTTRAN_Msk)
                retval = true;
            break;
        case Dmac_irq_err:
            if(reg & DMAC_STATUSINT_L_ERR_Msk)
                retval = true;
            break;
        default:
            break;
    }

    return retval;
}

/****f* dmac.functions/dw_dmac_setChannelConfig
 * DESCRIPTION
 *  This function sets configuration parameters in the DMAC's
 *  channel registers on the specified DMA channel.
 *  Only 1 DMA channel can be specified for the DMA_ChannelDef
 *  argument.
 */
int DMA_SetChannelConfig(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ChannelConfigDef *pConfig)
{
    int errorCode = -1;
    uint8_t chIndex;
    uint32_t reg;

    chIndex = DMA_GetChannelIndex(Ch);

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        if( ((1 << (pConfig->ctlSrcMsize + 1)) > CHx_MAX_MULT_SIZE) || \
            ((1 << (pConfig->ctlDstMsize + 1)) > CHx_MAX_MULT_SIZE))
        {
            errorCode = -1;
        }
        if(pConfig->cfgDstPer >= NUM_HS_INT || pConfig->cfgSrcPer >= NUM_HS_INT)
        {
            errorCode = -1;
        }
        if(pConfig->cfgChPrior > NUM_CHANNELS)
        {
            errorCode = -1;
        }

    }

    if(errorCode != -1)
    {
        // Set the control register
        reg = DMA_Def->CH[chIndex].CTL_L;
        reg &= ~(DMAC_CTL_L_INT_EN_Msk | DMAC_CTL_L_DST_TR_WIDTH_Msk | DMAC_CTL_L_SRC_TR_WIDTH_Msk   |    \
                DMAC_CTL_L_DINC_Msk | DMAC_CTL_L_SINC_Msk          |                                    \
                DMAC_CTL_L_DEST_MSIZE_Msk | DMAC_CTL_L_SRC_MSIZE_Msk |                                  \
                DMAC_CTL_L_TT_FC_Msk);
        reg |= (                                                                                         \
              (pConfig->ctlIntEn << DMAC_CTL_L_INT_EN_Pos) & DMAC_CTL_L_INT_EN_Msk |                    \
              (pConfig->ctlDstTrWidth << DMAC_CTL_L_DST_TR_WIDTH_Pos) & DMAC_CTL_L_DST_TR_WIDTH_Msk |       \
              (pConfig->ctlSrcTrWidth << DMAC_CTL_L_SRC_TR_WIDTH_Pos) & DMAC_CTL_L_SRC_TR_WIDTH_Msk |   \
              (pConfig->ctlDinc << DMAC_CTL_L_DINC_Pos) & DMAC_CTL_L_DINC_Msk |                         \
              (pConfig->ctlSinc << DMAC_CTL_L_SINC_Pos) & DMAC_CTL_L_SINC_Msk |                         \
              (pConfig->ctlDstMsize << DMAC_CTL_L_DEST_MSIZE_Pos) & DMAC_CTL_L_DEST_MSIZE_Msk |         \
              (pConfig->ctlSrcMsize << DMAC_CTL_L_SRC_MSIZE_Pos) & DMAC_CTL_L_SRC_MSIZE_Msk |           \
              (pConfig->ctlTtFc << DMAC_CTL_L_TT_FC_Pos) & DMAC_CTL_L_TT_FC_Msk                         \
              );

        DMA_Def->CH[chIndex].CTL_L = reg;

        reg = DMA_Def->CH[chIndex].CTL_H;
        reg &= ~(DMAC_CTL_H_BLOCK_TS_Msk);

        reg |= (pConfig->ctlBlockTs << DMAC_CTL_H_BLOCK_TS_Pos) & DMAC_CTL_H_BLOCK_TS_Msk;
        DMA_Def->CH[chIndex].CTL_H = reg;

        // Set the config register
        reg = DMA_Def->CH[chIndex].CFG_L;
        reg &= ~(DMAC_CFG_L_CH_PRIOR_Msk   |                             \
                 DMAC_CFG_L_HS_SEL_DST_Msk | DMAC_CFG_L_HS_SEL_SRC_Msk | \
                 DMAC_CFG_L_DST_HS_POL_Msk | DMAC_CFG_L_SRC_HS_POL_Msk);
        reg |= (                                                                                     \
              (pConfig->cfgChPrior << DMAC_CFG_L_CH_PRIOR_Pos) & DMAC_CFG_L_CH_PRIOR_Msk |          \
              (pConfig->cfgHsSelDst << DMAC_CFG_L_HS_SEL_DST_Pos) & DMAC_CFG_L_HS_SEL_DST_Msk |     \
              (pConfig->cfgHsSelSrc << DMAC_CFG_L_HS_SEL_SRC_Pos) & DMAC_CFG_L_HS_SEL_SRC_Msk |     \
              (pConfig->cfgDstHsPol << DMAC_CFG_L_DST_HS_POL_Pos) & DMAC_CFG_L_DST_HS_POL_Msk |     \
              (pConfig->cfgSrcHsPol << DMAC_CFG_L_SRC_HS_POL_Pos) & DMAC_CFG_L_SRC_HS_POL_Msk       \
              );
        DMA_Def->CH[chIndex].CFG_L = reg;

        reg = DMA_Def->CH[chIndex].CFG_H;
        reg &= ~(DMAC_CFG_H_FCMODE_Msk | DMAC_CFG_H_FIFO_MODE_Msk | \
                 DMAC_CFG_H_SRC_PER_Msk | DMAC_CFG_H_DEST_PER_Msk);
        reg |= (                                                     \
              (pConfig->cfgFcMode << DMAC_CFG_H_FCMODE_Pos) & DMAC_CFG_H_FCMODE_Msk|           \
              (pConfig->cfgFifoMode << DMAC_CFG_H_FIFO_MODE_Pos) & DMAC_CFG_H_FIFO_MODE_Msk |  \
              (pConfig->cfgSrcPer << DMAC_CFG_H_SRC_PER_Pos) & DMAC_CFG_H_SRC_PER_Msk|         \
              (pConfig->cfgDstPer << DMAC_CFG_H_DEST_PER_Pos) & DMAC_CFG_H_DEST_PER_Msk        \
              );
        DMA_Def->CH[chIndex].CFG_H = reg;

        // set the SAR/DAR registers
        DMA_Def->CH[chIndex].SAR_L = pConfig->sar;
        DMA_Def->CH[chIndex].DAR_L = pConfig->dar;
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_getChannelConfig
 * DESCRIPTION
 *  This function gets configuration parameters in the DMAC's
 *  channel registers for the specified DMA channel.
 *  Only 1 DMA channel can be specified for the DMA_ChannelDef
 *  argument.
 */
int DMA_GetChannelConfig(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ChannelConfigDef *pConfig)
{

    int errorCode = -1;
    uint8_t chIndex;
    uint32_t reg;

    errorCode = 0;
    chIndex = DMA_GetChannelIndex(Ch);
    // Check for valid channel number - can only specify one channel */
    if(DMAC_CH_MASK & Ch || Ch == Dmac_no_channel | Ch == DMAC_MAX_CHANNELS)
    {
        errorCode = -1;
    }
    if(errorCode != -1)
    {
        // Get the control register
        reg = DMA_Def->CH[chIndex].CTL_L;
        pConfig->ctlIntEn = (bool)((reg & DMAC_CTL_L_INT_EN_Msk) >> DMAC_CTL_L_INT_EN_Pos);
        pConfig->ctlDstTrWidth = (DMA_TransferWidthDef)((reg & DMAC_CTL_L_DST_TR_WIDTH_Msk) >> DMAC_CTL_L_DST_TR_WIDTH_Pos);
        pConfig->ctlSrcTrWidth = (DMA_TransferWidthDef)((reg & DMAC_CTL_L_SRC_TR_WIDTH_Msk) >> DMAC_CTL_L_SRC_TR_WIDTH_Pos);
        pConfig->ctlDinc = (DMA_AddressIncrementDef)((reg & DMAC_CTL_L_DINC_Msk) >> DMAC_CTL_L_DINC_Pos);
        pConfig->ctlSinc = (DMA_AddressIncrementDef)((reg & DMAC_CTL_L_SINC_Msk) >> DMAC_CTL_L_SINC_Pos);

        pConfig->ctlDstMsize = (DMA_BurstTransLengthDef)((reg & DMAC_CTL_L_DEST_MSIZE_Msk) >> DMAC_CTL_L_DEST_MSIZE_Pos);
        pConfig->ctlSrcMsize = (DMA_BurstTransLengthDef)((reg & DMAC_CTL_L_SRC_MSIZE_Msk) >> DMAC_CTL_L_SRC_MSIZE_Pos);
        pConfig->ctlTtFc = (DMA_TransferFlowDef)((reg & DMAC_CTL_L_TT_FC_Msk) >> DMAC_CTL_L_TT_FC_Pos);


        reg = DMA_Def->CH[chIndex].CTL_H;
        pConfig->ctlBlockTs = (uint32_t)((reg & DMAC_CTL_H_BLOCK_TS_Msk) >> DMAC_CTL_H_BLOCK_TS_Pos);

        // Get the config register
        reg = DMA_Def->CH[chIndex].CFG_L;
        pConfig->cfgChPrior = (DMA_ChannelPriorityDef)((reg & DMAC_CFG_L_CH_PRIOR_Msk) >> DMAC_CFG_L_CH_PRIOR_Pos);
        pConfig->cfgHsSelDst = (DMA_SwHwHsSelectDef)((reg & DMAC_CFG_L_HS_SEL_DST_Msk) >> DMAC_CFG_L_HS_SEL_DST_Pos);
        pConfig->cfgHsSelSrc = (DMA_SwHwHsSelectDef)((reg & DMAC_CFG_L_HS_SEL_SRC_Msk) >> DMAC_CFG_L_HS_SEL_SRC_Pos);
        pConfig->cfgDstHsPol = (DMA_PolarityLevelDef)((reg & DMAC_CFG_L_DST_HS_POL_Msk) >> DMAC_CFG_L_DST_HS_POL_Pos);
        pConfig->cfgSrcHsPol = (DMA_PolarityLevelDef)((reg & DMAC_CFG_L_SRC_HS_POL_Msk) >> DMAC_CFG_L_SRC_HS_POL_Pos);

        reg = DMA_Def->CH[chIndex].CFG_H;
        pConfig->cfgFcMode = (DMA_FlowCtrlModeDef)((reg & DMAC_CFG_H_FCMODE_Msk) >> DMAC_CFG_H_FCMODE_Pos);
        pConfig->cfgFifoMode = (DMA_FifoModeDef)((reg & DMAC_CFG_H_FIFO_MODE_Msk) >> DMAC_CFG_H_FIFO_MODE_Pos);
        pConfig->cfgSrcPer = (DMA_HsInterfaceDef)((reg & DMAC_CFG_H_SRC_PER_Msk) >> DMAC_CFG_H_SRC_PER_Pos);
        pConfig->cfgDstPer = (DMA_HsInterfaceDef)((reg & DMAC_CFG_H_DEST_PER_Msk) >> DMAC_CFG_H_DEST_PER_Pos);

        // Get the SAR/DAR registers
        pConfig->sar = DMA_Def->CH[chIndex].SAR_L;
        pConfig->dar = DMA_Def->CH[chIndex].DAR_L;
    }

    return errorCode;

}
/*****/


/****f* dmac.functions/dw_dmac_isBlockTransDone
 * DESCRIPTION
 *  This function returns whether the block transfer of the selected
 *  channel has completed.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
bool DMA_IsBlockTransDone(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    uint8_t chIndex;
    uint32_t reg;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        reg = DMA_Def->CH[chIndex].CTL_H;

        if(reg & DMAC_CTL_H_DONE_Msk)
            retval = true;
        else
            retval = false;
    }

    return retval;

}
/*****/


/****f* dmac.functions/dw_dmac_setAddress
 * DESCRIPTION
 *  This function sets the address on the specified source or/and
 *  destination register of the specified channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Both source and destination can
 *  be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_SetAddress(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, uint32_t address)
{
    int x,errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
                {
                    DMA_Def->CH[x].SAR_L = address;
                }
                if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
                {
                    DMA_Def->CH[x].DAR_L = address;
                }
            }
        }
    }

    return errorCode;

}

/*****/

/****f* dmac.functions/dw_dmac_getAddress
 * DESCRIPTION
 *  This function returns the address on the specified source or
 *  destination register of the specified channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1, source or destination,
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_GetAddress(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, uint32_t * pAddr)
{
    uint8_t chIndex;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
        {
            *pAddr = DMA_Def->CH[chIndex].SAR_L;
        }
        if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
        {
            *pAddr = DMA_Def->CH[chIndex].DAR_L;
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setBlockTransSize
 * DESCRIPTION
 *  This function sets the block size of a transfer on the specified
 *  DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_SetBlockTransSize(DMA_T *DMA_Def, DMA_ChannelDef Ch, uint16_t blockSize)
{
    uint32_t reg;
    int x,errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CTL_H;
                if(blockSize != ((reg & DMAC_CTL_H_BLOCK_TS_Msk) >> DMAC_CTL_H_BLOCK_TS_Pos ))
                {
                    reg &= ~(DMAC_CTL_H_BLOCK_TS_Msk);
                    reg |= (blockSize << DMAC_CTL_H_BLOCK_TS_Pos) & DMAC_CTL_H_BLOCK_TS_Msk;

                    DMA_Def->CH[x].CTL_H = reg;
                }
                DMA_Def->CH[x].CTL_H = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getBlockTransSize
 * DESCRIPTION
 *  This function returns the block size of a transfer
 *  on the specified DMA channel.
 *  Only ONE DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_GetBlockTransSize(DMA_T *DMA_Def, DMA_ChannelDef Ch, uint16_t * pBlockSize)
{
    uint8_t chIndex;
    int errorCode = -1;
    uint32_t reg;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CTL_H;

        *pBlockSize = (reg & DMAC_CTL_H_BLOCK_TS_Msk) >> DMAC_CTL_H_BLOCK_TS_Pos;

    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setMemPeriphFlowCtl
 * DESCRIPTION
 *  This function sets the transfer device type and flow control
 *  (TT_FC) for the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_SetMemPeriphFlowCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_TransferFlowDef tt_fc)
{
    uint32_t reg;
    int x,errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CTL_L;
                if(tt_fc != ((reg & DMAC_CTL_L_TT_FC_Msk) >> DMAC_CTL_L_TT_FC_Pos ))
                {
                    reg &= ~(DMAC_CTL_L_TT_FC_Msk);
                    reg |= (tt_fc << DMAC_CTL_L_TT_FC_Pos) & DMAC_CTL_L_TT_FC_Msk;

                    DMA_Def->CH[x].CTL_L = reg;
                }
                DMA_Def->CH[x].CTL_L = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getMemPeriphFlowCtl
 * DESCRIPTION
 *  This function returns the transfer device type and flow control
 *  (TT_FC) for the specified DMA channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_GetMemPeriphFlowCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_TransferFlowDef * const pTranferFlow)
{
    uint8_t chIndex;
    int errorCode = -1;
    uint32_t reg;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CTL_L;

        *pTranferFlow = (DMA_TransferFlowDef)((reg & DMAC_CTL_L_TT_FC_Msk) >> DMAC_CTL_L_TT_FC_Pos);
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setBurstTransLength
 * DESCRIPTION
 *  This function sets the specified source and/or destination
 *  burst size on the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Both source and destination
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_SetBurstTransLength(DMA_T *DMA_Def, DMA_ChannelDef Ch,
    DMA_SrcDstSelectDef sdSel, DMA_BurstTransLengthDef xfLength)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CTL_L;
                if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CTL_L_SRC_MSIZE_Msk;
                    reg |= (xfLength << DMAC_CTL_L_SRC_MSIZE_Pos) & DMAC_CTL_L_SRC_MSIZE_Msk;
                }
                if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CTL_L_DEST_MSIZE_Msk;
                    reg |= (xfLength << DMAC_CTL_L_DEST_MSIZE_Pos) & DMAC_CTL_L_DEST_MSIZE_Msk;
                }
                DMA_Def->CH[x].CTL_L = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getBurstTransLength
 * DESCRIPTION
 *  This function returns the specified source or destination
 *  burst size on the specified DMA channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1, source or destination,
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
 int DMA_GetBurstTransLength( DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel,
                                                             DMA_BurstTransLengthDef * pBurstLen)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CTL_L;

        if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
        {
            *pBurstLen = (DMA_BurstTransLengthDef)((reg & DMAC_CTL_L_SRC_MSIZE_Msk) >> DMAC_CTL_L_SRC_MSIZE_Pos);
        }
        if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
        {
            *pBurstLen = (DMA_BurstTransLengthDef)((reg & DMAC_CTL_L_DEST_MSIZE_Msk) >> DMAC_CTL_L_DEST_MSIZE_Pos);
        }
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_setAddressInc
 * DESCRIPTION
 *  This function sets the address increment type on the specified
 *  source and/or destination on the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Both source and destination
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_SetAddressInc(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_AddressIncrementDef addrInc)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CTL_L;
                if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CTL_L_SINC_Msk;
                    reg |= (addrInc << DMAC_CTL_L_SINC_Pos) & DMAC_CTL_L_SINC_Msk;
                }
                if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CTL_L_DINC_Msk;
                    reg |= (addrInc << DMAC_CTL_L_DINC_Pos) & DMAC_CTL_L_DINC_Msk;
                }
                DMA_Def->CH[x].CTL_L = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getAddressInc
 * DESCRIPTION
 *  This function returns the address increment type on the specified
 *  source or destination on the specified DMA channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1, source or destination,
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_GetAddressInc(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel,
                                                                DMA_AddressIncrementDef *pAddrInc)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CTL_L;

        if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
        {
            *pAddrInc = (DMA_AddressIncrementDef)((reg & DMAC_CTL_L_SINC_Msk) >> DMAC_CTL_L_SINC_Pos);
        }
        if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
        {
            *pAddrInc = (DMA_AddressIncrementDef)((reg & DMAC_CTL_L_DINC_Msk) >> DMAC_CTL_L_DINC_Pos);
        }
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_setTransWidth
 * DESCRIPTION
 *  This function sets the specified source and/or destination
 *  transfer width on the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Both source and destination
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_SetTransWidth(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_TransferWidthDef xfWidth)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CTL_L;
                if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CTL_L_SRC_TR_WIDTH_Msk;
                    reg |= (xfWidth << DMAC_CTL_L_SRC_TR_WIDTH_Pos) & DMAC_CTL_L_SRC_TR_WIDTH_Msk;
                }
                if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CTL_L_DST_TR_WIDTH_Msk;
                    reg |= (xfWidth << DMAC_CTL_L_DST_TR_WIDTH_Pos) & DMAC_CTL_L_DST_TR_WIDTH_Msk;
                }
                DMA_Def->CH[x].CTL_L = reg;
            }
        }
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_getTransWidth
 * DESCRIPTION
 *  This function returns the specified source or destination
 *  transfer width on the specified DMA channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1, source or destination,
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_GetTransWidth(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel,
                                                                                   DMA_TransferWidthDef *pWidth)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CTL_L;

        if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
        {
            *pWidth = (DMA_TransferWidthDef)((reg & DMAC_CTL_L_SRC_TR_WIDTH_Msk) >> DMAC_CTL_L_SRC_TR_WIDTH_Pos);
        }
        if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
        {
            *pWidth = (DMA_TransferWidthDef)((reg & DMAC_CTL_L_DST_TR_WIDTH_Msk) >> DMAC_CTL_L_DST_TR_WIDTH_Pos);
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setHsInterface
 * DESCRIPTION
 *  This function sets the handshaking interface on the specified
 *  source or destination on the specified DMA channel.
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Both source and destination
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_SetHsInterface(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel,DMA_HsInterfaceDef hsIf)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_H;
                if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CFG_H_SRC_PER_Msk;
                    reg |= (hsIf << DMAC_CFG_H_SRC_PER_Pos) & DMAC_CFG_H_SRC_PER_Msk;
                }
                if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CFG_H_DEST_PER_Msk;
                    reg |= (hsIf << DMAC_CFG_H_DEST_PER_Pos) & DMAC_CFG_H_DEST_PER_Msk;
                }
                DMA_Def->CH[x].CFG_H = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getHsInterface
 * DESCRIPTION
 *  This function returns the handshaking interface on the specified
 *  source or destination on the specified DMA channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1, source or destination,
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_GetHsInterface(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel,
                                                                                 DMA_HsInterfaceDef *pHsIf)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CFG_H;

        if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
        {
            *pHsIf = (DMA_HsInterfaceDef)((reg & DMAC_CFG_H_SRC_PER_Msk) >> DMAC_CFG_H_SRC_PER_Pos);
        }
        if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
        {
            *pHsIf = (DMA_HsInterfaceDef)((reg & DMAC_CFG_H_DEST_PER_Msk) >> DMAC_CFG_H_DEST_PER_Pos);
        }
    }

    return errorCode;

}
/*****/



/****f* dmac.functions/dw_dmac_setProtCtl
 * DESCRIPTION
 *  This function sets the prot level for the AMBA bus
 *   on the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_SetProtCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ProtLevelDef protLvl)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_H;
                {
                    reg &= ~DMAC_CFG_H_PROTCTL_Msk;
                    reg |= (protLvl << DMAC_CFG_H_PROTCTL_Pos) & DMAC_CFG_H_PROTCTL_Msk;
                }
                DMA_Def->CH[x].CFG_H = reg;
            }
        }
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_getProtCtl
 * DESCRIPTION
 *  This function returns the prot level for the AMBA bus on the
 *  specified DMA channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_GetProtCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ProtLevelDef * pLvl)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CFG_H;
        {
            *pLvl = (DMA_ProtLevelDef)((reg & DMAC_CFG_H_PROTCTL_Msk) >> DMAC_CFG_H_PROTCTL_Pos);
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setFifoMode
 * DESCRIPTION
 *  This function sets the FIFO mode on the specified DMA channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_SetFifoMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FifoModeDef fifoMode)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_H;
                {
                    reg &= ~DMAC_CFG_H_FIFO_MODE_Msk;
                    reg |= (fifoMode << DMAC_CFG_H_FIFO_MODE_Pos) & DMAC_CFG_H_FIFO_MODE_Msk;
                }
                DMA_Def->CH[x].CFG_H = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getFifoMode
 * DESCRIPTION
 *  This function returns the FIFO mode on the specified DMA channel(s)
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_GetFifoMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FifoModeDef *pFifoMode)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CFG_H;
        {
            *pFifoMode = (DMA_FifoModeDef)((reg & DMAC_CFG_H_FIFO_MODE_Msk) >> DMAC_CFG_H_FIFO_MODE_Pos);
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setFlowCtlMode
 * DESCRIPTION
 *  This function sets the flow control mode on the specified DMA
 *  channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_SetFlowCtlMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FlowCtrlModeDef fcMode)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_H;
                {
                    reg &= ~DMAC_CFG_H_FCMODE_Msk;
                    reg |= (fcMode << DMAC_CFG_H_FCMODE_Pos) & DMAC_CFG_H_FCMODE_Msk;
                }
                DMA_Def->CH[x].CFG_H = reg;
            }
        }
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_getFlowCtlMode
 * DESCRIPTION
 *  This function returns the flow control mode on the specified DMA
 *  channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_GetFlowCtlMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FlowCtrlModeDef *pFcMode)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CFG_H;
        {
            *pFcMode = (DMA_FlowCtrlModeDef)((reg & DMAC_CFG_H_FCMODE_Msk) >> DMAC_CFG_H_FCMODE_Pos);
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setHsPolarity
 * DESCRIPTION
 *  This function sets the handshaking interface polarity on the
 *  specified source and/or destination on the specified DMA
 *  channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Both source and destination
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_SetHsPolarity(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_PolarityLevelDef polLevel)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_L;
                if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CFG_L_SRC_HS_POL_Msk;
                    reg |= (polLevel << DMAC_CFG_L_SRC_HS_POL_Pos) & DMAC_CFG_L_SRC_HS_POL_Msk;
                }
                if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CFG_L_DST_HS_POL_Msk;
                    reg |= (polLevel << DMAC_CFG_L_DST_HS_POL_Pos) & DMAC_CFG_L_DST_HS_POL_Msk;
                }
                DMA_Def->CH[x].CFG_L = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getHsPolarity
 * DESCRIPTION
 *  This function returns the handshaking interface polarity on the
 *  specified source or destination on the specified DMA channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1, source or destination,
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_GetHsPolarity(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_PolarityLevelDef *pPolLevel)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CFG_L;

        if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
        {
            *pPolLevel = (DMA_PolarityLevelDef)((reg & DMAC_CFG_L_SRC_HS_POL_Msk) >> DMAC_CFG_L_SRC_HS_POL_Pos);
        }
        if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
        {
            *pPolLevel = (DMA_PolarityLevelDef)((reg & DMAC_CFG_L_DST_HS_POL_Msk) >> DMAC_CFG_L_DST_HS_POL_Pos);
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_setHandshakingMode
 * DESCRIPTION
 *  This function sets the handshaking mode from hardware to software
 *  on the specified source and/or destination on the specified DMA
 *  channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument. Both source and destination
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_SetHandshakingMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel, DMA_SwHwHsSelectDef hsHwSwSel)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_L;
                if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CFG_L_HS_SEL_SRC_Msk;
                    reg |= (hsHwSwSel << DMAC_CFG_L_HS_SEL_SRC_Pos) & DMAC_CFG_L_HS_SEL_SRC_Msk;
                }
                if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
                {
                    reg &= ~DMAC_CFG_L_HS_SEL_DST_Msk;
                    reg |= (hsHwSwSel << DMAC_CFG_L_HS_SEL_DST_Pos) & DMAC_CFG_L_HS_SEL_DST_Msk;
                }
                DMA_Def->CH[x].CFG_L = reg;

            }
        }
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_getHandshakingMode
 * DESCRIPTION
 *  This function returns the handshaking mode hardware or software
 *  on the specified source or destination on the specified DMA
 *  channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument. Only 1, source or destination,
 *  can be specified for the dw_dmac_src_dst_select argument.
 */
int DMA_GetHandshakingMode(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_SwHwHsSelectDef *pHsSel)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CFG_L;

        if(sdSel == Dmac_src || sdSel == Dmac_src_dst)
        {
            *pHsSel = (DMA_SwHwHsSelectDef)((reg & DMAC_CFG_L_HS_SEL_SRC_Msk) >> DMAC_CFG_L_HS_SEL_SRC_Pos);
        }
        if(sdSel == Dmac_dst || sdSel == Dmac_src_dst)
        {
            *pHsSel = (DMA_SwHwHsSelectDef)((reg & DMAC_CFG_L_HS_SEL_DST_Msk) >> DMAC_CFG_L_HS_SEL_DST_Pos);
        }
    }

    return errorCode;
}
/*****/

/****f* dmac.functions/dw_dmac_isFifoEmpty
 * DESCRIPTION
 *  This function returns whether the FIFO is empty on the
 *  specified channel. Only 1 DMA channel can be specified
 *  for the DMA_ChannelDef argument.
 */
bool DMA_IsFifoEmpty(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    uint8_t chIndex;
    uint32_t reg;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        reg = DMA_Def->CH[chIndex].CFG_L;

        if(reg & DMAC_CFG_L_FIFO_EMPTY_Msk)
            retval = true;
        else
            retval = false;
    }

    return retval;

}

/*
 * DESCRIPTION
 *  This function suspends transfers on the specified channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_SuspendChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_H;
                {
                    reg |= DMAC_CFG_L_CH_SUSP_Msk;
                }
                DMA_Def->CH[x].CFG_L = reg;

            }
        }
    }

    return errorCode;

}
/*****/

/*
 * DESCRIPTION
 *  This function resumes (remove suspend) on the specified channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_ResumeChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_H;
                {
                    reg &= ~DMAC_CFG_L_CH_SUSP_Msk;
                }
                DMA_Def->CH[x].CFG_L = reg;

            }
        }
    }

    return errorCode;

}
/*****/

/*
 * DESCRIPTION
 *  This function returns whether the specified channel is suspended.
 *  Only ONE DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
bool DMA_IsChannelSuspended(DMA_T *DMA_Def, DMA_ChannelDef Ch)
{
    uint8_t chIndex;
    uint32_t reg;
    bool retval = false;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        reg = DMA_Def->CH[chIndex].CFG_L;

        if(reg & DMAC_CFG_L_CH_SUSP_Msk)
            retval = true;
        else
            retval = false;
    }

    return retval;

}
/*****/


/****f* dmac.functions/dw_dmac_setChannelPriority
 * DESCRIPTION
 *  This function sets the priority level on the specified DMA
 *  channel(s).
 *  Multiple DMA channels can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_SetChannelPriority(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ChannelPriorityDef chPriority)
{
    uint32_t reg;
    int x, errorCode = -1;

    if(!DMA_CheckChannelBusy(DMA_Def, Ch))
    {
        errorCode = 0;
        for(x = 0; x < NUM_CHANNELS; x++)
        {
            if(Ch & (1 << x))
            {
                reg = DMA_Def->CH[x].CFG_L;
                {
                    reg &= ~DMAC_CFG_L_CH_PRIOR_Msk;
                    reg |= (chPriority << DMAC_CFG_L_CH_PRIOR_Pos) & DMAC_CFG_L_CH_PRIOR_Msk;
                }
                DMA_Def->CH[x].CFG_L = reg;
            }
        }
    }

    return errorCode;

}
/*****/

/****f* dmac.functions/dw_dmac_getChannelPriority
 * DESCRIPTION
 *  This function returns the priority level on the specified DMA
 *  channel.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
int DMA_GetChannelPriority(DMA_T * DMA_Def, DMA_ChannelDef Ch, DMA_ChannelPriorityDef *pPriority)
{
    uint8_t chIndex;
    uint32_t reg;
    int errorCode = -1;

    chIndex = DMA_GetChannelIndex(Ch);

    if(DMA_CheckChannelRange(DMA_Def, Ch))
    {
        errorCode = 0;
        reg = DMA_Def->CH[chIndex].CFG_L;
        {
            *pPriority = (DMA_ChannelPriorityDef)((reg & DMAC_CFG_L_CH_PRIOR_Msk) >> DMAC_CFG_L_CH_PRIOR_Pos);
        }
    }

    return errorCode;

}

/****f* dmac.functions/dw_dmac_getChannelIndex
 * DESCRIPTION
 *  This function returns the channel index from the specified channel
 *  enumerated type.
 *  Only 1 DMA channel can be specified for the
 *  DMA_ChannelDef argument.
 */
unsigned DMA_GetChannelIndex(DMA_ChannelDef Ch)
{
    unsigned chEnum = 1;
    unsigned chIndex = 0;

    Ch &= DMAC_MAX_CH_MASK;

    while(chIndex < DMAC_MAX_CHANNELS)
    {
        if(chEnum == Ch)
        {
            break;
        }
        chEnum *= 2;
        chIndex++;
    }

    return chIndex;
}
/*****/

uint32_t DMA_GetIntStatusReg(DMA_T * DMA_Def)
{
    uint32_t reg;

    reg = DMA_Def->STATUS_INT_L;

    return reg;

}




