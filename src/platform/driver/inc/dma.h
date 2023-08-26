

#ifndef __DMAC_H__
#define __DMAC_H__

/* Allow C++ to use this header */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Macro definitions for DMA controller limits
#define DMAC_MAX_CHANNELS    8
#define DMAC_MAX_INTERRUPTS  5

// Macro definitions for the flow control mode of the DMA Controller
#define DMAC_DMA_FC_ONLY     0x0
#define DMAC_SRC_FC_ONLY     0x1
#define DMAC_DST_FC_ONLY     0x2
#define DMAC_ANY_FC          0x3

// Macro definitions for channel masks

// i.e if DMAC_MAX_CHANNELS = 8 : = 0xff
#define DMAC_MAX_CH_MASK  \
     (~(~0 << DMAC_MAX_CHANNELS))

// i.e. if NUM_CHANNELS = 3 : = 0xf8
#define DMAC_CH_MASK \
     (DMAC_MAX_CH_MASK & (DMAC_MAX_CH_MASK << NUM_CHANNELS))

// i.e. if NUM_CHANNELS = 3 : = 0x7
#define DMAC_CH_EN_MASK \
     (DMAC_MAX_CH_MASK & ~(DMAC_CH_MASK))

// i.e. if NUM_CHANNELS = 3 : = 0x707
#define DMAC_CH_ALL_MASK \
     ((DMAC_CH_EN_MASK) + ((DMAC_CH_EN_MASK) << DMAC_MAX_CHANNELS))

// get a dw_dmac_channel_number from a channel index
#define DMAC_CH_NUM(ch_idx)     (0x101 << (ch_idx))

//Parameters definitions
//Valid channels
#define NUM_CHANNELS         3

// Maximum value of burst transaction size that can be programmed
// for channel x (CTLx.SRC_MSIZE and CTLx.DEST_MSIZE).
#define CHx_MAX_MULT_SIZE 8
//Creates the specified number of handshaking interfaces
#define NUM_HS_INT        16




typedef enum DMA_Channel {
    Dmac_no_channel   = 0x0000,
    Dmac_channel0     = 0x0101,
    Dmac_channel1     = 0x0202,
    Dmac_channel2     = 0x0404,
    Dmac_channel3     = 0x0808,
    Dmac_channel4     = 0x1010,
    Dmac_channel5     = 0x2020,
    Dmac_channel6     = 0x4040,
    Dmac_channel7     = 0x8080,
    Dmac_all_channels = 0xffff
}DMA_ChannelDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used to select the source and/or the
 *  destination for a specific DMA channel when using some
 *  of the driver's API functions.
 */
typedef enum DMA_SrcDstSelect {
    Dmac_src     = 0x1,
    Dmac_dst     = 0x2,
    Dmac_src_dst = 0x3
}DMA_SrcDstSelectDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used to select the interrupt type on a specified
 *  DMA channel when using the specified driver API functions to access
 *  interrupt registers within the DMA Controller.
 */
typedef enum DMA_Irq {
    Dmac_irq_none       = 0x00,     // no interrupts
    Dmac_irq_tfr        = 0x01,     // transfer complete
    Dmac_irq_block      = 0x02,     // block transfer complete
    Dmac_irq_srctran    = 0x04,     // source transaction complete
    Dmac_irq_dsttran    = 0x08,     // destination transaction complete
    Dmac_irq_err        = 0x10,     // error
    Dmac_irq_all        = 0x1f      // all interrupts
}DMA_IrqDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the transfer flow device
 *  (memory or peripheral device) and for setting the flow control
 *  device for the DMA transfer when using the specified driver
 *  API functions.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit field(s): (x = channel number)
 *    - CTLx.TT_FC
 */
typedef enum DMA_TransferFlow {
    Dmac_mem2mem_dma    = 0x0, /* mem to mem - DMAC   flow ctlr */
    Dmac_mem2prf_dma    = 0x1, /* mem to prf - DMAC   flow ctlr */
    Dmac_prf2mem_dma    = 0x2, /* prf to mem - DMAC   flow ctlr */
    Dmac_prf2prf_dma    = 0x3, /* prf to prf - DMAC   flow ctlr */
    Dmac_prf2mem_prf    = 0x4, /* prf to mem - periph flow ctlr */
    Dmac_prf2prf_srcprf = 0x5, /* prf to prf - source flow ctlr */
    Dmac_mem2prf_prf    = 0x6, /* mem to prf - periph flow ctlr */
    Dmac_prf2prf_dstprf = 0x7  /* prf to prf - dest   flow ctlr */
}DMA_TransferFlowDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the burst transfer length
 *  on the source and/or destination of a DMA channel when using the
 *  specified driver API functions. These transfer length values do
 *  not relate to the AMBA HBURST parameter.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit field(s): (x = channel number)
 *    - CTLx.SRC_MSIZE, CTLx.DEST_MSIZE
 */
typedef enum DMA_BurstTransLength {
    Dmac_msize_1   = 0x0,
    Dmac_msize_4   = 0x1,
    Dmac_msize_8   = 0x2,
    Dmac_msize_16  = 0x3,
    Dmac_msize_32  = 0x4,
    Dmac_msize_64  = 0x5,
    Dmac_msize_128 = 0x6,
    Dmac_msize_256 = 0x7
}DMA_BurstTransLengthDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the address increment
 *  type for the source and/or destination on a DMA channel when using
 *  the specified driver API functions.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit-field(s): (x = channel number)
 *    - CTLx.SINC, CTLx.DINC
 */
typedef enum DMA_AddressIncrement {
    Dmac_addr_increment = 0x0,
    Dmac_addr_decrement = 0x1,
    Dmac_addr_nochange  = 0x2
}DMA_AddressIncrementDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the transfer width for the
 *  source and/or destination on a DMA channel when using the specified
 *  driver API functions. This data type maps directly to the AMBA AHB
 *  HSIZE parameter.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit field(s): (x = channel number)
 *    - CTLx.SRC_TR_WIDTH, CTLx.DST_TR_WIDTH
 */
typedef enum DMA_TransferWidth {
    Dmac_trans_width_8   = 0x0,
    Dmac_trans_width_16  = 0x1,
    Dmac_trans_width_32  = 0x2,
    Dmac_trans_width_64  = 0x3,
    Dmac_trans_width_128 = 0x4,
    Dmac_trans_width_256 = 0x5
}DMA_TransferWidthDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used to select a software or hardware interface
 *  when using the specified driver API functions to access the
 *  handshaking interface on a specified DMA channel.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit field(s): (x = channel number)
 *    - CFGx.HS_SEL_SRC, CFGx.HS_SEL_DST
 */
typedef enum DMA_SwHwHsSelect {
    Dmac_hs_hardware   = 0x0,
    Dmac_hs_software   = 0x1
}DMA_SwHwHsSelectDef;

/****
 * DESCRIPTION
 *  This data type is used for selecting the handshaking interface
 *  number for the source and/or destination on a DMA channel when
 *  using the specified driver API functions.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit-field(s): (x = channel number)
 *    - CFGx.DEST_PER, CFGx.SRC_PER
 */
typedef enum DMA_HsInterface {
    Dmac_hs_if0  = 0x0, //uart0_tx
    Dmac_hs_if1  = 0x1, //uart0_rx
    Dmac_hs_if2  = 0x2, //uart1_tx
    Dmac_hs_if3  = 0x3, //uart1_rx
    Dmac_hs_if4  = 0x4, //i2c0_tx
    Dmac_hs_if5  = 0x5, //i2c0_rx
    Dmac_hs_if6  = 0x6, //i2c1_tx
    Dmac_hs_if7  = 0x7, //i2c1_rx
    Dmac_hs_if8  = 0x8, //spi0_tx
    Dmac_hs_if9  = 0x9, //spi0_rx
    Dmac_hs_if10 = 0xa, //spi1_tx
    Dmac_hs_if11 = 0xb, //spi1_rx
    Dmac_hs_if12 = 0xc, //spi2_tx
    Dmac_hs_if13 = 0xd, //spi2_rx
    Dmac_hs_if14 = 0xe, //spi3_tx
    Dmac_hs_if15 = 0xf  //spi3_rx
}DMA_HsInterfaceDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the tranfer protection level
 *  on a DMA channel when using the specified driver API functions.
 *  This data type maps directly to the AMBA AHB HPROT parameter.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit-field(s): (x = channel number)
 *    - CFGx.PROTCTL
 */
typedef enum DMA_ProtLevel {
   Dmac_noncache_nonbuff_nonpriv_opcode = 0x0, /* default prot level */
   Dmac_noncache_nonbuff_nonpriv_data   = 0x1,
   Dmac_noncache_nonbuff_priv_opcode    = 0x2,
   Dmac_noncache_nonbuff_priv_data      = 0x3,
   Dmac_noncache_buff_nonpriv_opcode    = 0x4,
   Dmac_noncache_buff_nonpriv_data      = 0x5,
   Dmac_noncache_buff_priv_opcode       = 0x6,
   Dmac_noncache_buff_priv_data         = 0x7,
   Dmac_cache_nonbuff_nonpriv_opcode    = 0x8,
   Dmac_cache_nonbuff_nonpriv_data      = 0x9,
   Dmac_cache_nonbuff_priv_opcode       = 0xa,
   Dmac_cache_nonbuff_priv_data         = 0xb,
   Dmac_cache_buff_nonpriv_opcode       = 0xc,
   Dmac_cache_buff_nonpriv_data         = 0xd,
   Dmac_cache_buff_priv_opcode          = 0xe,
   Dmac_cache_buff_priv_data            = 0xf
}DMA_ProtLevelDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the FIFO mode on a DMA
 *  channel when using the specified driver API functions.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit field(s): (x = channel number)
 *    - CFGx.FIFO_MODE
 */
typedef enum DMA_FifoMode {
    Dmac_fifo_mode_single = 0x0,
    Dmac_fifo_mode_half   = 0x1
}DMA_FifoModeDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the flow control mode on a
 *  DMA channel when using the specified driver API functions.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit-field(s): (x = channel number)
 *    - CFGx.FCMODE
 */
typedef enum DMA_FlowCtrlMode {
    Dmac_data_prefetch_enabled  = 0x0,
    Dmac_data_prefetch_disabled = 0x1
}DMA_FlowCtrlModeDef;
/*****/

/****
 * DESCRIPTION
 *  This data type is used for selecting the polarity level for the
 *  source and/or destination on a DMA channel's handshaking interface
 *  when using the specified driver API functions.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s) / bit-field(s): (x = channel number)
 *    - CFGx.SRC_HS_POL, CFGx.DST_HS_POL
 */
typedef enum DMA_PolarityLevel {
    Dmac_active_high = 0x0,
    Dmac_active_low  = 0x1
}DMA_PolarityLevelDef;
/*****/


/****
 * DESCRIPTION
 *  This data type is used for selecting the priority level of a DMA
 *  channel when using the specified driver API functions.
 * NOTES
 *  This data type relates directly to the following DMA Controller
 *  register(s)/bit field(s): (x = channel number)
 *    - CFGx.CH_PRIOR
 */
typedef enum DMA_ChannelPriority {
    Dmac_priority_0 = 0x0,
    Dmac_priority_1 = 0x1,
    Dmac_priority_2 = 0x2,
    Dmac_priority_3 = 0x3,
    Dmac_priority_4 = 0x4,
    Dmac_priority_5 = 0x5,
    Dmac_priority_6 = 0x6,
    Dmac_priority_7 = 0x7
}DMA_ChannelPriorityDef;
/*****/

/****
 * DESCRIPTION
 *  This structure is used to set configuration parameters for
 *  a channel in the DMA Controller. All of these configuration
 *  parameters must be programmed into the DMA controller before
 *  enabling the channel. The members of this structure map directly
 *  to a channel's register/bit field within the DMAC device.
 * NOTES
 *  To initialize the structure, the user should call the
 *  dw_dmac_getChannelConfig() API function after reset of the DMA
 *  controller. This sets the dw_dmac_channel_config structure members
 *  to the DMA controllers reset values. This allows the user to
 *  change only the structure members that need to be different from
 *  the default values and then call the dw_dmac_setChannelConfig()
 *  function to set up the DMA channel transfer.
 */
typedef struct DMA_ChannelConfig {
    //CTLx
    uint32_t                        sar;
    uint32_t                        dar;
    DMA_BurstTransLengthDef         ctlSrcMsize;
    DMA_BurstTransLengthDef         ctlDstMsize;
    DMA_AddressIncrementDef         ctlSinc;
    DMA_AddressIncrementDef         ctlDinc;
    DMA_TransferWidthDef            ctlSrcTrWidth;
    DMA_TransferWidthDef            ctlDstTrWidth;
    //CFGx
    DMA_HsInterfaceDef              cfgDstPer;
    DMA_HsInterfaceDef              cfgSrcPer;

    DMA_PolarityLevelDef            cfgSrcHsPol;
    DMA_PolarityLevelDef            cfgDstHsPol;
    DMA_SwHwHsSelectDef             cfgHsSelSrc;
    DMA_SwHwHsSelectDef             cfgHsSelDst;

    uint32_t                        ctlBlockTs;
    DMA_TransferFlowDef             ctlTtFc;
    bool                            ctlIntEn;
    DMA_ProtLevelDef                cfgProtCtl;
    DMA_FifoModeDef                 cfgFifoMode;
    DMA_FlowCtrlModeDef             cfgFcMode;

    DMA_ChannelPriorityDef          cfgChPrior;
}DMA_ChannelConfigDef;

void DMA_EnableClock(void);
void DMA_DisableClock(void);

int DMA_Init(DMA_T *DMA_Def);

void DMA_Enable(DMA_T *DMA_Def);
int DMA_Disable(DMA_T *DMA_Def);
bool DMA_IsEnabled(DMA_T *DMA_Def);

int DMA_EnableChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch);
int DMA_DisableChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch);
bool DMA_IsChannelEnabled(DMA_T *DMA_Def, DMA_ChannelDef Ch);

uint8_t DMA_GetChannelEnableReg(DMA_T *DMA_Def);

int DMA_EnableChannelIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch);
int DMA_DisableChannelIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch);
bool DMA_IsChannelIrqEnabled(DMA_T *DMA_Def, DMA_ChannelDef Ch);

DMA_ChannelDef DMA_GetFreeChannel(DMA_T *DMA_Def);

int DMA_SuspendChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch);
int DMA_ResumeChannel(DMA_T *DMA_Def, DMA_ChannelDef Ch);
bool DMA_IsChannelSuspended(DMA_T *DMA_Def, DMA_ChannelDef Ch);

int DMA_ClearIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq);
int DMA_MaskIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq);
int DMA_UnmaskIrq(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq);
bool DMA_IsIrqMasked(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq);
bool DMA_IsRawIrqActive(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq);
bool DMA_IsIrqActive(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_IrqDef Irq);
bool DMA_IsCombinedIrqActive(DMA_T *DMA_Def, DMA_IrqDef Irq);

int DMA_SetChannelConfig(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ChannelConfigDef *pConfig);
int DMA_GetChannelConfig(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ChannelConfigDef *pConfig);

bool DMA_IsBlockTransDone(DMA_T *DMA_Def, DMA_ChannelDef Ch);


int DMA_SetAddress(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, uint32_t address);
int DMA_GetAddress(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, uint32_t *pAddr);

int DMA_SetBlockTransSize(DMA_T *DMA_Def, DMA_ChannelDef Ch, uint16_t blockSize);
int DMA_GetBlockTransSize(DMA_T *DMA_Def, DMA_ChannelDef Ch, uint16_t * pBlockSize)
;

int DMA_SetMemPeriphFlowCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_TransferFlowDef tt_fc);
int DMA_GetMemPeriphFlowCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_TransferFlowDef * const pTranferFlow)
;

int DMA_SetBurstTransLength(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel, DMA_BurstTransLengthDef xfLength);
 int DMA_GetBurstTransLength( DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel,
                                                             DMA_BurstTransLengthDef * pBurstLen);

int DMA_SetAddressInc(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_AddressIncrementDef addrInc);
int DMA_GetAddressInc(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel,
                                                                DMA_AddressIncrementDef *pAddrInc);

int DMA_SetTransWidth(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_TransferWidthDef xfWidth);
int DMA_GetTransWidth(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel,
                                                                                   DMA_TransferWidthDef *pWidth);

int DMA_SetHsInterface(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel,DMA_HsInterfaceDef hsIf);
int DMA_GetHsInterface(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_SrcDstSelectDef sdSel,
                                                                                 DMA_HsInterfaceDef *pHsIf);

int DMA_SetProtCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ProtLevelDef protLvl);
int DMA_GetProtCtl(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ProtLevelDef * pLvl);

int DMA_SetFifoMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FifoModeDef fifoMode);
int DMA_GetFifoMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FifoModeDef *pFifoMode);

int DMA_SetFlowCtlMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FlowCtrlModeDef fcMode);
int DMA_GetFlowCtlMode(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_FlowCtrlModeDef *pFcMode);

int DMA_SetHsPolarity(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_PolarityLevelDef polLevel);
int DMA_GetHsPolarity(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_PolarityLevelDef *pPolLevel);

int DMA_SetHandshakingMode(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_SwHwHsSelectDef hsHwSwSel);
int DMA_GetHandshakingMode(DMA_T *DMA_Def, DMA_ChannelDef Ch,DMA_SrcDstSelectDef sdSel, DMA_SwHwHsSelectDef *pHsSel);

bool DMA_IsFifoEmpty(DMA_T *DMA_Def, DMA_ChannelDef Ch);

int DMA_SetChannelPriority(DMA_T *DMA_Def, DMA_ChannelDef Ch, DMA_ChannelPriorityDef chPriority);
int DMA_GetChannelPriority(DMA_T * DMA_Def, DMA_ChannelDef Ch, DMA_ChannelPriorityDef *pPriority);

unsigned DMA_GetChannelIndex(DMA_ChannelDef Ch);

uint32_t DMA_GetIntStatusReg(DMA_T * DMA_Def);
/*****/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* DW_AHB_DMAC_PUBLIC_H */

