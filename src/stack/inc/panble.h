

#ifndef __PANBLE_H__
#define __PANBLE_H__

#include "pn102Series.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char      uint8;   //  8 bits
typedef char               int8;    //  8 bits
typedef unsigned short     uint16;  // 16 bits
typedef short              int16;   // 16 bits
typedef unsigned long      uint32;  // 32 bits
typedef long               int32;   // 32 bits
typedef unsigned long long uint64;  // 64 bits
typedef long long          int64;   // 64 bits

/*---------------------- BLE Baseband Register -------------------------*/

typedef struct {

    /**
     * RWBTLECNTL
     * ===================================================================================================
     * Offset: 0x00  BT LE Control Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t CNTL;

    /**
     * VERSION
     * ===================================================================================================
     * Offset: 0x04  Version Register
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t VERSION;
	  __IO  uint32_t VERSION;

    /**
     * RWBTLECONF
     * ===================================================================================================
     * Offset: 0x08  BT LE Config Register
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t CONF;
    __IO  uint32_t CONF;

    /**
     * INTCNTL
     * ===================================================================================================
     * Offset: 0x0C  Interrupt Control Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t INTCNTL;

    /**
     * INTSTAT
     * ===================================================================================================
     * Offset: 0x10  Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t INTSTAT;
    __IO  uint32_t INTSTAT;

    /**
     * INTRAWSTAT
     * ===================================================================================================
     * Offset: 0x14  Interrupt Raw Status Register
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t INTRAWSTAT;
    __IO  uint32_t INTRAWSTAT;

    /**
     * INTACK
     * ===================================================================================================
     * Offset: 0x18  Interrupt Acknowledgement Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t INTACK;

    /**
     * BASETIMECNT
     * ===================================================================================================
     * Offset: 0x1C  Base Time Counter Register
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t BASETIMECNT;
    __IO  uint32_t BASETIMECNT;

    /**
     * FINETIMECNT
     * ===================================================================================================
     * Offset: 0x20  Fine Time Counter Register
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t FINETIMECNT;
    __IO  uint32_t FINETIMECNT;

    /**
     * BDADDRL
     * ===================================================================================================
     * Offset: 0x24  Bluetooth Low Energy Device Address Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t BDADDRL;

    /**
     * BDADDRU
     * ===================================================================================================
     * Offset: 0x28  Bluetooth Low Energy Device Address and Privacy indicator Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t BDADDRU;

    /**
     * CURRENTRXDESCPTR
     * ===================================================================================================
     * Offset: 0x2C  Current RX Descriptor Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t CURRENTRXDESCPTR;

    /**
     * DEEPSLCNTL
     * ===================================================================================================
     * Offset: 0x30  Deep Sleep Counter Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t DEEPSLCNTL;

    /**
     * DEEPSLWKUP
     * ===================================================================================================
     * Offset: 0x34  Deep Sleep Wake Up Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t DEEPSLWKUP;

    /**
     * DEEPSLSTAT
     * ===================================================================================================
     * Offset: 0x38  Deep Sleep Status Register
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t DEEPSLSTAT;
		__IO  uint32_t DEEPSLSTAT;

    /**
     * ENBPRESET
     * ===================================================================================================
     * Offset: 0x3C  Interrupt Raw Status Register
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t ENBPRESET;

    /**
     * FINECNTCORR
     * ===================================================================================================
     * Offset: 0x40
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t FINECNTCORR;

    /**
     * BASETIMECNTCORR
     * ===================================================================================================
     * Offset: 0x44
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t BASETIMECNTCORR;

    /**
     * NULL0
     * ===================================================================================================
     * Offset: 0x48
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t NULL0;

    /**
     * NULL1
     * ===================================================================================================
     * Offset: 0x4C
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t NULL1;

    /**
     * DIAGCNTL
     * ===================================================================================================
     * Offset: 0x50
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t DIAGCNTL;

    /**
     * DIAGSTAT
     * ===================================================================================================
     * Offset: 0x54
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t DIAGSTAT;
    __IO  uint32_t DIAGSTAT;

    /**
     * DEBUGADDMAX
     * ===================================================================================================
     * Offset: 0x58
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t DEBUGADDMAX;

    /**
     * DEBUGADDMIN
     * ===================================================================================================
     * Offset: 0x5C
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t DEBUGADDMIN;

    /**
     * ERRORTYPESTAT
     * ===================================================================================================
     * Offset: 0x60
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t ERRORTYPESTAT;
    __IO  uint32_t ERRORTYPESTAT;

    /**
     * SWPROFILING
     * ===================================================================================================
     * Offset: 0x64
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t SWPROFILING;

    /**
     * Null2
     * ===================================================================================================
     * Offset: 0x68
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t Null2;

    /**
     * Null3
     * ===================================================================================================
     * Offset: 0x6C
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t Null3;

    /**
     * RADIOCNTL0
     * ===================================================================================================
     * Offset: 0x70
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t RADIOCNTL0;

    /**
     * RADIOCNTL1
     * ===================================================================================================
     * Offset: 0x74
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t RADIOCNTL1;

    /**
     * RADIOCNTL2
     * ===================================================================================================
     * Offset: 0x78
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t RADIOCNTL2;

    /**
     * RADIOCNTL3
     * ===================================================================================================
     * Offset: 0x7C
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t RADIOCNTL3;

    /**
     * RADIOPWRUPDN
     * ===================================================================================================
     * Offset: 0x80
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t RADIOPWRUPDN;

    /**
     * RADIOCNTL4
     * ===================================================================================================
     * Offset: 0x84
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t RADIOCNTL4;

    /**
     * NULL4
     * ===================================================================================================
     * Offset: 0x88
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t NULL4;

    /**
     * NULL5
     * ===================================================================================================
     * Offset: 0x8C
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t NULL5;

    /**
     * ADVCHMAP
     * ===================================================================================================
     * Offset: 0x90
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t ADVCHMAP;


		__IO  uint32_t NULL6;
		__IO  uint32_t NULL7;
		__IO  uint32_t NULL8;

    /**
     * ADVTIM
     * ===================================================================================================
     * Offset: 0xA0
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t ADVTIM;

    /**
     * ACTSCANSTAT
     * ===================================================================================================
     * Offset: 0xA4
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t ACTSCANSTAT;
    __IO  uint32_t ACTSCANSTAT;

		__IO  uint32_t NULL9;
		__IO  uint32_t NULL10;

    /**
     * WLPUBADDPTR
     * ===================================================================================================
     * Offset: 0xB0
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t WLPUBADDPTR;

    /**
     * WLPRIVADDPTR
     * ===================================================================================================
     * Offset: 0xB4
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t WLPRIVADDPTR;

    /**
     * WLNBDEV
     * ===================================================================================================
     * Offset: 0xB8
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t WLNBDEV;

		__IO  uint32_t NULL11;

    /**
     * AESCNTL
     * ===================================================================================================
     * Offset: 0xC0
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t AESCNTL;

    /**
     * AESKEY31_0
     * ===================================================================================================
     * Offset: 0xC4
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t AESKEY31_0;

    /**
     * AESKEY63_32
     * ===================================================================================================
     * Offset: 0xC8
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t AESKEY63_32;

    /**
     * AESKEY95_64
     * ===================================================================================================
     * Offset: 0xCC
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t AESKEY95_64;

    /**
     * AESKEY127_96
     * ===================================================================================================
     * Offset: 0xD0
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t AESKEY127_96;

    /**
     * AESPTR
     * ===================================================================================================
     * Offset: 0xD4
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t AESPTR;

    /**
     * TXMICVAL
     * ===================================================================================================
     * Offset: 0xD8
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t TXMICVAL;
    __IO  uint32_t TXMICVAL;

    /**
     * RXMICVAL
     * ===================================================================================================
     * Offset: 0xDC
     * ---------------------------------------------------------------------------------------------------
    */
//     __I  uint32_t RXMICVAL;
    __IO  uint32_t RXMICVAL;

    /**
     * RFTESTCNTL
     * ===================================================================================================
     * Offset: 0xE0
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t RFTESTCNTL;

		__IO  uint32_t NULL12;
		__IO  uint32_t NULL13;
		__IO  uint32_t NULL14;


    /**
     * TIMGENCNTL
     * ===================================================================================================
     * Offset: 0xF0
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t TIMGENCNTL;

    /**
     * GROSSTIMTGT
     * ===================================================================================================
     * Offset: 0xF4
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t GROSSTIMTGT;

    /**
     * FINETIMTGT
     * ===================================================================================================
     * Offset: 0xF8
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t FINETIMTGT;

    /**
     * SAMPLECLK
     * ===================================================================================================
     * Offset: 0xFC
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t SAMPLECLK;

    /**
     * WLANCNTL
     * ===================================================================================================
     * Offset: 0x100
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t WLANCNTL;

    /**
     * BLEMPRIO0
     * ===================================================================================================
     * Offset: 0x104
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t BLEMPRIO0;

    /**
     * BLEMPRIO1
     * ===================================================================================================
     * Offset: 0x108
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t BLEMPRIO1;

		__IO  uint32_t NULL15;

    /**
     * BLEPRIOSCHARB
     * ===================================================================================================
     * Offset: 0x110
     * ---------------------------------------------------------------------------------------------------
    */
    __IO  uint32_t BLEPRIOSCHARB;

} BB_T;

/**@}*/ /* end of BLEBB register group */

/*---------------------- BLE Modem Register -------------------------*/


typedef struct {
	__IO  uint32_t VERSION;						// 0x80'
	__IO  uint32_t MDM_CNTL;					// 0x81'
	__IO  uint32_t CLKCNTL;						// 0x82'
	__IO  uint32_t RX_STARTUPDEL;			// 0x83'
	__IO  uint32_t TX_STARTUPDEL;			// 0x84'
	__IO  uint32_t TX_GFSKMODE;				// 0x85'
	__IO  uint32_t DIAGCNTL;					// 0x86'
	__IO  uint32_t RX_PWR;						// 0x87'
	__IO  uint32_t NULL16;
	__IO  uint32_t NULL17;
	__IO  uint32_t NULL18;
	__IO  uint32_t NULL19;
	__IO  uint32_t NULL20;
	__IO  uint32_t NULL21;
	__IO  uint32_t NULL22;
	__IO  uint32_t NULL23;
	__IO  uint32_t RXFE_CNTL;					// 0x90'
	__IO  uint32_t FCS_IFMHZ;					// 0x91'
	__IO  uint32_t NULL25;
	__IO  uint32_t NULL26;
	__IO  uint32_t NULL27;
	__IO  uint32_t NULL28;
	__IO  uint32_t NULL29;
	__IO  uint32_t NULL30;
	__IO  uint32_t NULL31;
	__IO  uint32_t NULL32;
	__IO  uint32_t NULL33;
	__IO  uint32_t NULL34;
	__IO  uint32_t NULL35;
	__IO  uint32_t NULL36;
	__IO  uint32_t NULL37;
  __IO  uint32_t NULL38;

	__IO  uint32_t RXGFSK_CNTL;				// 0xA0'
	__IO  uint32_t GFO_P2PTHR;				// 0xA1'
	__IO  uint32_t GFO_REFINIT;				// 0xA2'
	__IO  uint32_t GFO_GFSKDETECT;		// 0xA3'
	__IO  uint32_t GFO_SETKDELSW;			// 0xA4'
	__IO  uint32_t GFO_SETKDELPL;			// 0xA5'
	__IO  uint32_t GFO_CONVDEL;				// 0xA6'
	__IO  uint32_t GFO_ESTSW;					// 0xA7'
	__IO  uint32_t GFO_ESTPL;					// 0xA8'
	__IO  uint32_t GFO_INIT;					// 0xA9'
	__IO  uint32_t ACSYNCTUNE;				// 0xAA'
	__IO  uint32_t PE_POWERTHR;				// 0xAB'
	__IO  uint32_t DPLATENCY_CNTL;		// 0xAC'
  __IO  uint32_t NULL0;
  __IO  uint32_t NULL1;
  __IO  uint32_t NULL2;
	__IO  uint32_t RXDPSK_CNTL;				// 0xB0'
	__IO  uint32_t TE_TIMEINIT;				// 0xB1'
	__IO  uint32_t DFD_KFACTOR;				// 0xB2'
	__IO  uint32_t COC_KFACTOR;				// 0xB3'
	__IO  uint32_t COC_THR;						// 0xB4'
	__IO  uint32_t COC_TD;						// 0xB5'
	__IO  uint32_t COC_TENABLE;				// 0xB6'
	__IO  uint32_t POW_DIGGAINOFF;		// 0xB7'
	__IO  uint32_t FOE_STATUS;				// 0xB8'
	__IO  uint32_t TECOC_STATUS;			// 0xB9'
  __IO  uint32_t NULL3;
	__IO  uint32_t NULL4;
  __IO  uint32_t NULL5;
  __IO  uint32_t NULL6;
	__IO  uint32_t NULL7;
	__IO  uint32_t NULL8;
	__IO  uint32_t GSG_DEN;						// 0xC0'
	__IO  uint32_t GSG_LSTVAL;				// 0xC1'
	__IO  uint32_t GSG_NOM;						// 0xC2'
	__IO  uint32_t GSG_THREPS;					// 0xC3'
	__IO  uint32_t GSG_VCO_DEN;				// 0xC4'
	__IO  uint32_t GSG_VCO_NOM;				// 0xC5'
	__IO  uint32_t FM2PSW_LAT;				// 0xC6'
	__IO  uint32_t GSG_DPHI_DEN;				// 0xC7'
	__IO  uint32_t GSG_DPHI_NOM;				// 0xC8'
	__IO  uint32_t NULL9;
	__IO  uint32_t NULL10;
	__IO  uint32_t NULL11;
	__IO  uint32_t NULL12;
	__IO  uint32_t NULL13;
	__IO  uint32_t NULL14;
	__IO  uint32_t NULL15;
	__IO  uint32_t DSG_DEN;						// 0xD0'
	__IO  uint32_t DSG_NOM;						// 0xD1'

} MDM_T;


/**@}*/ /* end of BLEMDM register group */

/*---------------------- BLE Control Structure -------------------------*/

typedef struct {
	__IO	uint32_t	CS_STR00;
	__IO	uint32_t	CS_STR04;
	__IO	uint32_t	CS_STR08;
	__IO	uint32_t	CS_STR0C;
	__IO	uint32_t	CS_STR10;
	__IO	uint32_t	CS_STR14;
	__IO	uint32_t	CS_STR18;
	__IO	uint32_t	CS_STR1C;
	__IO	uint32_t	CS_STR20;
	__IO	uint32_t	CS_STR24;
	__IO	uint32_t	CS_STR28;
	__IO	uint32_t	CS_STR2C;
	__IO	uint32_t	CS_STR30;
	__IO	uint32_t	CS_STR34;
	__IO	uint32_t	CS_STR38;
	__IO	uint32_t	CS_STR3C;
	__IO	uint32_t	CS_STR40;
	__IO	uint32_t	CS_STR44;
	__IO	uint32_t	CS_STR48;
	__IO	uint32_t	CS_STR4C;
	__IO	uint32_t	CS_STR50;
	__IO	uint32_t	CS_STR54;
	__IO	uint32_t	CS_STR58;
	__IO	uint32_t	CS_STR5C;
	__IO	uint32_t	CS_STR60;
	__IO	uint32_t	CS_STR64;
	__IO	uint32_t	CS_STR68;
	__IO	uint32_t	CS_STR6C;
}CS_T;


/* end of BLE Control Structure */

#define BB_CS_STR00_FORMAT_Pos                (0)
#define BB_CS_STR00_FORMAT_Msk                (0x1Ful << BB_CS_STR00_FORMAT_Pos)

#define BB_CS_STR00_RXCRYPT_EN_Pos            (6)
#define BB_CS_STR00_RXCRYPT_EN_Msk            (0x1ul << BB_CS_STR00_RXCRYPT_EN_Pos)

#define BB_CS_STR00_TXCRYPT_EN_Pos            (7)
#define BB_CS_STR00_TXCRYPT_EN_Msk            (0x1ul << BB_CS_STR00_TXCRYPT_EN_Pos)

#define BB_CS_STR00_DNABORT_EN_Pos            (8)
#define BB_CS_STR00_DNABORT_EN_Msk            (0x1ul << BB_CS_STR00_DNABORT_EN_Pos)

#define BB_CS_STR00_RXBSY_EN_Pos              (9)
#define BB_CS_STR00_RXBSY_EN_Msk              (0x1ul << BB_CS_STR00_RXBSY_EN_Pos)

#define BB_CS_STR00_TXBSY_EN_Pos              (10)
#define BB_CS_STR00_TXBSY_EN_Msk              (0x1ul << BB_CS_STR00_TXBSY_EN_Pos)

#define BB_CS_STR00_PTI_Pos                   (12)                                      /*!< BB::CS: PTI Position            */
#define BB_CS_STR00_PTI_Msk                   (0xFul << BB_CS_STR00_PTI_Pos)                  /*!< BB::CS: PTI Mask                */

#define BB_CS_STR00_SYNCWORDL_Pos             (16)
#define BB_CS_STR00_SYNCWORDL_Msk             (0xFFFFul << BB_CS_STR00_SYNCWORDL_Pos)

#define BB_CS_STR04_SYNCWORDH_Pos             (0)
#define BB_CS_STR04_SYNCWORDH_Msk             (0xFFFFul << BB_CS_STR04_SYNCWORDH_Pos)

#define BB_CS_STR04_CRCINIT0_Pos              (16)
#define BB_CS_STR04_CRCINIT0_Msk              (0xFFFFul << BB_CS_STR04_CRCINIT0_Pos)

#define BB_CS_STR08_CRCINIT1_Pos              (0)
#define BB_CS_STR08_CRCINIT1_Msk              (0xFFul << BB_CS_STR08_CRCINIT1_Pos)

#define BB_CS_STR08_FILTER_POLICY_Pos         (8)
#define BB_CS_STR08_FILTER_POLICY_Msk         (0xFFul << BB_CS_STR08_FILTER_POLICY_Pos)

#define BB_CS_STR08_CH_IDX_Pos                (16)
#define BB_CS_STR08_CH_IDX_Msk                (0x3Ful << BB_CS_STR08_CH_IDX_Pos)

#define BB_CS_STR08_HOPINT_Pos                (24)
#define BB_CS_STR08_HOPINT_Msk                (0x1Ful << BB_CS_STR08_HOPINT_Pos)

#define BB_CS_STR08_FH_EN_Pos                 (31)
#define BB_CS_STR08_FH_EN_Msk                 (0x1Ful << BB_CS_STR08_FH_EN_Pos)

#define BB_CS_STR0C_TXPWR_Pos                 (0)
#define BB_CS_STR0C_TXPWR_Msk                 (0xFFul << BB_CS_STR0C_TXPWR_Pos)

#define BB_CS_STR0C_RXTHR_Pos                 (8)
#define BB_CS_STR0C_RXTHR_Msk                 (0xFul << BB_CS_STR0C_RXTHR_Pos)

#define BB_CS_STR0C_NESN_Pos                  (12)
#define BB_CS_STR0C_NESN_Msk                  (0x1ul << BB_CS_STR0C_NESN_Pos)

#define BB_CS_STR0C_SN_Pos                    (13)
#define BB_CS_STR0C_SN_Msk                    (0x1ul << BB_CS_STR0C_SN_Pos)

#define BB_CS_STR0C_LASTEMPTY_Pos             (14)
#define BB_CS_STR0C_LASTEMPTY_Msk             (0x1ul << BB_CS_STR0C_LASTEMPTY_Pos)

#define BB_CS_STR0C_RXBUFF_FULL_Pos           (15)
#define BB_CS_STR0C_RXBUFF_FULL_Msk           (0x1ul << BB_CS_STR0C_RXBUFF_FULL_Pos)

#define BB_CS_STR0C_RXWINSZ_Pos               (16)
#define BB_CS_STR0C_RXWINSZ_Msk               (0x3FFFul << BB_CS_STR0C_RXWINSZ_Pos)

#define BB_CS_STR0C_RXWIDE_Pos                (31)
#define BB_CS_STR0C_RXWIDE_Msk                (0x1ul << BB_CS_STR0C_RXWIDE_Pos)

#define BB_CS_STR10_TXDESCPTR_Pos             (0)
#define BB_CS_STR10_TXDESCPTR_Msk             (0x3FFFul << BB_CS_STR10_TXDESCPTR_Pos)

#define BB_CS_STR10_RXDESCPTR_Pos             (16)
#define BB_CS_STR10_RXDESCPTR_Msk             (0x3FFFul << BB_CS_STR10_RXDESCPTR_Pos)

#define BB_CS_STR10_RXNRDY_Pos                (31)
#define BB_CS_STR10_RXNRDY_Msk                (0x1ul << BB_CS_STR10_RXNRDY_Pos)

#define BB_CS_STR14_WINOFFSET_Pos             (0)
#define BB_CS_STR14_WINOFFSET_Msk             (0xFFFFul << BB_CS_STR14_WINOFFSET_Pos)

#define BB_CS_STR14_MINEVTIME_Pos             (0)
#define BB_CS_STR14_MINEVTIME_Msk             (0xFFFFul << BB_CS_STR14_MINEVTIME_Pos)

#define BB_CS_STR14_MAXEVTIME_Pos             (16)
#define BB_CS_STR14_MAXEVTIME_Msk             (0xFFFFul << BB_CS_STR14_MAXEVTIME_Pos)

#define BB_CS_STR18_LLCHMAP0_Pos              (0)
#define BB_CS_STR18_LLCHMAP0_Msk              (0xFFFFul << BB_CS_STR18_LLCHMAP0_Pos)

#define BB_CS_STR18_LLCHMAP1_Pos              (16)
#define BB_CS_STR18_LLCHMAP1_Msk              (0xFFFFul << BB_CS_STR18_LLCHMAP1_Pos)

#define BB_CS_STR1C_LLCHMAP2_Pos              (0)
#define BB_CS_STR1C_LLCHMAP2_Msk              (0x1Ful << BB_CS_STR1C_LLCHMAP2_Pos)

#define BB_CS_STR1C_NBCHGOOD_Pos              (8)
#define BB_CS_STR1C_NBCHGOOD_Msk              (0x3Ful << BB_CS_STR1C_NBCHGOOD_Pos)

#define BB_CS_STR1C_FCNTOFFSET_Pos            (16)
#define BB_CS_STR1C_FCNTOFFSET_Msk            (0x3FFul << BB_CS_STR1C_FCNTOFFSET_Pos)

#define BB_CS_STR20_SK0_Pos                   (0)
#define BB_CS_STR20_SK0_Msk                   (0xFFFFul << BB_CS_STR20_SK0_Pos)

#define BB_CS_STR20_ADV_BD_ADDR0_Pos          (0)
#define BB_CS_STR20_ADV_BD_ADDR0_Msk          (0xFFFFul << BB_CS_STR20_ADV_BD_ADDR0_Pos)

#define BB_CS_STR20_SK1_Pos                   (16)
#define BB_CS_STR20_SK1_Msk                   (0xFFFFul << BB_CS_STR20_SK1_Pos)

#define BB_CS_STR20_ADV_BD_ADDR1_Pos          (16)
#define BB_CS_STR20_ADV_BD_ADDR1_Msk          (0xFFFFul << BB_CS_STR20_ADV_BD_ADDR1_Pos)

#define BB_CS_STR24_SK2_Pos                   (0)
#define BB_CS_STR24_SK2_Msk                   (0xFFFFul << BB_CS_STR24_SK2_Pos)

#define BB_CS_STR24_ADV_BD_ADDR2_Pos          (0)
#define BB_CS_STR24_ADV_BD_ADDR2_Msk          (0xFFFFul << BB_CS_STR24_ADV_BD_ADDR2_Pos)

#define BB_CS_STR24_SK3_Pos                   (16)
#define BB_CS_STR24_SK3_Msk                   (0xFFFFul << BB_CS_STR24_SK3_Pos)

#define BB_CS_STR24_ADV_BD_ADDR3_Pos          (16)
#define BB_CS_STR24_ADV_BD_ADDR3_Msk          (0x1ul << BB_CS_STR24_ADV_BD_ADDR3_Pos)

#define BB_CS_STR28_SK4_Pos                   (0)
#define BB_CS_STR28_SK4_Msk                   (0xFFFFul << BB_CS_STR28_SK4_Pos)

#define BB_CS_STR28_SK5_Pos                   (16)
#define BB_CS_STR28_SK5_Msk                   (0xFFFFul << BB_CS_STR28_SK5_Pos)

#define BB_CS_STR2C_SK6_Pos                   (0)
#define BB_CS_STR2C_SK6_Msk                   (0xFFFFul << BB_CS_STR2C_SK6_Pos)

#define BB_CS_STR2C_SK7_Pos                   (16)
#define BB_CS_STR2C_SK7_Msk                   (0xFFFFul << BB_CS_STR2C_SK7_Pos)

#define BB_CS_STR30_IV0_Pos                   (0)
#define BB_CS_STR30_IV0_Msk                   (0xFFFFul << BB_CS_STR30_IV0_Pos)

#define BB_CS_STR30_IV1_Pos                   (16)
#define BB_CS_STR30_IV1_Msk                   (0xFFFFul << BB_CS_STR30_IV1_Pos)

#define BB_CS_STR34_IV2_Pos                   (0)
#define BB_CS_STR34_IV2_Msk                   (0xFFFFul << BB_CS_STR34_IV2_Pos)

#define BB_CS_STR34_IV3_Pos                   (16)
#define BB_CS_STR34_IV3_Msk                   (0xFFFFul << BB_CS_STR34_IV3_Pos)

#define BB_CS_STR38_TXCCMPKTCNT0_Pos          (0)
#define BB_CS_STR38_TXCCMPKTCNT0_Msk          (0xFFFFul << BB_CS_STR38_TXCCMPKTCNT0_Pos)

#define BB_CS_STR38_TXCCMPKTCNT1_Pos          (16)
#define BB_CS_STR38_TXCCMPKTCNT1_Msk          (0xFFFFul << BB_CS_STR38_TXCCMPKTCNT1_Pos)

#define BB_CS_STR3C_TXCCMPKTCNT2_Pos          (0)
#define BB_CS_STR3C_TXCCMPKTCNT2_Msk          (0xFFFFul << BB_CS_STR3C_TXCCMPKTCNT2_Pos)

#define BB_CS_STR3C_RXCCMPKTCNT0_Pos          (16)
#define BB_CS_STR3C_RXCCMPKTCNT0_Msk          (0xFFFFul << BB_CS_STR3C_RXCCMPKTCNT0_Pos)

#define BB_CS_STR40_RXCCMPKTCNT1_Pos          (0)
#define BB_CS_STR40_RXCCMPKTCNT1_Msk          (0xFFFFul << BB_CS_STR40_RXCCMPKTCNT1_Pos)

#define BB_CS_STR40_RXCCMPKTCNT2_Pos          (16)
#define BB_CS_STR40_RXCCMPKTCNT2_Msk          (0xFFFFul << BB_CS_STR40_RXCCMPKTCNT2_Pos)

#define BB_CS_STR44_BTCNTSYNC0_Pos          (0)
#define BB_CS_STR44_BTCNTSYNC0_Msk          (0xFFFFul << BB_CS_STR44_BTCNTSYNC0_Pos)

#define BB_CS_STR44_BTCNTSYNC1_Pos          (16)
#define BB_CS_STR44_BTCNTSYNC1_Msk          (0x3Ful << BB_CS_STR44_BTCNTSYNC1_Pos)

#define BB_CS_STR48_FCNTSYNC_Pos            (0)
#define BB_CS_STR48_FCNTSYNC_Msk            (0x3FFul << BB_CS_STR48_FCNTSYNC_Pos)

#define BB_CS_STR48_TXDESCCNT_Pos             (16)
#define BB_CS_STR48_TXDESCCNT_Msk             (0xFFul << BB_CS_STR48_TXDESCCNT_Pos)

#define BB_CS_STR48_RXDESCCNT_Pos             (24)
#define BB_CS_STR48_RXDESCCNT_Msk             (0xFFul << BB_CS_STR48_RXDESCCNT_Pos)

#define BB_CS_STR4C_CURRENTPRIO_Pos           (0)
#define BB_CS_STR4C_CURRENTPRIO_Msk           (0x1Ful << BB_CS_STR4C_CURRENTPRIO_Pos)

#define BB_CS_STR4C_CONFLICT_Pos              (7)
#define BB_CS_STR4C_CONFLICT_Msk              (0x1ul << BB_CS_STR4C_CONFLICT_Pos)

#define BB_CS_STR4C_MINPRIO_Pos               (8)
#define BB_CS_STR4C_MINPRIO_Msk               (0x1Ful << BB_CS_STR4C_MINPRIO_Pos)

#define BB_CS_STR4C_PRIOINCSTEP_Pos           (14)
#define BB_CS_STR4C_PRIOINCSTEP_Msk           (0x7ul << BB_CS_STR4C_PRIOINCSTEP_Pos)

/*---------------------- BLE TX Descriptor -------------------------*/

typedef struct {
		__IO uint32_t DESC_CFG;
		__IO uint32_t txdata[32] ;
} TX_DESC_T;

#define BB_TXDESC_NEXTPTR_Pos           (0)
#define BB_TXDESC_NEXTPTR_Msk           (0x3FFFul << BB_TXDESC_NEXTPTR_Pos)

#define BB_TXDESC_TXCRYPTRDY_Pos        (14)
#define BB_TXDESC_TXCRYPTRDY_Msk        (0x1ul << BB_TXDESC_TXCRYPTRDY_Pos)

#define BB_TXDESC_TXDONE_Pos            (15)
#define BB_TXDESC_TXDONE_Msk            (0x1ul << BB_TXDESC_TXDONE_Pos)

/* TX Descripter For Data Packet */
#define BB_TXDESC_DAT_TXLLLID_Pos           (16)
#define BB_TXDESC_DAT_TXLLLID_Msk           (0x3ul << BB_TXDESC_DAT_TXLLLID_Pos)

#define BB_TXDESC_DAT_TXNESN_Pos            (18)
#define BB_TXDESC_DAT_TXNESN_Msk            (0x1ul << BB_TXDESC_DAT_TXNESN_Pos)

#define BB_TXDESC_DAT_TXSN_Pos              (19)
#define BB_TXDESC_DAT_TXSN_Msk              (0x1ul << BB_TXDESC_DAT_TXSN_Pos)

#define BB_TXDESC_DAT_TXMD_Pos              (20)
#define BB_TXDESC_DAT_TXMD_Msk              (0x1ul << BB_TXDESC_DAT_TXMD_Pos)

#define BB_TXDESC_DAT_TXLEN_Pos             (24)
#define BB_TXDESC_DAT_TXLEN_Msk             (0x1Ful << BB_TXDESC_DAT_TXLEN_Pos)

/* TX Descripter For Adv Packet */
#define BB_TXDESC_ADV_TXTYPE0_Pos           (16)
#define BB_TXDESC_ADV_TXTYPE0_Msk           (0xFul << BB_TXDESC_ADV_TXTYPE0_Pos)

#define BB_TXDESC_ADV_TXTXADD_Pos           (22)
#define BB_TXDESC_ADV_TXTXADD_Msk           (0xFul << BB_TXDESC_ADV_TXTXADD_Pos)

#define BB_TXDESC_ADV_TXRXADD_Pos           (23)
#define BB_TXDESC_ADV_TXRXADD_Msk           (0xFul << BB_TXDESC_ADV_TXRXADD_Pos)

#define BB_TXDESC_ADV_TXLEN_Pos             (24)
#define BB_TXDESC_ADV_TXLEN_Msk             (0x3Ful << BB_TXDESC_ADV_TXLEN_Pos)

/* end of BLE TX Descriptor */

/*---------------------- BLE RX Descriptor -------------------------*/


typedef struct {
	__IO  uint32_t rx_desc_cfg1 ;
	__IO  uint32_t rx_desc_cfg2	;
	//__IO  uint32_t rxdata[10] ;
	__IO  uint8_t rxdata[37] ;

} RX_DESC_T;



#define BB_RXDESC_CFG1_NEXTPTR_Pos             (0)
#define BB_RXDESC_CFG1_NEXTPTR_Msk             (0x3FFFul << BB_RXDESC_CFG1_NEXTPTR_Pos)

#define BB_RXDESC_CFG1_RXDONE_Pos              (15)
#define BB_RXDESC_CFG1_RXDONE_Msk              (0x1ul << BB_RXDESC_CFG1_RXDONE_Pos)

#define BB_RXDESC_CFG1_SYNCERR_Pos             (16)
#define BB_RXDESC_CFG1_SYNCERR_Msk             (0x1ul << BB_RXDESC_CFG1_SYNCERR_Pos)

#define BB_RXDESC_CFG1_TYPEERR_Pos             (17)
#define BB_RXDESC_CFG1_TYPEERR_Msk             (0x1ul << BB_RXDESC_CFG1_TYPEERR_Pos)

#define BB_RXDESC_CFG1_LENERR_Pos              (18)
#define BB_RXDESC_CFG1_LENERR_Msk              (0x1ul << BB_RXDESC_CFG1_LENERR_Pos)

#define BB_RXDESC_CFG1_CRCERR_Pos              (19)
#define BB_RXDESC_CFG1_CRCERR_Msk              (0x1ul << BB_RXDESC_CFG1_CRCERR_Pos)

#define BB_RXDESC_CFG1_MICERR_Pos              (20)
#define BB_RXDESC_CFG1_MICERR_Msk              (0x1ul << BB_RXDESC_CFG1_MICERR_Pos)

#define BB_RXDESC_CFG1_SNERR_Pos               (21)
#define BB_RXDESC_CFG1_SNERR_Msk               (0x1ul << BB_RXDESC_CFG1_SNERR_Pos)

#define BB_RXDESC_CFG1NESNERR_Pos             (22)
#define BB_RXDESC_CFG1NESNERR_Msk             (0x1ul << BB_RXDESC_CFG1NESNERR_Pos)

#define BB_RXDESC_CFG1BDADDR_MATCH_Pos        (23)
#define BB_RXDESC_CFG1BDADDR_MATCH_Msk        (0x1ul << BB_RXDESC_CFG1BDADDR_MATCH_Pos)

/* RX Descrpitor For Data Packet */
#define BB_RXDESC_CFG2_RXLLLID_Pos             (0)
#define BB_RXDESC_CFG2_RXLLLID_Msk             (0x3ul << BB_RXDESC_CFG2_RXLLLID_Pos)

#define BB_RXDESC_CFG2_RXNESN_Pos              (2)
#define BB_RXDESC_CFG2_RXNESN_Msk              (0x1ul << BB_RXDESC_CFG2_RXNESN_Pos)

#define BB_RXDESC_CFG2_RXSN_Pos                (3)
#define BB_RXDESC_CFG2_RXSN_Msk                (0x1ul << BB_RXDESC_CFG2_RXSN_Pos)

#define BB_RXDESC_CFG2_RXMD_Pos                (4)
#define BB_RXDESC_CFG2_RXMD_Msk                (0x1ul << BB_RXDESC_CFG2_RXMD_Pos)

#define BB_RXDESC_CFG2_RXLEN_Pos                (8)
#define BB_RXDESC_CFG2_RXLEN_Msk                (0x1Ful << BB_RXDESC_CFG2_RXLEN_Pos)
/* RX Descrpitor For Data Packet End */

/* RX Descrpitor For ADV Packet */

#define BB_RXDESC_CFG2_RXTYPE_Pos                (0)
#define BB_RXDESC_CFG2_RXTYPE_Msk                (0xFul << BB_RXDESC_CFG2_RXTYPE_Pos)

#define BB_RXDESC_CFG2_RXTXADD_Pos                (6)
#define BB_RXDESC_CFG2_RXTXADD_Msk                (0x1ul << BB_RXDESC_CFG2_RXTXADD_Pos)

#define BB_RXDESC_CFG2_RXRXADD_Pos                (7)
#define BB_RXDESC_CFG2_RXRXADD_Msk                (0x1ul << BB_RXDESC_CFG2_RXRXADD_Pos)

#define BB_RXDESC_CFG2_RXADVLEN_Pos                (8)
#define BB_RXDESC_CFG2_RXADVLEN_Msk                (0x3Ful << BB_RXDESC_CFG2_RXADVLEN_Pos)
/* RX Descrpitor For ADV Packet End */

#define BB_RXDESC_CFG2RXRSS_Pos                (16)
#define BB_RXDESC_CFG2RXRSS_Msk                (0xFFul << BB_RXDESC_CFG2RXRSS_Pos)

#define BB_RXDESC_CFG2USED_CH_IDX_Pos                (24)
#define BB_RXDESC_CFG2USED_CH_IDX_Msk                (0x3Ful << BB_RXDESC_CFG2USED_CH_IDX_Pos)

/*---------------------- BLE Exchange Table -------------------------*/

typedef struct {
	__IO uint32_t STR00_ET01 ;
	__IO uint32_t STR04_ET23 ;
	__IO uint32_t STR08_ET45 ;
	__IO uint32_t STR0C_ET67 ;
	__IO uint32_t STR10_ET89 ;
	__IO uint32_t STR14_ETAB ;
	__IO uint32_t STR18_ETCD ;
	__IO uint32_t STR1C_ETEF ;

} ET_T;

#define BB_ET00_ET0_Pos                (0)
#define BB_ET00_ET0_Msk                (0xFFFFul << BB_ET00_ET0_Pos)

#define BB_ET00_ET1_Pos                (16)
#define BB_ET00_ET1_Msk                (0xFFFFul << BB_ET00_ET1_Pos)

#define BB_ET04_ET2_Pos                (0)
#define BB_ET04_ET2_Msk                (0xFFFFul << BB_ET04_ET2_Pos)

#define BB_ET04_ET3_Pos                (16)
#define BB_ET04_ET3_Msk                (0xFFFFul << BB_ET04_ET3_Pos)

#define BB_ET08_ET4_Pos                (0)
#define BB_ET08_ET4_Msk                (0xFFFFul << BB_ET08_ET4_Pos)

#define BB_ET08_ET5_Pos                (16)
#define BB_ET08_ET5_Msk                (0xFFFFul << BB_ET08_ET5_Pos)

#define BB_ET0C_ET6_Pos                (0)
#define BB_ET0C_ET6_Msk                (0xFFFFul << BB_ET0C_ET6_Pos)

#define BB_ET0C_ET7_Pos                (16)
#define BB_ET0C_ET7_Msk                (0xFFFFul << BB_ET0C_ET7_Pos)

#define BB_ET10_ET8_Pos                (0)
#define BB_ET10_ET8_Msk                (0xFFFFul << BB_ET10_ET8_Pos)

#define BB_ET10_ET9_Pos                (16)
#define BB_ET10_ET9_Msk                (0xFFFFul << BB_ET10_ET9_Pos)

#define BB_ET14_ETA_Pos                (0)
#define BB_ET14_ETA_Msk                (0xFFFFul << BB_ET14_ETA_Pos)

#define BB_ET14_ETB_Pos                (16)
#define BB_ET14_ETB_Msk                (0xFFFFul << BB_ET14_ETB_Pos)

#define BB_ET18_ETC_Pos                (0)
#define BB_ET18_ETC_Msk                (0xFFFFul << BB_ET18_ETC_Pos)

#define BB_ET18_ETD_Pos                (16)
#define BB_ET18_ETD_Msk                (0xFFFFul << BB_ET18_ETD_Pos)

#define BB_ET1C_ETE_Pos                (0)
#define BB_ET1C_ETE_Msk                (0xFFFFul << BB_ET1C_ETE_Pos)

#define BB_ET1C_ETF_Pos                (16)
#define BB_ET1C_ETF_Msk                (0xFFFFul << BB_ET1C_ETF_Pos)

/* end of BLE Exchange Table */


//==================== Frequency index Table=================
typedef struct{
	__IO uint32_t FIT_STR20 ;
	__IO uint32_t FIT_STR24 ;
	__IO uint32_t FIT_STR28 ;
	__IO uint32_t FIT_STR2C ;
	__IO uint32_t FIT_STR30 ;
	__IO uint32_t FIT_STR34 ;
	__IO uint32_t FIT_STR38 ;
	__IO uint32_t FIT_STR3C ;
	__IO uint32_t FIT_STR40 ;
	__IO uint32_t FIT_STR44 ;
}FIT_T ;
/* end of BLE Frequency Index Table */

//==================== White List Table   ===================
typedef struct{
	__IO uint32_t WL_STR0 ;
	__IO uint32_t WL_STR4 ;
	__IO uint32_t WL_STR8 ;
}WL_T ;


/*ANAC_BA define*/
typedef struct
{
    __IO uint32_t AGC_TB0;
    __IO uint32_t AGC_TB1;
    __IO uint32_t AGC_TB2;
    __IO uint32_t AGC_TB3;

    __IO uint32_t GAIN0;
    __IO uint32_t GAIN1;
    __IO uint32_t GAIN2;

    __IO uint32_t AGC_CTL;

    __IO uint32_t AGC_ADC;
    //ANAC_BASE + 0x24
    __IO uint32_t MCU_RF;
    __IO uint32_t FSM_RF;

    __IO uint32_t PLL_CTL;
    __IO uint32_t SD_CTL;
    __IO uint32_t GS_CTL;
    __IO uint32_t TP_CTL;
    __IO uint32_t TP_STS;
    __IO uint32_t VCO_CTL;
    __IO uint32_t RX_CTL;
    __IO uint32_t TX_CTL;

    __IO uint32_t RF_PLL;
    __IO uint32_t RF_PLL2;

    __IO uint32_t LDO_CTL;
    __IO uint32_t RCC_CTL;
		
	__IO uint32_t FAR_CTL;
	__IO uint32_t LDO2_CTL;
	__IO uint32_t PARAMP_CTL;
	
		__IO uint32_t ANAC_3VCTL;			//0X68
		__IO uint32_t temp3_reg;			//0X6c

		__IO uint32_t AGC00;			//0X70
		__IO uint32_t AGC01;			//0X74
		__IO uint32_t AGC02;			//0X78
		__IO uint32_t AGC03;			//0X7c

		__IO uint32_t AGC04;			//0X80
		__IO uint32_t AGC05;			//0X84
		__IO uint32_t AGC06;			//0X88
		__IO uint32_t AGC07;			//0X8c

		__IO uint32_t AGC08;			//0X90
		__IO uint32_t AGC09;			//0X94
		__IO uint32_t AGC10;			//0X98
		__IO uint32_t AGC11;			//0X9c

		__IO uint32_t AGC12;			//0Xa0
		__IO uint32_t AGC13;			//0Xa4
		__IO uint32_t AGC14;			//0Xa8
		__IO uint32_t AGC15;			//0Xac

		__IO uint32_t AGC16;			//0Xb0
		__IO uint32_t AGC17;			//0Xb4
		__IO uint32_t AGC18;			//0Xb8
		__IO uint32_t AGC19;			//0Xbc

		__IO uint32_t AGC20;			//0Xc0
		__IO uint32_t AGC21;			//0Xc4
		__IO uint32_t AGC22;			//0Xc8
		__IO uint32_t AGC23;			//0Xcc

		__IO uint32_t AGC24;			//0Xd0
		__IO uint32_t AGC25;			//0Xd4
		__IO uint32_t AGC26;			//0Xd8
		__IO uint32_t AGC27;			//0Xdc

		__IO uint32_t AGC28;			//0Xe0
		__IO uint32_t AGC29;			//0Xe4
		__IO uint32_t AGC30;			//0Xe8
		__IO uint32_t AGC31;			//0Xec

		__IO uint32_t AGC_ST00;			//0Xf0
		__IO uint32_t AGC_ST01;			//0Xf4
		__IO uint32_t AGC_ST02;			//0Xf8
		__IO uint32_t AGC_ST03;			//0Xfc
}ANAC_T;
#define ANAC_BASE  (APB1PERIPH_BASE + 0x70000)    ///< ANAC_BASE register base address

#define ANAC ((ANAC_T *)ANAC_BASE)

#define BB_WL_STR0_DEV0L_Pos                (0)
#define BB_WL_STR0_DEV0L_Msk                (0xFFFFul << BB_WL_STR0_DEV0L_Pos)

#define BB_WL_STR0_DEV0M_Pos                (16)
#define BB_WL_STR0_DEV0M_Msk                (0xFFFFul << BB_WL_STR0_DEV0M_Pos)

#define BB_WL_STR4_DEV0H_Pos                (0)
#define BB_WL_STR4_DEV0H_Msk                (0xFFFFul << BB_WL_STR4_DEV0H_Pos)

#define BB_WL_STR4_DEV1L_Pos                (16)
#define BB_WL_STR4_DEV1L_Msk                (0xFFFFul << BB_WL_STR4_DEV1L_Pos)

#define BB_WL_STR8_DEV1M_Pos                (0)
#define BB_WL_STR8_DEV1M_Msk                (0xFFFFul << BB_WL_STR8_DEV1M_Pos)

#define BB_WL_STR8_DEV1H_Pos                (16)
#define BB_WL_STR8_DEV1H_Msk                (0xFFFFul << BB_WL_STR8_DEV1H_Pos)




/* end of BLE White List Table */

#define BLEMDM_BASE						(APB1PERIPH_BASE + 0x60200)    	///< BLE modem base address

#define BLEBB_BASE						(AHBPERIPH_BASE + 0x01000)    	///< BLE baseband base address   qshi added

#define BLEEM_BASE						(AHBPERIPH_BASE + 0x01400UL)		///< BLE Exchange base address	 qshi added

#define	BLEFQTBL_BASE					(AHBPERIPH_BASE + 0X01420)			///Freq table


/* Qshi added */
#define BLEBB							((BB_T *) 	BLEBB_BASE)			///< Pointer to BLE BB register structure
#define BLEMDM							((MDM_T *) 	BLEMDM_BASE)			///< Pointer to BLE Modem register structure
#define BLEET							((ET_T *) 	BLEEM_BASE)
#define	BLEFQTBL						((FIT_T *) 	BLEFQTBL_BASE)



/* EM operation */
#define EM_BLE_WR(addr, value)       	(*(volatile uint16_t *)(addr)) = (value)
#define EM_BLE_RD(addr)              	(*(volatile uint16_t *)(addr))


#define	set_ui32_bits(reg,field,pos,val)	(reg = (reg & ~field) | ((uint32_t)val<<pos))		// set bit-field of register
#define	set_ET(reg,pos,val)					(reg = ((uint32_t)val<<pos))						// set ET of register
#define	set_ui32_reg(reg, val)				(reg = val)
#define read_ui32_bits(reg,field,pos,val) 	(val = (reg & field ) >> pos )
#define	get_bits(reg,field,pos)				((reg & field)>>pos)
#define read_ui32_reg(reg, val)				(val = reg)
#define set_ui32_mask(reg,mask)             (reg=((reg)|(mask)))
#define clear_uin32_mask(reg,mask)          (reg=((reg)&~(mask)))

/// Macro to read a BLE register
#define REG_BLE_RD(addr)             (*(volatile uint32_t *)(addr))

/// Macro to write a BLE register
#define REG_BLE_WR(addr, value)      (*(volatile uint32_t *)(addr)) = (value)


/*===========================================================================*/
/*========================== Start of footer part ===========================*/
/*===========================================================================*/
/*----------------------------*/
/* Data access macros         */
/*----------------------------*/

#define SHIF16(a) ((a)&0x0001?0: (a)&0x0002?1: (a)&0x0004?2: (a)&0x0008?3:\
                   (a)&0x0010?4: (a)&0x0020?5: (a)&0x0040?6: (a)&0x0080?7:\
                   (a)&0x0100?8: (a)&0x0200?9: (a)&0x0400?10:(a)&0x0800?11:\
                   (a)&0x1000?12:(a)&0x2000?13:(a)&0x4000?14: 15)

#define SHIF32(a)((a)&0x00000001?0: (a)&0x00000002?1: (a)&0x00000004?2: (a)&0x00000008?3:\
                  (a)&0x00000010?4: (a)&0x00000020?5: (a)&0x00000040?6: (a)&0x00000080?7:\
                  (a)&0x00000100?8: (a)&0x00000200?9: (a)&0x00000400?10:(a)&0x00000800?11:\
                  (a)&0x00001000?12:(a)&0x00002000?13:(a)&0x00004000?14:(a)&0x00008000?15:\
                  (a)&0x00010000?16:(a)&0x00020000?17:(a)&0x00040000?18:(a)&0x00080000?19:\
                  (a)&0x00100000?20:(a)&0x00200000?21:(a)&0x00400000?22:(a)&0x00800000?23:\
                  (a)&0x01000000?24:(a)&0x02000000?25:(a)&0x04000000?26:(a)&0x08000000?27:\
                  (a)&0x10000000?28:(a)&0x20000000?29:(a)&0x40000000?30: 31)

#define SetWord8(a,d)       (* ( volatile uint8*) (a)=(d) )
#define SetWord16(a,d)      (* ( volatile uint16*)(a)=(d) )
#define SetWord32(a,d)      (* ( volatile uint32*)(a)=(d) )
#define SetWord64(a,d)      (* ( volatile uint64*)(a)=(d) )

#define GetWord8(a)         (* ( volatile uint8*) (a) )
#define GetWord16(a)        (* ( volatile uint16*)(a) )
#define GetWord32(a)        (* ( volatile uint32*)(a) )
#define GetWord64(a)        (* ( volatile uint64*)(a) )


/* Aliases for backwards compatibility (only the Byte versions). */
#define SetByte(a,d)        SetWord8((a),(d))
#define GetByte(a)          GetWord8((a))

#define SetBits16(a,f,d)    ( SetWord16( (a), (GetWord16(a)&(~(uint16)(f))) | (((uint16)(d))<<SHIF16((f))) ))
#define SetBits32(a,f,d)    ( SetWord32( (a), (GetWord32(a)&(~(uint32)(f))) | (((uint32)(d))<<SHIF32((f))) ))

//#define GetBits(a,f)      (( ( volatile struct __##a *)(a))->BITFLD_##f )

#define GetBits16(a,f)  ( (GetWord16(a)&( (uint16)(f) )) >> SHIF16(f) )
#define GetBits32(a,f)  ( (GetWord32(a)&( (uint32)(f) )) >> SHIF32(f) )

/**
 ****************************************************************************************
 * @brief Initialize the BLE stack.
 ****************************************************************************************
 */
extern void rwble_init(void);

/**
 ****************************************************************************************
 * @brief Reset the BLE stack.
 ****************************************************************************************
 */
extern void rwble_reset(void);

/**
 ****************************************************************************************
 * @brief Gives FW/HW versions of RW-BLE stack.
 *
 ****************************************************************************************
 */
extern void rwble_version(uint8_t* fw_version, uint8_t* hw_version);

/**
 ****************************************************************************************
 * @brief Retrieve the slot clock.
 *
 * @return  Slot clock (x 625us)
 ****************************************************************************************
 */
extern uint32_t rwble_get_clock(void);

#if RW_BLE_SUPPORT && HCIC_ITF
/**
 ****************************************************************************************
 * @brief Send an error message to Host.
 *
 * This function is used to send an error message to Host from platform.
 *
 * @param[in] error    Error detected by FW
 ****************************************************************************************
 */
extern void rwble_send_message(uint32_t error);
#endif //RW_BLE_SUPPORT && HCIC_ITF

extern void ble_int_cfg(void);
#endif  // __PANBLE_H__
