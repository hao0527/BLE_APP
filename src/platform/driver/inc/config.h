#ifndef		_CONFIG_H_
#define		_CONFIG_H_

#define		DEBUG_EN						0
#define		TASK_EN						0//	 1
//==========================================================
//			TIMER Frequency Definition
//==========================================================
#define		TIMER0_FREQ						1000
#define		TIMER1_FREQ						10

//==========================================================
//			CONFIG & SEC Address Definition
//==========================================================
		// compile for pn102 ( 256K flash )
#define		CMPL_SEC_MODE_ADDR			0x0003FDFC
#define		CMPL_CONFIG0_ADDR			0x0003FE00
#define		CMPL_CONFIG1_ADDR			0x0003FE04
#define		CMPL_CONFIG2_ADDR			0x0003FE08
#define		CMPL_CONFIG3_ADDR			0x0003FE0C

#define		CMPL_COMPANYID_ADDR					0x00200040												// ID page
#define		CMPL_PRODUCTID_ADDR					0x00200084
#define		CMPL_UNIQUEID_ADDRw0				0x00200020
#define		CMPL_UNIQUEID_ADDRw1				0x00200024
#define		CMPL_UNIQUEID_ADDRw2				0x00200028
#define		CMPL_UNQCOMID_ADDRw0				0x00200030
#define		CMPL_UNQCOMID_ADDRw1				0x00200034
#define		CMPL_UNQCOMID_ADDRw2				0x00200038
#define		CMPL_UNQCOMID_ADDRw3				0x0020003C

//-----------------------------------------------------------//
#define		ID_BLOCK_START_ADDRESS			0x00200000
#define		ID_BLOCK_END_ADDRESS				0x002001FF
#define		CONFIG_BLOCK_END_ADDRESS		0x00007FFF
//**************************************************************//



#define PN102B (1)
#define UART0_ENABLE (1)
#define SLEEP_TEST (1)



#ifdef CP_TEST

#define BUFSIZE  64

//Speed Define
#define I2C_SPEED_DFT_100K               (100000)
#define I2C_SPEED_DFT_400K               (400000)

#define SPI_SPEED_DFT_400K               (400000)

//Interface Defined
#define MODE_SWD 		(0XA5)
#define MODE_IIC 		(0XA6)
#define MODE_A7 		(0XA7)
#define MODE_SPI 		(0XA8)
#define MODE_SWD_I2C 	(0XA9)

//SPI Mask Select
#define CP_SPI0     (0x01)
#define CP_SPI1     (0x02)
#define CP_SPI2     (0x04)
#define CP_SPI3     (0x08)

#define ROM_SIZE	(0x6000)
#define SUPPORT_3_3V  (1)
//For EAD Simulation
// #define SIMULATE 
  #define DEBUG

#ifdef SIMULATE
#undef DEBUG
#endif

#ifdef DEBUG
//#define CMD_DEBUG
//#define SPI_DEBUG
//#define UART_DEBUG
//#define I2C_DEBUG
#define DEBUG_LDO
#define DEBUG_ADC

#define CP_LOG(...)         \
do {                        \
    printf(__VA_ARGS__);    \
} while(0)

#else

#define CP_LOG(...)

#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "PN102Series.h"
#include "cp_ram.h"
#include "cp_romcrc.h"

#include "my_sys.h"
#include "CP_Timer.h"

#include "cp_i2c.h"
#include "cp_cmd.h"
#include "cp_bgap.h"
#include "cp_id.h"
#include "CP_32K_CLK.h"
#include "Flash_Download.h"

// Declare global variables
extern volatile bool tfr_flag;
extern volatile bool err_flag;

void CP_I2C_SysInit(void);
void CP_UART_SysInit(void);
void CP_SPI_SysInit(uint8_t spiSelectMask);

uint32_t GetRomMode(void);


#endif


#endif
