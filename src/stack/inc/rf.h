

#ifndef __RF_H_
#define __RF_H_

#include "PN102Series.h"
#include "panip.h"


//under 0dbm
#define TX_POWER__14DBM 0xF2
#define TX_POWER__8DBM  0xF8
#define TX_POWER__5DBM  0xFB
#define TX_POWER__2DBM  0xFE

//up odbm
#define TX_POWER_0DBM   0
#define TX_POWER_1DBM   1
#define TX_POWER_2DBM   2
#define TX_POWER_3DBM   3
#define TX_POWER_4DBM   4
#define TX_POWER_5DBM   5
#define TX_POWER_6DBM   6
#define TX_POWER_7DBM   7
#define TX_POWER_8DBM   8
#define TX_POWER_9DBM   9
#define TX_POWER_10DBM  10


void rf_init ( struct rwip_rf_api *api );

/**
 * rf 校准初始化 (两点式校准)
 **/
extern void rf_calibration_init ( void );

/**
 * 温度变化时调用此函数, 从新执行 rf 校准 
 **/
extern void rf_calibration_detect ( void );

/**
 * 遍历所有的有效参数, 生成 tpcode table, 仅用于测试
 **/
extern void rf_calibration_tpcode_table_generate ( void );

void en_bb_debug(void);

void rc_internal_init(void);

void rf_pa_ramp_init(void);

#endif

