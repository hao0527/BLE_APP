#ifndef __KEY_APP_H_
#define __KEY_APP_H_

#include "panble.h"

typedef enum KeyEvents {
	KEY_PRESS = 0,      // 检测到按键按下
	KEY_PRESS_3SECS,    // 检测到按下3秒
	KEY_RELEASE,        // 检测到按键释放
	KEY_SCAN_END,       // 按键检测结束（无后续连按）
} KeyEvents_t;

typedef void (*KeyEventFuncPtr)(KeyEvents_t, uint8);    // 无返回 输入事件与连按次数

typedef struct KeyInfo {
	uint8 taskStatus;      // 0:未周期扫描 1:正在周期扫描
	uint8 keyStatus;       // 0:未按下 1:按下
	uint8 pressCnt;        // 连按次数
	uint16 pressTime;      // 最后一次按下时长(单位10ms)
	uint16 releaseTime;    // 松开时长(单位10ms)
} KeyInfo_t;

typedef struct KeyCfg {
	uint8 timerPeriod;             // 定时器周期、检测周期(单位10ms)
	uint8 pressIntervalTime;       // 连按最大间隔时间(单位10ms 需为检测周期整数倍)
	KeyEventFuncPtr keyEventCB;    // 按键事件回调函数
} KeyCfg_t;

extern KeyInfo_t keyInfo;

void key_monitorPress(void);
void key_scanTimerCB(void);
void key_taskReset(void);

void key_resetInit(void);

#endif    //__KEY_APP_H_
