#ifndef __LED_APP_H_
#define __LED_APP_H_

#include "panble.h"

typedef enum LedMode {
	LED_MODE_OFF = 0,    // 常灭模式
	LED_MODE_ON,         // 常亮模式
	LED_MODE_BLINK,      // 闪模式
} LedMode_t;

typedef struct LedInfo {
	uint8 ledStatus;    // 0:灭 1:亮
	LedMode_t ledMode;
	uint16 ledOffTime;    // 闪模式下灭的时间 单位10ms
	uint16 ledOnTime;     // 闪模式下亮的时间 单位10ms
} LedInfo_t;

void led_resetInit(void);
void led_setMode(LedMode_t mode);
void led_setBlinkTime(uint16 offTime, uint16 onTime);
void led_ledBlinkTimerCb(void);

#endif    //__LED_APP_H_
