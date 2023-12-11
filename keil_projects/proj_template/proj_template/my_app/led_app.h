#ifndef __LED_APP_H_
#define __LED_APP_H_

#include "panble.h"

typedef enum LedMode {
	LED_MODE_OFF = 0,    // ����ģʽ
	LED_MODE_ON,         // ����ģʽ
	LED_MODE_BLINK,      // ��ģʽ
} LedMode_t;

typedef struct LedInfo {
	uint8 ledStatus;    // 0:�� 1:��
	LedMode_t ledMode;
	uint16 ledOffTime;    // ��ģʽ�����ʱ�� ��λ10ms
	uint16 ledOnTime;     // ��ģʽ������ʱ�� ��λ10ms
} LedInfo_t;

void led_resetInit(void);
void led_setMode(LedMode_t mode);
void led_setBlinkTime(uint16 offTime, uint16 onTime);
void led_ledBlinkTimerCb(void);

#endif    //__LED_APP_H_
