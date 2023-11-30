#ifndef __KEY_APP_H_
#define __KEY_APP_H_

#include "panble.h"

typedef enum KeyEvents {
	KEY_PRESS = 0,      // ��⵽��������
	KEY_PRESS_3SECS,    // ��⵽����3��
	KEY_RELEASE,        // ��⵽�����ͷ�
	KEY_SCAN_END,       // �������������޺���������
} KeyEvents_t;

typedef void (*KeyEventFuncPtr)(KeyEvents_t, uint8);    // �޷��� �����¼�����������

typedef struct KeyInfo {
	uint8 taskStatus;      // 0:δ����ɨ�� 1:��������ɨ��
	uint8 keyStatus;       // 0:δ���� 1:����
	uint8 pressCnt;        // ��������
	uint16 pressTime;      // ���һ�ΰ���ʱ��(��λ10ms)
	uint16 releaseTime;    // �ɿ�ʱ��(��λ10ms)
} KeyInfo_t;

typedef struct KeyCfg {
	uint8 timerPeriod;             // ��ʱ�����ڡ��������(��λ10ms)
	uint8 pressIntervalTime;       // ���������ʱ��(��λ10ms ��Ϊ�������������)
	KeyEventFuncPtr keyEventCB;    // �����¼��ص�����
} KeyCfg_t;

extern KeyInfo_t keyInfo;

void key_monitorPress(void);
void key_scanTimerCB(void);
void key_taskReset(void);

void key_resetInit(void);

#endif    //__KEY_APP_H_
