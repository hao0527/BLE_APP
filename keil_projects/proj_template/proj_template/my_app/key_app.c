/**
 * @file key_app.c
 * @note ��ⰴ�����º�����ʱ����ÿ200ms���һ�ΰ����Ƿ񻹱����£�
 *       �������ɿ�2s���ް������¹رն�ʱ����2s�ڼ���������������
 *       �ܼ�¼���һ�εİ���ʱ�䣬���ܵİ��´�����
 *       ��⵽�����¼����ܻص��û��ṩ�Ĵ�������
 *
 */
#include "key_app.h"
#include "mcu_hal.h"
#include "stack_svc_api.h"

#define KEY_PRESSED     TRUE     // ����
#define KEY_NOT_PRESSED FALSE    // δ����

KeyInfo_t keyInfo;
static KeyCfg_t *pKeyCfg;

static void key_scanTimerOff(void)
{
	// ���Ƿ��������������ر�
	if (((ke_timer_active_handler)SVC_ke_timer_active)(APP_KEY_SCAN_TIMER, TASK_APP)) {
		((ke_timer_clear_handler)SVC_ke_timer_clear)(APP_KEY_SCAN_TIMER, TASK_APP);
	}
}

static void key_scanTimerOn(uint16 time)
{
	key_scanTimerOff();	// ��ֹ֮ǰ���Ŷ�ʱ�����ȳ��Թ�
	((ke_timer_set_handler)SVC_ke_timer_set)(APP_KEY_SCAN_TIMER, TASK_APP, time);
}

static void key_scan(void)
{
	if (mcu_gpio_key_pressed()) {
		keyInfo.keyStatus = KEY_PRESSED;
	} else {
		keyInfo.keyStatus = KEY_NOT_PRESSED;
	}
}

void key_init(KeyCfg_t* pCfg)
{
	pKeyCfg = pCfg;
}

/**
 * @brief ����״ΰ���
 * 
 */
void key_monitorPress(void)
{
	// �����������ɨ���ѿ������������״ΰ���
	if(keyInfo.taskStatus)
		return;
	key_scan();
	if (keyInfo.keyStatus == KEY_PRESSED) {
		// �״ΰ��� ����ɨ�谴����ʱ��
		key_scanTimerOn(pKeyCfg->timerPeriod);
		keyInfo.taskStatus = TRUE;
		keyInfo.pressTime = 0;	// ����ʱ����0
		keyInfo.pressCnt = 0;	// ����������0
		pKeyCfg->keyEventCB(KEY_PRESS, keyInfo.pressCnt);
	}
}

void key_taskReset(void)
{
	key_scanTimerOff();
	memset(&keyInfo, 0, sizeof(KeyInfo_t));
}

void key_scanTimerCB(void)
{
	uint8 lastKeyStatus = keyInfo.keyStatus;
	uint16 lastPressTime = keyInfo.pressTime;
	uint16 lastReleaseTime = keyInfo.releaseTime;
	key_scan();

	if (lastKeyStatus != keyInfo.keyStatus) {
		if (keyInfo.keyStatus == KEY_PRESSED) {
			// ��⵽����
			keyInfo.pressTime = 0;
			keyInfo.pressCnt++;
			pKeyCfg->keyEventCB(KEY_PRESS, keyInfo.pressCnt);
		} else {
			// ��⵽�ͷ�
			keyInfo.releaseTime = 0;
			pKeyCfg->keyEventCB(KEY_RELEASE, keyInfo.pressCnt);
		}
	}

	if (keyInfo.keyStatus == KEY_PRESSED) {
		keyInfo.pressTime += pKeyCfg->timerPeriod;
		if (keyInfo.pressTime >= 300 && lastPressTime < 300){
			pKeyCfg->keyEventCB(KEY_PRESS_3SECS, keyInfo.pressCnt);
		}
	} else {
		keyInfo.releaseTime += pKeyCfg->timerPeriod;
		if (keyInfo.releaseTime >= pKeyCfg->pressIntervalTime && lastReleaseTime < pKeyCfg->pressIntervalTime){
			key_scanTimerOff();
			pKeyCfg->keyEventCB(KEY_SCAN_END, keyInfo.pressCnt);
		}
	}
}

////////////////////////////////////////////key_user/////////////////////////////////////////////
static KeyCfg_t KeyCfg;

void key_EventCB(KeyEvents_t event, uint8 pressCnt)
{
	;
}

void key_resetInit(void)
{
	KeyCfg.keyEventCB = key_EventCB;
	KeyCfg.pressIntervalTime = 200;
	KeyCfg.timerPeriod = 10;
	key_init(&KeyCfg);
	memset(&keyInfo, 0, sizeof(KeyInfo_t));
}
