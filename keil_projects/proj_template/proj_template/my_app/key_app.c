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
void key_task(void)
{
	// �����������ѿ������������״ΰ���
	if(keyInfo.scanStatus)
		return;
	key_scan();
	if (keyInfo.keyStatus == KEY_PRESSED && !keyInfo.scanStatus) {
		// �״ΰ��� ����ɨ�谴����ʱ��
		key_scanTimerOn(pKeyCfg->timerPeriod);
		keyInfo.scanStatus = TRUE;
	}
}

void key_scanTimerCB(void)
{
	;
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
}
