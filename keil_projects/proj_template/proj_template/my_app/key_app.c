/**
 * @file key_app.c
 * @note ��ⰴ�����º�����ʱ����ÿ100ms���һ�ΰ����Ƿ񻹱����£�
 *       �������ɿ�2s���ް������¹رն�ʱ����2s�ڼ���������������
 *       �ܼ�¼���һ�εİ���ʱ�䣬���ܵİ��´�����
 *       ��⵽�����¼����ܻص��û��ṩ�Ĵ�������
 *       ������ں����������ʱ������û����á�
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
	keyInfo.taskStatus = FALSE;
}

static void key_scanTimerOn(uint16 time)
{
	key_scanTimerOff();    // ��ֹ֮ǰ���Ŷ�ʱ�����ȳ��Թ�
	((ke_timer_set_handler)SVC_ke_timer_set)(APP_KEY_SCAN_TIMER, TASK_APP, time);
	keyInfo.taskStatus = TRUE;
}

static void key_scan(void)
{
	if (mcu_gpio_key_pressed()) {
		keyInfo.keyStatus = KEY_PRESSED;
	} else {
		keyInfo.keyStatus = KEY_NOT_PRESSED;
	}
}

void key_init(KeyCfg_t *pCfg)
{
	pKeyCfg = pCfg;
}

/**
 * @brief ����״ΰ���
 */
void key_monitorPress(void)
{
	// �����������ɨ���ѿ������������״ΰ���
	if (keyInfo.taskStatus)
		return;
	key_scan();
	if (keyInfo.keyStatus == KEY_PRESSED) {
		// �״ΰ��� ����ɨ�谴����ʱ��
		key_scanTimerOn(pKeyCfg->timerPeriod);
		keyInfo.pressTime = 0;      // ����ʱ����0
		keyInfo.releaseTime = 0;    // �ͷ�ʱ����0
		keyInfo.pressCnt = 0;       // ����������0
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
	bool timerContinue = TRUE;

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
		if (keyInfo.pressTime >= 100 && lastPressTime < 100) {
			pKeyCfg->keyEventCB(KEY_PRESS_1SECS, keyInfo.pressCnt);
		} else if (keyInfo.pressTime >= 300 && lastPressTime < 300) {
			pKeyCfg->keyEventCB(KEY_PRESS_3SECS, keyInfo.pressCnt);
		}
	} else {
		keyInfo.releaseTime += pKeyCfg->timerPeriod;
		if (keyInfo.releaseTime >= pKeyCfg->pressIntervalTime && lastReleaseTime < pKeyCfg->pressIntervalTime) {
			timerContinue = FALSE;
			pKeyCfg->keyEventCB(KEY_SCAN_END, keyInfo.pressCnt);
		}
	}

	if (timerContinue) {
		key_scanTimerOn(pKeyCfg->timerPeriod);
	} else {
		key_scanTimerOff();
	}
}

////////////////////////////////////////////key_user/////////////////////////////////////////////
typedef enum KeyUserFlag {
	KEY_NO_FLAG = 0,
	KEY_POWER_ON_FLAG,
	KEY_3SECS_UNLOCK_FLAG,
	KEY_POWER_OFF_FLAG,
} KeyUserFlag_t;

typedef struct KeyUserInfo {
	uint8 runMode;    // 0:�ػ� 1:����
	KeyUserFlag_t flag;
} KeyUserInfo_t;

KeyUserInfo_t keyUserInfo;
static KeyCfg_t keyCfg;

void key_eventCB(KeyEvents_t event, uint8 pressCnt)
{
	if (event == KEY_PRESSED && pressCnt == 0) {
		// �״ΰ������־λ
		keyUserInfo.flag = KEY_NO_FLAG;
	}

	if (keyUserInfo.runMode) {
		if (event == KEY_PRESS_3SECS && pressCnt == 0) {
			// ����3s������������
			keyUserInfo.flag = KEY_3SECS_UNLOCK_FLAG;
		} else if (event == KEY_PRESS && pressCnt == 1) {
			if (keyUserInfo.flag == KEY_3SECS_UNLOCK_FLAG) {
				// �������ٰ��¾���POW_EN���ͣ����û��ͷŰ���ʱ��ȫ�ϵ�
				keyUserInfo.flag = KEY_POWER_OFF_FLAG;
				mcu_gpio_en_pow(FALSE);
			}
		} else if (event == KEY_RELEASE) {
			if (keyUserInfo.flag == KEY_NO_FLAG) {
				// δ�������������ͷŰ���ʱ���ð���
				key_taskReset();
			}
		}
	} else {
		if (event == KEY_PRESS_1SECS) {
			// ����1s����
			mcu_gpio_en_pow(TRUE);
			keyUserInfo.flag = KEY_POWER_ON_FLAG;
		} else if (event == KEY_RELEASE) {
			if (keyUserInfo.flag == KEY_NO_FLAG) {
				// δ����1s���ͷŰ���ʱ�µ�
				mcu_gpio_en_pow(FALSE);
			}
		} else if (event == KEY_SCAN_END) {
			if (keyUserInfo.flag == KEY_POWER_ON_FLAG) {
				// �Ȱ���ɨ�������������runMode����ֹ�����������ֱ�����
				keyUserInfo.runMode = TRUE;
			}
		}
	}
}

void key_resetInit(void)
{
	keyCfg.keyEventCB = key_eventCB;
	keyCfg.pressIntervalTime = 200;
	keyCfg.timerPeriod = 10;
	key_init(&keyCfg);
	memset(&keyInfo, 0, sizeof(KeyInfo_t));

	memset(&keyUserInfo, 0, sizeof(KeyUserInfo_t));
}
