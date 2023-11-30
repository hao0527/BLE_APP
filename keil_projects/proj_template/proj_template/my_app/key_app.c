/**
 * @file key_app.c
 * @note 检测按键按下后开启定时器，每200ms检测一次按键是否还被按下，
 *       到按键松开2s内无按键按下关闭定时器，2s内继续按下算连按，
 *       能记录最后一次的按下时间，和总的按下次数，
 *       检测到特殊事件后能回调用户提供的处理函数。
 *
 */
#include "key_app.h"
#include "mcu_hal.h"
#include "stack_svc_api.h"

#define KEY_PRESSED     TRUE     // 按下
#define KEY_NOT_PRESSED FALSE    // 未按下

KeyInfo_t keyInfo;
static KeyCfg_t *pKeyCfg;

static void key_scanTimerOff(void)
{
	// 查是否开启，如果开启则关闭
	if (((ke_timer_active_handler)SVC_ke_timer_active)(APP_KEY_SCAN_TIMER, TASK_APP)) {
		((ke_timer_clear_handler)SVC_ke_timer_clear)(APP_KEY_SCAN_TIMER, TASK_APP);
	}
}

static void key_scanTimerOn(uint16 time)
{
	key_scanTimerOff();	// 防止之前开着定时器，先尝试关
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
 * @brief 监测首次按下
 * 
 */
void key_monitorPress(void)
{
	// 如果按键周期扫描已开启，无需检测首次按下
	if(keyInfo.taskStatus)
		return;
	key_scan();
	if (keyInfo.keyStatus == KEY_PRESSED) {
		// 首次按下 开启扫描按键定时器
		key_scanTimerOn(pKeyCfg->timerPeriod);
		keyInfo.taskStatus = TRUE;
		keyInfo.pressTime = 0;	// 按下时间清0
		keyInfo.pressCnt = 0;	// 连按次数清0
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
			// 检测到按下
			keyInfo.pressTime = 0;
			keyInfo.pressCnt++;
			pKeyCfg->keyEventCB(KEY_PRESS, keyInfo.pressCnt);
		} else {
			// 检测到释放
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
