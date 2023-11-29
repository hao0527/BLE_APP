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
 * @brief 检测首次按下
 * 
 */
void key_task(void)
{
	// 如果按键检测已开启，无需检测首次按下
	if(keyInfo.scanStatus)
		return;
	key_scan();
	if (keyInfo.keyStatus == KEY_PRESSED && !keyInfo.scanStatus) {
		// 首次按下 开启扫描按键定时器
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
