#include "led_app.h"
#include "mcu_hal.h"
#include "stack_svc_api.h"

#define LED_OFF FALSE    // ��
#define LED_ON  TRUE     // ��

LedInfo_t ledInfo;

static void led_BlinkTimerOff(void)
{
	// ���Ƿ��������������ر�
	if (((ke_timer_active_handler)SVC_ke_timer_active)(APP_LED_BLINK_TIMER, TASK_APP)) {
		((ke_timer_clear_handler)SVC_ke_timer_clear)(APP_LED_BLINK_TIMER, TASK_APP);
	}
}

static void led_BlinkTimerOn(uint16 time)
{
	led_BlinkTimerOff();	// ��ֹ֮ǰ���Ŷ�ʱ�����ȳ��Թ�
	((ke_timer_set_handler)SVC_ke_timer_set)(APP_LED_BLINK_TIMER, TASK_APP, time);
}

static void led_ctrlStatus(bool light)
{
	mcu_gpio_light_led(light);
	ledInfo.ledStatus = light;
}

void led_resetInit(void)
{
	led_ctrlStatus(LED_OFF);
	ledInfo.ledMode = LED_MODE_OFF;
	ledInfo.ledOffTime = 180;
	ledInfo.ledOnTime = 20;
	led_setMode(LED_MODE_BLINK);
}

void led_setMode(LedMode_t mode)
{
	if (ledInfo.ledMode == mode)
		return;    // ��ԭ��ģʽһ��

	switch (mode) {
	case LED_MODE_OFF:
		led_BlinkTimerOff();
		led_ctrlStatus(LED_OFF);
		break;

	case LED_MODE_ON:
		led_BlinkTimerOff();
		led_ctrlStatus(LED_ON);
		break;

	case LED_MODE_BLINK:
		led_BlinkTimerOn(ledInfo.ledOffTime);
		led_ctrlStatus(LED_OFF);
		break;

	default:
		return;
	}

	ledInfo.ledMode = mode;
}

void led_setBlinkTime(uint16 offTime, uint16 onTime)
{
	// �������βμ��
	ledInfo.ledOffTime = offTime;
	ledInfo.ledOnTime = onTime;
}

void led_ledBlinkTimerCb(void)
{
	if (ledInfo.ledStatus == LED_OFF) {
		led_BlinkTimerOn(ledInfo.ledOnTime);
		led_ctrlStatus(LED_ON);
	} else {
		led_BlinkTimerOn(ledInfo.ledOffTime);
		led_ctrlStatus(LED_OFF);
	}
}
