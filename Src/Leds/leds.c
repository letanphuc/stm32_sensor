#include "leds.h"

void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_PORTLEDS_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SIGNAL_LED_GPIO_Port, SIGNAL_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SIGNAL_LED_Pin */
	GPIO_InitStruct.Pin = SIGNAL_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SIGNAL_LED_GPIO_Port, &GPIO_InitStruct);
}

void led_toggle(led_id_t led)
{
	(void)led;
	HAL_GPIO_TogglePin(SIGNAL_LED_GPIO_Port, SIGNAL_LED_Pin);
}
