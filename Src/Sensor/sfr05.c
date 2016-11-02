#include "srf05.h"
#include "sensor.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "kalmanfilter.h"
#include "cmsis_os.h"

struct _srf05 srf05 = {
	STATE_S0,
	0,
	0,
	0,
	0
};

extern TIM_HandleTypeDef htim2;
extern osThreadId defaultTaskHandle;
void Error_Handler(void);

kalman_state kalman_filter;

/**
 * @brief delay us micro second
 * @param us: number of microsecond will be delayed
 */
static void delay_us(int us)
{
	while (us--)
	{
	}
}

/**
 * @brief gen a start pulse on trigger pin, this function takes about 12 us
 */
static void srf05_gen_pulse()
{
	delay_us(10);
	HAL_GPIO_WritePin(SRF05_TRIGGER_GPIO_Port, SRF05_TRIGGER_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(SRF05_TRIGGER_GPIO_Port, SRF05_TRIGGER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief run a state machine
 * @param event: event to be handled such as echo pin, time out ...
 */
void srf05_handle_event(event_t event)
{
	switch (srf05.state)
	{
	case STATE_S0:
		if (event == ECHO_GOES_HIGH)
		{
			/* start timer */
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_TIM_Base_Start(&htim2);

			srf05.timer_counter = 0;
			srf05.ms_counter = 0;
			srf05.state = STATE_S1;
		}
		break;
	case STATE_S1:
		if (event == ECHO_GOES_LOW)
		{
			srf05.need_perform = 1;
			srf05.timer_counter = srf05.ms_counter * 1000 +
					__HAL_TIM_GET_COUNTER(&htim2) * 999 / CALIB_TIMER;

			if (srf05.ms_counter < MINIMUM_DELAY)
			{
				srf05.state = STATE_S2;
			}
			else
			{
				HAL_TIM_Base_Stop(&htim2);
				srf05_gen_pulse();
				srf05.state = STATE_S0;
			}


			osSignalSet(defaultTaskHandle, 0x0001);

		}
		else if (event == TIMEOUT)
		{
			srf05.ms_counter ++;
		}
		break;
	case STATE_S2:
		if (event == TIMEOUT)
		{
			srf05.ms_counter ++;
			if (srf05.ms_counter >= MINIMUM_DELAY)
			{
				HAL_TIM_Base_Stop(&htim2);
				srf05_gen_pulse();
				srf05.state = STATE_S0;
			}
		}
		break;
	}
}

/**
 * @brief check if new data is available or not
 * @return 1 if there is new data else return 0
 */
static uint8_t srf05_new_data(void)
{
	return srf05.new_data;
}
/**
 * @brief get the latest data
 * @return data in uint32_t format
 */
static float srf05_get_data(void)
{
	srf05.new_data = 0;
	return (float)srf05.value;
}
/**
 * @brief run hard work, this function will takes time so it need to be run in
 * main loop
 * @return 1 if it is run, 0 if not
 */
static uint8_t srf05_perform(void)
{
	if (srf05.need_perform)
	{
		/* run filter than set new_data */
		double raw_value = srf05.timer_counter / 58.0;
//		kalman_update(&kalman_filter, raw_value);
//		srf05.value = kalman_filter.x;
		srf05.value = raw_value;
		srf05.new_data = 1;

		srf05.need_perform = 0;
		return 1;
	}
	return 0;
}
/**
 * @brief initialize gpio and timer to work with srf05 sensor
 */
static void srf05_init(void)
{
	kalman_init(&kalman_filter, 40.0, 70.0, 40.0, 0);
	srf05_gen_pulse();
}


const sensor_t sensor =
{
	"Ultrasonic",
	srf05_init,
	srf05_new_data,
	srf05_get_data,
	srf05_perform
}; /*< sensor variable for main function */

void srf05_timer_callback()
{
	srf05_handle_event(TIMEOUT);
}
void srf05_gpio_callback()
{
	if (HAL_GPIO_ReadPin(SRF05_ECHO_GPIO_Port, SRF05_ECHO_Pin) == GPIO_PIN_RESET)
		srf05_handle_event(ECHO_GOES_LOW);
	else
		srf05_handle_event(ECHO_GOES_HIGH);
}

void (*timer_callback) (void) = srf05_timer_callback;
void (*gpio_callback) (void) = srf05_gpio_callback;

__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	gpio_callback();
}
