/*
 * LED.cpp
 *
 *  Created on: Mar 16, 2021
 *      Author: tiwo
 */

#include "LED.h"

LED::LED() {
}

LED::~LED() {
	// TODO Auto-generated destructor stub
}

/* USER CODE BEGIN 4 */
void LED::led_on(uint16_t led_) {
	HAL_GPIO_WritePin(GPIOI, led_, GPIO_PIN_SET);
}

void LED::led_off(uint16_t led_) {
	HAL_GPIO_WritePin(GPIOI, led_, GPIO_PIN_RESET);
}

void LED::led_toggle(uint16_t led_) {
	if(HAL_GPIO_ReadPin(GPIOI, led_)) {
		HAL_GPIO_WritePin(GPIOI, led_, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOI, led_, GPIO_PIN_SET);
	}

}

namespace LEDns {
LED led;
}

/* USER CODE END 4 */
