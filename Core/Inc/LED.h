/*
 * LED.h
 *
 *  Created on: Mar 16, 2021
 *      Author: tiwo
 */

#define LED_YELLOW GPIO_PIN_5
#define LED_GREEN GPIO_PIN_6
#define LED_RED GPIO_PIN_7

#ifndef SRC_LED_H_
#define SRC_LED_H

#include "stm32f4xx_hal.h"

class LED {
public:
	LED();
	virtual ~LED();

	void led_on(uint16_t led_);
	void led_off(uint16_t led_);
	void led_toggle(uint16_t led_);
};

namespace LEDns {
extern LED led;
}

#endif /* SRC_LED_H_ */
