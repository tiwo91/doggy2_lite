/*
 * PowerCtrl.cpp
 *
 *  Created on: Mar 17, 2021
 *      Author: tiwo
 */

#include "PowerCtrl.h"

PowerCtrl::PowerCtrl() {

}

PowerCtrl::~PowerCtrl() {
	// TODO Auto-generated destructor stub
}

bool PowerCtrl::setVCCPower(bool state) {
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);	// SHDN set '1'

	uint32_t actTime = Timers::Systick::tim.getTime(); // store actual time

	while ((!getEN_VCC()) && (actTime + 5000 > Timers::Systick::tim.getTime())) {
		// waiting for power above Vcc-0.7V
		// or timeout after 2 seconds
	}
	return getEN_VCC();
}

bool PowerCtrl::getEN_VCC(void) {
	return GPIOD->IDR & GPIO_PIN_0;
}

namespace PowerNs {
PowerCtrl power;
}

