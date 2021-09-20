/*
 * PowerCtrl.h
 *
 *  Created on: Mar 17, 2021
 *      Author: tiwo
 */

#ifndef SRC_POWERCTRL_H_
#define SRC_POWERCTRL_H_

#include <stm32f4xx_hal.h>
#include <SysTickTimer.h>

class PowerCtrl {
public:
	PowerCtrl();
	virtual ~PowerCtrl();

	bool setVCCPower(bool state);

private:
	bool getEN_VCC(void);
};

namespace PowerNs {
static const bool powerOn = true;
static const bool powerOff = false;

extern PowerCtrl power;
}

#endif /* SRC_POWERCTRL_H_ */
