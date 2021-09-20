/*
 * SysTickTimer.cpp
 *
 *  Created on: Mar 16, 2021
 *      Author: tiwo
 */

#include "SysTickTimer.h"

namespace Timers {

namespace Systick {
SysTickTimer tim;
}
}

volatile std::atomic<uint32_t> SysTickTimer::time { 0 };
volatile std::atomic<bool> SysTickTimer::timer_overrun { false };
volatile std::atomic<bool> SysTickTimer::reset_switch { false };

/**
 * \brief This Method sleeps for t milliseconds
 * \param t Time to sleep for in milliseconds
 * \retval none
 */
void SysTickTimer::sleep(uint32_t t)
{
  uint32_t t_final = time.load() + t;

  // wait until time+t has been reached
  while (time.load() < t_final)
    ;

  reset_switch.store(false);  // reset flag which is probably set
  timer_overrun.store(false); // reset flag which is probably set
}

SysTickTimer::SysTickTimer() {
}

SysTickTimer::~SysTickTimer() {
	// TODO Auto-generated destructor stub
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority){
    /* Configure the SysTick to have interrupt in 1ms time basis*/
    if (HAL_SYSTICK_Config(SystemCoreClock / (Timers::Systick::FREQUENCY_HZ)) > 0U)
    {
        return HAL_ERROR;
    }

    /* Configure the SysTick IRQ priority */
    if (TickPriority < (1UL << __NVIC_PRIO_BITS))
    {
        HAL_NVIC_SetPriority(SysTick_IRQn, Timers::Systick::SYSTEMTICK_IRQ_PRIO, 0U);
        uwTickPrio = TickPriority;
    }
    else
    {
        return HAL_ERROR;
    }

    /* Return function status */
    return HAL_OK;
}

void SysTick_Handler(void){
    HAL_IncTick();
	SysTickTimer::tick();
}
