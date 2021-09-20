/*
 * SysTickTimer.h
 *
 *  Created on: Mar 16, 2021
 *      Author: tiwo
 */

#ifndef SRC_SYSTICKTIMER_H_
#define SRC_SYSTICKTIMER_H_

#include <stm32f4xx_hal.h>
#include <atomic>
//#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_it.h>
#include <core_cm4.h>
#include <Timersetting.h>

extern "C"
{
    HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
	void SysTick_Handler(void);
}

class SysTickTimer {
private:
	static volatile std::atomic<uint32_t> time; //! Actual time value starting at 0
	static volatile std::atomic<bool> timer_overrun; //! true if an overrun had occurred
	static volatile std::atomic<bool> reset_switch; //! acts like a watchdog - polling in between two cycles
public:
	SysTickTimer();
	virtual ~SysTickTimer();

	static void sleep(uint32_t t);
	void init_SysTick(void);

	/**
	 * \brief Restart with initial params
	 * \note This function has to be called, when starting a new loop with an
	 *       initial clock
	 */
	void start(void) {
		time.store(0);
		timer_overrun.store(false);
		reset_switch.store(false);
	}

	/**
	 * \brief Returns the actual system time
	 * \retval time - the actual time value in ms
	 */
	uint32_t getTime(void) {
		return time.load();
	}

	/**
	 * \brief This Method signals if an system tick has occurred
	 * \retval isSysTick - true if a system tick has occurred, false otherwise
	 * \note The process should be:
	 *   <CODE>
	 *   if (isSysTick())\n
	 *   {\n
	 *     // Do something meaningful\n
	 *     // We are in the timing of System Tick\n
	 *     resetTimingControl(); // if not reseted an overrun will occur\n
	 *   }\n
	 *   else\n
	 *   {\n
	 *     // SYSTEMTICK_PERIOD_MS has not been passed\n
	 *     if (isOutOfSync)\n
	 *     {\n
	 *       // Timer overrun - program not synchronised in time\n
	 *     }\n
	 *   }
	 *   </CODE>
	 */
	bool isSysTick(void) {
		return (reset_switch.load() & !timer_overrun.load());
	}

	/**
	 * \brief Displays if the timer is still in synchronisation or not
	 * \retval true - if not synchronised, false otherwise
	 */
	bool isOutOfSync(void) {
		return timer_overrun.load();
	}

	/**
	 * \brief This Method has to be called at least within every
	 * SYSTEMTICK_PERIOD_MS
	 */
	void resetTimingControl(void) {
		// only allowed to reset the switch if no timer overrun has occured
		if (!timer_overrun.load()) {
			reset_switch.store(false);
		}
	}

private:
	friend void SysTick_Handler(void);
    friend HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

	/* Make base class non copyable */
	SysTickTimer(const SysTickTimer&) = delete;
	const SysTickTimer& operator=(const SysTickTimer&) = delete;

	/**
	 * \brief Updates the actual timer value and checks for an overrun of the
	 *        Sample Period
	 */
	static void tick(void) {
		time.store(Timers::Systick::SYSTEMTICK_PERIOD_MS + time.load());
		// reset_switch hasn't been called since last
		// system tick => timing problem occurred
		if (reset_switch) {
			timer_overrun.store(true);
		} else {
			reset_switch.store(true);
		}
	}
};

namespace Timers {

namespace Systick {
extern SysTickTimer tim;
}
}

#endif /* SRC_SYSTICKTIMER_H_ */
