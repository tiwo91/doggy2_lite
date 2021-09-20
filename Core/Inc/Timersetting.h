/*
 * Timersetting.h
 *
 *  Created on: Mar 16, 2021
 *      Author: tiwo
 * \brief   Includes all settings for the used timers
 *
 * \defgroup Timers All Settings for used Timers
 * @{
 * \ingroup Timers
 */

#ifndef INC_TIMERSETTING_H_
#define INC_TIMERSETTING_H_

#include <stm32f4xx_hal.h>
#include "hw_config.h"

extern TIM_HandleTypeDef htim4;

/**
 * \namespace Timers
 * \brief All Settings for used Timers including static members for specific timers
 */
namespace Timers {
/**\addtogroup SystemTick
 * @{
 */
    namespace Systick {
        static constexpr uint8_t FREQ_DIV = 4; /** < Divides the set frequency by a given Number, such that
 * \f$ F_\mathrm{s} = \frac{\SI{1}{\kilo\hertz}}{n}\f$.
 * The FREQ_DIV is \f$ n\f$ .
 * Only the following numbers are allowed:\n
 * \begin{description}
 * \item[n=1] \f$ F_\mathrm{s} = \SI{1}{\kilo\hertz}\f$
 * \item[n=2] \f$ F_\mathrm{s} = \SI{500}{\hertz}\f$
 * \item[n=4] \f$ F_\mathrm{s} = \SI{250}{\hertz}\f$
 * \item[n=5] \f$ F_\mathrm{s} = \SI{200}{\hertz}\f$
 * \item[n=8] \f$ F_\mathrm{s} = \SI{125}{\hertz}\f$
 * \item[n=10] \f$ F_\mathrm{s} = \SI{100}{\hertz}\f$
 * \item[n=20] \f$ F_\mathrm{s} = \SI{50}{\hertz}\f$
 * \item[n=25] \f$ F_\mathrm{s} = \SI{40}{\hertz}\f$
 * \end{description}
 */
        static constexpr uint32_t FREQUENCY_HZ = 1000 / FREQ_DIV; //! System Tick Frequency
        static constexpr uint32_t SYSTEMTICK_PERIOD_MS = 1000 / FREQUENCY_HZ; //! 1000 ticks / Hz = ms
        static constexpr uint32_t SYSTEMTICK_IRQ_PRIO = 0; //! SysTick Timer interrupt priority
    }

    namespace Encoder{
        static constexpr uint32_t INIT_VAL = 0x00007FFF;
    }

    namespace Motor {
    }
}

#endif /* INC_TIMERSETTING_H_ */
