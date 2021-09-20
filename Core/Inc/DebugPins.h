/**
 * Created by tiwo on 25/03/2021.
 *
 * toggle pins to do some debug stuff using the oscilloscope e.g.
 */
//
//

#ifndef DOGGY2_LITE_DEBUGPINS_H
#define DOGGY2_LITE_DEBUGPINS_H

#include "stm32f4xx_hal.h"

#define PIN7 GPIO_PIN_7
#define PIN8 GPIO_PIN_8
#define PIN9 GPIO_PIN_9

class DebugPins{
public:
    DebugPins();
    ~DebugPins();

    void pin_set(uint16_t pin_);
    void pin_reset(uint16_t pin_);
    void pin_toggle(uint16_t pin_);
};

namespace UserDebug {
    extern DebugPins pin;
}

#endif //DOGGY2_LITE_DEBUGPINS_H
