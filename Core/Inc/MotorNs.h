//
// Created by tiwo on 30/03/2021.
//

#ifndef DOGGY2_LITE_MOTORNS_H
#define DOGGY2_LITE_MOTORNS_H

namespace MotorNs {
    constexpr uint8_t MotorA = 0;
    constexpr uint8_t MotorB = 1;
    constexpr uint8_t MotorC = 2;

    constexpr uint8_t AXES = 3;

    constexpr uint16_t MotorPwmPeriod = 99;

    constexpr float max_power_break = -0.2; /// Watt max power from motors in generator mode
}

#endif //DOGGY2_LITE_MOTORNS_H
