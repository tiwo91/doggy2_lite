//
// Created by tiwo on 25/03/2021.
//

#include "DebugPins.h"

DebugPins::DebugPins() {}
DebugPins::~DebugPins() {}

void DebugPins::pin_set(uint16_t pin_) {
    HAL_GPIO_WritePin(GPIOE, pin_, GPIO_PIN_SET);
}

void DebugPins::pin_reset(uint16_t pin_){
    HAL_GPIO_WritePin(GPIOE, pin_, GPIO_PIN_RESET);
}

void DebugPins::pin_toggle(uint16_t pin_) {
    if(HAL_GPIO_ReadPin(GPIOE, pin_)) {
        HAL_GPIO_WritePin(GPIOE, pin_, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOE, pin_, GPIO_PIN_SET);
    }
}

namespace UserDebug {
    DebugPins pin;
}