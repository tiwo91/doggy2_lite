//
// Created by tiwo on 19/03/2021.
//

#include <hw_config.h>

HW_Config::HW_Config() {}

HW_Config::~HW_Config() {}

void HW_Config::init_HW(void) {
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM4_Init();
    MX_TIM12_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_USART3_UART_Init();
}

namespace HW {
    HW_Config config;
}