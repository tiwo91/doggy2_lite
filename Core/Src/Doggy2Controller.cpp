
#include "Doggy2Controller.h"

namespace MotorNs {
    Doggy2Controller doggy2motors;
}

Doggy2Controller::Doggy2Controller() : Motor(new Doggy2Motor *[MotorNs::AXES]), isSendState(false), axesInitialized(false), angPos(new float[MotorNs::AXES]), angVel(new float[MotorNs::AXES]), encVal(new float[MotorNs::AXES]) {

    /*---------- Init completed ------------*/

    for (int i = 0; i < MotorNs::AXES; i++) {
        Motor[i] = new Doggy2Motor(i, false, tauGravity + i);
        Motor[i]->setMotorPWM(0);
        angPos[i] = 0;
        angVel[i] = 0;
    }
}

void Doggy2Controller::setIOPins(void) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);	//< RC_SHDN_VSS "Break_Resistor"
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);	//< VREF_A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);	//< VREF_B
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);	//< VREF_C
    /** Decay mode pins **/
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);     // fast decay mode
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);    // fast decay mode
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);    // fast decay mode

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);	//< RST_ABC

    /** start encoders **/

    HW::config.getTim1()->Instance->CNT = Timers::Encoder::INIT_VAL;
    HW::config.getTim3()->Instance->CNT = Timers::Encoder::INIT_VAL;
    HW::config.getTim5()->Instance->CNT = Timers::Encoder::INIT_VAL;

    HAL_TIM_Encoder_Start(HW::config.getTim1(), TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(HW::config.getTim3(), TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(HW::config.getTim5(), TIM_CHANNEL_ALL);
}

Doggy2Controller::~Doggy2Controller() {
    delete Motor;
}