//
// Created by tiwo on 02/02/2021.
//

#include "Doggy2Motor.h"

Doggy2Motor::Doggy2Motor(uint8_t i, bool _initState, float *_tauGravity) : jointNo(i),
                                                                           tauGravity(_tauGravity),
                                                                           posState(startRoutine),
                                                                           currentLimit(I_max),
                                                                           swTriggerLeftSet(false),
                                                                           swTriggerRightSet(false),
                                                                           swTriggerLeft(0),
                                                                           emergencyOff(false),
                                                                           swTriggerRight(0),
                                                                           motorDir(ROTATION_COUNTERCLOCKWISE),
                                                                           encVal(Timers::Encoder::INIT_VAL),
                                                                           encValAim(Timers::Encoder::INIT_VAL),
                                                                           motorPwm(0),
                                                                           initCycleCounter(0),
                                                                           initErrorCounter(0),
                                                                           rpm(0),
                                                                           rpmAim(0),
                                                                           rpmDiff(0),
                                                                           integral_rpmDiff(0),
                                                                           angVelPreCtrl(0),
                                                                           tauController(0),
                                                                           tauInertia(0){}

Doggy2Motor::~Doggy2Motor() {}