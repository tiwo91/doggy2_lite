/**
 * Created by tiwo on 02/02/2021.
 *
 * containing the controller for each joint motor
 */

#ifndef DOGGY_DOGGY2MOTOR_H
#define DOGGY_DOGGY2MOTOR_H

#include "Timersetting.h"
#include "LED.h"
#include <cmath>

#define PI 3.14159265

static constexpr bool ROTATION_CLOCKWISE = true;
static constexpr bool ROTATION_COUNTERCLOCKWISE = false;
static constexpr float MAXON_GEAR = 13/3;
static constexpr float GEAR_RATIO[] = {MAXON_GEAR * 105/15, MAXON_GEAR * 90/15, MAXON_GEAR * 90/15}; /**< Gear ratio in between motor and tooth wheel*/
// 105
static constexpr uint32_t ticksPerMotorRevolution = 2000; ///< Encoder Ticks per motor revolution

static constexpr float adjustmentVelocity = 3; ///< rpm in init pos fct

static constexpr uint32_t tolleranceValEnc = 0x02;
static constexpr float VCC = 30.08; ///< supply voltage of actuators
static constexpr float I_max = 1.89; ///< max. const motor current
static constexpr float I_overcurrent = 3.0; ///< Continuous motor drive output current (see DRV8842 Motor Driver datasheet p. 6)
static constexpr int32_t zeroPosOffset[] = {0, 0, 0};


static constexpr float R_Motor = 3.9;   ///< ohmic portion of resistance in motor-windings.
static constexpr float TORQUE_CONST = 0.242;  ///< torque constant parameter of motor in [Nm/A] (see DC-Motor RE50 data sheet)
static constexpr float RPM_VOLTAGE_CONST = 39.5; ///< [rpm/V] (see DC-Motor RE50 data sheet)
static constexpr float R_HBridge = 0.52; ///< ohmic portion of resistance in motor-driver H-bridge.
static constexpr float R_Loss = R_Motor + R_HBridge;
static constexpr uint8_t numberOfSystemTicks = 5; /// for calc of Powerconsumtion the Power over \var powerApproxTickVal Ticks is aporoximated
static constexpr float K_thermal = 5; /// Gain of thermal behavior function
static constexpr float F_thermal = 0.0194; /// Factor of thermal regression function
static constexpr float MaxTemperature = 100; ///< Max winding temperature of motor in deg Celsius
static constexpr float TEMPERATURE_ENV = 25;


namespace BreakResistor {
    // PD7
    static constexpr float MAX_BREAK_OFFSET = -1.0;  ///< power [Watt] when BreakResistor is switched on
}

namespace varControll {
    static constexpr float Kp0[3] = {0.8, 0.5, 0.5};
    static constexpr float Kp[3] = {Kp0[0] * 360 / (GEAR_RATIO[0]*ticksPerMotorRevolution), Kp0[1] * 360 / (GEAR_RATIO[1]*ticksPerMotorRevolution), Kp0[2] * 360 / (GEAR_RATIO[2]*ticksPerMotorRevolution)};    ///< outer loop Kp gain value
    static constexpr float Kv[3] = {1.2, 2, 2};        ///< inner loop Kv gain value
    static constexpr float Ki[3] = {20, 10, 10};         ///< inner loop Ki integrator gain Value
}

enum initState {
    startRoutine,
    zeroPosOK,
    findZeroSw,
    findMiddlePos,
    findSwTriggerLeft,
    findSwTriggerRight,
};

class Doggy2Motor {
private:
    uint8_t jointNo;        ///< Joint number of motor
    int16_t motorPwm;
    bool motorDir;
    bool emergencyOff;

    initState posState;     ///< init Statemachine state
    uint8_t initCycleCounter;       ///< counter to check weather the joint is moving towards a limitation
    uint8_t initErrorCounter;
    uint32_t swTriggerRight;    ///< enc val on the right side of the switch
    uint32_t swTriggerLeft;     ///< enc val on the left side of the switch
    uint32_t zeroSwMiddlePos;   ///< enc val of middle pos \var{zeroPosOffset} is addet to the calculated value
    uint32_t encVal;        ///< enc val of current systick
    uint32_t encValAim;     ///< enc val of desired angle
    uint32_t encValLastSysTick;
    int32_t encValDiff;     ///< encoder value difference between current sysTick and the last one
    bool swTriggerRightSet;     ///< true if the value of \var{swTriggerRight} is valid
    bool swTriggerLeftSet;      ///< true if the value of \var{swTriggerLeft} is valid
    int32_t posDiff;    ///< position error input of outer control loop

    float rpm;      ///< rotation speed in current sysTick
    float rpmAim;   ///< desired speed
    float rpmDiff;  ///< velocity error input of inner control loop
    float integral_rpmDiff; ///< integrated error
    float angVelPreCtrl;     ///< precontroller velocity from doggy2PC ros node

    float powerSysTick;
    float motorTemperature;
    float currentLimit;     ///< limitares the motor curren due to thermal protection
    float tauController;    ///< calculated torque from controller circuit
    float tauInertia;       ///< pre controller torque send by USB
    float *tauGravity;      ///< pre controller torque calculated by Doggy2Controller class in dependencie of joint positions

    bool isClamping;    ///< if hardware is in saturation clamping is true


    bool zeroSwVal;

public:

    Doggy2Motor(uint8_t i, bool _initState, float *_tauGravity);

    virtual ~Doggy2Motor();

    void updateMotor(void) {
        encValLastSysTick = encVal;
        encVal = getEncVal();
        encValDiff = encVal - encValLastSysTick;
        updateZeroSw();
        rpm = calcRpm(encValDiff);

        if(posState == zeroPosOK){
            goToAimPos();
        }

        posDiff = static_cast<int32_t>(encValAim - encVal);

        if(posDiff > 100){
            LEDns::led.led_on(LED_GREEN);
        } else {
            LEDns::led.led_off(LED_GREEN);
        }

        rpmDiff = rpmAim - rpm;        /// error of controller input
        calcPowerConsumption();         /// calculates the power consumption powerMotor of motor in approximation over x Ticks
        currentLimitation();            /// according to calculated Motor temperature the max current is toggled between 1.89A and 5A
        outerControllLoop();
    }

    /**
     *
     * @return
     */

    bool initPosition(void) {
        uint32_t swHysteresis = 100;

        if(initErrorCounter >= 250){
            initErrorCounter = 0;
            zeroSwMiddlePos = Timers::Encoder::INIT_VAL;
            swTriggerRightSet = false;
            swTriggerLeftSet = false;
            posState = startRoutine;
        }

        switch (posState) {
            case startRoutine:
                setMotorSpeed(0);
                posState = findZeroSw;
                break;
            case zeroPosOK:
                setAimPos(0);
                return true;
            case findZeroSw:
                if (zeroSwVal) {
                    posState = findMiddlePos;
                } else {
                    if ((-3 < encValDiff) && (encValDiff < 3)) {
                        if (initCycleCounter <= 50) {
                            setMotorSpeed(5);
                            LEDns::led.led_on(LED_YELLOW);
                            LEDns::led.led_off(LED_GREEN);
                            initCycleCounter++;
                        } else if (initCycleCounter <= 100) {
                            setMotorSpeed(-5);
                            LEDns::led.led_off(LED_YELLOW);
                            LEDns::led.led_on(LED_GREEN);
                            initCycleCounter++;
                        } else {
                            initCycleCounter = 0;
                        }
                    }
                }
                break;
            case findMiddlePos:
                if (zeroSwVal == true) {
                    if (!swTriggerRightSet && motorDir) {
                        setMotorSpeed(-adjustmentVelocity);
                        swTriggerRight = getEncVal();   // actually its swTriggerLeft but it might be overwritten if just set before
                        posState = findSwTriggerRight;
                        break;
                    } else if (!swTriggerLeftSet && !motorDir) {
                        setMotorSpeed(adjustmentVelocity);
                        swTriggerLeft = getEncVal();
                        posState = findSwTriggerLeft;
                        break;
                    } else if (swTriggerRight && swTriggerLeft) {
                        zeroSwMiddlePos =
                                ((swTriggerLeft + swTriggerRight) / 2) +
                                zeroPosOffset[jointNo];
                        posState = zeroPosOK;
                        break;
                    }
                } else {
                    if ((-3 < encValDiff) && (encValDiff < 3)) {
                        initErrorCounter++;
                    }
                    break;
                }
            case findSwTriggerLeft:
                if (zeroSwVal == true) {
                    setMotorSpeed(adjustmentVelocity);
                } else {
                    if (getEncVal() > (swTriggerLeft + swHysteresis)) {
                        setMotorSpeed(-adjustmentVelocity);
                        swTriggerLeft = getEncVal();
                        swTriggerLeftSet = true;
                        posState = findMiddlePos;
                        break;
                    } else {
                        setMotorSpeed(adjustmentVelocity);
                        if ((-3 < encValDiff) && (encValDiff < 3)) {
                            initErrorCounter++;
                        }
                    }
                }
                break;
            case findSwTriggerRight:
                if (zeroSwVal == true) {
                    setMotorSpeed(-adjustmentVelocity);
                } else {
                    if (getEncVal() < (swTriggerRight - swHysteresis)) {
                        setMotorSpeed(adjustmentVelocity);
                        swTriggerRight = getEncVal();
                        swTriggerRightSet = true;
                        posState = findMiddlePos;
                        break;
                    } else {
                        setMotorSpeed(-adjustmentVelocity);
                        if ((-3 < encValDiff) && (encValDiff < 3)) {
                            initErrorCounter++;
                        }
                    }
                }
                break;
            default:
                break;
        }
        return false;
    }


    /**
 * @return the value of the encoder
 */

    uint32_t getEncVal(void) {
        switch (jointNo) {
            case MotorNs::MotorA:
                return HW::config.getTim1()->Instance->CNT;
            case MotorNs::MotorB:
                return HW::config.getTim3()->Instance->CNT;
            case MotorNs::MotorC:
                return HW::config.getTim5()->Instance->CNT;
            default:
                return 0;
        }
    }

    /**
     * updates the value of the zero position switch
     */

    void updateZeroSw(void) {
        switch (jointNo) {
            case MotorNs::MotorA:
                zeroSwVal = !(GPIOA->IDR & GPIO_PIN_11);
                break;
            case MotorNs::MotorB:
                zeroSwVal = !(GPIOC->IDR & GPIO_PIN_9);
                break;
            case MotorNs::MotorC:
                zeroSwVal = !(GPIOH->IDR & GPIO_PIN_8);
                break;
            default:
                break;
        }

    }

    bool goToAimPos(void) {
        bool ret;

        if (encVal < (encValAim - tolleranceValEnc)) {
            setMotorSpeed(posController());
            ret = false;

        } else if (encVal > (encValAim + tolleranceValEnc)) {
            setMotorSpeed(posController());
            ret = false;
        } else {
            setMotorSpeed(0);
            ret = true;
        }
        return ret;
    }

    /**
     * calculates the power consumption powerMotor of motor in approximation over x Ticks
     */

    void calcPowerConsumption(void) {
        powerSysTick = fabsf(rpmDiff) * VCC * VCC / (R_Motor * 100 / 2.00243) / Timers::Systick::FREQUENCY_HZ;
        if(fabsf(rpm) > fabsf(rpmAim)){
            powerSysTick *= -1;
        }

        calcWindTemp((float) numberOfSystemTicks / Timers::Systick::FREQUENCY_HZ);
    }

    float getPowerConsumption(void){return powerSysTick;}

    /**
     * according to calculated Motor temperature the max current is toggled between 1.89A and 5A
     */

    void currentLimitation(void) {
        if(motorTemperature >= MaxTemperature) {
            currentLimit = I_max;
        } else currentLimit = I_overcurrent;
    }


    /**
         *  the winding temperature of the Motor is calculated. the function represents a simplification of the thermal behavior of the joint motor
         * @param power
         * @param deltaTime
         */

    void calcWindTemp(float deltaTime) {
        motorTemperature += deltaTime * ((fabsf(powerSysTick) * K_thermal) + TEMPERATURE_ENV - motorTemperature) * F_thermal / K_thermal;
    }

    void outerControllLoop(void) {
        innerControllLoop();           /// Controller algorithm calculates the desired speed

        float pwm0 = rpm * GEAR_RATIO[jointNo] * 100 / (RPM_VOLTAGE_CONST * VCC);  /// pwm Value for zero torque in current cycle

        int pwmTauMax = static_cast<int>(currentLimit * R_Motor * 100 / VCC);

        float pwmTau = tauController * 100 * R_Motor / (TORQUE_CONST * GEAR_RATIO[jointNo] * VCC);
        if (pwmTau > pwmTauMax) {
            pwmTau = pwmTauMax;
            isClamping = true;
        } else if (pwmTau < -pwmTauMax) {
            pwmTau = -pwmTauMax;
            isClamping = true;
        } else isClamping = false;

        int16_t pwmZwischenwert = static_cast<int>(pwm0 + pwmTau);

        if (pwmZwischenwert > MotorNs::MotorPwmPeriod) {
            motorPwm = MotorNs::MotorPwmPeriod;
            isClamping = true;
        } else if (pwmZwischenwert < -MotorNs::MotorPwmPeriod) {
            motorPwm = -MotorNs::MotorPwmPeriod;
            isClamping = true;
        } else {
            motorPwm = pwmZwischenwert;
            isClamping = false;
        }

        if (pwmZwischenwert >= 0) {
            motorDir = ROTATION_COUNTERCLOCKWISE;
        } else {
            motorDir = ROTATION_CLOCKWISE;
        }

        setMotorPWM(motorPwm);
    }

    void innerControllLoop(void) {
        float tau_g,
                tau_v,
                tau_i;

        float i_max = 0.5;

        //float inductiveVoltage = rpm * GEAR_RATIO / RPM_VOLTAGE_CONST;
        if (posState == zeroPosOK) {
            tau_g = *tauGravity*1/3;
            if (rpmDiff >= 10 || -10 >= rpmDiff) isClamping = true;
            else {
                if(emergencyOff) isClamping = true;
                else isClamping = false;
            }
        } else {
            tau_g = 0;
            isClamping = true;
        }
        tau_v = varControll::Kv[jointNo] * rpmDiff;
        if (!isClamping) integral_rpmDiff += rpmDiff / Timers::Systick::FREQUENCY_HZ;
        if(integral_rpmDiff >= i_max) integral_rpmDiff = i_max;
        else if(integral_rpmDiff < -integral_rpmDiff) integral_rpmDiff = -integral_rpmDiff;

        tau_i = varControll::Ki[jointNo] * integral_rpmDiff;

        //__testVal[0] = tau_g;
        //__testVal[1] = tau_v;
        //__testVal[2] = tau_i;

        tauController = tau_g + tau_v + tau_i + tauInertia + *tauGravity;
    }

    float posController(void) {
        float _rpm;

        _rpm = posDiff * varControll::Kp[jointNo];                    /// P position controller
        _rpm += angVelPreCtrl / 6;     /// angVel / 6 = rpm
        return _rpm;
    }

    /**
 * calculates the rotation speed of the joint during the last sysTick in rpm
 * @param _encValDiff
 * @return
 */

    float calcRpm(int32_t _encValDiff) {
        return ((((float) _encValDiff) * (float) Timers::Systick::FREQUENCY_HZ)) * 60 /
                (GEAR_RATIO[jointNo]*ticksPerMotorRevolution);
    }

    void setMotorSpeed(float _rpm_val) {
        rpmAim = _rpm_val;
    }

    void setMotorPWM(int16_t pwmVal) {
        switch (jointNo){
            case MotorNs::MotorA:
                if ((pwmVal >= 0) && (pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.getTim4()->Instance->CCR3 = 0x00000000 | static_cast<uint16_t>(pwmVal);
                    HW::config.getTim4()->Instance->CCR4 = 0x00000000;
                } else if ((pwmVal < 0) && (-pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.getTim4()->Instance->CCR3 = 0x00000000;
                    HW::config.getTim4()->Instance->CCR4 = 0x00000000 | static_cast<uint16_t>(-pwmVal);
                }
                break;
            case MotorNs::MotorB:   /// MotorB direction is reversed
                if ((pwmVal >= 0) && (pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.getTim4()->Instance->CCR1 = 0x00000000;
                    HW::config.getTim4()->Instance->CCR2 = 0x00000000 | static_cast<uint16_t>(pwmVal);
                } else if ((pwmVal < 0) && (-pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.getTim4()->Instance->CCR1 = 0x00000000 | static_cast<uint16_t>(-pwmVal);
                    HW::config.getTim4()->Instance->CCR2 = 0x00000000;
                }
                break;
            case MotorNs::MotorC:
                if ((pwmVal >= 0) && (pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.getTim12()->Instance->CCR1 = 0x00000000 | static_cast<uint16_t>(pwmVal);
                    HW::config.getTim12()->Instance->CCR2 = 0x00000000;
                } else if ((pwmVal < 0) && (-pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.getTim12()->Instance->CCR1 = 0x00000000;
                    HW::config.getTim12()->Instance->CCR2 = 0x00000000 | static_cast<uint16_t>(-pwmVal);
                }
                break;
            default:
                break;
        }
    }

    /*{
        switch (jointNo){
            case MotorNs::MotorA:
                if ((pwmVal >= 0) && (pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_3, MotorNs::MotorPwmPeriod, static_cast<uint16_t>(pwmVal));
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_4, MotorNs::MotorPwmPeriod, 0);
                } else if ((pwmVal < 0) && (-pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_3, MotorNs::MotorPwmPeriod, 0);
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_4, MotorNs::MotorPwmPeriod, static_cast<uint16_t>(-pwmVal));
                }
                break;
            case MotorNs::MotorB:
                if ((pwmVal >= 0) && (pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_1, MotorNs::MotorPwmPeriod, static_cast<uint16_t>(pwmVal));
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_2, MotorNs::MotorPwmPeriod, 0);
                } else if ((pwmVal < 0) && (-pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_1, MotorNs::MotorPwmPeriod, 0);
                    HW::config.setPWM(HW::config.getTim4(), TIM_CHANNEL_2, MotorNs::MotorPwmPeriod, static_cast<uint16_t>(-pwmVal));
                }
                break;
            case MotorNs::MotorC:
                if ((pwmVal >= 0) && (pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.setPWM(HW::config.getTim12(), TIM_CHANNEL_1, MotorNs::MotorPwmPeriod, static_cast<uint16_t>(pwmVal));
                    HW::config.setPWM(HW::config.getTim12(), TIM_CHANNEL_2, MotorNs::MotorPwmPeriod, 0);
                } else if ((pwmVal < 0) && (-pwmVal <= MotorNs::MotorPwmPeriod)) {
                    HW::config.setPWM(HW::config.getTim12(), TIM_CHANNEL_1, MotorNs::MotorPwmPeriod, 0);
                    HW::config.setPWM(HW::config.getTim12(), TIM_CHANNEL_2, MotorNs::MotorPwmPeriod, static_cast<uint16_t>(-pwmVal));
                }
                break;
            default:
                break;
        }
    }*/

    void setAimPos(float _angAimPos) {
        encValAim = static_cast<uint32_t>(static_cast<int32_t >(_angAimPos * (GEAR_RATIO[jointNo]*ticksPerMotorRevolution) / 360) +
                                          static_cast<int32_t>(zeroSwMiddlePos));
    }

    void setInertia(float _inertiaTorque) {
        tauInertia = _inertiaTorque;
    }

    void setVelocityPreCtrl(float _vel) {
        angVelPreCtrl = _vel;
    }

    float getAngleRad(void) {
        return (static_cast<float>(encVal) - static_cast<float>(zeroSwMiddlePos)) * 2 * PI /
                (GEAR_RATIO[jointNo]*ticksPerMotorRevolution);
    }

    /**
     * \brief getAngPos
     * @return angular position
     */

    float getAngPos(void) {
        return (static_cast<float>(encVal) - static_cast<float>(zeroSwMiddlePos)) * 360 / (GEAR_RATIO[jointNo]*ticksPerMotorRevolution);
    }

    /**
     * \brief getAngVel
     * @return angular velocity
     */

    float getAngVel(void) {
        return rpm * 6; /// deg/s is equal to 6 * U/min
    }

    void setEmergencyOff(bool state) {
        emergencyOff = state;
    }
};


#endif //DOGGY_DOGGY2MOTOR_H
