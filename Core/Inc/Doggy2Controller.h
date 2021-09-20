/**
 * Created by tiwo on 02/02/2021.
 *
 * here all the motors are managed. doggy2motors contains the 3 joint motors.
 *
 */

#ifndef DOGGY_DOGGY2CONTROLLER_H
#define DOGGY_DOGGY2CONTROLLER_H

#include <Doggy2Motor.h>
#include <MotorNs.h>

class Doggy2Controller {

private:

    float tauGravity[3];
    float *angVel;
    float *angPos;
    float *encVal;
    float powerSysTickTotal;
    bool isSendState;
    bool axesInitialized;
    uint8_t msgCounter;
    uint8_t breakResistorHysteresis;

public:
    Doggy2Motor** Motor; ///< HAL Class for Motor A to C

    Doggy2Controller();
    virtual ~Doggy2Controller();

    void setIOPins(void);

    /**
     * calls the initialisation sequence
     * @return true if all joints initialized the zero position correctly
     */

    bool initMotors(){
        axesInitialized = true;
        for(int i=0; i<MotorNs::AXES; i++) {
            axesInitialized &= Motor[i]->initPosition();
        }
        //axesInitialized = Motor[0]->initPosition();
        return axesInitialized;
    }

    bool areAxesInitialized(void) {
        return axesInitialized;
    }

    /**
     * updates the motors state
     * this function has to be called every µC cycle
     */

    void updateMotors(void) {

        if(axesInitialized) calcGravity();   // Gravity force calculation
        calcBreakResistorState();
        calcEmergencyButtonPressed();

        for(int i=0; i<MotorNs::AXES; i++) {
            Motor[i]->updateMotor();
        }
        //calcGravity();
    }

    /**
     * sets angular positiom
     * @param _q1
     * @param _q2
     * @param _q3
     */

    void setAngPos(float _q1, float _q2, float _q3){
        Motor[MotorNs::MotorA]->setAimPos(_q1);
        Motor[MotorNs::MotorB]->setAimPos(_q2);
        Motor[MotorNs::MotorC]->setAimPos(_q3);
    }

    /**
     * sets angular velocity
     * @param _qd1
     * @param _qd2
     * @param _qd3
     */

    void setAngVel(float _qd1, float _qd2, float _qd3) {
        Motor[MotorNs::MotorA]->setVelocityPreCtrl(_qd1);
        Motor[MotorNs::MotorB]->setVelocityPreCtrl(_qd2);
        Motor[MotorNs::MotorC]->setVelocityPreCtrl(_qd3);
    }

    /**
     * set inertia torque from pre controller
     * @param _tau1
     * @param _tau2
     * @param _tau3
     */

    void setInertia(float _tau1, float _tau2, float _tau3) {
        Motor[MotorNs::MotorA]->setInertia(_tau1);
        Motor[MotorNs::MotorB]->setInertia(_tau2);
        Motor[MotorNs::MotorC]->setInertia(_tau3);
    }

    /**
     * response is triggered to be send in next cycle
     */

    void triggerResponse(void) {
        isSendState = true;
        msgCounter++;
    }

    /**
     *
     * @return true if a response is pending
     */

    bool isResTriggered(void) {
        if(isSendState) {
            isSendState = false;
            return true;
        } else return false;
    }

    float * getAngPos(void) {
        for(int i=0; i<MotorNs::AXES; i++) {
            angPos[i] = Motor[i]->getAngPos();
        }
        return angPos;
    }

    float * getAngVel(void) {
        for(int i=0; i<MotorNs::AXES; i++) {
            angVel[i] = Motor[i]->getAngVel();
        }
        return angVel;
    }

    float *getEncVal(void) {
        for(int i=0; i<MotorNs::AXES; i++) {
            encVal[i] = static_cast<float>(Motor[i]->getEncVal());
        }
        return encVal;
    }

    uint8_t getMsgCounter(void) {
        return msgCounter;
    }

    /**
     * calculates the gravitation influence on each joint.
     * The math is done in Peter corkes MATLAB Robotics Toolbox
     */

    void calcGravity(void)
    {
        float q2 = Motor[1]->getAngPos() * PI/180;
        float q3 = Motor[2]->getAngPos() * PI/180;

        tauGravity[0] = 0;
        tauGravity[1] = cos(q2)*(-1.25568680814) + sin(q2)*4.96584162e-1 - cos(q3)*sin(q2)*6.39093062772 + sin(q2)*sin(q3)*2.282755608e-2;
        tauGravity[2] = cos(q2) * (cos(q3)*3.486434629931106e+16 + sin(q3)*9.760817925442303e+18) * (-6.547535950918171e-19);
    }

    /**
     * calcs if break res has to be switched on
     */

    void calcBreakResistorState(void) {
        powerSysTickTotal = 0;
        for(int i=0; i<MotorNs::AXES; i++) {
            powerSysTickTotal += Motor[i]->getPowerConsumption();
        }

        if(powerSysTickTotal <= MotorNs::max_power_break) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);	/// set Break_Resistor
        }
        else {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);	/// set Break_Resistor
        }
    }

    void calcEmergencyButtonPressed(void) {
        if((!GPIOH->IDR & GPIO_PIN_13)) {
            LEDns::led.led_on(LED_RED);
            for(int i=0; i<MotorNs::AXES; i++) {
                Motor[i]->setEmergencyOff(true);
            }
        }
        else {
            LEDns::led.led_off(LED_RED);
            for(int i=0; i<MotorNs::AXES; i++) {
                Motor[i]->setEmergencyOff(false);
            }
        }
    }

};

namespace MotorNs
{
    extern Doggy2Controller doggy2motors;
}


#endif //DOGGY_DOGGY2CONTROLLER_H
