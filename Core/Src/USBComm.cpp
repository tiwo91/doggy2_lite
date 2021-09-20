//
// Created by tiwo on 19/03/2021.
//


#include "USBComm.h"
#include "Doggy2Controller.h"


USBComm::USBComm() : enable_Rx(true), enable_Tx(true) {}

USBComm::~USBComm() {}

void USBComm::update(void) {

    if (has_recv) {
        has_recv = false;
        if ((rxDataBuffer[62] == COMM_FLAGS::EOT) && (rxDataBuffer[63] == COMM_FLAGS::EOT)) {
            if(checkMessage()) MotorNs::doggy2motors.triggerResponse();
        }
    } else {

    }
}

void USBComm::sendUpdate(void) {

    char text[] = "  Hallo es geht!\n";
    txDataBuffer[0] = RCV_ID::microConState;
    for (uint8_t i = 1; i < USB_DATA_LENGTH; i++) {
        if (i <= sizeof(text)) {
            txDataBuffer[i] = static_cast<uint8_t>(text[i - 1]);
        } else {
            txDataBuffer[i] = 0x20; // ascii code for space key
        }

    }

    txDataBuffer[62] = COMM_FLAGS::EOT;
    txDataBuffer[63] = COMM_FLAGS::EOT;

    UserDebug::pin.pin_set(PIN7);
    //HAL_UART_Transmit(HW::config.getUart3(), txDataBuffer, sizeof(txDataBuffer), HAL_MAX_DELAY);
    //UserDebug::pin.pin_reset(PIN7);
}

bool USBComm::triggerTx(void) {
    if (enable_Tx) {
        HAL_UART_Transmit_IT(HW::config.getUart3(), txDataBuffer, txDataLength);
        enable_Tx = false;
    }
}

void USBComm::sendInitialized() {

    dataAdd((uint8_t) RCV_ID::isInitialized);
    dataAdd((uint8_t) COMM_FLAGS::ACK);
    dataAdd((uint8_t) 0x01);
    dataAdd((uint8_t) COMM_FLAGS::EOT);
    dataAdd((uint8_t) COMM_FLAGS::EOT);


#ifdef DOGGY2_DEBUG_USB_ON
    UserDebug::pin.pin_set(PIN7);
#endif
    triggerTx();
}

void USBComm::sendResponse(uint8_t count, float *_ang, float *_vel, int32_t *testVal) {
    dataAdd((uint8_t) RCV_ID::microConState);
    dataAdd(count);
    dataAdd(_ang[0]);
    dataAdd(_ang[1]);
    dataAdd(_ang[2]);

    dataAdd(_vel[0]);
    dataAdd(_vel[1]);
    dataAdd(_vel[2]);

    dataAdd(testVal[0]);
    dataAdd(testVal[1]);
    dataAdd(testVal[2]);

    dataAdd((uint8_t) 100);
    dataAdd((uint8_t) 100);
    dataAdd((uint8_t) 100);
    dataAdd((uint8_t) COMM_FLAGS::EOT);
    dataAdd((uint8_t) COMM_FLAGS::EOT);

    triggerTx();
}

bool USBComm::dataAdd(uint8_t _data) {
    if (txDataLength <= 63) {
        txDataBuffer[txDataLength] = _data;
        txDataLength++;
        return true;
    } else {
        return false;
    }
}

bool USBComm::dataAdd(float _data) {
    uint8_t *p = (uint8_t *) &_data;

    if (txDataLength <= 59) {
        txDataBuffer[txDataLength] = p[0];
        txDataLength++;
        txDataBuffer[txDataLength] = p[1];
        txDataLength++;
        txDataBuffer[txDataLength] = p[2];
        txDataLength++;
        txDataBuffer[txDataLength] = p[3];
        txDataLength++;
        return true;
    } else return false;
}

bool USBComm::dataAdd(int32_t _data) {
    uint8_t *p = (uint8_t *) &_data;

    if (txDataLength <= 59) {
        txDataBuffer[txDataLength] = p[0];
        txDataLength++;
        txDataBuffer[txDataLength] = p[1];
        txDataLength++;
        txDataBuffer[txDataLength] = p[2];
        txDataLength++;
        txDataBuffer[txDataLength] = p[3];
        txDataLength++;
        return true;
    } else return false;
}

void USBComm::receiveData(void) {
    HAL_UART_Receive_IT(HW::config.getUart3(), rxDataBuffer, 64);
}

void USBComm::sendInitialized_test() {

    uint16_t _dataLength = 5;
    uint8_t _data[_dataLength];

    _data[0] = RCV_ID::isInitialized;
    _data[1] = COMM_FLAGS::ACK;
    _data[2] = 0x01;
    _data[3] = COMM_FLAGS::EOT;
    _data[4] = COMM_FLAGS::EOT;

    UserDebug::pin.pin_set(PIN7);
    HAL_UART_Transmit(HW::config.getUart3(), _data, _dataLength, HAL_MAX_DELAY);
    UserDebug::pin.pin_reset(PIN7);
}

bool USBComm::checkMessage(void) {
    uint8_t id = rxDataBuffer[0];
    float a,b,c,d,e,f,g,h,i;

    switch (id) {
        case RCV_ID::setAngles:
            a = *((float *) &rxDataBuffer[1]);
            b = *((float *) &rxDataBuffer[5]);
            c = *((float *) &rxDataBuffer[9]);
            d = *((float *) &rxDataBuffer[13]);
            e = *((float *) &rxDataBuffer[17]);
            f = *((float *) &rxDataBuffer[21]);
            g = *((float *) &rxDataBuffer[25]);
            h = *((float *) &rxDataBuffer[29]);
            i = *((float *) &rxDataBuffer[33]);
            MotorNs::doggy2motors.setAngPos(a, b, c);
            MotorNs::doggy2motors.setAngVel(d, e, f);
            MotorNs::doggy2motors.setInertia(g, h, i);
            return true;
        case RCV_ID::setAngle:
            a = *((float *) &rxDataBuffer[1]);
            b = *((float *) &rxDataBuffer[5]);
            c = *((float *) &rxDataBuffer[9]);
            MotorNs::doggy2motors.setAngPos(a, b, c);
            return true;
        case RCV_ID::deviceHardReset:
            return false;
        default:
            return false;
    }
}

void USBComm::initUsb() {
    //__HAL_UART_ENABLE_IT(HW::config.getUart3(), UART_IT_RXNE);
    //__HAL_UART_ENABLE_IT(HW::config.getUart3(), UART_IT_TXE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    Comm::usb.enableTransmission();
#ifdef DOGGY2_DEBUG_USB_ON
    UserDebug::pin.pin_reset(PIN7);
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#ifdef DOGGY2_DEBUG_USB_ON
    LEDns::led.led_toggle(LED_YELLOW);
#endif
    __HAL_UART_CLEAR_FLAG(HW::config.getUart3(), UART_FLAG_RXNE);


    Comm::usb.setRx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

    //LEDns::led.led_on(LED_RED);
}

void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(HW::config.getUart3());
}

namespace Comm {
    USBComm usb;
}