/**
 * Created by tiwo on 19/03/2021.
 *
 * config of the USB interface
 */

#ifndef DOGGY2_LITE_USBCOMM_H
#define DOGGY2_LITE_USBCOMM_H

#include <atomic>
#include "hw_config.h"
#include "DebugPins.h"

#define USB_DATA_LENGTH 64

extern "C" {
void USART3_IRQHandler();
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);


};

class USBComm {
public:
    USBComm();

    virtual ~USBComm();

private:
    /* Private Members */
    bool has_recv;     //! Indicates if a byte or more has been sent
    bool response_pending; //! response has to be send in next systick step
    bool enable_Tx; //! Transmission switch
    bool enable_Rx; //! Reception switch

    uint8_t bData_In[USB_DATA_LENGTH];
    uint8_t bData_Out[USB_DATA_LENGTH];
    uint8_t byteIn; //!< For reading in a byte and use it more than once
    std::atomic<bool> isMsgInUpdated; //!< true if a message has received and read out for usage
    int pwmSetToMotor; //!< The Motor Number the pwm limitation is set to, a 0 means all motors should be set
    uint8_t stateFlag; //!< One of the \ref REV_ID flags to operate
    bool isEOTFirst; //!< Indicates the first End of transmission flag has arrived

    inline static uint8_t txDataBuffer[USB_DATA_LENGTH];
    inline static uint8_t rxDataBuffer[USB_DATA_LENGTH];
    inline static uint8_t txDataLength;

    bool checkMessage(void);

    friend void USART3_IRQHandler(void);

    friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

    friend void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

    friend void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

protected:
    /* Make base class non copyable */
    USBComm(const USBComm &) = delete;

    const USBComm &operator=(const USBComm &) = delete;

public:

    void setRx(void) { has_recv = true; }

    void setTx(void) { response_pending = true; }

    void enableTransmission(void) {
        enable_Tx = true;
        txDataLength = 0;   // reset TxDataBuffer
    }

    void enableReception(void) { enable_Rx = true; }

    bool isResponsePending(void) { return response_pending; }

    void clearBuffer(void) {

    }

    void initUsb(void);

    void emptyOutputBuff(void) {
        uint16_t _dataLength = 5;
        uint8_t _data[_dataLength];
        HAL_UART_Transmit(HW::config.getUart3(), _data, _dataLength, HAL_MAX_DELAY);
    }

    bool triggerTx(void);

    void sendInitialized(void);

    void sendResponse(uint8_t count, float *_ang, float *_vel, int32_t *testVal);

    bool dataAdd(uint8_t _data);

    bool dataAdd(float _data);
    bool dataAdd(int32_t _data);

    void sendInitialized_test(void);

    void update(void);

    void sendUpdate(void);

    void receiveData(void);

};

namespace Comm {
    extern USBComm usb;
}


#endif //DOGGY2_LITE_USBCOMM_H
