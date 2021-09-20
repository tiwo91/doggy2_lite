/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - HW configuration is done in hw_config.h
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hw_config.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <SysTickTimer.h>
#include <DebugPins.h>
#include <PowerCtrl.h>
#include "Doggy2Controller.h"
#include <USBComm.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// ----- main() ---------------------------------------------------------------

enum mainStateMachine {
    initSeq,
    mainSeq,
    testSeq
} uCState = initSeq;

void mainFSM(Doggy2Controller &motors, uint32_t &i) {

    switch (uCState) {
        case initSeq:
            if (motors.initMotors()) {
                Comm::usb.sendInitialized();
                uCState = mainSeq;
            }
            break;
        case mainSeq:

            break;
        case testSeq:

            break;
        default:
            break;
    };
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    HW::config.init_HW();
    /* USER CODE BEGIN 2 */
    LEDns::led.led_on(LED_YELLOW);
    if (PowerNs::power.setVCCPower(PowerNs::powerOn)) LEDns::led.led_on(LED_YELLOW);
    else LEDns::led.led_on(LED_RED);
    Comm::usb.initUsb();

    uint32_t _time = Timers::Systick::tim.getTime();
    while (_time + 5000 > Timers::Systick::tim.getTime());
    SystemCoreClockUpdate();
    _time = Timers::Systick::tim.getTime();
    while (_time + 500 > Timers::Systick::tim.getTime());
    LEDns::led.led_on(LED_GREEN);

    MotorNs::doggy2motors.setIOPins();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t i, cnt = 0;

    //Comm::usb.emptyOutputBuff();
    Timers::Systick::tim.start();

    LEDns::led.led_off(LED_YELLOW);

    //MotorNs::doggy2motors.updateMotors();

    int32_t testValues[3] = {2,5,7};

//    while(1) {
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);	/// set Break_Resistor
//    }

    while (1) {
        /* USER CODE END WHILE */
        if (Timers::Systick::tim.isSysTick()) {

            mainFSM(MotorNs::doggy2motors, i);

            Comm::usb.receiveData();
            Comm::usb.update();

            MotorNs::doggy2motors.updateMotors();



            if(MotorNs::doggy2motors.isResTriggered()){
                Comm::usb.sendResponse(MotorNs::doggy2motors.getMsgCounter(), MotorNs::doggy2motors.getAngPos(), MotorNs::doggy2motors.getAngVel(), testValues);
            }

            //HW::config.setPWM(MotorNs::MotorA, 5);

            cnt++;

            if(cnt % 5 == 0) {
                //Comm::usb.sendResponse(MotorNs::doggy2motors.getMsgCounter(), MotorNs::doggy2motors.getEncVal(), MotorNs::doggy2motors.getAngVel(), testValues);
            }

            if (cnt % 250 == 0) {
                //LEDns::led.led_toggle(LED_GREEN);
                cnt = 0;
            }
            Timers::Systick::tim.resetTimingControl();
        }
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
