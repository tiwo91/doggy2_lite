/**
 *
 */

#ifndef DOGGY2_LITE_HW_CONFIG_H
#define DOGGY2_LITE_HW_CONFIG_H

#include <stm32f4xx_hal.h>
#include "GlobalEnumsStructures.h"
#include "MotorNs.h"
#include <cstdint>
#include <Timersetting.h>

namespace Comm {

//    static constexpr USART_TypeDef* Port           = USART3; //!< USART3 is used for the communication via USB using FTDI driver
//    static constexpr GPIO_TypeDef*  GpioPort       = GPIOC;  //!< USART3 is used on Port C with Pins 10 and 11 for the RX/TX
//    static constexpr uint32_t       PortClock      = RCC_APB1Periph_USART3;
//    static constexpr uint32_t       PeriphClock    = RCC_AHB1Periph_GPIOC; //!< Clock for Port
//    static constexpr uint16_t       RxPin          = GPIO_Pin_10; //!< RX Pin on MC
//    static constexpr uint8_t        AF_PinSourceRx = GPIO_PinSource10; //!< RX Pin alternate function on MC
//    static constexpr uint16_t       TxPin          = GPIO_Pin_11; //!< TX Pin on MC
//    static constexpr uint8_t        AF_PinSourceTx = GPIO_PinSource11; //!< TX Pin alternate function on MC
//    static constexpr uint8_t        GpioAF         = GPIO_AF_USART3; //!< GPIO's alternate function
    static constexpr uint8_t        TxBuffLen      = 255U; //!< Length of reception buffer
    static constexpr uint8_t        RxBuffLen      = 128U; //!< Length of transmission buffer
    static constexpr uint32_t       Baudrate       = 921600; //!< Baudrate to be set to 921600 Hz, which is \f$ f_\mathrm{baud} = \frac{f_\mathrm{bit}}{n_\mathrm{BitPerSymbol}} = \frac{921600\,\mathrm{Hz}}{10} = 92.16\,\mathrm{kBaud/s} \f$
//    static constexpr uint16_t       Wordlength     = USART_WordLength_8b; //!< Using standard 8bits as word length
//    static constexpr uint16_t       Stopbits       = USART_StopBits_1; //!< Only one stop bit to be used
//    static constexpr uint16_t       Parity         = USART_Parity_No; //!< No parity bit used
//    static constexpr uint16_t       Flowcontrol    = USART_HardwareFlowControl_None; //!< Do not use Flowcontrol

    static constexpr uint32_t       FRAC2          = 100;     ///< Fractional Part for float number - here two digits
    static constexpr uint32_t       FRAC3          = 1000;    ///< Fractional Part for float number - here three digits
    static constexpr uint32_t       FRAC4          = 10000;   ///< Fractional Part for float number - here four digits
    static constexpr uint32_t       FRAC5          = 100000;  ///< Fractional Part for float number - here five digits
    static constexpr uint32_t       FRAC6          = 1000000; ///< Fractional Part for float number - here six digits

    static constexpr uint8_t        EOM            = COMM_FLAGS::EOT; ///< Is the End Of Message received
    static constexpr uint8_t        GS             = COMM_FLAGS::SPACE_CHAR; //!< Group Seperator
}

extern "C" {
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
}

class HW_Config {
public:
    HW_Config();
    ~HW_Config();

    void init_HW(void);

    void Error_Handler(void){}

    TIM_HandleTypeDef * getTim1(void) { return &htim1;}
    TIM_HandleTypeDef * getTim3(void) { return &htim3;}
    TIM_HandleTypeDef * getTim4(void) { return &htim4;}
    TIM_HandleTypeDef * getTim5(void) { return &htim5;}
    TIM_HandleTypeDef * getTim12(void) { return &htim12;}
    UART_HandleTypeDef * getUart3(void) { return &huart3;}

    TIM_OC_InitTypeDef sConfigOC_TIM4 = {0};
    TIM_OC_InitTypeDef sConfigOC_TIM12 = {0};

private:
    TIM_HandleTypeDef htim1;
    TIM_HandleTypeDef htim3;
    TIM_HandleTypeDef htim4;
    TIM_HandleTypeDef htim5;
    TIM_HandleTypeDef htim12;

    UART_HandleTypeDef huart3;

    /**
  * @brief System Clock Configuration
  * @retval None
  */
    void SystemClock_Config(void)
    {
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

        /** Configure the main internal regulator output voltage
        */
        __HAL_RCC_PWR_CLK_ENABLE();
        __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
        /** Initializes the RCC Oscillators according to the specified parameters
        * in the RCC_OscInitTypeDef structure.
        */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 4;
        RCC_OscInitStruct.PLL.PLLN = 128;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
        RCC_OscInitStruct.PLL.PLLQ = 4;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            Error_Handler();
        }
        /** Initializes the CPU, AHB and APB buses clocks
        */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        {
            Error_Handler();
        }
    }

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
    void MX_TIM1_Init(void)
    {
        /* USER CODE BEGIN TIM1_Init 0 */

        /* USER CODE END TIM1_Init 0 */

        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        /* USER CODE BEGIN TIM1_Init 1 */

        /* USER CODE END TIM1_Init 1 */
        htim1.Instance = TIM1;
        htim1.Init.Prescaler = 0;
        htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim1.Init.Period = 0x0000FFFF;
        htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim1.Init.RepetitionCounter = 0;
        htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
        {
            Error_Handler();
        }
        /* USER CODE BEGIN TIM1_Init 2 */

        /* USER CODE END TIM1_Init 2 */

    }

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
    void MX_TIM3_Init(void)
    {

        /* USER CODE BEGIN TIM3_Init 0 */

        /* USER CODE END TIM3_Init 0 */

        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        /* USER CODE BEGIN TIM3_Init 1 */

        /* USER CODE END TIM3_Init 1 */
        htim3.Instance = TIM3;
        htim3.Init.Prescaler = 0;
        htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim3.Init.Period = 0x0000FFFF;
        htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
        {
            Error_Handler();
        }
        /* USER CODE BEGIN TIM3_Init 2 */

        /* USER CODE END TIM3_Init 2 */

    }

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
    void MX_TIM4_Init(void)
    {

        /* USER CODE BEGIN TIM4_Init 0 */

        /* USER CODE END TIM4_Init 0 */

        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};


        /* USER CODE BEGIN TIM4_Init 1 */

        /* USER CODE END TIM4_Init 1 */
        htim4.Instance = TIM4;
        htim4.Init.Prescaler = 72;
        htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim4.Init.Period = MotorNs::MotorPwmPeriod;
        htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
        {
            Error_Handler();
        }
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
        {
            Error_Handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
        {
            Error_Handler();
        }
        sConfigOC_TIM4.OCMode = TIM_OCMODE_PWM1;
        sConfigOC_TIM4.Pulse = 0;
        sConfigOC_TIM4.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC_TIM4.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC_TIM4, TIM_CHANNEL_1) != HAL_OK)
        {
            Error_Handler();
        } else {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        }
        if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC_TIM4, TIM_CHANNEL_2) != HAL_OK)
        {
            Error_Handler();
        } else {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
        }
        if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC_TIM4, TIM_CHANNEL_3) != HAL_OK)
        {
            Error_Handler();
        } else {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        }
        if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC_TIM4, TIM_CHANNEL_4) != HAL_OK)
        {
            Error_Handler();
        } else {
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        }
        /* USER CODE BEGIN TIM4_Init 2 */

        /* USER CODE END TIM4_Init 2 */
        HAL_TIM_MspPostInit(&htim4);

    }

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
    void MX_TIM5_Init(void)
    {

        /* USER CODE BEGIN TIM5_Init 0 */

        /* USER CODE END TIM5_Init 0 */

        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        /* USER CODE BEGIN TIM5_Init 1 */

        /* USER CODE END TIM5_Init 1 */
        htim5.Instance = TIM5;
        htim5.Init.Prescaler = 0;
        htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim5.Init.Period = 0x0000FFFF;
        htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC2Filter = 0;
        if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
        {
            Error_Handler();
        }
        /* USER CODE BEGIN TIM5_Init 2 */

        /* USER CODE END TIM5_Init 2 */

    }

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
    void MX_TIM12_Init(void)
    {

        /* USER CODE BEGIN TIM12_Init 0 */

        /* USER CODE END TIM12_Init 0 */

        TIM_ClockConfigTypeDef sClockSourceConfig = {0};

        /* USER CODE BEGIN TIM12_Init 1 */

        /* USER CODE END TIM12_Init 1 */
        htim12.Instance = TIM12;
        htim12.Init.Prescaler = 72;
        htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim12.Init.Period = MotorNs::MotorPwmPeriod;
        htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
        {
            Error_Handler();
        }
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
        {
            Error_Handler();
        }
        sConfigOC_TIM12.OCMode = TIM_OCMODE_PWM1;
        sConfigOC_TIM12.Pulse = 0;
        sConfigOC_TIM12.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC_TIM12.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC_TIM12, TIM_CHANNEL_1) != HAL_OK)
        {
            Error_Handler();
        } else {
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
        }
        if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC_TIM12, TIM_CHANNEL_2) != HAL_OK)
        {
            Error_Handler();
        } else {
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
        }
        /* USER CODE BEGIN TIM12_Init 2 */

        /* USER CODE END TIM12_Init 2 */
        HAL_TIM_MspPostInit(&htim12);

    }

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
    void MX_USART3_UART_Init(void)
    {

        /* USER CODE BEGIN USART3_Init 0 */

        /* USER CODE END USART3_Init 0 */

        /* USER CODE BEGIN USART3_Init 1 */

        /* USER CODE END USART3_Init 1 */
        huart3.Instance = USART3;
        huart3.Init.BaudRate = Comm::Baudrate;
        huart3.Init.WordLength = UART_WORDLENGTH_8B;
        huart3.Init.StopBits = UART_STOPBITS_1;
        huart3.Init.Parity = UART_PARITY_NONE;
        huart3.Init.Mode = UART_MODE_TX_RX;
        huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart3.Init.OverSampling = UART_OVERSAMPLING_16;
        
        if (HAL_UART_Init(&huart3) != HAL_OK)
        {
            Error_Handler();
        }
        /* USER CODE BEGIN USART3_Init 2 */

        /* USER CODE END USART3_Init 2 */

    }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
    void MX_GPIO_Init(void)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        /* GPIO Ports Clock Enable */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        __HAL_RCC_GPIOI_CLK_ENABLE();

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14, GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

        /*Configure GPIO pins : PB8 PB9 PB10 PB14 */
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pins : PH8 PH13 PH14 PH15 */
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

        /*Configure GPIO pins : PG2 PG9 PG10 PG11 */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        /*Configure GPIO pin : PC9 PC11 */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /*Configure GPIO pin : PA11 */
        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*Configure GPIO pins : PD0 PD2 PD3 */
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /*Configure GPIO pin : PD7 */
        GPIO_InitStruct.Pin = GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /*Configure GPIO pins : PI5 PI6 PI7 */
        GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

        /*Configure GPIO pins : PE7 PE8 PE9 */
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /*Configure GPIO pins : PF9 */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    }

};

namespace HW{
    extern HW_Config config;
}
#endif //DOGGY2_LITE_HW_CONFIG_H
