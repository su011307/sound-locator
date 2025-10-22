/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.hpp"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.hpp"
#include "dma.hpp"
#include "tim.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "motor.hpp"
#include "led.hpp"
#include "OLED.h"
#include "OLED_Font.h"
#include "mic.hpp"
#include "tdoa.hpp"
#include <cmath>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
constexpr std::array<Point, 4> MIC_LOCATIONS =
{
    {
        {4.5f, 4.5f},
        {4.5f, -4.5f},
        {-4.5f, 4.5f},
        {-4.5f, -4.5f}
    }
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MicrophoneMatrix *g_mic_matrix = nullptr;
Motor *g_motor = nullptr;
LEDMatrix *g_led_matrix = nullptr;
// OLED *g_oled = nullptr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void GlobalClock_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// 返回全局时钟的计数。不处理溢出，直接回绕
static inline uint32_t micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

/* USER CODE END 0 */

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
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    /* USER CODE BEGIN 2 */
    GlobalClock_Init();
    g_mic_matrix = new MicrophoneMatrix();

    static auto g_enc_gpio_1 = new GPIO(GPIOB, GPIO_PIN_0);
    static auto g_enc_gpio_2 = new GPIO(GPIOB, GPIO_PIN_1);

    static auto g_led_gpio_1 = new GPIO(GPIOA, GPIO_PIN_8);
    static auto g_led_gpio_2 = new GPIO(GPIOA, GPIO_PIN_9);
    static auto g_led_gpio_3 = new GPIO(GPIOA, GPIO_PIN_10);

    static I2CPort *g_i2cport = new I2CPort(&hi2c1);

    static auto g_encoder = new Encoder(&htim3);
    static auto g_pid_param = new PIDParams{
        15.0, 1.0, 1.0, 0.0, 0.0
    };

    g_motor = new Motor(&htim2, TIM_CHANNEL_2, g_enc_gpio_1, g_enc_gpio_2, g_encoder, g_pid_param);
    g_led_matrix = new LEDMatrix(g_led_gpio_1, g_led_gpio_2, g_led_gpio_3);

    HAL_Delay(100);
    OLED_Init();
    OLED_Clear();
    char str[] = "Hello";
    OLED_ShowString(1, 1, str);

    HAL_ADC_Start_DMA(&hadc1, g_mic_matrix->dma_ptr(), 4);
    // 在这里启动几个时钟
    HAL_TIM_Base_Start_IT(&htim9);  // 定期读取编码器度数
    HAL_TIM_Base_Start_IT(&htim10);  // 定期触发ADC
    HAL_TIM_Base_Start_IT(&htim11);  // 定期调整电机

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */
        HAL_Delay(1);
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM10) {
        // 软件触发ADC扫描，因为低贱的TIM10不支持直接硬件启动ADC
        // 只能在中断这里曲线救国了
        ADC1->CR2 |= ADC_CR2_SWSTART;
    } else if (htim->Instance == TIM9) {
        // 突然发现TIM9是没有必要的，直接在TIM11更新poll就好了
        // 然后换用了uint16_t，应该是能正确处理回绕问题的
    } else if (htim->Instance == TIM11) {
        // 更新PID响应
        g_motor->poll();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        HAL_TIM_Base_Stop_IT(&htim10);
        if (g_mic_matrix) {
            g_mic_matrix->update(micros());
            if (g_mic_matrix->is_ok()) {
                auto ts = g_mic_matrix->get_timestamps();
                auto loc = compute_position(MIC_LOCATIONS, ts);
                if (loc.has_value()) {
                    auto f = loc.value().first, s = loc.value().second;
                    auto d = sqrtf(f * f + s * s);
                    auto str = "dist.: " + std::to_string(d); // 生成需要显示的字符串
                    // g_oled->write(str);

                    // 计算角度
                    auto angle = atan2(loc.value().second, loc.value().first) * 57.29578; // 换算成度
                    g_motor->move_to(angle);
                } else {
                    // g_oled->write("?");
                }
            }
        }
        if (g_mic_matrix != nullptr) {
            HAL_ADC_Start_DMA(&hadc1, g_mic_matrix->dma_ptr(), 4);
        }
        HAL_TIM_Base_Start_IT(&htim10);
    }
}

void HAL_EXTI_GPIO_IRQHandler(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        HAL_Delay(20);
        g_mic_matrix->switch_mode();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
