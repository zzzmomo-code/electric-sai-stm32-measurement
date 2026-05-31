/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system.h"
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
  #define BUF_SAMPLES  25U
  #define BUF_CH       8U
  #define BUF_WORDS    (BUF_SAMPLES * BUF_CH)
  #define BUF_BYTES    (BUF_WORDS * 2U)

  static int16_t buf0[BUF_WORDS];
  static int16_t buf1[BUF_WORDS];
  static int16_t *volatile fill_buf = buf0;
  static int16_t *volatile send_buf;
  static volatile uint8_t fill_idx;
  static volatile uint8_t send_ready;
  static volatile uint8_t uart_tx_idle = 1U;
  static volatile uint8_t decim;
  static volatile uint8_t adc_read_active;
  static volatile uint8_t adc_store_sample;
  static volatile uint32_t sample_ok_cnt;
  static volatile uint32_t sample_skip_cnt;
  static volatile uint32_t sample_drop_cnt;
  static volatile uint32_t uart_dma_fail_cnt;
  static volatile uint32_t uart_error_cnt;
  static uint8_t tx_buf[2U + BUF_BYTES];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void APP_ProcessAd7606(void);
static void APP_StoreAd7606Sample(const int16_t raw[BUF_CH]);
static void APP_ProcessUartTx(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  System_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    APP_ProcessUartTx();
    APP_ProcessAd7606();
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void APP_ProcessAd7606(void)
{
    int16_t raw[BUF_CH];
    uint8_t store_sample;
    uint8_t ret;

    AD7606_BusyPoll();

    if (ad7606_complete_flag == 0U) {
        return;
    }

    __disable_irq();
    if ((ad7606_complete_flag != 0U) &&
        (ad7606_conv_busy_flag == 0U) &&
        (adc_read_active == 0U)) {
        adc_read_active = 1U;
        store_sample = adc_store_sample;
    } else {
        __enable_irq();
        return;
    }
    __enable_irq();

    ret = AD7606_ReadData(raw);

    __disable_irq();
    adc_read_active = 0U;
    __enable_irq();

    if (ret == AD7606_OK) {
        sample_ok_cnt++;
        if (store_sample != 0U) {
            APP_StoreAd7606Sample(raw);
        }
    } else {
        sample_drop_cnt++;
    }
}

static void APP_StoreAd7606Sample(const int16_t raw[BUF_CH])
{
    uint8_t idx = fill_idx;

    if (idx < BUF_SAMPLES) {
        for (uint8_t ch = 0U; ch < BUF_CH; ch++) {
            fill_buf[(uint16_t)idx * BUF_CH + ch] = raw[ch];
        }
        fill_idx = idx + 1U;
    }

    if (fill_idx >= BUF_SAMPLES) {
        if (send_ready == 0U) {
            send_buf = fill_buf;
            fill_buf = (fill_buf == buf0) ? buf1 : buf0;
            fill_idx = 0U;
            send_ready = 1U;
        } else {
            sample_drop_cnt += BUF_SAMPLES;
            fill_idx = 0U;
        }
    }
}

static void APP_ProcessUartTx(void)
{
    HAL_StatusTypeDef status;

    if ((send_ready == 0U) || (uart_tx_idle == 0U)) {
        return;
    }

    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    memcpy(tx_buf + 2, (uint8_t *)send_buf, BUF_BYTES);

    status = HAL_UART_Transmit_DMA(&huart3, tx_buf, sizeof(tx_buf));
    if (status == HAL_OK) {
        uart_tx_idle = 0U;
        send_ready = 0U;
    } else {
        uart_dma_fail_cnt++;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        decim++;
        if ((ad7606_conv_busy_flag == 0U) &&
            (ad7606_complete_flag == 0U) &&
            (adc_read_active == 0U)) {
            adc_store_sample = decim & 1U;
            AD7606_ConvStart();
        } else {
            sample_skip_cnt++;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_3) return;

    AD7606_Busy_IRQHandler();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart_tx_idle = 1U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart_error_cnt++;
        uart_tx_idle = 1U;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);
  Usart_Send_Computer(&huart3, "error");
  while (1)
  {
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
