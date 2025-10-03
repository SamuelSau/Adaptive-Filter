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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "dsp/filtering_functions.h"
#include <stdio.h>
#include <string.h>

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
// NLMS parameters
#define BUFFER_SIZE   256
#define BLOCK_SIZE    (BUFFER_SIZE/2)
#define NUM_TAPS      128
#define MU            0.05f

//testing for ADC buffer being filled through DMA
#define BUFFER_SIZE      256                // total DMA buffer size (must be even for half-buffer)
volatile uint16_t adc_buffer[BUFFER_SIZE];   // DMA destination (static/global)
volatile uint8_t adc_half_ready = 0;
volatile uint8_t adc_full_ready = 0;

#define VREF             3.3f
#define ADC_MAX          4095.0f

//initializing test filter
/*
arm_fir_instance_f32 S;
float32_t state[32 + 64];
const float32_t coeffs[32];
arm_fir_init_f32(&S, 32, coeffs, state, 64);
*/

/* CMSIS-DSP NLMS instance */
arm_lms_norm_instance_f32 nlms;

/* Filter state */
static float32_t coeffs[NUM_TAPS];
static float32_t state[NUM_TAPS + BLOCK_SIZE];

/* Processing buffers */
static float32_t proc_ref[BLOCK_SIZE];  // reference noise input
static float32_t proc_des[BLOCK_SIZE];  // desired (speech+noise) input
static float32_t proc_out[BLOCK_SIZE];  // estimated noise output
static float32_t proc_err[BLOCK_SIZE];  // error (clean speech)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void convert_adc_block_to_float(const uint16_t *src, float32_t *dst, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        dst[i] = ((float32_t)src[i] / 4095.0f) * 2.0f - 1.0f; // normalize to [-1,1]
    }
}

static void process_block(uint16_t *src_adc)
{
    /* For now, demo assumes single ADC channel.
       In practice: one channel = desired mic, one = reference mic.
       Replace this with proper channel splitting if using dual ADC or I2S codec. */
    convert_adc_block_to_float(src_adc, proc_des, BLOCK_SIZE);
    convert_adc_block_to_float(src_adc, proc_ref, BLOCK_SIZE);

    /* Run NLMS filtering */
    arm_lms_norm_f32(&nlms, proc_ref, proc_des, proc_out, proc_err, BLOCK_SIZE);

    /* Debug: print first sample of error */
    printf("err[0]=%f\r\n", proc_err[0]);
}
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  printf("HCLK = %lu Hz\r\n", HAL_RCC_GetHCLKFreq());

  /*
  //testing to convert ADC buffer into voltages for DSP processing (filtering)
  float adc_volt[BUFFER_SIZE];
  for (int i = 0; i < BUFFER_SIZE; i++) {
      adc_volt[i] = (adc_buffer[i] / ADC_MAX) * VREF;
      printf("adc_volt[i]: %d\r\n", (uint8_t)adc_volt[i]);
  }
  */  /* Initialize NLMS */
  memset(coeffs, 0, sizeof(coeffs));
  arm_lms_norm_init_f32(&nlms, NUM_TAPS, coeffs, state, MU, BLOCK_SIZE);

  /* Start ADC DMA */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BUFFER_SIZE) != HAL_OK) {
      Error_Handler();
  }

  printf("NLMS initialized: taps=%d, mu=%.3f, block=%d\r\n", NUM_TAPS, MU, BLOCK_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*if (toggling_flag) {
		printf("GPIO_PIN_5 toggled\r\n");
		toggling_flag = 0;
		break;
	}*/
	if (adc_half_ready) {
		adc_half_ready = 0;
		process_block((uint16_t*)&adc_buffer[0]);
	}
	if (adc_full_ready) {
		adc_full_ready = 0;
		process_block((uint16_t*)&adc_buffer[BLOCK_SIZE]);
	}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)   // Check if it's TIM2
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Toggle NUCLEO LED for test
    }
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) adc_half_ready = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) adc_full_ready = 1;
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
