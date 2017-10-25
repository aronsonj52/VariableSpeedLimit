/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "cJSON.c"
#include <time.h>

// Radar
#define NUM_RADAR_PINS 11
#define NUM_RADAR_DATA_PINS 7
#define NUM_RADAR_SEL_PINS 4
#define RADAR_THRESH_HIGH 0xF0
#define RADAR_THRESH_LOW 0x10

// Speed
#define MAX_DISTRIBUTION_SIZE 100
#define MAX_DISTRIBUTION_AGE_MIN 30
#define SPEED_PERCENTILE .85 // 85th percentile

// ADC
#define ADC_CHANNELS 12

// LED
#define LED_LOGIC_0_HIGH_COUNT 17
#define LED_LOGIC_0_LOW_COUNT 38
#define LED_LOGIC_1_HIGH_COUNT 33
#define LED_LOGIC_1_LOW_COUNT 29
#define NUM_LEDS 150
#define BITS_PER_BYTE 8
#define BYTES_PER_WORD 3

// Battery
#define BATTERY_THRESH (.75 * 0xFF)

// Timer interrupt constants
#define RADAR_INTR_COUNT_MS 500 // ms
#define RADAR_TRIGGER_COUNT_MS 7 // ms
#define WEATHER_INTR_COUNT_MIN 15 // min
#define DISPLAY_SPEED_INTR_COUNT_MIN 15 // min
#define DISPLAY_CHECK_INTR_COUNT_S 5 // sec
#define BATTERY_INTR_COUNT_MIN 5 // min

// Time constants
#define MS_ROLLOVER 1000
#define SEC_ROLLOVER 60
#define MIN_ROLLOVER 60
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t prev_pb = RESET;

// Time counts
uint16_t ms = 0, target_ms;
uint8_t sec = 0, min = 0;

// Timer interrupt counts
uint16_t radar_intr_count = RADAR_INTR_COUNT_MS;
uint16_t radar_trigger_count = RADAR_TRIGGER_COUNT_MS;
uint8_t weather_intr_count = WEATHER_INTR_COUNT_MIN;
uint8_t display_speed_intr_count = DISPLAY_SPEED_INTR_COUNT_MIN;
uint8_t display_check_intr_count = DISPLAY_CHECK_INTR_COUNT_S;
uint8_t battery_intr_count = BATTERY_INTR_COUNT_MIN;

// Radar
typedef struct S_T {
	time_t timestamp;
	uint32_t speed;
} Speed_Time;

uint8_t radar_flag = 0;
uint8_t radar_data [NUM_RADAR_SEL_PINS];
Speed_Time speed_distribution [MAX_DISTRIBUTION_SIZE];
Speed_Time speed_distribution_sorted [MAX_DISTRIBUTION_SIZE];
Speed_Time speed_time;
uint8_t distribution_index = 0;
uint32_t variable_speed, display_speed;

// ADC
uint32_t adc_data [(ADC_CHANNELS + 1) / 2];
uint8_t adc_channels [ADC_CHANNELS];
uint8_t adc_conv_cplt_flag = 0;

// Weather
//cJSON weather_data;

// LED
uint8_t led_data [NUM_LEDS][BYTES_PER_WORD]; // LED data to shift out
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void radarMain(void);
void triggerRadar(void);
void readRadar(void);
void storeSpeedTime(void);
uint32_t getRadarValue(void);
uint32_t segmentToInt(uint8_t seg);
void sortDistributionTime(void);
void cleanDistribution(void);
void removeSpeedTime(uint8_t index);
uint32_t getSpeedPercentile(void);
void sortDistributionSpeed(void);

void weatherMain(void);
void pingWeatherAPI(void);

void displaySpeedMain(void);
void checkWeather(void);
void checkSpeedPrediction(void);
void roundVariableSpeed(void);
void displaySpeedLimit(uint32_t speed);
void sendLEDData(void);
void sendLEDWord(uint8_t * word);
void sendLEDByte(uint8_t byte);
void sendLEDBit(uint8_t bit);

void displayCheckMain(void);

void batteryMain(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // initialize LED_Out_Pin to not low
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  // radar read test
	  /*if (radar_flag == 0x0F) {
		  speed_distribution[distribution_index] = getRadarValue();
	  }
	  */

	  // LED output test
	  led_data[0][0] = 0x00;
	  led_data[0][1] = 0x00;
	  led_data[0][2] = 0x00;
	  sendLEDWord(led_data[0]);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Radar_Trig_Pin|LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Cell_Power_GPIO_Port, Cell_Power_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Radar_Trig_Pin LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = Radar_Trig_Pin|LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Cell_Power_Pin */
  GPIO_InitStruct.Pin = Cell_Power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Cell_Power_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Out_Pin */
  GPIO_InitStruct.Pin = LED_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Out_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// TIM1 interrupt @ 1 Hz / every 1 s
	if (htim->Instance == TIM1) {
		sec++;
		if (sec == SEC_ROLLOVER) {
			sec = 0;
			min = (min + 1) % MIN_ROLLOVER;
		}

		if (min == weather_intr_count) {
			//weatherMain();
			weather_intr_count = (weather_intr_count + WEATHER_INTR_COUNT_MIN) % MIN_ROLLOVER;
		}
		if (min == display_speed_intr_count) {
			//displaySpeedMain();
			display_speed_intr_count = (display_speed_intr_count + DISPLAY_SPEED_INTR_COUNT_MIN) % MIN_ROLLOVER;
		}
		if (sec == display_check_intr_count) {
			//displayCheckMain();
			display_check_intr_count = (display_check_intr_count + DISPLAY_CHECK_INTR_COUNT_S) % SEC_ROLLOVER;
		}
		if (min == battery_intr_count) {
			//batteryMain();
			battery_intr_count = (battery_intr_count + BATTERY_INTR_COUNT_MIN) % MIN_ROLLOVER;
		}
	}

	// TIM3 interrupt @ 1 kHz / every 1 ms
	if (htim->Instance == TIM3) {
		ms = (ms + 1) % MS_ROLLOVER;

		// push button
		uint8_t pb = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if (prev_pb == RESET && pb == SET) {
			radar_flag = 0;
			distribution_index = (distribution_index + 1) % MAX_DISTRIBUTION_SIZE;
		}
		prev_pb = pb;

		// radar gun
		//HAL_ADC_Start_DMA(&hadc, adc_data, (uint32_t)ADC_CHANNELS);
		if (ms == radar_intr_count) {
			//radarMain();
			radar_intr_count = (radar_intr_count + RADAR_INTR_COUNT_MS) % MIN_ROLLOVER;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	uint8_t i;
	for (i = 0; i < ADC_CHANNELS / 2; i++) {
		// store ADC data in buffer
		adc_channels[2 * i] = (uint8_t)(adc_data[i] & 0xFF);
		adc_channels[2 * i + 1] = (uint8_t)(adc_data[i] >> 16);
	}

	uint8_t j;
	for (i = 0; i < NUM_RADAR_SEL_PINS; i++) {
		if (adc_channels[i + NUM_RADAR_DATA_PINS] > RADAR_THRESH_HIGH) {
			radar_data[i] = 0;
			for (j = 0; j < NUM_RADAR_DATA_PINS; j++) {
				radar_data[i] <<= 1;
				radar_data[i] |= (adc_channels[j] < RADAR_THRESH_LOW);
			}
			radar_flag |= (0x01 << i);
		}
	}

	adc_conv_cplt_flag = 1;
}

///////////////////////////////////////////////////////////////////////////////
// RADAR																	 //
///////////////////////////////////////////////////////////////////////////////

void radarMain(void) {
	triggerRadar();
	readRadar();
	storeSpeedTime();
	sortDistributionTime();
	cleanDistribution();
	variable_speed = getSpeedPercentile();
}

void triggerRadar(void) {
	radar_trigger_count = (ms + RADAR_TRIGGER_COUNT_MS) % MS_ROLLOVER; // set ms count
	HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_SET); // set radar trigger high
	while (ms != radar_trigger_count); // wait for counter
	HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_RESET); // set radar trigger low
}

void readRadar(void) {
	radar_flag = 0;
	adc_conv_cplt_flag = 1;

	while (radar_flag != 0x0F) {
		HAL_ADC_Start_DMA(&hadc, adc_data, (uint32_t)ADC_CHANNELS);
		while (adc_conv_cplt_flag == 0); // wait for ADC conversion to complete
	}
}

void storeSpeedTime(void) {
	if (distribution_index == MAX_DISTRIBUTION_SIZE) {
		distribution_index--;
	}

	speed_distribution[distribution_index].speed = getRadarValue();
	speed_distribution[distribution_index].timestamp = time(NULL);
	distribution_index++;
}

uint32_t getRadarValue(void) {
	uint8_t hundreds = 0;
	hundreds |= ((radar_data[3] & (0x01 << (6 - 1))) > 0) ? (0x01 << 6) : 0;
	hundreds |= ((radar_data[3] & (0x01 << (6 - 0))) > 0) ? (0x01 << 5) : 0;
	hundreds |= ((radar_data[2] & (0x01 << (6 - 1))) > 0) ? (0x01 << 4) : 0;
	hundreds |= ((radar_data[2] & (0x01 << (6 - 0))) > 0) ? (0x01 << 3) : 0;
	hundreds |= ((radar_data[1] & (0x01 << (6 - 0))) > 0) ? (0x01 << 2) : 0;
	hundreds |= ((radar_data[1] & (0x01 << (6 - 1))) > 0) ? (0x01 << 1) : 0;
	hundreds |= ((radar_data[0] & (0x01 << (6 - 1))) > 0) ? 0x01 : 0;

	uint8_t tens = 0;
	tens |= ((radar_data[3] & (0x01 << (6 - 3))) > 0) ? (0x01 << 6) : 0;
	tens |= ((radar_data[3] & (0x01 << (6 - 2))) > 0) ? (0x01 << 5) : 0;
	tens |= ((radar_data[2] & (0x01 << (6 - 3))) > 0) ? (0x01 << 4) : 0;
	tens |= ((radar_data[2] & (0x01 << (6 - 2))) > 0) ? (0x01 << 3) : 0;
	tens |= ((radar_data[1] & (0x01 << (6 - 2))) > 0) ? (0x01 << 2) : 0;
	tens |= ((radar_data[1] & (0x01 << (6 - 3))) > 0) ? (0x01 << 1) : 0;
	tens |= ((radar_data[0] & (0x01 << (6 - 3))) > 0) ? 0x01 : 0;

	uint8_t ones = 0;
	ones |= ((radar_data[3] & (0x01 << (6 - 5))) > 0) ? (0x01 << 6) : 0;
	ones |= ((radar_data[3] & (0x01 << (6 - 4))) > 0) ? (0x01 << 5) : 0;
	ones |= ((radar_data[2] & (0x01 << (6 - 5))) > 0) ? (0x01 << 4) : 0;
	ones |= ((radar_data[2] & (0x01 << (6 - 4))) > 0) ? (0x01 << 3) : 0;
	ones |= ((radar_data[1] & (0x01 << (6 - 4))) > 0) ? (0x01 << 2) : 0;
	ones |= ((radar_data[1] & (0x01 << (6 - 5))) > 0) ? (0x01 << 1) : 0;
	ones |= ((radar_data[0] & (0x01 << (6 - 5))) > 0) ? 0x01 : 0;

	return (uint32_t)(100 * segmentToInt(hundreds) + 10 * segmentToInt(tens) + segmentToInt(ones));
}

uint32_t segmentToInt(uint8_t seg) {
	switch (seg) {
		case 0x77: return 0;
		case 0x12: return 1;
		case 0x5D: return 2;
		case 0x5B: return 3;
		case 0x3A: return 4;
		case 0x6B: return 5;
		case 0x6F: return 6;
		case 0x52: return 7;
		case 0x7F: return 8;
		case 0x7B: return 9;
		case 0x00: return 0;
		default:   return -1;
	}
}

void sortDistributionTime(void) {
	uint8_t i, j, max_time_index;

	// selection sort
	for (i = 0; i < distribution_index - 1; i++) {
		max_time_index = i;

		for (j = i + 1; j < distribution_index; j++) {
			if (speed_distribution[j].timestamp > speed_distribution[max_time_index].timestamp) {
				max_time_index = j;
			}
		}

		// swap
		if (max_time_index != i) {
			speed_time = speed_distribution[i];
			speed_distribution[i] = speed_distribution[max_time_index];
			speed_distribution[max_time_index] = speed_time;
		}
	}
}

void cleanDistribution(void) {
	time_t now = time(NULL);
	uint8_t i = 0;
	while (i < distribution_index) {
		// if Speed_Time is aged, remove from distribution; else, continue
		if (now - speed_distribution[i].timestamp > (MAX_DISTRIBUTION_AGE_MIN * 60)) {
			removeSpeedTime(i);
		} else {
			i++;
		}
	}
}

void removeSpeedTime(uint8_t index) {
	// shift down all Speed_Time's after index
	uint8_t i;
	for (i = index; i < distribution_index; i++) {
		speed_distribution[i] = speed_distribution[i + 1];
	}

	// adjust distribution index/size
	distribution_index--;
}

uint32_t getSpeedPercentile(void) {
	sortDistributionSpeed();
	return speed_distribution_sorted[(int)(SPEED_PERCENTILE * distribution_index)].speed;
}

void sortDistributionSpeed(void) {
	// copy to speed_distribution_sorted
	uint8_t i;
	for (i = 0; i < distribution_index; i++) {
		speed_distribution_sorted[i] = speed_distribution[i];
	}

	uint8_t j, min_speed_index;

	// selection sort
	for (i = 0; i < distribution_index - 1; i++) {
		min_speed_index = i;

		for (j = i + 1; j < distribution_index; j++) {
			if (speed_distribution_sorted[j].speed < speed_distribution_sorted[min_speed_index].speed) {
				min_speed_index = j;
			}
		}

		// swap
		if (min_speed_index != i) {
			speed_time = speed_distribution_sorted[i];
			speed_distribution_sorted[i] = speed_distribution_sorted[min_speed_index];
			speed_distribution_sorted[min_speed_index] = speed_time;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Weather																	 //
///////////////////////////////////////////////////////////////////////////////

void weatherMain(void) {
	pingWeatherAPI();
}

void pingWeatherAPI(void) {
	//TODO: configure AT commands
	//TODO: send AT commands to 4G module
	//TODO: receive data back from 4G module
}

///////////////////////////////////////////////////////////////////////////////
// Display Speed															 //
///////////////////////////////////////////////////////////////////////////////

void displaySpeedMain(void) {
	checkWeather();
	checkSpeedPrediction();
	roundVariableSpeed();
	if (variable_speed < display_speed) {
		display_speed = variable_speed;
		displaySpeedLimit(display_speed);
	}
}

void checkWeather(void) {
	//TODO: adjust speed based on weather data
}

void checkSpeedPrediction(void) {
	//TODO: adjust speed based on speed prediction table
}

void roundVariableSpeed(void) {
	uint8_t ones = variable_speed % 10;
	if (ones <= 2) { // x0-x2 -> x0
		variable_speed = (int)(variable_speed / 10) * 10;
	} else if (ones <= 7) { // x3-x7 -> x5
		variable_speed = (int)(variable_speed / 10) * 10 + 5;
	} else { // x8-x9 -> (x+1)0
		variable_speed = (int)(variable_speed / 10) * 10 + 10;
	}
}

void displaySpeedLimit(uint32_t speed) {
	//TODO: send speed to LEDs
	//uint8_t tens = speed / 10;
	//uint8_t ones = speed % 10;
	//TODO: translate tens and ones to LED segment addresses using led_data
}

void sendLEDData(void) {
	uint8_t i;
	for (i = 0; i < NUM_LEDS; i++) {
		sendLEDWord(led_data[i]);
	}
}

void sendLEDWord(uint8_t * word) {
	uint16_t i;
	/*
	for (i = 0; i < BYTES_PER_WORD; i++) {
		sendLEDByte(word[i]);
	}
	*/

	uint8_t j, k, delay_count;
	for (k = 0; k < BYTES_PER_WORD; k++) {
		for (j = 0; j < BITS_PER_BYTE; j++) {
			HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET); // set GPIO pin not high
			delay_count = (word[k] & (0x01 << j)) == 0 ? 5 : 8;
			for (i = 0; i < delay_count; i++);
			HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // set GPIO pin not low
		}
	}

	// delay 50 us
	for (i = 0; i < 350; i++);
}

void sendLEDByte(uint8_t byte) {
	/*
	uint8_t i;
	for (i = 0; i < BITS_PER_BYTE; i++) {
		sendLEDBit((uint8_t)(byte & (0x01 << i)));
	}
	sendLEDBit((uint8_t)(byte & (0x01)));
	sendLEDBit((uint8_t)(byte & (0x02)));
	sendLEDBit((uint8_t)(byte & (0x04)));
	sendLEDBit((uint8_t)(byte & (0x08)));
	sendLEDBit((uint8_t)(byte & (0x10)));
	sendLEDBit((uint8_t)(byte & (0x20)));
	sendLEDBit((uint8_t)(byte & (0x40)));
	sendLEDBit((uint8_t)(byte & (0x80)));
	*/
	uint8_t i, j, delay_count;
	for (j = 0; j < BITS_PER_BYTE; j++) {
		HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET); // set GPIO pin not high
		delay_count = (byte & (0x01 << j)) == 0 ? 5 : 8;
		for (i = 0; i < delay_count; i++);
		HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // set GPIO pin not low
	}
}

void sendLEDBit(uint8_t bit) {
	//htim2.Instance->CNT = 0;
	uint8_t i = 0;
	bit = 0;
	if (bit == 0) {
		HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET); // set GPIO pin not high

		//HAL_TIM_Base_Start(&htim2);
		//while (htim2.Instance->CNT < LED_LOGIC_0_HIGH_COUNT);
		for (i = 0; i < 6; i++);

		HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // set GPIO pin not low

		//htim2.Instance->CNT = 0;
		//while (htim2.Instance->CNT < LED_LOGIC_0_LOW_COUNT);
		//for (i = 0; i < 8; i++);
	} else {
		HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET); // set GPIO pin not high

		//HAL_TIM_Base_Start(&htim2);
		//while (htim2.Instance->CNT < LED_LOGIC_1_HIGH_COUNT);
		for (i = 0; i < 9; i++);

		HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // set GPIO pin not low

		//htim2.Instance->CNT = 0;
		//while (htim2.Instance->CNT < LED_LOGIC_1_LOW_COUNT);
		//for (i = 0; i < 1; i++);
	}

	//HAL_TIM_Base_Stop(&htim2);
	//htim2.Instance->CNT = 0;
}

///////////////////////////////////////////////////////////////////////////////
// Display Check															 //
///////////////////////////////////////////////////////////////////////////////

void displayCheckMain(void) {
	//TODO: turn on/off display according to latest Speed_Time.timestamp
}

///////////////////////////////////////////////////////////////////////////////
// Battery / State of Charge												 //
///////////////////////////////////////////////////////////////////////////////

void batteryMain(void) {
	//TODO: check battery level against threshold, power save mode if necessary
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
