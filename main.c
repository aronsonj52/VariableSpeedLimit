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

// Time constants
#define MS_ROLLOVER 1000
#define SEC_ROLLOVER 60
#define MIN_ROLLOVER 60
#define HOUR_ROLLOVER 24

// Radar
#define NUM_RADAR_PINS 11
#define NUM_RADAR_DATA_PINS 7
#define NUM_RADAR_SEL_PINS 4
#define NUM_RADAR_READ_PASSES 20
#define RADAR_THRESH_HIGH 0xF0
#define RADAR_THRESH_LOW 0x10
#define RADAR_TRIGGER_MASK 0x02

// Speed
#define MAX_SPEED 999
#define MAX_DISTRIBUTION_SIZE 10
#define MAX_DISTRIBUTION_AGE_MIN 30
#define SPEED_PERCENTILE .85 // 85th percentile

// ADC
#define ADC_CHANNELS 12
#define ADC_CONV_CPLT_MASK 0x01

// LED
#define NUM_LEDS 150
#define GREEN 0
#define RED 1
#define BLUE 2
#define BITS_PER_BYTE 8
#define BYTES_PER_WORD 3

// Battery
#define BATTERY_THRESH (.75 * 0xFF) // 75% of 12 V operating voltage
#define BATTERY_ADC_CHANNEL (ADC_CHANNELS - 1)

// Speed predict table
#define MAX_PREDICT_SIZE 10
#define DAYS_PER_WEEK 7
#define TIMES_PER_DAY (HOUR_ROLLOVER * 2) // 30 min intervals

// Timer interrupt constants and masks
#define RADAR_INTR_COUNT_MS 500 // ms
#define RADAR_TRIGGER_COUNT_MS 999 // <-- radar trigger test val; actual val --> 7 // ms
#define WEATHER_INTR_COUNT_MIN 15 // min
#define DISPLAY_SPEED_INTR_COUNT_MIN 15 // min
#define DISPLAY_CHECK_INTR_COUNT_S 15 // sec
#define BATTERY_INTR_COUNT_MIN 5 // min

#define RADAR_INTR_MASK 0x01
#define WEATHER_INTR_MASK 0x02
#define DISPLAY_SPEED_INTR_MASK 0x04
#define DISPLAY_CHECK_INTR_MASK 0x08
#define BATTERY_INTR_MASK 0x10
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
uint8_t pb_flag = 0;

// Time counts
uint16_t ms = 0;
uint8_t sec = 0, min = 0, hour = 0, day = 0;

// Timer interrupt counts and flags
uint16_t radar_intr_count = RADAR_INTR_COUNT_MS;
uint16_t radar_trigger_count = RADAR_TRIGGER_COUNT_MS;
uint8_t weather_intr_count = WEATHER_INTR_COUNT_MIN;
uint8_t display_speed_intr_count = DISPLAY_SPEED_INTR_COUNT_MIN;
uint8_t display_check_intr_count = DISPLAY_CHECK_INTR_COUNT_S;
uint8_t battery_intr_count = BATTERY_INTR_COUNT_MIN;

uint8_t intr_flag = 0;

// Radar
uint8_t radar_flag = 0;
uint8_t radar_data [NUM_RADAR_SEL_PINS];
uint8_t radar_read [NUM_RADAR_SEL_PINS];

typedef struct S_T {
	uint8_t time_hour;
	uint8_t time_min;
	uint16_t speed;
} Speed_Time;
Speed_Time speed_distribution [MAX_DISTRIBUTION_SIZE];
Speed_Time speed_time;

uint8_t distribution_index = 0;
uint16_t variable_speed, display_speed;

// ADC
uint32_t adc_data [(ADC_CHANNELS + 1) / 2];
uint8_t adc_channels [ADC_CHANNELS];

// Weather
//cJSON weather_data;

// LED
uint8_t led_data [NUM_LEDS][BYTES_PER_WORD]; // LED data to shift out
// GREEN : led_data[i][0]
// RED   : led_data[i][1]
// BLUE  : led_data[i][2]

// Speed prediction table
typedef struct Predict_Arr {
	uint32_t speed_arr [MAX_PREDICT_SIZE];
	uint8_t index;
} Predict_Element;
//Predict_Element predict_table [DAYS_PER_WEEK][TIMES_PER_DAY];

// Battery
typedef enum VSL_State {
	VARIABLE = 0,
	STATIC = 1,
	DISPLAY_OFF = 2,
	LOW_POWER = 3
} State;
State state = VARIABLE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// Radar
void radarMain(void);
void triggerRadar(void);
void readRadar(void);
void storeSpeedTime(void);
uint16_t getRadarValue(void);
uint32_t segmentToInt(uint8_t seg);
void sortDistributionTime(void);
void cleanDistribution(void);
void removeSpeedTime(uint8_t index);
uint16_t getSpeedPercentile(void);
void sortDistributionSpeed(void);

// Weather
void weatherMain(void);
void pingWeatherAPI(void);

// Display speed
void displaySpeedMain(void);
void checkWeather(void);
void checkSpeedPrediction(void);
uint16_t getPredictSpeed(void);
uint8_t getTableIndex(void);
uint16_t getAvgSpeed(Predict_Element p_e);
void roundVariableSpeed(void);
void storeSpeedInTable(void);
void displaySpeedLimit(uint32_t speed);
void clearLEDData(void);
void sendLEDData(void);
void sendLEDWord(uint8_t * word);

// Display check
void displayCheckMain(void);

// Battery
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
  // start TIM1 and TIM3 interrupts
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // initialize LED_Out_Pin to not low for IDLE
  HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_SET); // set radar trigger high for IDLE
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  uint16_t mode = 0;

  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//	  uint16_t i, j;

	  // LED output test
/*
	  clearLEDData();

	  // grow and shrink rainbow
	  for (i = 0; i < NUM_LEDS; i++) {
		  if (mode < NUM_LEDS && i < mode) {
			  if (i < 50) {
				  led_data[i][RED] = (uint8_t)((50 - i) * 1.0 / 50.0 * 0xFF); // decrease RED
				  led_data[i][GREEN] = (uint8_t)((i) * 1.0 / 50.0 * 0xFF); // increase GREEN
			  } else if (i < 100) {
				  led_data[i][GREEN] = (uint8_t)((100 - i) * 1.0 / 50.0 * 0xFF); // decrease GREEN
				  led_data[i][BLUE] = (uint8_t)((i - 50) * 1.0 / 50.0 * 0xFF); // increase BLUE
			  } else if (i < 150) {
				  led_data[i][BLUE] = (uint8_t)((150 - i) * 1.0 / 50.0 * 0xFF); // decrease BLUE
				  led_data[i][RED] = (uint8_t)((i - 100) * 1.0 / 50.0 * 0xFF); // increase RED
			  }
		  } else if (mode >= NUM_LEDS && (NUM_LEDS - i - 1) >= (mode - NUM_LEDS)) { // 149->0 >= 0->149
			  if (i < 50) {
				  led_data[i][RED] = (uint8_t)((50 - i) * 1.0 / 50.0 * 0xFF); // decrease RED
				  led_data[i][GREEN] = (uint8_t)((i) * 1.0 / 50.0 * 0xFF); // increase GREEN
			  } else if (i < 100) {
				  led_data[i][GREEN] = (uint8_t)((100 - i) * 1.0 / 50.0 * 0xFF); // decrease GREEN
				  led_data[i][BLUE] = (uint8_t)((i - 50) * 1.0 / 50.0 * 0xFF); // increase BLUE
			  } else if (i < 150) {
				  led_data[i][BLUE] = (uint8_t)((150 - i) * 1.0 / 50.0 * 0xFF); // decrease BLUE
				  led_data[i][RED] = (uint8_t)((i - 100) * 1.0 / 50.0 * 0xFF); // increase RED
			  }
		  }
	  }

  	  // 1 every 10 LEDs rainbow
	  for (j = 0; j < NUM_LEDS; j += 10) {
		  uint8_t k = (mode + j) % NUM_LEDS;
		  if (k < 50) {
			  led_data[k][RED] = (uint8_t)((50 - k) * 1.0 / 50.0 * 0xFF); // decrease RED
			  led_data[k][GREEN] = (uint8_t)((k) * 1.0 / 50.0 * 0xFF); // increase GREEN
		  } else if (k < 100) {
			  led_data[k][GREEN] = (uint8_t)((100 - k) * 1.0 / 50.0 * 0xFF); // decrease GREEN
			  led_data[k][BLUE] = (uint8_t)((k - 50) * 1.0 / 50.0 * 0xFF); // increase BLUE
		  } else if (k < 150) {
			  led_data[k][BLUE] = (uint8_t)((150 - k) * 1.0 / 50.0 * 0xFF); // decrease BLUE
			  led_data[k][RED] = (uint8_t)((k - 100) * 1.0 / 50.0 * 0xFF); // increase RED
		  }
	  }


	  sendLEDData();

	  // latch high and wait
	  HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET); // set GPIO pin not high
	  for (i = 0; i < 1; i++) {
		  for (j = 0; j < 10000; j++);
	  }

	  mode = (mode + 1) % (NUM_LEDS * 2);
*/

	  // Radar Trigger Test

	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	  while (pb_flag == 0); // wait for user to push button
	  pb_flag = 0;
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	  radar_flag = 0;
	  triggerRadar();
	  readRadar();

	  // 111
//	  radar_data[0] = 0x00;
//	  radar_data[1] = 0x2A;
//	  radar_data[2] = 0x2A;
//	  radar_data[3] = 0x00;

	  // 1
//	  radar_data[0] = 0x00;
//	  radar_data[1] = 0x02;
//	  radar_data[2] = 0x02;
//	  radar_data[3] = 0x00;

	  // 19
//	  radar_data[0] = 0x02;
//	  radar_data[1] = 0x0A;
//	  radar_data[2] = 0x0E;
//	  radar_data[3] = 0x06;

	  storeSpeedTime();

	  // Normal Operation
/*
	  if (intr_flag & RADAR_INTR_MASK) {
		  intr_flag &= ~RADAR_INTR_MASK; // clear bit
		  //radarMain();
	  }
	  if (intr_flag & WEATHER_INTR_MASK) {
		  intr_flag &= ~WEATHER_INTR_MASK; // clear bit
		  //weatherMain();
	  }
	  if (intr_flag & DISPLAY_SPEED_INTR_MASK) {
		  intr_flag &= ~DISPLAY_SPEED_INTR_MASK; // clear bit
		  //displaySpeedMain();
	  }
	  if (intr_flag & DISPLAY_CHECK_INTR_MASK) {
		  intr_flag &= ~DISPLAY_CHECK_INTR_MASK; // clear bit
		  //displayCheckMain();
	  }
	  if (intr_flag & BATTERY_INTR_MASK) {
		  intr_flag &= ~BATTERY_INTR_MASK; // clear bit
		  //batteryMain();
	  }
*/
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
  htim2.Init.Period = 50;
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_Out_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// TIM1 interrupt @ 1 Hz / every 1 s
	if (htim->Instance == TIM1) {
		sec++;
		if (sec == SEC_ROLLOVER) {
			sec = 0;
			min++;

			if (min == MIN_ROLLOVER) {
				min = 0;
				hour++;

				if (hour == HOUR_ROLLOVER) {
					hour = 0;
					day = (day + 1) % DAYS_PER_WEEK;
				}
			}
		}

		if (min == weather_intr_count) {
			//weatherMain();
			intr_flag |= WEATHER_INTR_MASK;
			weather_intr_count = (weather_intr_count + WEATHER_INTR_COUNT_MIN) % MIN_ROLLOVER;
		}
		if (min == display_speed_intr_count) {
			//displaySpeedMain();
			intr_flag |= DISPLAY_SPEED_INTR_MASK;
			display_speed_intr_count = (display_speed_intr_count + DISPLAY_SPEED_INTR_COUNT_MIN) % MIN_ROLLOVER;
		}
		if (sec == display_check_intr_count) {
			//displayCheckMain();
			intr_flag |= DISPLAY_CHECK_INTR_MASK;
			display_check_intr_count = (display_check_intr_count + DISPLAY_CHECK_INTR_COUNT_S) % SEC_ROLLOVER;
		}
		if (min == battery_intr_count) {
			//batteryMain();
			intr_flag |= BATTERY_INTR_MASK;
			battery_intr_count = (battery_intr_count + BATTERY_INTR_COUNT_MIN) % MIN_ROLLOVER;
		}
	}

	// TIM3 interrupt @ 1 kHz / every 1 ms
	if (htim->Instance == TIM3) {
		ms = (ms + 1) % MS_ROLLOVER;

		// push button
		uint8_t pb = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if (prev_pb == RESET && pb == SET) {
			pb_flag = 1;
		}
		prev_pb = pb;

		// radar gun
		if (ms == radar_intr_count) {
			//radarMain();
			intr_flag |= RADAR_INTR_MASK;
			radar_intr_count = (radar_intr_count + RADAR_INTR_COUNT_MS) % MIN_ROLLOVER;
		}

		if (ms == radar_trigger_count && (radar_flag & RADAR_TRIGGER_MASK) == 0) {
			radar_flag |= RADAR_TRIGGER_MASK;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	uint8_t i, j, k;
	for (i = 0; i < (ADC_CHANNELS + 1) / 2; i++) {
		// store ADC data in buffer
		adc_channels[2 * i] = (uint8_t)(adc_data[i] & 0xFF);
		adc_channels[2 * i + 1] = (uint8_t)(adc_data[i] >> 16);
	}

	for (i = 0; i < NUM_RADAR_SEL_PINS; i++) {
		if (adc_channels[i + NUM_RADAR_DATA_PINS] > RADAR_THRESH_HIGH && radar_read[i] < NUM_RADAR_READ_PASSES) {
			for (j = 0; j < NUM_RADAR_DATA_PINS; j++) {
				k = (adc_channels[j] < RADAR_THRESH_LOW) ? (0x01 << (6 - j)) : 0;
				radar_data[i] |= k;
			}
//			radar_flag |= (0x01 << i); // set digit read bit
			radar_read[i]++;
		}
	}

	radar_flag |= ADC_CONV_CPLT_MASK; // set conversion complete bit
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
	HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_RESET); // set radar trigger low
	while ((radar_flag & RADAR_TRIGGER_MASK) == 0); // wait for counter
	radar_flag &= ~RADAR_TRIGGER_MASK; // clear bit
	HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_SET); // set radar trigger high
}

void readRadar(void) {
	radar_flag = 0;
	radar_data[0] = 0;
	radar_data[1] = 0;
	radar_data[2] = 0;
	radar_data[3] = 0;
	radar_read[0] = 0;
	radar_read[1] = 0;
	radar_read[2] = 0;
	radar_read[3] = 0;

//	while ((radar_flag & 0x0F) != 0x0F) {
	while (radar_read[0] < NUM_RADAR_READ_PASSES ||
		   radar_read[1] < NUM_RADAR_READ_PASSES ||
		   radar_read[2] < NUM_RADAR_READ_PASSES ||
		   radar_read[3] < NUM_RADAR_READ_PASSES) {
		HAL_ADC_Start_DMA(&hadc, adc_data, (uint32_t)ADC_CHANNELS);
		while ((radar_flag & ADC_CONV_CPLT_MASK) == 0); // wait for ADC conversion to complete
		radar_flag &= ~ADC_CONV_CPLT_MASK; // clear bit
	}
}

void storeSpeedTime(void) {
	uint16_t speed = getRadarValue();
	// do not store unread speeds
	if (speed == 0 || speed > MAX_SPEED) {
		return;
	}

	if (distribution_index == MAX_DISTRIBUTION_SIZE) {
		distribution_index--;
	}

	speed_distribution[distribution_index].speed = speed;
	speed_distribution[distribution_index].time_hour = hour;
	speed_distribution[distribution_index].time_min = min;
	distribution_index++;
}

uint16_t getRadarValue(void) {
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

	return (uint16_t)(100 * segmentToInt(hundreds) + 10 * segmentToInt(tens) + segmentToInt(ones));
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
			if (speed_distribution[j].time_hour >= (speed_distribution[max_time_index].time_hour + 1) % HOUR_ROLLOVER) {
				max_time_index = j;
			} else if (speed_distribution[j].time_hour == speed_distribution[max_time_index].time_hour) {
				if (speed_distribution[j].time_min > speed_distribution[max_time_index].time_min) {
					max_time_index = j;
				}
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
	uint8_t now_min = min, now_hour = hour;
	uint8_t diff_min = 0, i = 0;

	while (i < distribution_index) {
		// if Speed_Time is aged, remove from distribution; else, continue
		if (now_hour >= (speed_distribution[i].time_hour + 1) % HOUR_ROLLOVER) {
			diff_min = ((now_hour + HOUR_ROLLOVER - speed_distribution[i].time_hour - 1) % HOUR_ROLLOVER) * MIN_ROLLOVER;
			diff_min += (now_min + MIN_ROLLOVER - speed_distribution[i].time_min) % MIN_ROLLOVER;
		} else {
			diff_min = (now_min + MIN_ROLLOVER - speed_distribution[i].time_min) % MIN_ROLLOVER;
		}

		if (diff_min > MAX_DISTRIBUTION_AGE_MIN) {
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

uint16_t getSpeedPercentile(void) {
	sortDistributionSpeed();
	return speed_distribution[(uint16_t)(SPEED_PERCENTILE * distribution_index)].speed;
}

void sortDistributionSpeed(void) {
	uint8_t i, j, min_speed_index;

	// selection sort
	for (i = 0; i < distribution_index - 1; i++) {
		min_speed_index = i;

		for (j = i + 1; j < distribution_index; j++) {
			if (speed_distribution[j].speed < speed_distribution[min_speed_index].speed) {
				min_speed_index = j;
			}
		}

		// swap
		if (min_speed_index != i) {
			speed_time = speed_distribution[i];
			speed_distribution[i] = speed_distribution[min_speed_index];
			speed_distribution[min_speed_index] = speed_time;
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
	storeSpeedInTable();
	if (variable_speed < display_speed) {
		display_speed = variable_speed;
		displaySpeedLimit(display_speed);
	}
}

void checkWeather(void) {
	//TODO: adjust speed based on weather data
	uint32_t weather_speed = variable_speed;
	if (weather_speed <= variable_speed - 5) {
		variable_speed -= 5;
	}
}

void checkSpeedPrediction(void) {
	//TODO: adjust speed based on speed prediction table
	uint32_t predict_speed = getPredictSpeed();
	if (predict_speed <= variable_speed - 5) {
		variable_speed -= 5;
	}
}

//uint16_t getPredictSpeed(void) {
//	uint8_t day_of_week = day;
//	uint8_t time_of_day = getTableIndex();
//	time_of_day = (time_of_day + 1) % TIMES_PER_DAY; // move to next time interval index
//	//TODO: more statistics on predict_table
//	return getAvgSpeed(predict_table[day_of_week][time_of_day]);
//}

uint8_t getTableIndex(void) {
	uint8_t table_index = min;

	// round to nearest 30 min
	if (table_index < (MIN_ROLLOVER / 4)) {
		table_index = 0;
	} else if (table_index < (MIN_ROLLOVER * 3 / 4)) {
		table_index = MIN_ROLLOVER / 2;
	} else {
		table_index = MIN_ROLLOVER;
	}
	table_index += hour * MIN_ROLLOVER; // add hours converted to minutes
	table_index /= (MIN_ROLLOVER * HOUR_ROLLOVER / TIMES_PER_DAY); // convert to predict table index

	return table_index;
}

uint16_t getAvgSpeed(Predict_Element p_e) {
	uint16_t sum = 0;
	uint8_t i;
	for (i = 0; i < p_e.index; i++) {
		sum += p_e.speed_arr[i];
	}
	return (uint16_t)(sum / p_e.index);
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

//void storeSpeedInTable(void) {
//	uint8_t day_of_week = day;
//	uint8_t time_of_day = getTableIndex();
//	predict_table[day_of_week][time_of_day].speed_arr[predict_table[day_of_week][time_of_day].index] = variable_speed;
//	predict_table[day_of_week][time_of_day].index = (predict_table[day_of_week][time_of_day].index + 1) % MAX_PREDICT_SIZE;
//}

void displaySpeedLimit(uint32_t speed) {
	//TODO: send speed to LEDs
	//uint8_t tens = speed / 10;
	//uint8_t ones = speed % 10;
	//TODO: translate tens and ones to LED segment addresses using led_data
	sendLEDData();
}

void clearLEDData(void) {
	uint8_t i;
	for (i = 0; i < NUM_LEDS; i++) {
		led_data[i][RED] = 0x00;
		led_data[i][GREEN] = 0x00;
		led_data[i][BLUE] = 0x00;
	}
}

void sendLEDData(void) {
	uint8_t i;
	for (i = 0; i < NUM_LEDS; i++) {
		sendLEDWord(led_data[i]);
	}

	// latch high to display LEDs
	HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET); // set GPIO pin not high
}

void sendLEDWord(uint8_t * word) {
	uint16_t i, j;
	HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // set GPIO pin not low

	for (i = 0; i < BYTES_PER_WORD; i++) {
		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x80) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}

		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x40) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}

		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x20) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}

		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x10) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}

		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x08) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}

		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x04) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}

		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x02) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}

		HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not high

		if ((word[i] & 0x01) == 0) {
			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low

			// wait for count
			for (j = 0; j < 1; j++);
		} else {
			//wait for count
			for (j = 0; j < 1; j++);

			// send low
			HAL_GPIO_TogglePin(LED_Out_GPIO_Port, LED_Out_Pin); // set GPIO pin not low
		}
	}

	// delay 50 us
	for (i = 0; i < 350; i++);
}

///////////////////////////////////////////////////////////////////////////////
// Display Check															 //
///////////////////////////////////////////////////////////////////////////////

void displayCheckMain(void) {
	//TODO: turn on/off display according to latest Speed_Time.timestamp
	sortDistributionTime();
	speed_time = speed_distribution[distribution_index - 1];
}

///////////////////////////////////////////////////////////////////////////////
// Battery / State of Charge												 //
///////////////////////////////////////////////////////////////////////////////

void batteryMain(void) {
	//TODO: check battery level against threshold, enter low power mode if necessary
	if (adc_channels[BATTERY_ADC_CHANNEL] < BATTERY_THRESH) {
		state = LOW_POWER;
	}
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
