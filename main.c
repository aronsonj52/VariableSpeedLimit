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
#include "cJSON.h"

// Time constants
#define MS_ROLLOVER 1000
#define SEC_ROLLOVER 60
#define MIN_ROLLOVER 60
#define HOUR_ROLLOVER 24
#define DAYS_PER_WEEK 7

// Radar
#define NUM_RADAR_PINS 11
#define NUM_RADAR_DATA_PINS 7
#define NUM_RADAR_SEL_PINS 4
#define NUM_RADAR_READ_PASSES 20
#define RADAR_THRESH_HIGH 0xF0
#define RADAR_THRESH_LOW 0x10
#define RADAR_TRIGGER_MASK 0x40
#define NUM_RADAR_ATTEMPTS 5

// Speed
#define MAX_SPEED 999
#define STATIC_SPEED 65 // default speed
#define MIN_SPEED 40
#define MAX_DISTRIBUTION_SIZE 100
#define MAX_DISTRIBUTION_AGE_MIN 30
#define SPEED_PERCENTILE 0.85 // 85th percentile

// ADC
#define ADC_CHANNELS 12
#define ADC_CONV_CPLT_MASK 0x20

// LED
#define NUM_LEDS 76
#define BITS_PER_BYTE 8
#define BYTES_PER_WORD 3
#define RED 1
#define GREEN 0
#define BLUE 2
//TODO: find good color values
#define RED_COLOR 0xFF
#define GREEN_COLOR 0xFF
#define BLUE_COLOR 0xFF
#define SEGMENT0_ADDR 0
#define SEGMENT1_ADDR 6
#define SEGMENT2_ADDR 11
#define SEGMENT3_ADDR 17
#define SEGMENT4_ADDR 22
#define SEGMENT5_ADDR 27
#define SEGMENT6_ADDR 33
//TODO: find good brightness values
#define BRIGHTNESS 1 // 100% brightness
#define LP_BRIGHTNESS 0.5 // 50% brightness for low power mode

// Weather
#define WEATHER_API_STR "GET /data/2.5/weather?id=4928096&appid=37813a3a15e507a6206d0a276ca84b29 \n\n"
#define WEATHER_DATA_SIZE 600
#define NUM_WEATHER_ATTEMPTS 5
#define NUM_WEATHER_CHECKS 3

// Battery
#define BATTERY_THRESH (0.75 * 0xFF) // 75% of 12 V operating voltage
#define BATTERY_ADC_CHANNEL (ADC_CHANNELS - 1)

// Speed predict table - DEPRECATED
//#define MAX_PREDICT_SIZE 10
//#define TIMES_PER_DAY (HOUR_ROLLOVER * 2) // 30 min intervals

// Display check
#define DISPLAY_OFF_MIN 2

// Timer interrupt constants and masks
#define RADAR_INTR_COUNT_MS 500 // ms
#define RADAR_TRIGGER_COUNT_MS 50 // ms
#define WEATHER_INTR_COUNT_MIN 15 // min
#define DISPLAY_SPEED_INTR_COUNT_MIN 15 // min
#define DISPLAY_CHECK_INTR_COUNT_S 5 // sec
#define BATTERY_INTR_COUNT_MIN 5 // min

#define RADAR_INTR_LP_COUNT_MS 999 // ms - 2x
#define WEATHER_INTR_LP_COUNT_MIN 30 // min - 2x
#define DISPLAY_SPEED_INTR_LP_COUNT_MIN 15 // min - 1x
#define DISPLAY_CHECK_INTR_LP_COUNT_S 1 // sec - 1/5x
#define BATTERY_INTR_LP_COUNT_MIN 1 // min - 1/5x

#define RADAR_INTR_MASK 0x01
#define WEATHER_INTR_MASK 0x02
#define DISPLAY_SPEED_INTR_MASK 0x04
#define DISPLAY_CHECK_INTR_MASK 0x08
#define BATTERY_INTR_MASK 0x10

#define RADAR_POWER_COUNT 50
#define RADAR_POWER_MASK 0x01
#define RADAR_CHECK_MASK 0x02
#define WEATHER_CHECK_MASK 0x04

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
uint16_t radar_power_count = RADAR_POWER_COUNT;
uint8_t weather_intr_count = WEATHER_INTR_COUNT_MIN;
uint8_t display_speed_intr_count = DISPLAY_SPEED_INTR_COUNT_MIN;
uint8_t display_check_intr_count = DISPLAY_CHECK_INTR_COUNT_S;
uint8_t battery_intr_count = BATTERY_INTR_COUNT_MIN;

uint8_t intr_flag = 0;

// Radar
uint8_t radar_data [NUM_RADAR_SEL_PINS];
uint8_t radar_read [NUM_RADAR_SEL_PINS];

typedef struct S_T {
	uint8_t time_hour;
	uint8_t time_min;
	uint8_t time_sec;
	uint16_t speed;
} Speed_Time;
Speed_Time speed_distribution [MAX_DISTRIBUTION_SIZE];
Speed_Time speed_time;

uint8_t distribution_index = 0;
uint16_t variable_speed = STATIC_SPEED, display_speed = STATIC_SPEED;

// ADC
uint32_t adc_data [(ADC_CHANNELS + 1) / 2];
uint8_t adc_channels [ADC_CHANNELS];

// Weather
cJSON * weather_data;
uint8_t weather_buffer [WEATHER_DATA_SIZE];

// LED
uint8_t led_data [NUM_LEDS][BYTES_PER_WORD]; // LED data to shift out
// RED   : led_data[i][1]
// GREEN : led_data[i][0]
// BLUE  : led_data[i][2]

// Speed prediction table - DEPRECATED
//typedef struct Predict_Arr {
//	uint32_t speed_arr [MAX_PREDICT_SIZE];
//	uint8_t index;
//} Predict_Element;
//Predict_Element predict_table [DAYS_PER_WEEK][TIMES_PER_DAY];

// Battery
typedef enum VSL_State {
	STATIC = 0,
	VARIABLE = 1,
	DISPLAY_OFF = 2,
	LOW_POWER = 3
} State;
State state = VARIABLE, prev_state = VARIABLE;

// Hardware/Software Checks
uint8_t module_check = 0;
uint8_t radar_fails = 0;
uint8_t weather_fails = 0;

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

void init(void);
void turnOnRadar(void);
void checkModuleStatus(void);

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
void retrieveWeatherData(void);
void checkValidWeather(void);
uint8_t endOfWeatherData(void);

// Display speed
void displaySpeedMain(void);
void checkWeather(void);
//double convertKtoF(double k); // DEPRECATED
//void checkSpeedPrediction(void); // DEPRECATED
//uint16_t getPredictSpeed(void); // DEPRECATED
//uint8_t getTableIndex(uint8_t h, uint8_t m); // DEPRECATED
//uint16_t getAvgSpeed(Predict_Element p_e); // DEPRECATED
void roundVariableSpeed(void);
//void storeSpeedInTable(void); // DEPRECATED
void displaySpeedLimit(uint32_t speed);
uint8_t intToSegment(uint8_t num);
void clearLEDData(void);
void sendLEDData(void);
void sendLEDWord(uint8_t * word);

// Display check
void displayCheckMain(void);
uint16_t getTimeDeltaMin(Speed_Time * s_t, uint8_t cur_hour, uint8_t cur_min);

// Battery
void batteryMain(void);
void enterLowPower(void);
void sendAlert(void);

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

//  init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t mode = 0;
  clearLEDData();

  while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  uint16_t i, j;

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
*/
	  if (pb_flag == 1) {
//		  displaySpeedLimit(mode);
//		  mode = (mode + 10) % 100;
		  for (i = 0; i < NUM_LEDS; i++) {
			  led_data[i][RED] = 0xFF;
			  led_data[i][GREEN] = 0xFF;
			  led_data[i][BLUE] = 0xFF;
		  }
		  sendLEDData();
		  // wait
		  for (i = 0; i < 10; i++) {
			  for (j = 0; j < 10000; j++);
		  }
		  pb_flag = 0;
	  }
/*
	  for (i = 0; i < NUM_LEDS; i++) {
		  led_data[i][RED] = 0xFF;
		  led_data[i][GREEN] = 0xFF;
		  led_data[i][BLUE] = 0xFF;
	  }

	  clearLEDData();
	  sendLEDData();

	  // wait
	  for (i = 0; i < 10; i++) {
		  for (j = 0; j < 10000; j++);
	  }

	  mode = (mode + 1) % (NUM_LEDS * 2);
*/
	  // Radar Trigger Test
/*
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	  while (pb_flag == 0); // wait for user to push button
	  pb_flag = 0;
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	  triggerRadar();
	  readRadar();
	  storeSpeedTime();
*/

	  // Radar Power Test
/*
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	  while (pb_flag == 0); // wait for user to push button
	  pb_flag = 0;
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	  turnOnRadar();
*/

	  // Weather Data Test
/*
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
//	  while (pb_flag == 0); // wait for user to push button
//	  pb_flag = 0;
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	  weatherMain();
	  checkWeather();
	  while (pb_flag == 0);
*/
	  // Normal Operation
/*
	  if (intr_flag & RADAR_INTR_MASK) {
		  intr_flag &= ~RADAR_INTR_MASK; // clear bit
//		  radarMain();
	  }
	  if (intr_flag & WEATHER_INTR_MASK) {
		  intr_flag &= ~WEATHER_INTR_MASK; // clear bit
//		  weatherMain();
	  }
	  if (intr_flag & DISPLAY_SPEED_INTR_MASK) {
		  intr_flag &= ~DISPLAY_SPEED_INTR_MASK; // clear bit
//		  displaySpeedMain();
	  }
	  if (intr_flag & DISPLAY_CHECK_INTR_MASK) {
		  intr_flag &= ~DISPLAY_CHECK_INTR_MASK; // clear bit
//		  displayCheckMain();
	  }
	  if (intr_flag & BATTERY_INTR_MASK) {
		  intr_flag &= ~BATTERY_INTR_MASK; // clear bit
//		  batteryMain();
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
  huart1.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOB, LED_Out_Pin|Radar_Power_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Radar_Power_Pin */
  GPIO_InitStruct.Pin = Radar_Power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Radar_Power_GPIO_Port, &GPIO_InitStruct);

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
			intr_flag |= WEATHER_INTR_MASK; // set weather intr bit
			weather_intr_count += (state == LOW_POWER) ? WEATHER_INTR_LP_COUNT_MIN : WEATHER_INTR_COUNT_MIN;
			weather_intr_count %= MIN_ROLLOVER;
		}
		if (min == display_speed_intr_count) {
			if (state != STATIC) {
				intr_flag |= DISPLAY_SPEED_INTR_MASK; // set display speed intr bit
			}
			display_speed_intr_count += (state == LOW_POWER) ? DISPLAY_SPEED_INTR_LP_COUNT_MIN : DISPLAY_SPEED_INTR_COUNT_MIN;
			display_speed_intr_count %= MIN_ROLLOVER;
		}
		if (sec == display_check_intr_count) {
			intr_flag |= DISPLAY_CHECK_INTR_MASK; // set display check intr bit
			display_check_intr_count += (state == LOW_POWER) ? DISPLAY_CHECK_INTR_LP_COUNT_S : DISPLAY_CHECK_INTR_COUNT_S;
			display_check_intr_count %= SEC_ROLLOVER;
		}
		if (min == battery_intr_count) {
			intr_flag |= BATTERY_INTR_MASK; // set battery intr bit
			battery_intr_count += (state == LOW_POWER) ? BATTERY_INTR_LP_COUNT_MIN : BATTERY_INTR_COUNT_MIN;
			battery_intr_count %= MIN_ROLLOVER;
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
			intr_flag |= RADAR_INTR_MASK; // set radar intr bit
			radar_intr_count += (state == LOW_POWER) ? RADAR_INTR_LP_COUNT_MS : RADAR_INTR_COUNT_MS;
			radar_intr_count %= MIN_ROLLOVER;
		}

		if (ms == radar_trigger_count && (intr_flag & RADAR_TRIGGER_MASK) == 0) {
			intr_flag |= RADAR_TRIGGER_MASK; // set radar trigger bit
		}

		if (ms == radar_power_count && (module_check & RADAR_POWER_MASK)) {
			module_check &= ~RADAR_POWER_MASK;
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
			radar_read[i]++;
		}
	}

	intr_flag |= ADC_CONV_CPLT_MASK; // set conversion complete bit
}

void init(void) {
	// start TIM1 and TIM3 interrupts
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);

	HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET); // initialize LED_Out_Pin to not low for IDLE
	HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_SET); // set radar trigger high for IDLE
	HAL_GPIO_WritePin(Radar_Power_GPIO_Port, Radar_Power_Pin, GPIO_PIN_RESET);

	clearLEDData();

	turnOnRadar();

	uint8_t i;

	// check radar for correct operation
	for (i = 0; i < NUM_RADAR_ATTEMPTS; i++) {
		triggerRadar();
		readRadar();
		if (getRadarValue() != 0) {
			module_check |= RADAR_CHECK_MASK;
		}
	}

	// check WiFi module for correct operation
	for (i = 0; i < NUM_WEATHER_ATTEMPTS; i++) {
		weatherMain();
		if (weather_data != NULL && cJSON_HasObjectItem(weather_data, "main")) {
			module_check |= WEATHER_CHECK_MASK;
		}
	}

	checkModuleStatus();
}

void turnOnRadar(void) {
	radar_power_count = (ms + RADAR_POWER_COUNT) % MS_ROLLOVER; // set count
	module_check |= RADAR_POWER_MASK; // set bit

	HAL_GPIO_WritePin(Radar_Power_GPIO_Port, Radar_Power_Pin, GPIO_PIN_SET); // set radar power high
	while (module_check & RADAR_POWER_MASK); // wait for count
	HAL_GPIO_WritePin(Radar_Power_GPIO_Port, Radar_Power_Pin, GPIO_PIN_RESET); // set radar power low

	module_check |= RADAR_POWER_MASK;
	radar_power_count = (ms + 999) % MS_ROLLOVER;
	while (module_check & RADAR_POWER_MASK); // wait 1 second for radar to turn on
}

void checkModuleStatus(void) {
	if (state == STATIC) {
		if ((module_check & RADAR_CHECK_MASK) && (module_check & WEATHER_CHECK_MASK)) {
			state = prev_state;
			prev_state = STATIC;
			display_speed = STATIC_SPEED;
		}
	} else {
		if ((module_check & RADAR_CHECK_MASK) == 0 || (module_check & WEATHER_CHECK_MASK) == 0) {
			prev_state = state;
			state = STATIC;
			display_speed = STATIC_SPEED;
			displaySpeedLimit(display_speed);
		}
	}
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
	intr_flag &= ~RADAR_TRIGGER_MASK; // clear radar trigger bit
	HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_RESET); // set radar trigger low
	while ((intr_flag & RADAR_TRIGGER_MASK) == 0); // wait for counter
	intr_flag &= ~RADAR_TRIGGER_MASK; // clear radar trigger bit
	HAL_GPIO_WritePin(Radar_Trig_GPIO_Port, Radar_Trig_Pin, GPIO_PIN_SET); // set radar trigger high
}

void readRadar(void) {
	intr_flag &= ~ADC_CONV_CPLT_MASK; // clear conversion complete bit
	radar_data[0] = 0; // reset radar data
	radar_data[1] = 0;
	radar_data[2] = 0;
	radar_data[3] = 0;
	radar_read[0] = 0; // reset radar read counts
	radar_read[1] = 0;
	radar_read[2] = 0;
	radar_read[3] = 0;

	while (radar_read[0] < NUM_RADAR_READ_PASSES ||
		   radar_read[1] < NUM_RADAR_READ_PASSES ||
		   radar_read[2] < NUM_RADAR_READ_PASSES ||
		   radar_read[3] < NUM_RADAR_READ_PASSES) {
		HAL_ADC_Start_DMA(&hadc, adc_data, (uint32_t)ADC_CHANNELS);
		while ((intr_flag & ADC_CONV_CPLT_MASK) == 0); // wait for ADC conversion to complete
		intr_flag &= ~ADC_CONV_CPLT_MASK; // clear conversion complete bit
	}
}

void storeSpeedTime(void) {
	uint16_t speed = getRadarValue();

	// do not store unread speeds
	if (speed == 0 || speed > MAX_SPEED) {
		radar_fails++;
		if (radar_fails == NUM_RADAR_ATTEMPTS) {
			radar_fails = 0; // clear num radar fails
			module_check &= ~RADAR_CHECK_MASK; // clear radar check bit
			checkModuleStatus();
		}
		return;
	}

	radar_fails = 0; // clear num radar fails

	// check for STATIC state
	if (state == STATIC) {
		module_check |= RADAR_CHECK_MASK;
		checkModuleStatus();
	}

	// if speed_distribution is full, replace oldest speed
	if (distribution_index == MAX_DISTRIBUTION_SIZE) {
		sortDistributionTime();
		distribution_index--;
	}

	speed_distribution[distribution_index].speed = speed;
	speed_distribution[distribution_index].time_hour = hour;
	speed_distribution[distribution_index].time_min = min;
	speed_distribution[distribution_index].time_sec = sec;
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

// sorts speed_distribution by time most recent to oldest (decreasing)
void sortDistributionTime(void) {
	uint8_t i, j, k, max_time_index;

	// selection sort
	for (i = 0; i < distribution_index - 1; i++) {
		max_time_index = i;

		for (j = i + 1; j < distribution_index; j++) {
			if (speed_distribution[j].time_hour > speed_distribution[max_time_index].time_hour) {
				max_time_index = j;
			} else if (speed_distribution[j].time_hour == speed_distribution[max_time_index].time_hour) {
				if (speed_distribution[j].time_min > speed_distribution[max_time_index].time_min) {
					max_time_index = j;
				} else if (speed_distribution[j].time_min == speed_distribution[max_time_index].time_min) {
					if (speed_distribution[j].time_sec > speed_distribution[max_time_index].time_sec) {
						max_time_index = j;
					}
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

	// split around large timedeltas (rotate down)
	for (i = 0; i < distribution_index - 1; i++) {
		if (speed_distribution[i].time_hour > speed_distribution[i + 1].time_hour + (HOUR_ROLLOVER / 2)) {
			// rotate down (i + 1) times
			for (j = 0; j <= i; j++) {
				// rotate down
				speed_time = speed_distribution[0];
				for (k = 0; k < distribution_index - 1; k++) {
					speed_distribution[k] = speed_distribution[k + 1];
				}
				speed_distribution[distribution_index - 1] = speed_time;
			}
		}
	}
}

void cleanDistribution(void) {
	uint16_t timedelta_min = 0;
	uint8_t i = 0;

	while (i < distribution_index) {
		// if Speed_Time is aged, remove from distribution; else, continue
		timedelta_min = getTimeDeltaMin(&speed_distribution[i], hour, min);

		if (timedelta_min > MAX_DISTRIBUTION_AGE_MIN) {
			removeSpeedTime(i);
		} else {
			i++;
		}
	}
}

void removeSpeedTime(uint8_t index) {
	// shift down all Speed_Time's after index
	uint8_t i;

	for (i = index; i < distribution_index - 1; i++) {
		speed_distribution[i] = speed_distribution[i + 1];
	}

	// adjust distribution index/size
	distribution_index--;
}

uint16_t getSpeedPercentile(void) {
	sortDistributionSpeed();
	double index_d = SPEED_PERCENTILE * (distribution_index - 1);
	uint8_t index_i = (uint8_t)(index_d);
	double percentile = speed_distribution[index_i].speed;
	percentile += (index_d > index_i) ? (index_d - index_i) * (speed_distribution[index_i + 1].speed - speed_distribution[index_i].speed) : 0;
	return (uint16_t)(percentile + .5);
}

// sorts speed_distribution by speed lowest to highest (increasing)
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
	retrieveWeatherData();
	checkValidWeather();
}

void retrieveWeatherData(void) {
	HAL_UART_Transmit(&huart1, WEATHER_API_STR, sizeof(WEATHER_API_STR) - 1, HAL_MAX_DELAY);

	uint16_t i = 0;
	do {
		HAL_UART_Receive(&huart1, &weather_buffer[i++], 1, HAL_MAX_DELAY);
	} while (i < WEATHER_DATA_SIZE && !endOfWeatherData());

	// remove '*CLOS*' at end
	if (endOfWeatherData()) {
		uint16_t i = 0;
		while (weather_buffer[i++] != '\0');
		i -= 2;
		weather_buffer[i - 5] = '\0';
		weather_buffer[i - 4] = '\0';
		weather_buffer[i - 3] = '\0';
		weather_buffer[i - 2] = '\0';
		weather_buffer[i - 1] = '\0';
		weather_buffer[i] = '\0';
	}

	weather_data = cJSON_Parse((char *)(weather_buffer + 6));
}

void checkValidWeather(void) {
	// check for valid weather data
	if (weather_data == NULL || !cJSON_HasObjectItem(weather_data, "main")) {
		weather_fails++;
		if (weather_fails == NUM_WEATHER_ATTEMPTS) {
			weather_fails = 0; // clear num weather fails
			module_check &= ~WEATHER_CHECK_MASK; // clear weather check bit
			checkModuleStatus();
		}
		return;
	}

	weather_fails = 0; // clear num weather fails

	// check for STATIC state
	if (state == STATIC) {
		module_check |= WEATHER_CHECK_MASK; // set weather check bit
		checkModuleStatus();
	}
}

uint8_t endOfWeatherData(void) {
	// find end of string
	uint16_t i = 0;
	while (i < WEATHER_DATA_SIZE && weather_buffer[i++] != '\0');
	if (i < 7) {
		return 0;
	}
	i -= 2;

	// check for '*CLOS*' at end
	return (weather_buffer[i - 5] == 42 && weather_buffer[i - 4] == 67 &&
			weather_buffer[i - 3] == 76 && weather_buffer[i - 2] == 79 &&
			weather_buffer[i - 1] == 83 && weather_buffer[i] == 42);
}

///////////////////////////////////////////////////////////////////////////////
// Display Speed															 //
///////////////////////////////////////////////////////////////////////////////

void displaySpeedMain(void) {
	if (variable_speed < MIN_SPEED) {
		variable_speed = MIN_SPEED;
	}
//	storeSpeedInTable();
	checkWeather();
//	checkSpeedPrediction();
	roundVariableSpeed();
	if (variable_speed != display_speed) {
		display_speed = variable_speed;
		displaySpeedLimit(display_speed);
	}
}

void checkWeather(void) {
	int16_t weather_speed [NUM_WEATHER_CHECKS] = {variable_speed, variable_speed, variable_speed};

	// Wind
	cJSON * cJSON_Obj = NULL;
	if (cJSON_HasObjectItem(weather_data, "wind")) {
		cJSON_Obj = cJSON_GetObjectItem(cJSON_GetObjectItem(weather_data, "wind"), "speed");
	}
	double wind_speed = (cJSON_Obj == NULL) ? 0 : cJSON_Obj->valuedouble;
	if (wind_speed <= 10) {
		weather_speed[0] -= 0;
	} else if (wind_speed <= 14) {
		weather_speed[0] -= 5;
	} else if (wind_speed <= 17) {
		weather_speed[0] -= 10;
	} else {
		weather_speed[0] -= 15;
	}

	// Rain
	cJSON_Obj = NULL;
	if (cJSON_HasObjectItem(weather_data, "rain")) {
		cJSON_Obj = cJSON_GetObjectItem(cJSON_GetObjectItem(weather_data, "rain"), "3h");
	}
	uint8_t volume = (cJSON_Obj == NULL) ? 0 : cJSON_Obj->valueint;
	if (volume <= 7) {
		weather_speed[1] -= 0;
	} else if (volume <= 14) {
		weather_speed[1] -= 5;
	} else {
		weather_speed[1] -= 10;
	}

	// Snow
	cJSON_Obj = NULL;
	if (cJSON_HasObjectItem(weather_data, "snow")) {
		cJSON_Obj = cJSON_GetObjectItem(cJSON_GetObjectItem(weather_data, "snow"), "3h");
	}
	volume = (cJSON_Obj == NULL) ? 0 : cJSON_Obj->valueint;
	if (volume <= 4) {
		weather_speed[2] -= 0;
	} else if (volume <= 14) {
		weather_speed[2] -= 5;
	} else if (volume <= 80) {
		weather_speed[2] -= 15;
	} else {
		weather_speed[2] -= 30;
	}

	uint8_t i;
	for (i = 0; i < NUM_WEATHER_CHECKS; i++) {
		if (weather_speed[i] < MIN_SPEED) {
			weather_speed[i] = MIN_SPEED;
		}
		if (weather_speed[i] < variable_speed) {
			variable_speed = weather_speed[i];
		}
	}
}

// DEPRECATED

//double convertKtoF(double k) {
//	return (k - 273.15) * 9 / 5 + 32;
//}

// DEPRECATED

//void checkSpeedPrediction(void) {
//	uint32_t predict_speed = getPredictSpeed();
//	if (predict_speed <= variable_speed - 5) {
//		variable_speed -= 5;
//	}
//}

// DEPRECATED

//uint16_t getPredictSpeed(void) {
//	uint8_t day_of_week = day;
//	uint8_t time_of_day = getTableIndex(hour, min);
//	time_of_day = (time_of_day + 1) % TIMES_PER_DAY; // move to next time interval index
//	return getAvgSpeed(predict_table[day_of_week][time_of_day]);
//}

// DEPRECATED

//uint8_t getTableIndex(uint8_t h, uint8_t m) {
//	uint8_t table_index = m;
//
//	// round to nearest 30 min
//	if (table_index < (MIN_ROLLOVER / 4)) {
//		table_index = 0;
//	} else if (table_index < (MIN_ROLLOVER * 3 / 4)) {
//		table_index = MIN_ROLLOVER / 2;
//	} else {
//		table_index = MIN_ROLLOVER;
//	}
//	table_index += h * MIN_ROLLOVER; // add hours converted to minutes
//	table_index /= (MIN_ROLLOVER * HOUR_ROLLOVER / TIMES_PER_DAY); // convert to predict table index
//	table_index %= TIMES_PER_DAY;
//
//	return table_index;
//}

// DEPRECATED

//uint16_t getAvgSpeed(Predict_Element p_e) {
//	uint16_t sum = 0;
//	uint8_t i;
//
//	for (i = 0; i < p_e.index; i++) {
//		sum += p_e.speed_arr[i];
//	}
//	return (uint16_t)(sum / p_e.index);
//}

void roundVariableSpeed(void) {
	uint8_t ones = variable_speed % 10;
	if (ones <= 2) { // x0-x2 -> x0
		variable_speed = (uint16_t)(variable_speed / 10) * 10;
	} else if (ones <= 7) { // x3-x7 -> x5
		variable_speed = (uint16_t)(variable_speed / 10) * 10 + 5;
	} else { // x8-x9 -> (x+1)0
		variable_speed = (uint16_t)(variable_speed / 10) * 10 + 10;
	}
}

// DEPRECATED

//void storeSpeedInTable(void) {
//	uint8_t day_of_week = day;
//	uint8_t time_of_day = getTableIndex(hour, min);
//	predict_table[day_of_week][time_of_day].speed_arr[predict_table[day_of_week][time_of_day].index] = variable_speed;
//	predict_table[day_of_week][time_of_day].index = (predict_table[day_of_week][time_of_day].index + 1) % MAX_PREDICT_SIZE;
//}

void displaySpeedLimit(uint32_t speed) {
	clearLEDData();

	// get digit/segment data
	uint8_t tens_digit = intToSegment(speed / 10);
	if (speed < 10) {
		tens_digit = 0;
	}
	uint8_t ones_digit = intToSegment(speed % 10);

	uint8_t i, j;

	for (i = 0; i < BITS_PER_BYTE; i++) {
		if (tens_digit & (0x01 << i)) {
			if (i == 0) {
				for (j = SEGMENT0_ADDR; j < SEGMENT1_ADDR; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 1) {
				for (j = SEGMENT1_ADDR; j < SEGMENT2_ADDR; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 2) {
				for (j = SEGMENT2_ADDR; j < SEGMENT3_ADDR; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 3) {
				for (j = SEGMENT3_ADDR; j < SEGMENT4_ADDR; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 4) {
				for (j = SEGMENT4_ADDR; j < SEGMENT5_ADDR; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 5) {
				for (j = SEGMENT5_ADDR; j < SEGMENT6_ADDR; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 6) {
				for (j = SEGMENT6_ADDR; j < NUM_LEDS / 2; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			}
		}

		if (ones_digit & (0x01 << i)) {
			if (i == 0) {
				for (j = SEGMENT0_ADDR + (NUM_LEDS / 2); j < SEGMENT1_ADDR + (NUM_LEDS / 2); j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 1) {
				for (j = SEGMENT1_ADDR + (NUM_LEDS / 2); j < SEGMENT2_ADDR + (NUM_LEDS / 2); j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 2) {
				for (j = SEGMENT2_ADDR + (NUM_LEDS / 2); j < SEGMENT3_ADDR + (NUM_LEDS / 2); j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 3) {
				for (j = SEGMENT3_ADDR + (NUM_LEDS / 2); j < SEGMENT4_ADDR + (NUM_LEDS / 2); j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 4) {
				for (j = SEGMENT4_ADDR + (NUM_LEDS / 2); j < SEGMENT5_ADDR + (NUM_LEDS / 2); j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 5) {
				for (j = SEGMENT5_ADDR + (NUM_LEDS / 2); j < SEGMENT6_ADDR + (NUM_LEDS / 2); j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			} else if (i == 6) {
				for (j = SEGMENT6_ADDR + (NUM_LEDS / 2); j < NUM_LEDS; j++) {
					led_data[j][RED] = (uint8_t)(RED_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][GREEN] = (uint8_t)(GREEN_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
					led_data[j][BLUE] = (uint8_t)(BLUE_COLOR * (state == LOW_POWER ? LP_BRIGHTNESS : BRIGHTNESS));
				}
			}
		}
	}

	sendLEDData();
}

uint8_t intToSegment(uint8_t num) {
	switch (num) {
	case 0: return 0x7E;
	case 1: return 0x42;
	case 2: return 0x37;
	case 3: return 0x67;
	case 4: return 0x4B;
	case 5: return 0x6D;
	case 6: return 0x7D;
	case 7: return 0x46;
	case 8: return 0x7F;
	case 9: return 0x6F;
	default: return 0x00;
	}
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
	sortDistributionTime();
	uint16_t timedelta_min = getTimeDeltaMin(&speed_distribution[0], hour, min);

	if (state == DISPLAY_OFF) {
		if (timedelta_min <= DISPLAY_OFF_MIN) {
			// turn display on
			state = prev_state;
			prev_state = DISPLAY_OFF;
			displaySpeedLimit(display_speed);
		}
	} else {
		if (timedelta_min > DISPLAY_OFF_MIN) {
			// turn display off
			prev_state = state;
			state = DISPLAY_OFF;
			clearLEDData();
			sendLEDData();
		}
	}
}

uint16_t getTimeDeltaMin(Speed_Time * s_t, uint8_t cur_hour, uint8_t cur_min) {
	uint16_t timedelta_min = 0;

	// add hours between
	uint8_t hours = (cur_hour + HOUR_ROLLOVER - s_t->time_hour) % HOUR_ROLLOVER;
	timedelta_min += hours * MIN_ROLLOVER;

	// check for minute difference included in hours
	if (hours >= 1 && cur_min < s_t->time_min) {
		timedelta_min -= MIN_ROLLOVER;
	}

	// add minute difference
	timedelta_min += (cur_min + MIN_ROLLOVER - s_t->time_min) % MIN_ROLLOVER;

	return timedelta_min;
}

///////////////////////////////////////////////////////////////////////////////
// Battery / State of Charge												 //
///////////////////////////////////////////////////////////////////////////////

void batteryMain(void) {
	if (state == LOW_POWER) {
		if (adc_channels[BATTERY_ADC_CHANNEL] <= BATTERY_THRESH) {
			state = prev_state;
			prev_state = LOW_POWER;
		}
	} else {
		if (adc_channels[BATTERY_ADC_CHANNEL] > BATTERY_THRESH) {
			prev_state = state;
			state = LOW_POWER;
			enterLowPower();
			sendAlert();
		}
	}
}

void enterLowPower(void) {
	uint8_t i;
	for (i = 0; i < NUM_LEDS; i++) {
		led_data[i][RED] = (uint8_t)(led_data[i][RED] * LP_BRIGHTNESS);
		led_data[i][GREEN] = (uint8_t)(led_data[i][GREEN] * LP_BRIGHTNESS);
		led_data[i][BLUE] = (uint8_t)(led_data[i][BLUE] * LP_BRIGHTNESS);
	}
	sendLEDData();
}

void sendAlert(void) {
	//TODO: send alert through WiFi module or light up LED
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
