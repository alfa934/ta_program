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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "sampling.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/******************************************************************************
 * Function Definitions
 *****************************************************************************/
#define ADAPTIVE_SAMPLING
#define CONTROL_SYSTEM

uint8_t done_state = 0;
int reset = 0;
int test = 0;
uint32_t current_sys_tick = 0;
uint8_t	get_data[] = "R\r", get_data_cnt = 0;
uint8_t send_time = 0;
uint8_t ph_ready = 0, temp_ready = 0, do_ready = 0, turb_ready = 0;
uint8_t ph_cnt = 0, temp_cnt = 0, do_cnt = 0, turb_cnt = 0;
uint8_t send_cnt = 0;
uint32_t sys_tick_offset = 0;
uint8_t tick_once = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim4) //--- general timer (every 1 ms)
	{
		test++;

		if(adaptive_start)
		{
			globalTimerProcess(&Global_Time);
		}


#ifdef CONTROL_SYSTEM
		my_my_task();

		light_sequence();

//		Task_Pressure();

		Task_Control_Test_UpDown();

		if((depth_setpoint != -1) && (duration_min != 0))
		{
			pid_task.start = 1; //---set this to 1
		}

		Task_Control_PID();
#endif

#ifdef ADAPTIVE_SAMPLING

	if(adaptive_start)
	{

		if(Global_Time.minute_t >= duration_min)
		{
			adaptive_start = 0;
			done_state = 1;
			pid_task.state = 3;
		}


		if(send_ph)
		{
			ph_ready = 1;
			ph_cnt++;
			if(ph_cnt >= 99)
			{
				ph_cnt = 0;
				ph_ready = 0;
				send_ph = 0;
			}
		}

		if(send_temp)
		{
			temp_ready = 1;
			temp_cnt++;
			if(temp_cnt >= 99)
			{
				temp_cnt = 0;
				temp_ready = 0;
				send_temp = 0;
			}
		}

		if(send_do)
		{
			do_ready = 1;
			do_cnt++;
			if(do_cnt >= 99)
			{
				do_cnt = 0;
				do_ready = 0;
				send_do = 0;
			}
		}

		if(send_turb)
		{
			turb_ready = 1;
			turb_cnt++;
			if(turb_cnt >= 99)
			{
				turb_cnt = 0;
				turb_ready = 0;
				send_turb = 0;
			}
		}


		if(done_state)
		{
			ph_ready = temp_ready = do_ready = turb_ready = 0;
		}

		memcpy(transmit +  3, &Global_Time.hour_t, 2);
		memcpy(transmit +  5, &Global_Time.minute_t, 1);
		memcpy(transmit +  6, &Global_Time.second_t, 1);
		memcpy(transmit +  7, &Global_Time.millisecond_t, 2);

		memcpy(transmit +  9, &ph_ready, 1);
		memcpy(transmit + 10, &ph, 4);
		memcpy(transmit + 14, &pHSampler.currentInterval, 4);

		memcpy(transmit + 18, &temp_ready, 1);
		memcpy(transmit + 19, &temp_cal, 4);
		memcpy(transmit + 23, &tempSampler.currentInterval, 4);

		memcpy(transmit + 27, &do_ready, 1);
		memcpy(transmit + 28, &do_data.value, 4);
		memcpy(transmit + 32, &doSampler.currentInterval, 4);

		memcpy(transmit + 36, &turb_ready, 1);
		memcpy(transmit + 37, &turbid_data.value, 4);
		memcpy(transmit + 41, &turbidSampler.currentInterval, 4);

		send_time++;
		if(send_time >= 99)
		{
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *)transmit, sizeof(transmit));
			send_time = 0;
		}

	}

#endif
	}

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		memcpy(&depth_setpoint, UART1_RX_BUFFER + 3, 4);
		memcpy(&duration_min, UART1_RX_BUFFER + 7, 4);

		HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
	}

	if(huart == &huart2)
	{
		for (int i = 0 ; i< sizeof(rx) ; i++){
			if(rx[i] == '\r' || rx[i] == '\n'){
				rx[i] = '\0';
				break;
				}
			}
			ph = atof(rx);
			//ph_cal = (ph - 0.085) / 0.5383;

		    response_received = 1;

		    HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx, sizeof(rx));
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		if(!(UART1_RX_BUFFER[0] == 'A' && UART1_RX_BUFFER[1] == 'B' && UART1_RX_BUFFER[2] == 'C'))
		{
			HAL_UART_AbortReceive(&huart1);
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
		}
	}

	if(huart == &huart2)
	{
//		if(!(rx[0] == 'A' && rx[1] == 'B' && rx[2] == 'C'))
//		{
//			HAL_UART_AbortReceive(&huart2);
//			HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx, sizeof(rx));
//		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		HAL_UART_AbortReceive(&huart1);
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));
	}

	if(huart == &huart2)
	{
		HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx, sizeof(rx));
	}
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

#ifdef CONTROL_SYSTEM
  initSubmersible();
#endif

#ifdef ADAPTIVE_SAMPLING

  initSampler(&tempSampler, tempHistory, HISTORY_SIZE, 0.025, 1000, 1000, 10000);
  initSampler(&doSampler, doHistory, HISTORY_SIZE, 0.06, 90000, 90000, 300000);
  initSampler(&pHSampler, pHHistory, HISTORY_SIZE, 0.018, 60000, 60000, 300000);
  initSampler(&turbidSampler, turbidHistory, HISTORY_SIZE, 0.02, 1000, 1000, 10000);

  temp_cal = 25;

  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx, sizeof(rx));

  HAL_TIM_Base_Start_IT(&htim3);
#endif
//--- turn this ON
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)UART1_RX_BUFFER, sizeof(UART1_RX_BUFFER));

  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

#ifdef ADAPTIVE_SAMPLING

	  if(adaptive_start)
	  {
		  if(!tick_once)
		  {
			  tick_once++;
			  sys_tick_offset = HAL_GetTick();
		  }
	  current_sys_tick = HAL_GetTick() - sys_tick_offset;
	  switch (temp_state)
	  {
	  	  case TEMP_STATE_IDLE:
	  		  if (current_sys_tick - last_temp_tick >= tempSampler.currentInterval-750)
	  		  { // 1 detik interval pembacaan

	  			  Presence = DS18B20_Start();
	  			  if (Presence == 1)
	  			  {
	  				  DS18B20_Write(0xCC);  // skip ROM
	  				  DS18B20_Write(0x44);  // start temperature conversion
	  				  last_temp_tick = current_sys_tick;  // mulai tunggu 750ms
	  				  temp_state = TEMP_STATE_WAIT_CONVERT;
	  			  }
	  			  else
	  			  {
	  				  last_temp_tick = current_sys_tick;
	  				  temp_state = TEMP_STATE_IDLE;
	  			  }
	  		  }
	  		  break;

	  	  case TEMP_STATE_WAIT_CONVERT:
	  		  if (current_sys_tick - last_temp_tick >= 750)
	  		  {
	  			  temp_state = TEMP_STATE_READ_TEMP;
	  		  }
	  		  break;

	  	  case TEMP_STATE_READ_TEMP:
	  		  Presence = DS18B20_Start();
	  		  if (Presence == 1)
	  		  {
	  			  DS18B20_Write(0xCC);  // skip ROM
	  			  DS18B20_Write(0xBE);  // read scratchpad
	  			  Temp_byte1 = DS18B20_Read(); // LSB
	  			  Temp_byte2 = DS18B20_Read(); // MSB
	  			  temp_state = TEMP_STATE_DONE;
	  		  }
	  		  else
	  		  {
	  			  last_temp_tick = current_sys_tick;
	  			  temp_state = TEMP_STATE_IDLE; // gagal deteksi sensor
	  		  }
	  		  break;

	  	  case TEMP_STATE_DONE:
	  		  uint16_t TEMP = (Temp_byte2 << 8) | Temp_byte1;
	  		  temp_raw = (float)TEMP / 16.0;
	  		  temp_cal = 0.9722 * temp_raw + 0.6976;
	  		  float newIntervalTemp = updateSamplingInterval(&tempSampler, temp_cal);
	  		  tempSampler.currentInterval = newIntervalTemp;
	  		  send_temp = 1;
	  		  last_temp_tick = current_sys_tick;
	  		  temp_state = TEMP_STATE_IDLE;
	  		  break;

	  	  default:
	  		  temp_state = TEMP_STATE_IDLE;
	  		  break;
	  }


	  switch(do_state)
	  {
	  	  case DO_STATE_IDLE:
	  		  if(current_sys_tick - last_do_tick >= doSampler.currentInterval)
	  		  {
	  			  do_state = DO_STATE_READ_ADC;
	  		  }
	  		  break;

	  	  case DO_STATE_READ_ADC:
	  		  HAL_ADC_Start(&hadc1);
	  		  if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	  		  {
	  			  do_data.adc_raw = HAL_ADC_GetValue(&hadc1);
	  		  }
	  		  else
	  		  {
	  			  do_data.adc_raw = 0;
	  		  }
	  		  HAL_ADC_Stop(&hadc1);
	  		  do_state = DO_STATE_CALCULATE;
	  		  break;

	  	  case DO_STATE_CALCULATE:
	  		  do_data.adc_voltage = VREF * do_data.adc_raw/ADC_RES;
	  		  do_data.value = calculateDO(do_data.adc_voltage, temp_cal);
	  		  do_data.cal_value = (0.1012*do_data.value*do_data.value)+(0.2518*do_data.value)+0.3116;
	  		  do_state = DO_STATE_DONE;
	  		  break;
	  	  case DO_STATE_DONE:
	  		  float newIntervalDO = updateSamplingInterval(&doSampler, do_data.value);
	  		  doSampler.currentInterval = newIntervalDO;
	  		  last_do_tick = current_sys_tick;
	  		  do_state = DO_STATE_IDLE;
	  		  send_do = 1;
	  		  break;

	  	  default:
	  		  do_state = DO_STATE_IDLE;
	  		  break;
	  }


	  switch(turbid_state)
	  {
	  	  case TURBID_STATE_IDLE:
	  		  if(current_sys_tick - last_turbid_tick >= turbidSampler.currentInterval)
	  		  {
	  			  turbid_state = TURBID_STATE_READ_ADC;
	  		  }
	  		  break;

	  	  case TURBID_STATE_READ_ADC:
	  		  HAL_ADC_Start(&hadc2);
	  		  if(HAL_ADC_PollForConversion(&hadc2, 100) == HAL_OK)
	  		  {
	  			  turbid_data.adc_raw = HAL_ADC_GetValue(&hadc2);
	  		  }
	  		  else
	  		  {
	  			  turbid_data.adc_raw = 0;
	  		  }
	  		  HAL_ADC_Stop(&hadc2);
	  		  turbid_state = TURBID_STATE_FILTER;
	  		  break;

	  	  case TURBID_STATE_FILTER:
	  		  // Hapus nilai lama dari jumlah total
	  		  if (turbid_buffer_count == MOV_AVER_TURB)
	  		  {
	  			  turbid_buffer_sum -= turbid_values_buffer[turbid_buffer_index];
	  		  }

	  		  // Tambahkan nilai ADC raw baru ke buffer dan jumlah total
	  		  turbid_values_buffer[turbid_buffer_index] = turbid_data.adc_raw;
	  		  turbid_buffer_sum += turbid_data.adc_raw;

	  		  // Majukan indeks buffer
	  		  turbid_buffer_index = (turbid_buffer_index + 1) % MOV_AVER_TURB;

	  		  // Hitung berapa banyak data yang sudah ada di buffer (maks TURBID_MOVING_AVERAGE_N)
	  		  if (turbid_buffer_count < MOV_AVER_TURB)
	  		  {
	  			  turbid_buffer_count++;
	  		  }

	  		  // Hitung nilai rata-rata ADC mentah yang difilter
	  		  turbid_data.filtered_adc_raw = (uint32_t)(turbid_buffer_sum / turbid_buffer_count);

	  		  turbid_state = TURBID_STATE_CALCULATE; // Lanjut ke state perhitungan
	  		  break;

	  	  case TURBID_STATE_CALCULATE:

	  		  turbid_data.adc_voltage = VREF * (((float)turbid_data.filtered_adc_raw * 2) / ADC_RES)/1000;

	  		  turbid_data.cal_value = (turbid_data.adc_voltage - 2.1733) / -0.003;

	  		  if(turbid_data.cal_value < 0){

	  			  turbid_data.value = 0;

	  		  }

	  		  else turbid_data.value = turbid_data.cal_value;

	  		  turbid_state = TURBID_STATE_DONE;

	  		  break;

	  	  case TURBID_STATE_DONE:
	  		  float newIntervalTurbid = updateSamplingInterval(&turbidSampler, turbid_data.value);

	  		  turbidSampler.currentInterval = newIntervalTurbid;

	  		  last_turbid_tick = current_sys_tick;

	  		  turbid_state = TURBID_STATE_IDLE;

	  		  send_turb = 1;

	  		  break;

	  	  default:
	  		  turbid_state = TURBID_STATE_IDLE;
	  		  break;
	  }


	  switch(ph_state)
	  {
	  	  case PH_STATE_IDLE:
	  		  if(current_sys_tick - last_ph_tick >= pHSampler.currentInterval)
	  		  {
	  			  ph_state = PH_STATE_REQUEST_DATA;
	  		  }
	  		  break;

	  	  case PH_STATE_REQUEST_DATA:
	  		  if(huart2.gState == HAL_UART_STATE_READY)
	  		  {
	  			HAL_UART_Transmit_DMA(&huart2, get_data, strlen((char*)get_data));

	  			response_received = 0;

	  			last_ph_tick = current_sys_tick;

	  			ph_state = PH_STATE_WAIT_RESPONSE;
	  		  }
	  		  break;

	  	  case PH_STATE_WAIT_RESPONSE:
	  		  if(response_received)
	  		  {
	  			  ph_state = PH_STATE_DONE;
	  		  }
//	  		  else
//	  		  {
//	  			  HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx, sizeof(rx));
//	  		  }
	  		  break;

	  	  case PH_STATE_DONE:
	  		  float newIntervalPH = updateSamplingInterval(&pHSampler, ph);

	  		  pHSampler.currentInterval = newIntervalPH;

	  		  last_ph_tick = current_sys_tick;

	  		  send_ph = 1;

	  		  ph_state = PH_STATE_IDLE;

	  		  break;

	  	  default:
	  		  ph_state = PH_STATE_IDLE;
	  		  break;
	  }

	  }
#endif


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
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
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB14 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
