/*
 * sampling.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Evander
 */


#include "sampling.h"

/******************************************************************************
 * Declarations
 *****************************************************************************/
const float DO_Table[41] =
{
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

Time_t			Global_Time			= {0};
//--- Temperature
uint8_t 		Presence;
uint8_t 		Temp_byte1;
uint8_t 		Temp_byte2;
TempState_t 	temp_state 			= TEMP_STATE_IDLE;
uint32_t 		last_temp_tick 		= 0;
float 			temp_raw;
float 			temp_cal;
//--- Dissolved Oxygen
Sensor 			do_data;
DOState_t 		do_state 			= DO_STATE_IDLE;
uint32_t 		last_do_tick 		= 0;
//--- Turbidity
Sensor 			turbid_data;
TurbidState_t 	turbid_state 		= TURBID_STATE_IDLE;
uint32_t 		last_turbid_tick 	= 0;
uint32_t 		turbid_values_buffer[MOV_AVER_TURB]; // Menyimpan ADC raw
uint8_t 		turbid_buffer_index = 0;
float 			turbid_buffer_sum 	= 0.0f;
uint8_t 		turbid_buffer_count = 0;
//--- pH
PHState_t 		ph_state = PH_STATE_IDLE;
uint32_t 		last_ph_tick 		= 0;
char 			rx[6];
//char 			tx[8] 				= "ABC";
float 			ph 					= 0;
float			ph_cal;
uint8_t 		response_received 	= 0;


//--- Adaptive Sampling
float tempHistory[HISTORY_SIZE];
AdaptiveSampler tempSampler;

float doHistory[HISTORY_SIZE];
AdaptiveSampler doSampler;

float turbidHistory[HISTORY_SIZE];
AdaptiveSampler turbidSampler;

float pHHistory[HISTORY_SIZE];
AdaptiveSampler pHSampler;

char transmit[53] 					= "ABC";
uint8_t send_temp					= 0;
uint8_t send_ph						= 0;
uint8_t send_do						= 0;
uint8_t send_turb					= 0;
/******************************************************************************
 * Functions
 *****************************************************************************/
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while((__HAL_TIM_GET_COUNTER(&htim3)) < us);
}

uint8_t DS18B20_Start(void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN); //set pin sebagai output
	HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);
	delay(480); //delay berdasarkan datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN); //set pin sebagai input
	delay(80);// delay berdasarkan datasheet

	if(!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;
	else Response = -1;
	delay(400);

	return Response;
}

void DS18B20_Write(uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN); //set pin sebagai output

	for(int i=0; i<8 ; i++){
		if((data & (1<<i)) != 0){
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);//set pin sebagai output
			HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, 0);
			delay(1);

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN); //set pin sebagai input
			delay(50);
		}
		else {// jika bit 0
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);
			delay (50);

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read(void)
{
	uint8_t value = 0;
	Set_Pin_Input (DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++){
		Set_Pin_Output (DS18B20_PORT, DS18B20_PIN); // set sebagai output
		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);
		delay (2);
		Set_Pin_Input (DS18B20_PORT, DS18B20_PIN);
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))
		{
			value |= 1<<i;
		}
		delay (60);
	}
	return value;
}

float calculateDO(uint32_t voltage_mv, float current_temp_cal)
{
    uint8_t temp_index = (uint8_t)roundf(current_temp_cal);
    if(temp_index > 40) temp_index = 40;
    if(temp_index < 0) temp_index = 0;

    // Kalibrasi satu titik dari kode Arduino Anda:
    float V_saturation = (float)CAL1_V + 35.0f * (current_temp_cal - (float)CAL1_T);

    if (fabsf(V_saturation) < 0.001f) V_saturation = 0.001f; // Hindari pembagian dengan nol

    float calculated_do = ((float)voltage_mv * DO_Table[temp_index] / V_saturation) / 1000.0f;

    // float do_final_cal = (0.1012f * calculated_do * calculated_do) +
    //                      (0.2518f * calculated_do) + 0.3116f;
    // return do_final_cal;
    // Jika tidak ada kalibrasi akhir, kembalikan nilai yang sudah dikompensasi suhu:
    return calculated_do;
}

void initSampler(AdaptiveSampler* sampler, float* historyBuffer, int historySize,
                float threshold, float initInterval, float minI, float maxI)
{
    sampler->history = historyBuffer;
    sampler->historySize = historySize;
    sampler->t = threshold;
    sampler->currentInterval = initInterval;
    sampler->minInterval = minI;
    sampler->maxInterval = maxI;
    sampler->dataIndex = 0;
    sampler->prevValue = 0.0f;
    sampler->initialized = 0;
}

float updateSamplingInterval(AdaptiveSampler* sampler, float newValue)
{
    // Fase inisialisasi: isi buffer historis
    if(!sampler->initialized) {
        sampler->history[sampler->dataIndex] = newValue;
        sampler->dataIndex++;

        if(sampler->dataIndex == sampler->historySize) {
        	sampler->initialized = 1; // fase inisialisasi selesai
            // Hitung rata-rata awal
            float avg = 0.0f;
            for(int i=0; i<sampler->historySize; i++) {
            	avg += sampler->history[i];
            }
            sampler->prevValue = avg/sampler->historySize;

            sampler->D = 0.0f;
            sampler->Y = 1.0f;
        }
        return sampler->currentInterval;
    }

    // Operasi normal: hitung interval adaptif
    float avg = 0.0f;
    for(int i=0; i<sampler->historySize; i++) {
    	avg += sampler->history[i];
    }

    avg /= sampler->historySize;

    if(fabsf(avg) < 0.001f) avg = 0.001f;

    sampler->D = fabsf(newValue - sampler->prevValue)/avg;
    sampler->Y = 2.0f/(1 + expf(-5.0*(sampler->D - sampler->t)));

    float newInterval = sampler->currentInterval / sampler->Y;
    newInterval = fmaxf(sampler->minInterval, fminf(newInterval, sampler->maxInterval));

    // Update buffer sirkuler
    sampler->history[sampler->dataIndex % sampler->historySize] = newValue;
    sampler->dataIndex++;
    sampler->prevValue = newValue;

    return newInterval;
}

void globalTimerProcess(Time_t *Global_Time)
{
	Global_Time->millisecond_t++;

	if(Global_Time->millisecond_t > 999)
	{
		Global_Time->second_t++;
		Global_Time->millisecond_t = 0;
	}

	if(Global_Time->second_t > 59)
	{
		Global_Time->minute_t++;
		Global_Time->second_t = 0;
	}

	if(Global_Time->minute_t > 59)
	{
		Global_Time->hour_t++;
		Global_Time->minute_t = 0;
	}
}
