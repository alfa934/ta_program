/*
 * sampling.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Evander
 */

#ifndef INC_SAMPLING_H_
#define INC_SAMPLING_H_

#include "main.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_0
// DO
#define VREF 	3300
#define ADC_RES 4095
#define CAL1_V 	400
#define CAL1_T 	23
// Turbidity
#define HISTORY_SIZE 	10
#define MOV_AVER_TURB 	10

extern const float DO_Table[41];

/******************************************************************************
 * Enums
 *****************************************************************************/
//Sensor Suhu
typedef enum
{
  TEMP_STATE_IDLE,
  TEMP_STATE_WAIT_CONVERT,
  TEMP_STATE_READ_TEMP,
  TEMP_STATE_DONE
} TempState_t;

typedef enum
{
	DO_STATE_IDLE,
	DO_STATE_READ_ADC,
	DO_STATE_CALCULATE,
	DO_STATE_DONE
} DOState_t;

typedef enum
{
	TURBID_STATE_IDLE,
	TURBID_STATE_READ_ADC,
	TURBID_STATE_FILTER,
	TURBID_STATE_CALCULATE,
	TURBID_STATE_DONE
} TurbidState_t;

typedef enum{
	PH_STATE_IDLE,
	PH_STATE_REQUEST_DATA,
	PH_STATE_WAIT_RESPONSE,
	PH_STATE_DONE
} PHState_t;

/******************************************************************************
 * Structs
 *****************************************************************************/
typedef struct
{
	uint16_t hour_t;
	uint8_t  minute_t;
	uint8_t  second_t;
	uint16_t millisecond_t;
} Time_t;

//Sensor DO
typedef struct
{
	uint32_t adc_raw;
	float adc_voltage;
	float value;
	float cal_value;
	uint32_t filtered_adc_raw;
} Sensor;

//DDASA
typedef struct {
    float* history;        // Buffer data historis
    int historySize;       // Ukuran buffer
    int dataIndex;         // Indeks sirkuler
    float prevValue;       // Nilai sebelumnya
    float currentInterval; // Interval saat ini (ms)
    float t;               // Threshold perubahan
    float minInterval;     // Interval minimum (ms)
    float maxInterval;     // Interval maksimum (ms)
    uint8_t initialized;
    float D;
    float Y;
} AdaptiveSampler;

/******************************************************************************
 * Declarations
 *****************************************************************************/
extern Time_t Global_Time;

//--- Temperature
extern uint8_t Presence;
extern uint8_t Temp_byte1;
extern uint8_t Temp_byte2;
extern TempState_t temp_state;
extern uint32_t last_temp_tick;
extern float temp_raw;
extern float temp_cal;
//--- Dissolved Oxygen
extern Sensor do_data;
extern DOState_t do_state;
extern uint32_t last_do_tick;
//--- Turbidity
extern Sensor turbid_data;
extern TurbidState_t turbid_state;
extern uint32_t last_turbid_tick;
extern uint32_t turbid_values_buffer[MOV_AVER_TURB]; // Menyimpan ADC raw
extern uint8_t turbid_buffer_index;
extern float turbid_buffer_sum;
extern uint8_t turbid_buffer_count;
//--- pH
extern PHState_t ph_state;
extern uint32_t last_ph_tick;
extern char rx[6];
//extern char tx[8];
extern float ph;
extern float ph_cal;
extern uint8_t response_received;

//--- Adaptive Sampling
extern float tempHistory[HISTORY_SIZE];
extern AdaptiveSampler tempSampler;

extern float doHistory[HISTORY_SIZE];
extern AdaptiveSampler doSampler;

extern float turbidHistory[HISTORY_SIZE];
extern AdaptiveSampler turbidSampler;

extern float pHHistory[HISTORY_SIZE];
extern AdaptiveSampler pHSampler;

extern char transmit[53];
extern uint8_t send_temp;
extern uint8_t send_ph;
extern uint8_t send_do;
extern uint8_t send_turb;

/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void 	Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void 	Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void 	delay(uint32_t us);
uint8_t DS18B20_Start(void);
void 	DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
float 	calculateDO(uint32_t voltage_mv, float current_temp_cal);
void 	initSampler(AdaptiveSampler* sampler, float* historyBuffer, int historySize,
                float threshold, float initInterval, float minI, float maxI);
float 	updateSamplingInterval(AdaptiveSampler* sampler, float newValue);

void 	globalTimerProcess(Time_t *Global_Time);

#endif /* INC_SAMPLING_H_ */
