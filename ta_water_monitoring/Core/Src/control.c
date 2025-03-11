/*
 * control.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Alfa
 */

#include "control.h"

//I2C_HandleTypeDef hi2c1;

/******************************************************************************
 * Variable Initialisation
 *****************************************************************************/
short int test_var = 0;

//--- Pressure Sensor
uint8_t current_command;
uint16_t prom_coefficients[8]; // PROM coefficients from the sensor
uint32_t D1, D2;               // Raw pressure and temperature values
uint32_t conversion_start_time = 0;
int conversion_in_progress = 0;
float pressure;

//--- Systems
short int system_start = 0;
short int system_reset = 0;


/******************************************************************************
 * Function Definitions
 *****************************************************************************/

void initSubmersible()
{
	while(1)
	{
		if(system_start)
		{
			break;
		}
	}
}


void MS5837_Reset(void)
{
    uint8_t cmd = MS5837_RESET_CMD;
    HAL_I2C_Master_Transmit(&hi2c1, MS5837_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10); // Wait for reset to complete
}

void MS5837_ReadPROM(void)
{
    uint8_t cmd;
    uint8_t data[2];

    for (int i = 0; i < 8; i++)
    {
        cmd = MS5837_PROM_READ + (i * 2);
        HAL_I2C_Master_Transmit(&hi2c1, MS5837_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, MS5837_I2C_ADDR, data, 2, HAL_MAX_DELAY);
        prom_coefficients[i] = (data[0] << 8) | data[1];
    }
}

void MS5837_StartConversion(uint8_t command)
{
    current_command = command;
    conversion_in_progress = 1;
    conversion_start_time = HAL_GetTick();
    HAL_I2C_Master_Transmit(&hi2c1, MS5837_I2C_ADDR, &command, 1, HAL_MAX_DELAY);
}

void MS5837_ReadADC(void)
{
    uint8_t cmd = MS5837_ADC_READ;
    uint8_t data[3];

    HAL_I2C_Master_Transmit(&hi2c1, MS5837_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, MS5837_I2C_ADDR, data, 3, HAL_MAX_DELAY);

    uint32_t adc_value = (data[0] << 16) | (data[1] << 8) | data[2];

    if (current_command == MS5837_CONVERT_D1) {
        D1 = adc_value;
    } else if (current_command == MS5837_CONVERT_D2) {
        D2 = adc_value;
        conversion_in_progress = 0;
    }
}

void MS5837_ProcessConversion(void)
{
    if (conversion_in_progress && (HAL_GetTick() - conversion_start_time >= 10)) {
        MS5837_ReadADC();
        if (current_command == MS5837_CONVERT_D1) {
            MS5837_StartConversion(MS5837_CONVERT_D2);
        }
    }
}

float MS5837_CalculatePressure(void)
{
    int32_t dT = D2 - ((int32_t)prom_coefficients[5] << 8);
//    int32_t TEMP = 2000 + ((int64_t)dT * prom_coefficients[6]) / 8388608;

    int64_t OFF = ((int64_t)prom_coefficients[2] << 17) + (((int64_t)dT * prom_coefficients[4]) / 64);
    int64_t SENS = ((int64_t)prom_coefficients[1] << 16) + (((int64_t)dT * prom_coefficients[3]) / 128);

    int32_t P = (((D1 * SENS) / 2097152) - OFF) / 8192;

    return P / 10.0f; // Return pressure in mbar
}


void initPID(float KP, float KI, float KD)
{
	kp = KP;
	ki = KI;
	kd = KD;
}

float updatePID(float setpoint, float feedback, float maximum_output)
{
	error = setpoint - feedback;

	proportional = kp * error;
	integral    += ki * error;
	derivative   = kd * (error - prev_error);
	prev_error   = error;

	if(integral >= maximum_output) 			{ integral =   maximum_output; }
	else if(integral < -(maximum_output)) 	{ integral = -(maximum_output); }

	float output = (proportional) + (integral) + (derivative);

	if(output >= maximum_output) 			{ output =   maximum_output; }
	else if(output < -(maximum_output)) 	{ output = -(maximum_output); }

	return output;
}


void writeMotor(int motor, int speed)
{
	int dir_a = (speed >= 0);
	int dir_b = (speed <  0);
	speed = abs(speed);

	switch(motor)
	{
		case 1:
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, dir_a);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, dir_b);
			TIM2->CCR1 = speed;
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, dir_a);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, dir_b);
			TIM2->CCR2 = speed;
			break;
		}
		default:
		{
			break;
		}

	}

}
