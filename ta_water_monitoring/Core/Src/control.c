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
short int control_state = 0;

//--- pid
pid_t motor_pid = {0};
pid_t position_pid = {0};
short int encoder_cnt = 0;
short int encoder_setpoint = 0;



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

/*
 * Pressure Sensor
 */
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

/*
 * PID Control
 */
void initPID(pid_t *uPID, float KP, float KI, float KD)
{
	uPID->kp = KP;
	uPID->ki = KI;
	uPID->kd = KD;
	uPID->proportional = 0;
	uPID->integral = 0;
	uPID->derivative = 0;
	uPID->error = 0;
	uPID->prev_error = 0;
}

float updatePID(pid_t *uPID, float setpoint, float feedback, float maximum_output)
{
	uPID->error = setpoint - feedback;

	uPID->proportional = uPID->kp * uPID->error;
	uPID->integral    += uPID->ki * uPID->error;
	uPID->derivative   = uPID->kd * (uPID->error - uPID->prev_error);
	uPID->prev_error   = uPID->error;

	if(uPID->integral >= maximum_output) 			{ uPID->integral =   maximum_output;  }
	else if(uPID->integral < -(maximum_output)) 	{ uPID->integral = -(maximum_output); }

	float output = (uPID->proportional) + (uPID->integral) + (uPID->derivative);

	if(output >= maximum_output) 			{ output =   maximum_output; }
	else if(output < -(maximum_output)) 	{ output = -(maximum_output); }

	return output;
}

void pidControl()
{
	encoder_cnt = TIM1->CCR1;
	TIM1->CCR1;

	short int pwm_output = (short int)(updatePID(&motor_pid, encoder_setpoint, encoder_cnt, 300));

	writeMotor(1, pwm_output);

}



void writeMotor(short int motor, short int speed)
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
//if (!conversion_in_progress)
//{
//    MS5837_StartConversion(MS5837_CONVERT_D1);
//}
//
//MS5837_ProcessConversion();
//
//if (!conversion_in_progress && D1 != 0 && D2 != 0)
//{
//    pressure = MS5837_CalculatePressure();
//}
