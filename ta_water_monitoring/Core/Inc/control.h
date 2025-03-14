/*
 * control.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Alfa
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
//--- Pressure Sensor
#define MS5837_I2C_ADDR 	0x76 << 1 	// MS5837 I2C address
#define MS5837_RESET_CMD 	0x1E      	// Reset command
#define MS5837_PROM_READ 	0xA0      	// PROM read base address
#define MS5837_CONVERT_D1 	0x40     	// Convert D1 command (pressure)
#define MS5837_CONVERT_D2 	0x50     	// Convert D2 command (temperature)
#define MS5837_ADC_READ 	0x00       	// ADC read command


/******************************************************************************
 * Structs
 *****************************************************************************/
//--- PID
typedef struct
{
	float   kp;
	float   ki;
	float   kd;
	float 	proportional;
	float 	integral;
	float 	derivative;
	float 	error;
	float 	prev_error;
} pid_t ;

/******************************************************************************
 * Variable Declarations
 *****************************************************************************/
extern short int test_var;

//--- Pressure Sensor
extern uint8_t current_command;
extern uint16_t prom_coefficients[8]; // PROM coefficients from the sensor
extern uint32_t D1, D2;               // Raw pressure and temperature values
extern uint32_t conversion_start_time;
extern int conversion_in_progress;
extern float pressure;

//--- Systems
extern short int system_start;
extern short int system_reset;
extern short int control_state;
/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void initSubmersible();

void MS5837_Reset(void);
void MS5837_ReadPROM(void);
void MS5837_StartConversion(uint8_t command);
void MS5837_ReadADC(void);
void MS5837_ProcessConversion(void);
float MS5837_CalculatePressure(void);
uint32_t HAL_GetTick(void);

void initPID(float KP, float KI, float KD);
float updatePID(float setpoint, float feedback, float maximum_output);

void writeMotor(int motor, int speed);

void pidControl();

#endif /* INC_CONTROL_H_ */
