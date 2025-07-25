/*
 * control.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Alfa
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"
#include "MS5837_lib.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define LIMIT_SW1 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)
#define LIMIT_SW2 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)

#define FULL_LENGTH_PULSE 30985
#define HALF_LENGTH_PULSE (FULL_LENGTH_PULSE/2)
#define QRTR_LENGTH_PULSE (FULL_LENGTH_PULSE/4)



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
	float 	setpoint;
	float	feedback;
	float 	error;
	float 	prev_error;
	float	output;
} pid_t ;

typedef struct
{
	GPIO_TypeDef *GPIO_A;
	GPIO_TypeDef *GPIO_B;
	uint16_t GPIO_PIN_A;
	uint16_t GPIO_PIN_B;
	TIM_HandleTypeDef *htimx;
	uint32_t channel;
} motor_t ;

typedef struct
{
	short int start;
	short int reset;
	short int state;
} system_t ;

/******************************************************************************
 * Variable Declarations
 *****************************************************************************/
extern short int lim_sw1_stat;
extern short int lim_sw2_stat;

//--- Pressure Sensor
extern MS5837_t MS5837;

//--- PID
extern short int enc_cnt;
extern long int total_enc_cnt;
extern pid_t motor_pid;
extern pid_t pressure_pid;

//--- Systems
extern system_t pressure_task;
extern system_t test_task;
extern system_t pid_task;
extern system_t main_task;
extern system_t my_task;
extern system_t light_task;

extern int16_t pwm_output;

extern uint8_t adaptive_start;
extern int32_t depth_setpoint;
extern int32_t duration_min;

extern char UART1_RX_BUFFER[12];
/******************************************************************************
 * Function Prototypes
 *****************************************************************************/
void 	initSubmersible();

void 	PID_Init(pid_t *uPID, float KP, float KI, float KD);
float 	PID_Update(pid_t *uPID, float setpoint, float feedback, float maximum_output);

void 	Motor_Init(motor_t *uMotor, GPIO_TypeDef *GPIO_A, uint16_t GPIO_PIN_A,
				GPIO_TypeDef *GPIO_B,  uint16_t GPIO_PIN_B, TIM_HandleTypeDef *htimx, uint32_t channel);
void 	Motor_Run(motor_t *uMotor, short int speed);
void 	Motor_Write(short int motor, short int speed);

int 	Task_Init(system_t *task);
void 	Task_Pressure();
void 	Task_Control_Test_UpDown();
void 	Task_Control_PID();
void 	light_sequence();
void 	my_my_task();
#endif /* INC_CONTROL_H_ */
