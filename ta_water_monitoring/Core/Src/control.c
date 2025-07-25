/*
 * control.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Alfa
 */

#include "control.h"

/******************************************************************************
 * Variable Initialisation
 *****************************************************************************/
short int lim_sw1_stat 		= 1;
short int lim_sw2_stat 		= 1;
uint32_t set_state = 0;
//--- Pressure Sensor
MS5837_t MS5837 			= {0};

//--- PID
short int enc_cnt 			= 0;
long int total_enc_cnt 		= 0;
pid_t motor_pid 			= {0};
pid_t pressure_pid 			= {0};

//--- Systems
system_t pressure_task 	= {0};
system_t test_task 		= {0};
system_t pid_task		= {0};
system_t main_task 		= {0};
system_t my_task		= {0};
system_t light_task		= {0};

int16_t pwm_output = 0;

uint8_t adaptive_start = 0;
int32_t depth_setpoint = -1;
int32_t duration_min = 0;

char UART1_RX_BUFFER[12];
/******************************************************************************
 * Function Definitions
 *****************************************************************************/

void initSubmersible()
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	PID_Init(&motor_pid, 0.6, 0, 0);
	PID_Init(&pressure_pid, 0, 0, 0);

	MS5837_Init(&hi2c1, &MS5837, 50);
}


/*
 * PID Control
 */
void PID_Init(pid_t *uPID, float KP, float KI, float KD)
{
	uPID->kp = KP;
	uPID->ki = KI;
	uPID->kd = KD;
	uPID->proportional = 0;
	uPID->integral = 0;
	uPID->derivative = 0;
	uPID->setpoint = 0;
	uPID->feedback = 0;
	uPID->error = 0;
	uPID->prev_error = 0;
	uPID->output = 0;
}

float PID_Update(pid_t *uPID, float setpoint, float feedback, float maximum_output)
{
	uPID->setpoint = setpoint;
	uPID->feedback = feedback;

	uPID->error = uPID->setpoint - uPID->feedback;

	uPID->proportional = uPID->kp * uPID->error;
	uPID->integral    += uPID->ki * uPID->error;
	uPID->derivative   = uPID->kd * (uPID->error - uPID->prev_error);
	uPID->prev_error   = uPID->error;

	if(uPID->integral >= maximum_output) 			{ uPID->integral =   maximum_output;  }
	else if(uPID->integral < -(maximum_output)) 	{ uPID->integral = -(maximum_output); }

	uPID->output = (uPID->proportional) + (uPID->integral) + (uPID->derivative);

	if(uPID->output >= maximum_output) 			{ uPID->output =   maximum_output;  }
	else if(uPID->output < -(maximum_output)) 	{ uPID->output = -(maximum_output); }

	return uPID->output;
}


void Motor_Init(motor_t *uMotor, GPIO_TypeDef *GPIO_A, uint16_t GPIO_PIN_A,
				GPIO_TypeDef *GPIO_B,  uint16_t GPIO_PIN_B, TIM_HandleTypeDef *htimx, uint32_t channel)
{
	uMotor->GPIO_A = GPIO_A;
	uMotor->GPIO_PIN_A = GPIO_PIN_A;
	uMotor->GPIO_B = GPIO_B;
	uMotor->GPIO_PIN_B = GPIO_PIN_B;
	uMotor->htimx = htimx;
	uMotor->channel = channel;

	HAL_TIM_PWM_Start(uMotor->htimx, uMotor->channel);
}

void Motor_Run(motor_t *uMotor, short int speed)
{
	short int dir_a = (speed >= 0);
	short int dir_b = (speed <  0);
	speed = abs(speed);

	HAL_GPIO_WritePin(uMotor->GPIO_A, uMotor->GPIO_PIN_A, dir_a);
	HAL_GPIO_WritePin(uMotor->GPIO_B, uMotor->GPIO_PIN_B, dir_b);

	switch (uMotor->channel)
	{
		case TIM_CHANNEL_1:
			uMotor->htimx->Instance->CCR1 = speed;
			break;
		case TIM_CHANNEL_2:
			uMotor->htimx->Instance->CCR2 = speed;
			break;
		case TIM_CHANNEL_3:
			uMotor->htimx->Instance->CCR3 = speed;
			break;
		case TIM_CHANNEL_4:
			uMotor->htimx->Instance->CCR4 = speed;
			break;
		default:
			break;
	}
}

void Motor_Write(short int motor, short int speed)
{
	int dir_a = (speed >= 0);
	int dir_b = (speed <  0);
	speed = abs(speed);

	switch(motor)
	{
		case 1:
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, dir_b);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, dir_a);
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

int Task_Init(system_t *task)
{
	if(!task -> start)
	{
		return 0;
	}
	if(task -> reset)
	{
		NVIC_SystemReset();
		task -> reset = 0;
		return 0;
	}

	return 1;
}

void Task_Pressure()
{
//	if(!Task_Init(&pressure_task))
//	{
//		return;
//	}

	static uint32_t pres_time = 0;

	pres_time++;

	if(pres_time >= MS5837.delay_ms)
	{
		MS5832_Process(&hi2c1, &MS5837);
		pres_time = 0;
	}

	return;
}


void Task_Control_Test_UpDown()
{
	if(!Task_Init(&test_task))
	{
		return;
	}

	Task_Pressure();

	static uint32_t counter_ms = 0;

	lim_sw1_stat = LIMIT_SW1;
	lim_sw2_stat = LIMIT_SW2;

	switch(test_task.state)
	{
		  default:
			  Motor_Write(1, 0);
			  break;

		  case 0: //-- go down
			  Motor_Write(1, -300);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);

			  if(lim_sw2_stat == 0)
			  {
				  Motor_Write(1, 0);
				  test_task.state++;
			  }
			  break;

		  case 1: //--- hold position;
			  counter_ms++;
			  if(counter_ms >= 20000)
			  {
				  counter_ms = 0;
				  test_task.state++;
			  }
			  break;

		  case 2:
			  Motor_Write(1, 250); //--- go up
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
			  if(lim_sw1_stat == 0)
			  {
				  Motor_Write(1, 0);
				  test_task.state++;
			  }
			  break;

		  case 3:
			  counter_ms++;
			  if(counter_ms >= 20000)
			  {
				  counter_ms = 0;
				  test_task.state = 0;
			  }
			  break;
	}
}

int32_t motor_set = 0;

void Task_Control_PID()
{
	if(!Task_Init(&pid_task))
	{
		return;
	}

	Task_Pressure();

	lim_sw1_stat = LIMIT_SW1;
	lim_sw2_stat = LIMIT_SW2;

	static uint32_t pid_cnt = 0;
	static uint32_t delay_10ms = 0;

	pid_cnt++;

	if(pid_cnt >= 10)
	{
		switch(pid_task.state)
		{
			default:
				Motor_Write(1, 0);
				break;

			case 0: //-- go down
				Motor_Write(1, -245);

				if(lim_sw2_stat == 0)
				{
					Motor_Write(1, 0);
					pid_task.state++;
				}
				break;

			case 1: // wait bruh
				delay_10ms++;
				if(delay_10ms >= 300)
				{
					TIM1 -> CNT = 0;
					delay_10ms = 0;
					pid_task.state++;
				}
				break;

			case 2: //--- hold position;

				if(fabs(motor_pid.error) < 50 && motor_pid.error != 0)
				{
					adaptive_start = 1;
				}

				delay_10ms++;

				if(delay_10ms >= 4)
				{
					pressure_pid.setpoint = depth_setpoint;
					pressure_pid.kp = 400;
					pressure_pid.ki = 0;
					pressure_pid.kd = 0;
					motor_pid.setpoint = PID_Update(&pressure_pid, pressure_pid.setpoint, MS5837.pressure_mbar, FULL_LENGTH_PULSE - 100);
					delay_10ms = 0;

				}

				enc_cnt = TIM1->CNT;
				total_enc_cnt += (long int)enc_cnt;
				TIM1->CNT = 0;

				pwm_output = (int16_t)(PID_Update(&motor_pid, motor_pid.setpoint, total_enc_cnt, 300));

			    if(lim_sw2_stat == 0)
			    {
			        pwm_output = (pwm_output > 0) ? pwm_output : 0;
			    }
			    else if(lim_sw1_stat == 0)
			    {
			        pwm_output = (pwm_output < 0) ? pwm_output : 0;
			    }


				Motor_Write(1, pwm_output);

				break;

			case 3:
				Motor_Write(1, -245);

				if(lim_sw2_stat == 0)
				{
					Motor_Write(1, 0);
					pid_task.state++;
				}
				break;

		}

		pid_cnt = 0;
	}
}
uint32_t light_light_cnt = 0;

void light_sequence()
{

	if(!Task_Init(&light_task))
	{
		return;
	}

	static uint32_t light_cnt = 0;
	static uint32_t light_global = 0;

	switch(light_task.state)
	{
		case 0:
			if(light_cnt >= 74)
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				light_global++;
				light_cnt = 0;
			}
			light_cnt++;

			if(light_global >= 11)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
				light_global = 0;
				light_task.state++;
			}
			break;
		case 1:
			if(light_cnt >= 99)
			{
				light_task.state ++;
				light_cnt = 0;
			}
			light_cnt++;
			break;
		case 2:
			light_light_cnt++;
			light_task.state = 0;
			break;
	}
}

uint32_t global_cnt = 0;

void my_my_task()
{
	if(my_task.reset)
	{
		NVIC_SystemReset();
		my_task.reset = 0;
	}
//	if(!Task_Init(&my_task))
//	{
//		return;
//	}
//
//	static uint32_t my_cnt = 0;
//
//	if(my_cnt >= 499)
//	{
//		global_cnt ++;
//		my_cnt = 0;
//	}
//	my_cnt++;


}
