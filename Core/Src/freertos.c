/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include "stdlib.h"
//#include "Accel_stepper.h"
#include "S_curve7_pro.h"
#include "Inverse_Arm.h"
#include "MY_FLASH.h"
#include "cli.h"
#include "Arm_CAN.h"
//#include "S_curve7_pro.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
	float pos_x;
	float pos_y;
	float pos_z;
}Arm_pos_t;

//Acceleration_t Stepper1;
//Acceleration_t Stepper2;
//Acceleration_t Stepper3;
//Acceleration_t Stepper4;
//Acceleration_t Stepper5;
//Acceleration_t Stepper6;

S_curve_t Stepper1;
S_curve_t Stepper2;
S_curve_t Stepper3;
S_curve_t Stepper4;
S_curve_t Stepper5;
S_curve_t Stepper6;

Inv_Arm_t Arm;
Arm_pos_t Arm_pos;
Arm_pos_t current_pos;
Memory_t memory;
Memory_t rxmemory;
CanMessage_t rxmsg;
extern UART_HandleTypeDef huart4;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define max(a,b) (a > b) ? a : b;
#define min(a,b) (a > b) ? b : a;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//=================================ARM=========================================//
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;
float theta4 = 0;
float theta5 = 0;
float theta6 = 0;
int set_theta1 = 0;
int set_theta2 = 0;
int set_theta3 = 0;
int set_theta4 = 0;
int set_theta5 = 0;
int set_theta6 = 0;
char uart_msg[100];
char uart_buff[100];
char msg_cmd[20];
extern char rx_data;
uint8_t freerun = 0;
uint8_t input_mode = 0;
uint8_t Lsw1 = 1;
uint8_t Lsw2 = 1;
uint8_t Lsw3 = 1;
uint8_t Lsw4 = 1;
uint8_t Lsw5 = 1;
uint8_t Lsw6 = 1;
uint16_t theta1dot = 0;
uint16_t theta2dot = 0;
uint16_t theta3dot = 0;
uint16_t theta4dot = 0;
uint16_t theta5dot = 0;
uint16_t theta6dot = 0;
uint16_t max_speed = 200;
uint16_t accel1 = 0;
uint16_t accel2 = 0;
uint16_t accel3 = 0;
uint16_t accel4 = 0;
uint16_t accel5 = 0;
uint16_t accel6 = 0;
uint16_t a_max = 10;
uint16_t j_max = 20;
uint16_t jerk1,jerk2,jerk3,jerk4,jerk5,jerk6;

uint32_t step1 = 0;
uint32_t step2 = 0;
uint32_t step3 = 0;
uint32_t step4 = 0;
uint32_t step5 = 0;
uint32_t step6 = 0;
//float Theta = 0;
//float Theta_mesure = 0;
float Speed_generate[6] = {0};
//float Speed_measure = 0;
//long enc_cnt =0;
//long old_cnt =0;
uint32_t old_delay = 0;
//
//float speed_error = 0;
//float pos_error = 0;
//signed long encoder(int i, uint8_t nowA, uint8_t nowB){
//	static uint8_t lastA[4] = {0};
//	static uint8_t lastB[4] = {0};
//	static signed long cnt[4] = {0};
//	if(nowA != lastA[i]){
//		lastA[i] = nowA;
//		if(lastA[i] == 0){
//			if(nowB == 0){
//				cnt[i]--;
//			}else{
//				cnt[i]++;
//			}
//		}else{
//			if(nowB == 1){
//				cnt[i]--;
//			}else{
//				cnt[i]++;
//			}
//		}
//	}
//	if(nowB != lastB[i]){
//		lastB[i] = nowB;
//		if(lastB[i] == 0){
//			if(nowA == 1){
//				cnt[i]--;
//			}else{
//				cnt[i]++;
//			}
//		}else{
//			if(nowA == 0){
//				cnt[i]--;
//			}else{
//				cnt[i]++;
//			}
//		}
//	}
//	return cnt[i];
//}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//
//  if(GPIO_Pin == ENCA_Pin||ENCB_Pin){//ROTARY_Y_A
//	  enc_cnt = encoder(0, HAL_GPIO_ReadPin(ENCA_GPIO_Port, ENCA_Pin), HAL_GPIO_ReadPin(ENCB_GPIO_Port, ENCB_Pin));
//  }
//}

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartMessages */
osMessageQueueId_t UartMessagesHandle;
const osMessageQueueAttr_t UartMessages_attributes = {
  .name = "UartMessages"
};
/* Definitions for CANRxBuffer */
osMessageQueueId_t CANRxBufferHandle;
const osMessageQueueAttr_t CANRxBuffer_attributes = {
  .name = "CANRxBuffer"
};
/* Definitions for UartCircleTimer */
osTimerId_t UartCircleTimerHandle;
const osTimerAttr_t UartCircleTimer_attributes = {
  .name = "UartCircleTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t UartSendHandle;
const osThreadAttr_t UartSend_attributes = {
  .name = "UartSend",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osMessageQueueId_t UartCommandHandle;
const osMessageQueueAttr_t UartCommand_attributes = {
  .name = "UartCommand"
};
osMessageQueueId_t FreeRunHandle;
const osMessageQueueAttr_t FreeRun_attributes = {
  .name = "FreeRun"
};

void read_cmd(char* s)
{
//    char a[20];
//    strcpy(a,"theta(4,3,2)");
//    char* cmd = strtok(a, "(");
//    if(cmd!=NULL){
//        if(strcmp(cmd, "theta")==0){
//            char *arg = strtok(NULL, ",");
//            char *arg1 = strtok(NULL, ",");
//            printf("%s", arg1);
//        }
//    }
    char* cmd = strtok(s, " ");// read the first argument
    if (cmd != NULL) {
        if (strcmp(cmd, "theta") == 0) {
        	input_mode = 0;
        	Inv_Arm_t Theta;
            char* arg1 = strtok(NULL, " "); // read theta1
            char* arg2 = strtok(NULL, " "); // read theta2
            char* arg3 = strtok(NULL, " "); // read theta3
            char* arg4 = strtok(NULL, " "); // read theta4
            char* arg5 = strtok(NULL, " "); // read theta5
            char* arg6 = strtok(NULL, " "); // read theta6
            Theta.theta1 = atof(arg1);// convert arg from ascii to integer
            Theta.theta2 = atof(arg2);
            Theta.theta3 = atof(arg3);
            Theta.theta4 = atof(arg4);
            Theta.theta5 = atof(arg5);
            Theta.theta6 = atof(arg6);
            osMessageQueuePut(UartMessagesHandle, &Theta, 0, 0);
        }
        else if (strcmp(cmd, "xyz") == 0) {

        	Arm_pos_t setpos;
        	input_mode = 1;
            char* arg0 = strtok(NULL, " "); // read the string after x
            char* arg1 = strtok(NULL, " "); // read the string after x
            char* arg2 = strtok(NULL, " "); // read the string after x

            /* convert arg from ascii to integer */
            setpos.pos_x = atof(arg0);
            setpos.pos_y = atof(arg1);
            setpos.pos_z = atof(arg2);
            osMessageQueuePut(UartMessagesHandle, &setpos, 0, 0);
        }else if(strcmp(cmd, "save_config") == 0){
        	char msg[20] = "save";
        	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
        }else if(strcmp(cmd, "set_amax") == 0){
        	char* arg = strtok(NULL, ")"); // read the string after x
        	memory.a_max = atoi(arg);
        }else if(strcmp(cmd, "set_vmax") == 0){
        	char* arg = strtok(NULL, ")"); // read the string after x
        	memory.v_max = atoi(arg);
        }else if(strcmp(cmd, "set_jmax") == 0){
        	char* arg = strtok(NULL, ")"); // read the string after x
        	memory.j_max = atoi(arg);
        }else if(strcmp(cmd, "set_traj") == 0){
        	char* arg = strtok(NULL, ")"); // read the string after x
        	memory.traj_mode = atoi(arg);
        }
//        else if (strcmp(cmd, "print") == 0)
//        {
//            uint8_t temp = 1;
//            arg = strtok(NULL, "\"");//in order to read char between ""
//        }
        else {
            printf("Command not recognized.\n");
        }
    }
}
int max6(int a, int b, int c, int d, int e, int f)
{
	int n1 = max(a, b);
	int n2 = max(c, d);
	int n3 = max(e, f);
	int n4 = max(n1, n2);
    return max(n3, n4);
}
void Limit_switch(){
//	Lsw1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);//CW -
//	Lsw2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);//CCW +
//	Lsw3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);//CCW +
//	Lsw4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);//CCW +
	 Lsw1 = HAL_GPIO_ReadPin(Ls1_GPIO_Port, Ls1_Pin);
	 Lsw2 = HAL_GPIO_ReadPin(Ls2_GPIO_Port, Ls2_Pin);
	 Lsw3 = HAL_GPIO_ReadPin(Ls3_GPIO_Port, Ls3_Pin);
	 Lsw4 = HAL_GPIO_ReadPin(Ls4_GPIO_Port, Ls4_Pin);
	 Lsw5 = HAL_GPIO_ReadPin(Ls5_GPIO_Port, Ls5_Pin);
	 Lsw6 = HAL_GPIO_ReadPin(Ls6_GPIO_Port, Ls6_Pin);
}
signed int angle2step(float angle, int ratio){
	return angle*SPR*ratio/360;//20=2*gearratio
}
signed int rad2step(float rads, int gear){
//	rads*SPR*10/(2*pi)=rads/ALPHA
	return rads*gear/ALPHA;//20=2*gearratio
}
void DelayUs_step(uint32_t us)
{
	HAL_TIM_Base_Start_IT(&htim14);
	//(&htim7)->Instance->CNT = (0);
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	while(__HAL_TIM_GET_COUNTER(&htim14) < us);
	HAL_TIM_Base_Stop_IT(&htim14);
}
void HOME(void){
	static uint8_t tmp = 0;
	HAL_GPIO_WritePin(dir_1_GPIO_Port, dir_1_Pin, 0);//CW -
	HAL_GPIO_WritePin(dir_2_GPIO_Port, dir_2_Pin, 1);//CCW +
	HAL_GPIO_WritePin(dir_3_GPIO_Port, dir_3_Pin, 0);//CW -
	HAL_GPIO_WritePin(dir_4_GPIO_Port, dir_4_Pin, 1);//CCW +
	HAL_GPIO_WritePin(dir_5_GPIO_Port, dir_5_Pin, 1);//CCW +
	HAL_GPIO_WritePin(dir_6_GPIO_Port, dir_6_Pin, 1);//CCW +
	while(Lsw2!=0 || Lsw3!=0 || (Lsw4!=0 && tmp != 1) || Lsw5!=0){//
		Limit_switch();
		if(Lsw2 != 0){
			HAL_GPIO_TogglePin(step_2_GPIO_Port, step_2_Pin);
		}
		if(Lsw3 != 0){
			HAL_GPIO_TogglePin(step_3_GPIO_Port, step_3_Pin);
		}
		if(Lsw4 != 0 && tmp != 1){
			HAL_GPIO_TogglePin(step_4_GPIO_Port, step_4_Pin);
		}else{

			if(tmp == 0){
				tmp = 1;
//				Accel_Stepper_Move(&Stepper4, angle2step(-40, 14), 1000, 1000, 500);//linear accel|decel * 10
				Accel_Stepper_Move(&Stepper4, angle2step(-45, 14), 10, 10, 800);//s-curve
			}
		}
		if(Lsw5!=0){
			HAL_GPIO_TogglePin(step_5_GPIO_Port, step_5_Pin);
		}
		DelayUs_step(400);
	}
	while(Lsw1!=0){//
			Limit_switch();
			if(Lsw1!=0){
				HAL_GPIO_TogglePin(step_1_GPIO_Port, step_1_Pin);
			}
			DelayUs_step(800);
	}

//	theta1 = 0.087266;
//	theta2 = 1.919862;
//	theta3 = -1.466076;
//	theta4 = 1.483529;
//	Manual position
//	step_cnt1 = 0;
//	step_cnt2 = 0;
//	step_cnt3 = 0;
//	step_cnt4 = 0;
//	step_cnt5 = 0;
}
void MoveTheta(int step1, int step2, int step3, int step4, int step5, int step6){
	int step_max = max6(abs(step1), abs(step2), abs(step3), abs(step4), abs(step5), abs(step6));
	float coef1 = fabs(step1)/step_max;
	float coef2 = fabs(step2)/step_max;
	float coef3 = fabs(step3)/step_max;
	float coef4 = fabs(step4)/step_max;
	float coef5 = fabs(step5)/step_max;
	float coef6 = fabs(step6)/step_max;
	theta1dot = max_speed * coef1;
	theta2dot = max_speed * coef2;
	theta3dot = max_speed * coef3;
	theta4dot = max_speed * coef4;
	theta5dot = max_speed * coef5;
	theta6dot = max_speed * coef6;
	accel1 = a_max * coef1;
	accel2 = a_max * coef2;
	accel3 = a_max * coef3;
	accel4 = a_max * coef4;
	accel5 = a_max * coef5;
	accel6 = a_max * coef6;

	jerk1 = j_max * coef1;
	jerk2 = j_max * coef2;
	jerk3 = j_max * coef3;
	jerk4 = j_max * coef4;
	jerk5 = j_max * coef5;
	jerk6 = j_max * coef6;
}
void MoveToPos(float x, float y, float z){
	cal_theta(x, y, z);
	set_theta1 = rad2step(Arm.theta1 - theta1, 10);
	set_theta2 = rad2step(Arm.theta2 - theta2, 20);
	set_theta3 = -1.0* rad2step(fabs(theta3 - Arm.theta3 -(Arm.theta2 - theta2)), 20);//in quadrant IV
	set_theta4 = rad2step(Arm.theta4 - theta4, 14);
	int step_max = max6(abs(set_theta1), abs(set_theta2), abs(set_theta3),abs(set_theta4), abs(set_theta5), abs(set_theta6));
	float coef1 = fabs(set_theta1)/step_max;
	float coef2 = fabs(set_theta2)/step_max;
	float coef3 = fabs(set_theta3)/step_max;
	float coef4 = fabs(set_theta4)/step_max;
	float coef5 = fabs(set_theta4)/step_max;
	float coef6 = fabs(set_theta6)/step_max;
	theta1dot = max_speed * coef1;
	theta2dot = max_speed * coef2;
	theta3dot = max_speed * coef3;
	theta4dot = max_speed * coef4;
	theta5dot = max_speed * coef5;
	theta6dot = max_speed * coef6;
	accel1 = a_max * coef1;
	accel2 = a_max * coef2;
	accel3 = a_max * coef3;
	accel4 = a_max * coef4;
	accel5 = a_max * coef5;
	accel6 = a_max * coef6;
}
void StartUartSend(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUartTask(void *argument);
void StartCANTask(void *argument);
void UartCircleCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of UartCircleTimer */
  UartCircleTimerHandle = osTimerNew(UartCircleCallback, osTimerPeriodic, NULL, &UartCircleTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(UartCircleTimerHandle, 10);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UartMessages */
  UartMessagesHandle = osMessageQueueNew (16, sizeof(Inv_Arm_t), &UartMessages_attributes);

  /* creation of CANRxBuffer */
  CANRxBufferHandle = osMessageQueueNew (8, sizeof(CanMessage_t), &CANRxBuffer_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  UartCommandHandle = osMessageQueueNew (16, sizeof(msg_cmd), &UartCommand_attributes);
  FreeRunHandle = osMessageQueueNew (16, sizeof(msg_cmd), &FreeRun_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew(StartCANTask, NULL, &CANTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  UartSendHandle = osThreadNew(StartUartSend, NULL, &UartSend_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	  MY_FLASH_SetSectorAddrs(11, 0x080E5000);
	  MY_FLASH_ReadN(0, &memory, sizeof(memory)/4, DATA_TYPE_32);
	  j_max = memory.j_max;
	  a_max = memory.a_max;
	  max_speed = memory.v_max;
	  vTaskDelay(pdMS_TO_TICKS(200));
//	static uint8_t tmp = 0;
	  Accel_Stepper_SetPin(&Stepper1, step_1_GPIO_Port, step_1_Pin, dir_1_GPIO_Port, dir_1_Pin);
	  Accel_Stepper_SetPin(&Stepper2, step_2_GPIO_Port, step_2_Pin, dir_2_GPIO_Port, dir_2_Pin);
	  Accel_Stepper_SetPin(&Stepper3, step_3_GPIO_Port, step_3_Pin, dir_3_GPIO_Port, dir_3_Pin);
	  Accel_Stepper_SetPin(&Stepper4, step_4_GPIO_Port, step_4_Pin, dir_4_GPIO_Port, dir_4_Pin);
	  Accel_Stepper_SetPin(&Stepper5, step_5_GPIO_Port, step_5_Pin, dir_5_GPIO_Port, dir_5_Pin);
	  Accel_Stepper_SetPin(&Stepper6, step_6_GPIO_Port, step_6_Pin, dir_6_GPIO_Port, dir_6_Pin);
	  Accel_Stepper_SetTimer(&Stepper1, &htim7);
	  Accel_Stepper_SetTimer(&Stepper2, &htim9);
	  Accel_Stepper_SetTimer(&Stepper3, &htim10);
	  Accel_Stepper_SetTimer(&Stepper4, &htim11);
	  Accel_Stepper_SetTimer(&Stepper5, &htim12);
	  Accel_Stepper_SetTimer(&Stepper6, &htim13);

	  vTaskDelay(pdMS_TO_TICKS(2000));
	  HAL_GPIO_WritePin(en_4_GPIO_Port, en_4_Pin, 1);
	  HAL_GPIO_WritePin(en_5_GPIO_Port, en_5_Pin, 1);
	  HAL_GPIO_WritePin(en_6_GPIO_Port, en_6_Pin, 1);
//	  for(int i=0;i<64000;i++){
////		  HAL_GPIO_TogglePin(step_1_GPIO_Port, step_1_Pin);
////		  HAL_GPIO_WritePin(dir_1_GPIO_Port, dir_1_Pin, 1);
////		  HAL_GPIO_TogglePin(step_2_GPIO_Port, step_2_Pin);
////		  HAL_GPIO_WritePin(dir_2_GPIO_Port, dir_2_Pin, 1);
////		  HAL_GPIO_TogglePin(step_3_GPIO_Port, step_3_Pin);
////		  HAL_GPIO_WritePin(dir_3_GPIO_Port, dir_3_Pin, 1);
////		  HAL_GPIO_TogglePin(step_4_GPIO_Port, step_4_Pin);
////		  HAL_GPIO_WritePin(dir_4_GPIO_Port, dir_4_Pin, 1);
////		  HAL_GPIO_TogglePin(step_5_GPIO_Port, step_5_Pin);
////		  HAL_GPIO_WritePin(dir_5_GPIO_Port, dir_5_Pin, 1);
//		  HAL_GPIO_TogglePin(step_3_GPIO_Port, step_3_Pin);
////		  HAL_GPIO_WritePin(dir_4_GPIO_Port, dir_6_Pin, 1);
////		  vTaskDelay(pdMS_TO_TICKS(1000));
//		  DelayUs_step(400);
//	  }
	  HOME();
//	  HAL_GPIO_WritePin(step_6_GPIO_Port, step_6_Pin, 1);
//	  MoveToPos(500, 500, 0);//xyz[mm]
//	  HAL_Delay(100);
//	  Accel_Stepper_Move(&Stepper6, angle2step(-90, 14), 10, 10, 500);
//	  Accel_Stepper_Move(&Stepper1, set_theta1, accel1, accel1, theta1dot);
	  Accel_Stepper_Move(&Stepper2, angle2step(-90, 20), 10, 10, 500);
	  Accel_Stepper_Move(&Stepper3, angle2step(90, 20), 10, 10, 500);
	  Accel_Stepper_Move(&Stepper5, angle2step(-90, 14), 10, 10, 500);
	  //	Initial angle
	  	theta1 = 90;
	  	theta2 = 90;
	  	theta3 = -90;
	  	theta4 = 0;
	  	theta5 = 0;
	  	Arm.theta1 = 90;
	  	Arm.theta2 = 90;
	  	Arm.theta3 = -90;
	  	Arm.theta4 = 0;
	  	Arm.theta5 = 0;
	  	Arm.theta6 = 0;

	  	vTaskDelay(pdMS_TO_TICKS(1000));
	  	freerun = 1;
  /* Infinite loop */
  for(;;)
  {

//	  if(input_mode == 0){
		  osMessageQueueGet(UartMessagesHandle, &Arm, NULL, osWaitForever);
		  set_theta1 = angle2step(Arm.theta1 - theta1, 5);
		  set_theta2 = angle2step(Arm.theta2 - theta2, 20);
		  set_theta3 = angle2step(Arm.theta3 - theta3, 20);
		  set_theta4 = angle2step(Arm.theta4 - theta4, 14);
		  set_theta5 = angle2step(Arm.theta5 - theta5, 14);
		  set_theta6 = angle2step(Arm.theta6 - theta6, 14);
		  MoveTheta(set_theta1, set_theta2, set_theta3, set_theta4, set_theta5, set_theta6);
//	  	  input_mode = 2;
//	  }
//	  else if(input_mode == 1){
//		  osMessageQueueGet(UartMessagesHandle, &Arm_pos, NULL, osWaitForever);
//		  MoveToPos(Arm_pos.pos_x, Arm_pos.pos_y, Arm_pos.pos_z);
//	  	  input_mode = 2;
//	  }

//	  if(Stepper1.run_status != 1){
//		  Accel_Stepper_Move(&Stepper1, set_theta1, accel1, accel1, theta1dot);
//	  }
//	  if(Stepper2.run_status != 1){
//		  Accel_Stepper_Move(&Stepper2, set_theta2, accel2, accel2, theta2dot);
//	  }
//	  if(Stepper3.run_status != 1){
//		  Accel_Stepper_Move(&Stepper3, set_theta3, accel3, accel3, theta3dot);
//	  }
//	  if(Stepper4.run_status != 1){
//		  Accel_Stepper_Move(&Stepper4, set_theta4, accel4, accel4, theta4dot);
//	  }
//	  if(Stepper5.run_status != 1){
//		  Accel_Stepper_Move(&Stepper5, set_theta5, accel5, accel5, theta5dot);
//	  }
//	  if(Stepper6.run_status != 1){
//		  Accel_Stepper_Move(&Stepper6, set_theta6, accel6, accel6, theta6dot);
//	  }
	  if(Stepper1.run_state != 1 && Stepper2.run_state != 1 && Stepper3.run_state != 1
			  && Stepper4.run_state != 1 && Stepper5.run_state != 1 && Stepper6.run_state != 1){//&& Stepper3.run_status != 1 && Stepper4.run_status != 1 && Stepper5.run_status != 1
//		  tmp = 0;
//		  switch(tmp){
//		  case 0:
//			 Arm.theta2 = 45; //180->0
//			 Arm.theta3 = -30;//-180->0
//			 Arm.theta4 = -90;//+-360
//			 Arm.theta5 = 30; //-90->90
//			 break;
//		  case 1:
//			  Arm.theta2 = 60;
//			  Arm.theta3 = -90;
//			  Arm.theta4 = 0;
//			  Arm.theta5 = 10;
//			 break;
//		  case 2:
//			  Arm.theta2 = 90;
//			  Arm.theta3 = -60;
//			  Arm.theta4 = 45;
//			  Arm.theta5 = -80;
//			 break;
//		  default:
//			  break;
//		  }
//		  set_theta2 = angle2step(Arm.theta2 - theta2, 20);//quardran I&II (+) Arm.theta2 - theta2
//		  set_theta3 = angle2step(Arm.theta3 - theta3, 20);//quardean III&IV (-)
//		  set_theta4 = angle2step(Arm.theta4 - theta4, 14);
//		  set_theta5 = angle2step(Arm.theta5 - theta5, 14);
//
//		  tmp ++;
//		  if(tmp>2) tmp = 0;
//		  Accel_Stepper_Move(&Stepper1, set_theta1, accel1, accel1, theta1dot);
//		  Accel_Stepper_Move(&Stepper2, set_theta2, accel2, accel2, theta2dot);
//		  Accel_Stepper_Move(&Stepper3, set_theta3, accel3, accel3, theta3dot);
//		  Accel_Stepper_Move(&Stepper4, set_theta4, accel4, accel4, theta4dot);
//		  Accel_Stepper_Move(&Stepper5, set_theta5, accel5, accel5, theta5dot);
//		  Accel_Stepper_Move(&Stepper6, set_theta6, accel6, accel6, theta6dot);

		  Accel_Stepper_Move(&Stepper1, set_theta1, accel1, jerk1, theta1dot);
		  Accel_Stepper_Move(&Stepper2, set_theta2, accel2, jerk2, theta2dot);
		  Accel_Stepper_Move(&Stepper3, set_theta3, accel3, jerk3, theta3dot);
		  Accel_Stepper_Move(&Stepper4, set_theta4, accel4, jerk4, theta4dot);
		  Accel_Stepper_Move(&Stepper5, set_theta5, accel5, jerk5, theta5dot);
		  Accel_Stepper_Move(&Stepper6, set_theta6, accel6, jerk6, theta6dot);
		  theta1 = Arm.theta1;
		  theta2 = Arm.theta2;
		  theta3 = Arm.theta3;
		  theta4 = Arm.theta4;
		  theta5 = Arm.theta5;
		  theta6 = Arm.theta6;
	  }
//	  if(Stepper1.run_status != 1){
//		  Accel_Stepper_Move(&Stepper1, set_theta1, 1000, 1000, 500);
//	  }
//	  if(Stepper2.run_status != 1){
//		  Accel_Stepper_Move(&Stepper2, set_theta2, 1000, 1000, 500);
//	  }
//	  if(Stepper3.run_status != 1){
//		  Accel_Stepper_Move(&Stepper3, set_theta3, 1000, 1000, 500);
//	  }
//	  if(Stepper4.run_status != 1){
//		  Accel_Stepper_Move(&Stepper4, set_theta4, 1000, 1000, 500);
//	  }
//	  if(Stepper5.run_status != 1){
//		  Accel_Stepper_Move(&Stepper5, set_theta5, 1000, 1000, 500);
//	  }
//	  if(Stepper6.run_status != 1){
//		  Accel_Stepper_Move(&Stepper6, set_theta6, 1000, 1000, 500);
//	  }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
//	char uart_buff[200];
//	Stepper1.arr = 1000;
//	Stepper6.arr = 1000;
	uint8_t cnt_arm = 0;

  /* Infinite loop */
  for(;;)
  {
	  if(Stepper1.run_state != 1 && Stepper2.run_state != 1 && Stepper3.run_state != 1
			  && Stepper4.run_state != 1 && Stepper5.run_state != 1 && Stepper6.run_state != 1){//&& Stepper3.run_status != 1 && Stepper4.run_status != 1 && Stepper5.run_status != 1
		  if(freerun == 1){
			  switch(cnt_arm){
			  case 0:
				  Arm.theta1 = 45;
				  Arm.theta2 = 45;
				  Arm.theta3 = -90;
				  Arm.theta4 = 0;
				  Arm.theta5 = 45;
				  Arm.theta6 = -60;
				  break;
			  case 1:
				  Arm.theta1 = 100;
				  Arm.theta2 = 30;
				  Arm.theta3 = -100;
				  Arm.theta4 = -45;
				  Arm.theta5 = 45;
				  Arm.theta6 = 90;
				  break;
			  case 2:
				  Arm.theta1 = 90;
				  Arm.theta2 = 90;
				  Arm.theta3 = -45;
				  Arm.theta4 = 0;
				  Arm.theta5 = -45;
				  Arm.theta6 = 45;
				  break;
			  case 3:
				  Arm.theta1 = 30;
				  Arm.theta2 = 80;
				  Arm.theta3 = -120;
				  Arm.theta4 = 0;
				  Arm.theta5 = 0;
				  Arm.theta6 = 0;
				  break;
			  case 4:
				  Arm.theta1 = 150;
				  Arm.theta2 = 130;
				  Arm.theta3 = -130;
				  Arm.theta4 = 90;
				  Arm.theta5 = 45;
				  Arm.theta6 = 0;
				  break;
			  }
			  set_theta1 = angle2step(Arm.theta1 - theta1, 5);
			  set_theta2 = angle2step(Arm.theta2 - theta2, 20);
			  set_theta3 = angle2step(Arm.theta3 - theta3, 20);
			  set_theta4 = angle2step(Arm.theta4 - theta4, 14);
			  set_theta5 = angle2step(Arm.theta5 - theta5, 14);
			  set_theta6 = angle2step(Arm.theta6 - theta6, 14);
			  vTaskDelay(pdMS_TO_TICKS(500));
			  MoveTheta(set_theta1, set_theta2, set_theta3, set_theta4, set_theta5, set_theta6);
			  Accel_Stepper_Move(&Stepper1, set_theta1, accel1, jerk1, theta1dot);
			  Accel_Stepper_Move(&Stepper2, set_theta2, accel2, jerk2, theta2dot);
			  Accel_Stepper_Move(&Stepper3, set_theta3, accel3, jerk3, theta3dot);
			  Accel_Stepper_Move(&Stepper4, set_theta4, accel4, jerk4, theta4dot);
			  Accel_Stepper_Move(&Stepper5, set_theta5, accel5, jerk5, theta5dot);
			  Accel_Stepper_Move(&Stepper6, set_theta6, accel6, jerk6, theta6dot);
			  theta1 = Arm.theta1;
			  theta2 = Arm.theta2;
			  theta3 = Arm.theta3;
			  theta4 = Arm.theta4;
			  theta5 = Arm.theta5;
			  theta6 = Arm.theta6;
		  }

		  cnt_arm = cnt_arm + 1;
		  if(cnt_arm > 4) cnt_arm = 0;
	  }
//	  Limit_switch();
//	  HAL_UART_Receive_IT(&huart4, (uint8_t*) uart_buff, sizeof(uart_buff));
//	  read_cmd(uart_buff);
//	  Speed_generate[0] = (ALPHA)/Stepper1.arr/2;
//	  Speed_generate[1] = (ALPHA)/Stepper2.arr/2;
//	  Speed_generate[2] = (ALPHA)/Stepper3.arr/2;
//	  Speed_generate[3] = (ALPHA)/Stepper4.arr/2;
//	  Speed_generate[4] = (ALPHA)/Stepper5.arr/2;
//	  Speed_generate[5] = (ALPHA)/Stepper6.arr/2;
    osDelay(10);
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void *argument)
{
  /* USER CODE BEGIN StartCANTask */
	arm_can_init();
  /* Infinite loop */
  for(;;)
  {
	    CanMessage_t msg;
	    osMessageQueueGet(CANRxBufferHandle, &msg, NULL, osWaitForever);
	    can_handle_msg(&msg);
	    osDelay(1);
  }
  /* USER CODE END StartCANTask */
}

/* UartCircleCallback function */
void UartCircleCallback(void *argument)
{
  /* USER CODE BEGIN UartCircleCallback */
	CLI_Process();
  /* USER CODE END UartCircleCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartUartSend(void *argument){
	/* USER CODE BEGIN StartUartTask */
	char msg[20];
	MY_FLASH_SetSectorAddrs(11, 0x080E5000);
	CLI_Init(&huart4);
	/* Infinite loop */
	for(;;)
	{
		osMessageQueueGet(UartCommandHandle, &msg, NULL, osWaitForever);
		if(strcmp(msg, "send_pos")==0){
			sprintf(uart_msg, "Current position [mm]: X%f Y%f Z%f", 200.56, 199.97, 100.12);
//			sprintf(uart_msg, "Current position: X%f Y%f Z%f", current_pos.pos_x, current_pos.pos_y, current_pos.pos_z);
//			HAL_UART_Transmit_IT(&huart4, (uint8_t*) uart_msg, sizeof(uart_msg));
			CLI_Printf(uart_msg);
		}else if(strcmp(msg, "set_home")==0){
			HOME();
		  	theta1 = 90;
		  	theta2 = 180;
		  	theta3 = -180;
		  	theta4 = 0;
		  	theta5 = 90;
			CLI_Printf("Done!");
		}else if(strcmp(msg, "set_amax")==0){
			char* cmd = strtok(uart_buff, " ");// read the first argument
			if (cmd != NULL) {
			    char* arg = strtok(NULL, " "); // read the string after x
			    uint32_t val = atoi(arg);
			    memcpy(&memory.a_max, &val, sizeof(uint32_t));
			}
		}else if(strcmp(msg, "set_vmax")==0){
			char* cmd = strtok(uart_buff, " ");// read the first argument
			if (cmd != NULL) {
		       	char* arg = strtok(NULL, " "); // read the string after x
		       	uint32_t val = atoi(arg);
		       	memcpy(&memory.v_max, &val, sizeof(uint32_t));
			}
		}else if(strcmp(msg, "set_jmax")==0){
			char* cmd = strtok(uart_buff, " ");// read the first argument
			if (cmd != NULL) {
		       	char* arg = strtok(NULL, " "); // read the string after x
		       	uint32_t val = atoi(arg);
		       	memcpy(&memory.j_max, &val, sizeof(uint32_t));
			}
		}else if(strcmp(msg, "set_theta")==0){
			char* cmd = strtok(uart_buff, " ");// read the first argument
			if (cmd != NULL) {
			    input_mode = 0;
			    Inv_Arm_t Theta;
			    char* arg1 = strtok(NULL, " "); // read theta1
			    char* arg2 = strtok(NULL, " "); // read theta2
			    char* arg3 = strtok(NULL, " "); // read theta3
			    char* arg4 = strtok(NULL, " "); // read theta4
			    char* arg5 = strtok(NULL, " "); // read theta5
			    char* arg6 = strtok(NULL, " "); // read theta6
			    Theta.theta1 = atof(arg1);// convert arg from ascii to integer
			    Theta.theta2 = atof(arg2);
			    Theta.theta3 = atof(arg3);
			    Theta.theta4 = atof(arg4);
			    Theta.theta5 = atof(arg5);
			    Theta.theta6 = atof(arg6);
			    osMessageQueuePut(UartMessagesHandle, &Theta, 0, 0);
			}
		}else if(strcmp(msg, "set_pos")==0){
			char* cmd = strtok(uart_buff, " ");// read the first argument
			if (cmd != NULL) {
	        	Arm_pos_t setpos;
	        	input_mode = 1;
	            char* arg0 = strtok(NULL, " "); // read the string after x
	            char* arg1 = strtok(NULL, " "); // read the string after x
	            char* arg2 = strtok(NULL, " "); // read the string after x
			            /* convert arg from ascii to integer */
	            setpos.pos_x = atof(arg0);
	            setpos.pos_y = atof(arg1);
	            setpos.pos_z = atof(arg2);
	            osMessageQueuePut(UartMessagesHandle, &setpos, 0, 0);
			}
		}else if(strcmp(msg, "freerun") == 0){
			char* cmd = strtok(uart_buff, " ");// read the first argument
			if (cmd != NULL) {
				char* arg0 = strtok(NULL, " "); // read the string after x
				if(strcmp(arg0, "true")==0){
					uint8_t run = 1;
					memcpy(&freerun, &run, 1);
//					char msg[20] = "true";
//					osMessageQueuePut(FreeRunHandle, &msg, 0, 0);
				}else if(strcmp(arg0, "false")==0){
					HAL_NVIC_SystemReset();
				}
			}
		}
		else if(strcmp(msg, "save")==0){
			MY_FLASH_WriteN(0, &memory, sizeof(memory)/4 , DATA_TYPE_32);
			HAL_NVIC_SystemReset();

		}


	    osDelay(1);
	}

	/* USER CODE END StartUartTask */
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0))
    {
        CAN_RxHeaderTypeDef header;
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
        {
//            CanMessage_t rxmsg;
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rxmsg.buf);

//            memcpy(&msg, &rxmsg.buf, 8);
            rxmsg.id = header.StdId;
            rxmsg.len = header.DLC;
            rxmsg.rtr = header.RTR;

            osMessageQueuePut(CANRxBufferHandle, &rxmsg, 0, 0);
        }
    }
}
/* USER CODE END Application */

