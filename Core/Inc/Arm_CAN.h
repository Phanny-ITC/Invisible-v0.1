/*
 * Arm_CAN.h
 *
 *  Created on: May 21, 2022
 *      Author: Phanny
 */

#ifndef INC_ARM_CAN_H_
#define INC_ARM_CAN_H_
#include "stdint.h"
#include "can.h"
#define dev_ID 1<<5 //device ID

typedef struct
{
    uint16_t id; // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    uint8_t rtr;
    uint8_t len;
    uint8_t buf[8];
} CanMessage_t;
typedef struct{
	uint8_t set_home;
	uint8_t start;
	uint8_t stop;
	uint8_t save_config;
	uint32_t a_max;
	uint32_t v_max;
	uint32_t j_max;
	float theta[6];
	float x_pos;
	float y_pos;
	float z_pos;

}Arm_param_t;

typedef enum{
	MSG_SET_HOME = 0x000,
	MSG_SET_ACCEL,
	MSG_SET_VEL,
	MSG_SET_JERK,
	MSG_SET_TH12,
	MSG_SET_TH34,
	MSG_SET_TH56,
	MSG_GET_POS,
	MSG_START,
	MSG_STOP,
	MSG_SAVE,
}Arm_Msg_t;
typedef enum{
	IDLE = 0,
	RUN
}Arm_status_t;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
uint8_t arm_can_init();
uint8_t can_handle_msg(CanMessage_t *msg);
uint8_t can_write(uint16_t dev_id, Arm_Msg_t msg);
#endif /* INC_ARM_CAN_H_ */
