/*
 * Arm_CAN.c
 *
 *  Created on: May 21, 2022
 *      Author: Phanny
 */

#include "Arm_CAN.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "cmsis_os.h"
#include "string.h"
#include "cli.h"

Arm_param_t Arm_get;
Arm_param_t Arm_set;
uint8_t arm_can_init()
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank = 10;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (status != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
        return 1;
    }

    status = HAL_CAN_Start(&hcan1);

    if (status != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
        return 1;
    }

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    return 0;
}
uint8_t can_handle_msg(CanMessage_t *msg)
{
    uint8_t first_word[4];
    uint8_t second_word[4];
    if (msg->len == 8)
    {
        memcpy(&first_word, &msg->buf, 4);
        second_word[3] = msg->buf[7];
        second_word[2] = msg->buf[6];
        second_word[1] = msg->buf[5];
        second_word[0] = msg->buf[4];
    }
    else if (msg->len == 4)
    {
        memcpy(&first_word, &msg->buf, 4);
    }
	Arm_param_t *arm_get;
	arm_get = &Arm_get;
    if ((msg->id & dev_ID) == dev_ID)
    {
        if (msg->rtr == 0)
        {
            switch (msg->id & 0b11111) // Mask out the first 5 bits
            {
            case (MSG_SET_HOME):
                memcpy(&arm_get->set_home, &first_word, sizeof(uint32_t));

                break;
            case (MSG_SET_ACCEL):
                memcpy(&arm_get->a_max, &first_word, sizeof(uint32_t));
            	CLI_Printf(BY"CAN Message:\n"BG"Set maximum acceleration: %d\n\r>>",arm_get->a_max);
                break;
            case (MSG_SET_VEL):
                memcpy(&arm_get->v_max, &first_word, sizeof(uint32_t));
            	CLI_Printf(BY"CAN Message:\n"BG"Set maximum velocity: %d\n\r>>",arm_get->v_max);
                break;
            case (MSG_SET_JERK):
				memcpy(&arm_get->j_max, &first_word, sizeof(uint32_t));
            	CLI_Printf(BY"CAN Message:\n"BG"Set maximum jerk: %d\n\r>>",arm_get->j_max);
                break;
            case (MSG_SET_TH12):
				memcpy(&arm_get->theta[0], &first_word, sizeof(uint32_t));
            	memcpy(&arm_get->theta[1], &second_word, sizeof(uint32_t));
                break;
            case (MSG_SET_TH34):
				memcpy(&arm_get->theta[2], &first_word, sizeof(uint32_t));
            	memcpy(&arm_get->theta[3], &second_word, sizeof(uint32_t));
                break;
            case (MSG_SET_TH56):
				memcpy(&arm_get->theta[4], &first_word, sizeof(uint32_t));
            	memcpy(&arm_get->theta[5], &second_word, sizeof(uint32_t));
                break;
            case (MSG_GET_POS):
				/* TODO: Implement */
                break;
            case (MSG_START):
				memcpy(&arm_get->start, &msg->buf[0], sizeof(uint8_t));
            	break;
            case (MSG_STOP):
				memcpy(&arm_get->stop, &msg->buf[0], sizeof(uint8_t));
            	break;
            case (MSG_SAVE):
				memcpy(&arm_get->save_config, &msg->buf[0], sizeof(uint8_t));
            	CLI_Printf(BY"CAN Message:\n"BG"Saving configuration...\n\r>>");
            	CLI_Printf(BY"System: Set home\nDone!\n\r>>");
            	break;
            default:
                return 1;
                break;
            }
        }

    }
    else
    {
        return 1; // Not correct device
    }

    return 0;
}

uint8_t can_write(uint16_t dev_id, Arm_Msg_t msg)
{
	Arm_param_t *arm_set;
	arm_set = &Arm_set;
    uint32_t can_error = HAL_CAN_GetError(&hcan1);
    if (can_error == HAL_CAN_ERROR_NONE)
    {
        CAN_TxHeaderTypeDef header;
        header.IDE = CAN_ID_STD;
        uint8_t data[8];
        float tmp[2];
        header.StdId = dev_id | msg;

        switch (msg)
        {
        case MSG_SET_HOME:
            memcpy(&data, &arm_set->set_home, 1);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 1;
            break;
        case MSG_SET_ACCEL:
            memcpy(&data, &arm_set->a_max, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_SET_VEL:
            memcpy(&data, &arm_set->v_max, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_SET_JERK:
            memcpy(&data, &arm_set->j_max, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_SET_TH12:
        	tmp[0] = arm_set->theta[0];
        	tmp[1] = arm_set->theta[1];
            memcpy(&data, &tmp, 8);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 8;
            break;
        case MSG_SET_TH34:
        	tmp[0] = arm_set->theta[2];
        	tmp[1] = arm_set->theta[3];
            memcpy(&data, &tmp, 8);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 8;
            break;
        case MSG_SET_TH56:
        	tmp[0] = arm_set->theta[4];
        	tmp[1] = arm_set->theta[5];
            memcpy(&data, &tmp, 8);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 8;
            break;
        case MSG_START:
            memcpy(&data, &arm_set->start, 1);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 1;
            break;
        case MSG_STOP:
            memcpy(&data, &arm_set->stop, 1);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 1;
            break;
        case MSG_SAVE:
            memcpy(&data, &arm_set->save_config, 1);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 1;
            break;
        default:
            break;
        }

        uint32_t retTxMailbox = 0;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
        {
            HAL_CAN_AddTxMessage(&hcan1, &header, data, &retTxMailbox);
        }

        return 1;
    }
    else
    {
        return can_error;
    }
}
