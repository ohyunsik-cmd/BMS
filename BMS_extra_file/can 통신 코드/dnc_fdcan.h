#ifndef DNC_FDCAN_H
#define DNC_FDCAN_H

#include "stm32h7xx_hal.h"

void DNC_CAN_ConfigFilter(FDCAN_HandleTypeDef *fhdcan);

void DNC_CAN_ConfigHeader(FDCAN_TxHeaderTypeDef *TxHeader);

void DNC_CAN_Start(FDCAN_HandleTypeDef *hfdcan);

#endif