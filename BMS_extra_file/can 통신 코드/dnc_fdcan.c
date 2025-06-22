#include "dnc_fdcan.h"

void DNC_CAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType        = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex   = 0;
    sFilterConfig.FilterType    = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig  = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1     = 0x22;
    sFilterConfig.FilterID2     = 0x22;
    sFilterConfig.RxBufferIndex = 0;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void DNC_CAN_ConfigHeader(FDCAN_TxHeaderTypeDef *TxHeader)
{
    TxHeader->Identifier             = 0x22;
    TxHeader->IdType                 = FDCAN_STANDARD_ID;
    TxHeader->TxFrameType            = FDCAN_DATA_FRAME;
    TxHeader->DataLength             = FDCAN_DLC_BYTES_8;
    TxHeader->ErrorStateIndicator    = FDCAN_ESI_ACTIVE;
    TxHeader->BitRateSwitch          = FDCAN_BRS_OFF;
    TxHeader->FDFormat               = FDCAN_CLASSIC_CAN;
    TxHeader->TxEventFifoControl     = FDCAN_NO_TX_EVENTS;
    TxHeader->MessageMarker          = 0;
}

void DNC_CAN_Start(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_Start(hfdcan);

    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}