/*
 * FDCAN.c
 *
 *  Created on: Mar 5, 2023
 *      Author: Jonathan
 */

#include "FDCAN.h"


extern FDCAN_HandleTypeDef hfdcan1;

//CAN variables here
uint32_t functionID,source_address, destination_address,data_size;
uint8_t motorstate_flyer[1], motorstate_bobbin[1];
uint8_t error_flyer[2], error_bobbin[2];
uint8_t drivecheck_response_flyer[1],drivecheck_response_bobbin[1];
uint8_t runtimedata_flyer[8], runtimedata_bobbin[8];
uint8_t diagnosticsdata_flyer[8], diagnosticsdata_bobbin[8];

FDCAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[16];
extern FDCAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               RxData[16];


void FDCAN_TxInit(void)
{
	if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
		{
		  //Error_Handler();
		}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		{
		  //Error_Handler();
		}
    TxHeader.Identifier = 0x0E090102;//set to transmit runtime data frame from flyer to motherboard
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
}

uint32_t FDCAN_generateIdentifier(uint16_t source, uint16_t destination, uint16_t functionID, uint8_t priority)
{
	uint32_t temp_identifier;
	temp_identifier=(priority<<24)|(functionID<<16)|(destination<<8)|(source);
	return(temp_identifier);

}

void FDCAN_RxFilterInit(void)
{
	  FDCAN_FilterTypeDef sFilterConfig;

	  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	  sFilterConfig.FilterIndex = 0;
	  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  sFilterConfig.FilterID1 = (uint32_t)S.CAN_ID<<8;//destination address of flyer 0x00000200 (uint32_t)S.CAN_ID<<8
	  sFilterConfig.FilterID2 = 0x0000FF00;
	  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    //Error_Handler();
	  }

	    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
}

