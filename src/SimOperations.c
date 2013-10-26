/** @addtogroup NetworkComm
 *@{*/

/******************************************************************************
 *  File       : SimOperation.c
 *****-----------------------------------------------------------------------------
 *  Object     :
 *
 *  Contents   :
 *
 *****************************************************************************/

/******************************************************************************
 * change history
 |*****************************************************************************
 |||||flag | author | date   | description
 ||||+-----+--------+--------+-----------------------------------------------------
 |     |        |        | create
 ||||+-----+--------+--------+-----------------------------------------------------
 |     |        |        |
 ||||+-----+--------+--------+-----------------------------------------------------
 |     |        |        |
 ||||+-----+--------+--------+-----------------------------------------------------
 |     |        |        |
 ||||+-----+--------+--------+-----------------------------------------------------
 *****************************************************************************/

/******************************************************************************
 *  Includes
 *****************************************************************************/

#include <adl_global.h>
#include "SimOperations.h"
//#include "queryapp.h"
#include "XactUtilities.h"

/******************************************************************************
 *  Defines
 *****************************************************************************/

/******************************************************************************
 *  Globals
 *****************************************************************************/
int g_sim_full_init_stat = 0;

static int SimInit = 0;

/******************************************************************************
 *  Prototypes
 *****************************************************************************/

/** @brief Handler of the SIM events
 *
 * @par
 * Handles events generated from the SIM operation
 *
 * @param Event
 * @return void
 */
void evhSimHandler(u8 Event)
{
//	s8 retCode = -1;

	TRACE((1, "Inside Sim_Handler"));
	adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1, "Inside Sim_Handler\n");

	switch (Event)
	{
	case ADL_SIM_STATE_INIT:
		TRACE((1, "SIM ADL_SIM_STATE_INIT"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1
		                       , "\r\nSIM ADL_SIM_STATE_INIT\n");
		break;

	case ADL_SIM_STATE_REMOVED:
		TRACE((1, "SIM ADL_SIM_STATE_REMOVED"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1
		                       , "\r\nSIM ADL_SIM_STATE_REMOVED\n");
		break;

	case ADL_SIM_STATE_INSERTED:
		TRACE((1, "SIM ADL_SIM_STATE_INSERTED"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1
		                       , "\r\nSIM ADL_SIM_STATE_INSERTED\n");
		break;

	case ADL_SIM_STATE_PIN_ERROR:
		TRACE((1, "SIM ADL_SIM_STATE_PIN_ERROR"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1
		                       , "\r\nSIM ADL_SIM_STATE_PIN_ERROR\n");
		break;

	case ADL_SIM_STATE_PIN_OK:
		TRACE((1, "SIM ADL_SIM_STATE_PIN_OK"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1
		                       , "\r\nSIM ADL_SIM_STATE_PIN_OK\n");
		break;

	case ADL_SIM_STATE_PIN_WAIT:
		TRACE((1, "SIM ADL_SIM_STATE_PIN_WAIT"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1
		                       , "\r\nSIM ADL_SIM_STATE_PIN_WAIT\n");
		break;

	case ADL_SIM_STATE_FULL_INIT:
		TRACE((1, "SIM ADL_SIM_STATE_FULL_INIT"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1
		                       , "\r\nSIM ADL_SIM_STATE_FULL_INIT\r\n");

		g_sim_full_init_stat = 1;
		SimInit = 1;
		break;

	default:
		wm_sprintf(g_traceBuf, "\r\nSIM Event return Value: %d\r\n", Event);
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1, g_traceBuf);
		TRACE((1, g_traceBuf));
		break;
	}
}


/** @brief determine current state of SIM operation
 *
 * @return  SimInit
 */
int IsSimInit(void)
{
	return SimInit;
}


/*@}*/
