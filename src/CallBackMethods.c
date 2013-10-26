/** @addtogroup OpenAT
 * @{ */
/*H****************************************************************************/

/** @file CallBackMethods.c
 * *----------------------------------------------------------------------------
 *  @brief This file contains callback methods used to service
 *         unsolicited messages from Open At
 *
 */

/******************************************************************************
 *  Includes
 *****************************************************************************/
#include <adl_global.h>
#include "gpioTest.h"
#include "XactUtilities.h"
#include "status.h"
#include "alarm_suppress.h"

/******************************************************************************
 *  Defines/Macros
 *****************************************************************************/

/******************************************************************************
 *  Globals/Prototypes
 *****************************************************************************/

/** @brief callback method for +CGEG unsolicted messages
 *
 * @par
 * Handler function which handles the +CREG indicaiton. This
 * function is used to suppress the unsolicited +CREG
 * indications so that the external mapping software does not
 * gets confused with these indications.
 *
 * @param paras
 *
 * @return FALSE
 */
bool CREGHandler(adl_atUnsolicited_t *paras)
{
	ascii respNum[10];
	ascii para1[10];
	ascii para2[10];

	TRACE((1, "Inside of CREG Handler"));
	wm_strGetParameterString(respNum, paras->StrData, 1);
	if ((strcmp(respNum, "1") == 0) || (strcmp(respNum, "5") == 0))
	{
		TRACE((1, "We got a 1 or a 5"));
		if (wm_strGetParameterString(para1, paras->StrData, 2) == NULL)
		{
			TRACE((1, "Region ID was null"));
		}
		else if (wm_strGetParameterString(para2, paras->StrData, 3) == NULL)
		{
			TRACE((1, "Cell ID was Null"));
		}
		else
		{
			TRACE((1, "We have GPRS available"));
			memcpy(&(g_status.LocationArea[0]), &para1, 4);
			memcpy(&(g_status.CellID[0]), &para2, 4);

			Status_setGSMStatus(GSM_WORKING);
		}
	}
	else
	{
		Status_setGSMStatus(GSM_NOT_WORKING);
	}

	return false;
}


/** @brief callback method for +CCED unsolicted messages
 *
 * @par
 * Handler function which handles the +CCED indicaiton. This
 * function is used to suppress the unsolicited +CCED
 * indications so that the external mapping software does not
 * gets confused with these indications.
 *
 * @param paras
 *
 * @return FALSE
 */
bool CCEDHandler(adl_atUnsolicited_t *paras)
{
	(void)paras;
	return FALSE;
}


#define RSSI_0          5
#define RSSI_1          10
#define RSSI_2          15
#define RSSI_3          20
#define RSSI_4          25
#define RSSI_ERROR      99

/** @brief callback method for +CSQ unsolicted messages
 *
 * @par
 * Handler function which handles the +CSQ indicaiton.
 * This handler provides the signal strength.
 *
 * @param paras
 *
 * @return FALSE
 */
bool CSQHandler(adl_atUnsolicited_t *paras)
{
	ascii rssi[10];

	int rssi_int;
	static char GSMAlarmGenerated = 0;

	wm_strGetParameterString(rssi, paras->StrData, 1);
	rssi_int = atoi(rssi);

	// rssi has a range from 0 - 31.
	// we have 6 ranges. Use the defines to define the 0-5 signal.
	if (rssi_int < RSSI_0)
	{
		g_status.GSMSignalStrength = '0';
	}
	else if (rssi_int < RSSI_1)
	{
		g_status.GSMSignalStrength = '1';
	}
	else if (rssi_int < RSSI_2)
	{
		g_status.GSMSignalStrength = '2';
	}
	else if (rssi_int < RSSI_3)
	{
		g_status.GSMSignalStrength = '3';
	}
	else if (rssi_int < RSSI_4)
	{
		g_status.GSMSignalStrength = '4';
	}
	else if (rssi_int == RSSI_ERROR)
	{
		g_status.GSMSignalStrength = '0';
	}
	else
	{
		g_status.GSMSignalStrength = '5';
	}

	wm_sprintf(g_traceBuf, "Sig strength = %c\r\n", g_status.GSMSignalStrength);
	DumpMessage(g_traceBuf);

	if (g_config.GSMAlertThresh != 'N')
	{
		if ((g_status.GSMSignalStrength <= g_config.GSMAlertThresh) && !GSMAlarmGenerated)
		{
			g_status.GSMAlarm = 'Y';
			GSMAlarmGenerated = 1;
			alarm_suppress_set_alarm_time(GSM_ALARM_SUP);
		}
		else if ((g_status.GSMSignalStrength > g_config.GSMAlertThresh) && (alarm_suppress_status(GSM_ALARM_SUP) == ALARM_EXPIRED))
		{
			GSMAlarmGenerated = 0;
		}
	}
	else
	{
		GSMAlarmGenerated = 0;
	}

	return FALSE;
}


/**@}*/
