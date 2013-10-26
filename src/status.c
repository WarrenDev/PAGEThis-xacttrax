/** @addtogroup NetworkComm
 *@{*/

/** @file status.c
 * @brief   Controls the status structure.
 *
 *
 */
#include "status.h"
#include "ConfigSettings.h"
#include "XactUtilities.h"
#include "InternalFlash.h"

/* main status structure */
STATUS g_status;

extern int VibrateDone;

/** @brief Initalize g_status global variable
 *
 * @par
 * Used to initialize the status struct on power up
 * @return void
 */
void InitStatus(void)
{
	NON_VOLATILE_STATUS nvs;
	if (InternalFlash_LoadNonVolatileStatus(&nvs) == -1)
	{
		DumpMessage("Error could not load load NVS\r\n");
		g_status.NumFences = 0;
		g_status.NumLogPktToSend = 0;
		g_status.LogPktToSendLatest = 0;
	}
	else
	{
		g_status.NumFences = nvs.NumFences;
		g_status.NumLogPktToSend = nvs.NumLogPktToSend;
		g_status.LogPktToSendLatest = nvs.LogPktToSendLatest;
	}

	g_status.BattLevel = 3;
	g_status.SystemFailure = 'N';
	g_status.GPSStatus = 'N';
	g_status.GPSSignalStrength1 = '0';
	g_status.GPSSignalStrength2 = '0';
	g_status.NumBirds = '0';
	g_status.GSMStatus = 'N';
	g_status.GSMSignalStrength = '0';
	g_status.ValidAccessCnt = 'A';
	g_status.BadAccessCnt = 'A';
	g_status.Mode = NO_CHANGE_MODE;
	g_status.SOSAlarm = 'N';
	g_status.GPSAlarm = 'N';
	g_status.BattAlarm = 'N';
	g_status.GSMAlarm = 'N';
	g_status.FenceAlarm = 'N';
	g_status.CurrentFenceIn = 0;
	g_status.MotionAlarm = 'N';
	g_status.OverSpeedAlarm = 'N';
	g_status.PowerDisconnAlarm = 'N';
	memset(&(g_status.LocationArea[0]), '0', 4);
	memset(&(g_status.CellID[0]), '0', 4);
}


/** @brief Clear all Alarms
 *
 * @par
 * Clear the generated alarms.
 * @return
 *  <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *  <TR>
 *      <TD><i>0</i></TD>
 *      <TD>we are delaying the alarm until the vibration is done </TD>
 *  </TR>
 *  <TR>
 *      <TD><i>1</i></TD>
 *      <TD>all alarms cleared </TD>
 *  </TR>
 *  </TABLE>
 */
// return 0 if we are delaying the alarm until vibration is done
// return 1 if cleared.
int ClearAlarms(void)
{
	DumpMessage("Clear Alarms!\r\n");

	if ((g_config.VibrationMotorDelayEn == ENABLE) && !VibrateDone)
	{
		DumpMessage("Waiting for vibrate\r\n");
		return 0;
	}

	if (g_status.SystemFailure == 'Y')
	{
		DumpMessage("SystemFailure alarm\r\n");
	}

	if (g_status.SOSAlarm == 'Y')
	{
		DumpMessage("SOSAlarm alarm\r\n");
	}

	if (g_status.GPSAlarm != 'N')
	{
		DumpMessage("GPSAlarm alarm\r\n");
	}
	if (g_status.BattAlarm != 'N')
	{
		DumpMessage("BattAlarm alarm\r\n");
	}
	if (g_status.GSMAlarm != 'N')
	{
		DumpMessage("GSMAlarm alarm\r\n");
	}
	if (g_status.FenceAlarm != 'N')
	{
		wm_sprintf(g_traceBuf, "FenceAlarm alarm: %x\r\n", g_status.FenceAlarm);
		DumpMessage(g_traceBuf);
	}
	if (g_status.RebootAlarm != 'N')
	{
		DumpMessage("RebootAlarm alarm\r\n");
	}
	if (g_status.MotionAlarm != 'N')
	{
		DumpMessage("MotionAlarm alarm\r\n");
	}
	if (g_status.OverSpeedAlarm != 'N')
	{
		DumpMessage("over speed alarm\r\n");
	}
	if (g_status.PowerDisconnAlarm != 'N')
	{
		DumpMessage("PowerDisconnAlarm alarm\r\n");
	}

#ifdef DEBUG_ALARMS
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.SystemFailure);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.SOSAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.GPSAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.BattAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.GSMAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.FenceAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.RebootAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.MotionAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.PowerDisconnAlarm);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "%x\r\n", g_status.OverSpeedAlarm);
	DumpMessage(g_traceBuf);
#endif

	g_status.SystemFailure = 'N';
	g_status.SOSAlarm = 'N';
	g_status.GPSAlarm = 'N';
	g_status.BattAlarm = 'N';
	g_status.GSMAlarm = 'N';
	g_status.FenceAlarm = 'N';
	g_status.RebootAlarm = 'N';
	g_status.MotionAlarm = 'N';
	g_status.PowerDisconnAlarm = 'N';
	g_status.OverSpeedAlarm = 'N';
	InternalFlash_WriteRebootAlarm('N');

	return 1;
}


void Status_setGSMStatus(GSM_STATUS gsm_stat)
{
	if (gsm_stat == GSM_WORKING)
	{
		g_status.GSMStatus = 'W';
	}
	else
	{
		g_status.GSMStatus = 'N';
	}
}


GSM_STATUS Status_getGSMStatus(void)
{
	if (g_status.GSMStatus == 'W')
	{
		return GSM_WORKING;
	}
	else
	{
		return GSM_NOT_WORKING;
	}
}


void Status_setResetReason(RESET_REASON reason)
{
	g_status.RebootAlarm = reason;
	if (InternalFlash_WriteRebootAlarm(reason) == -1)
	{
		DumpMessage("Error writing reboot alarm\r\n");
	}
}


/**@}*/
