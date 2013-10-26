#include "adl_global.h"
#include "XactUtilities.h"
#include "airplane_mode.h"
#include "gpioTest.h"

// local functions.
static bool WBHVHandler(adl_atResponse_t *paras);
static void AirplaneModeTimer(u8 Id, void *Context);

// local data.
static ascii    *RESTART_STRING = "AT+CFUN=1";
static ascii    *ENTER_AIRPLANE_STRING = "AT+WBHV=1,1";
static ascii    *EXIT_AIRPLANE_STRING = "AT+WBHV=1,0";

static bool in_airplane_mode = false;
// global data.

/** @brief evalute the status of airplane mode
 *
 * @return void
 */
void evalAirplaneMode(void)
{
	ascii *WBHV_QUERY_STRING = "AT+WBHV?";

	int Ret_Value;
	if ((Ret_Value = adl_atCmdCreate(WBHV_QUERY_STRING, FALSE, (adl_atRspHandler_t)WBHVHandler, "*", NULL)) != OK)
	{
		DisplayErrorCode("adl_atCmdCreate", __FILE__, __LINE__, Ret_Value);
	}
}


/** @brief Recv response from at+wbhv? string
 *
 * @return void
 */
static bool WBHVHandler(adl_atResponse_t *paras)
{
	// look for the " 1," string and capture the value after the comma.
	unsigned int ii;
	char ap_mode = '!';
	DumpMessage("WBHV HANDLER!\r\n");
	for (ii = 0; ii < strlen(paras->StrData) - 4; ii++)
	{
		if ((paras->StrData[ii] == ' ') && (paras->StrData[ii + 1] == '1') && (paras->StrData[ii + 2] == ','))
		{
			ap_mode = paras->StrData[ii + 3];
			break;
		}
	}

	if (ap_mode == '1')
	{
		// looks like we are in airplane mode
		in_airplane_mode = true;
		if (adl_tmrSubscribe(TRUE, 100, ADL_TMR_TYPE_100MS, AirplaneModeTimer) == NULL)
		{
			DumpMessage("Could not subscribe airplane mode timer -- restart\r\n");
			adl_atCmdCreate(RESTART_STRING, FALSE, NULL, NULL);
		}
	}

	return true;
}


/** @brief Timer that runs in airplane mode.
 *
 * @return void
 */
static void AirplaneModeTimer(u8 Id, void *Context)
{
	(void)Id;
	(void)Context;

	DumpMessage("Airplane Mode Timer!\r\n");

	// check if we can leave airplane mode.
	if (IsOnBatt())
	{
		// leave airplane mode.
		AirplaneModeExit();
	}
}


/** @brief Enter airplane mode and restart to take effect.
 *
 * @return void
 */
void AirplaneModeEnter(void)
{
	if (in_airplane_mode == false)
	{
		DumpMessage("Entering airplane mode...\r\n");
		adl_atCmdCreate(ENTER_AIRPLANE_STRING, FALSE, NULL, NULL);
		adl_atCmdCreate(RESTART_STRING, FALSE, NULL, NULL);
	}
}


/** @brief Exit airplane mode and restart to take effect.
 *
 * @return void
 */
void AirplaneModeExit(void)
{
	if (in_airplane_mode == true)
	{
		DumpMessage("Exiting airplane mode...\r\n");
		adl_atCmdCreate(EXIT_AIRPLANE_STRING, FALSE, NULL, NULL);
		adl_atCmdCreate(RESTART_STRING, FALSE, NULL, NULL);
	}
}
