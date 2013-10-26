#if !defined(ADC_H_)
#define ADC_H_

enum IginitionStatus
{
	Ign_Off   = 0, 
	Ign_On    = 1,
	Ign_Idle  = 2
};


void StartADCReadTimer(void);
void StopADCReadTimer(void);
void ReadAnalogSensorVoltage(u8 timerid, void *context);
void StartAnalogSensorReadTimer();
void StopAnalogSensorReadTimer();

#endif
