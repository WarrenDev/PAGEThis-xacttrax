#ifndef __GPIOTEST_H__
#define __GPIOTEST_H__

/***********************************************/
/*				PROTOTYPES                     */
/************************************************/

#define GPIO_TRACE_LEVEL        5   /*!< Trace level for the GPIO events */
#define DOG_PARK_THRESH         3   /*!< Number of seconds button pressed for dog park mode. */
#define DOG_WALK_THRESH         1   /*!< Number of seconds button pressed for dog walk mode. */
#define DOG_WALK_THRESH_MS      500 /*!< Number of milliseconds button pressed for dog walk mode. */
//#define SHUTDOWN_THRESH 5  /*!< Number of millseconds button pressed for shutdown event. */

#define SOS_THRESH              3 /*!< Number of secons the SOS button must be pressed to generate an alarm */
#define IGN_THRESH              10


//Ignition IO
#define GPIO_IGN_GPIO			(5)
#define GPIO_7					(7)

void InitializeUnusedGpio(void);
void TurnOffFF(void);
void test_write_gpio(void);
void InitializeButton(void);
void TestGPIO(int number, int level);
void ModeSwitch(char toMode);
void UnInitializeButton();
void GPIOTimer(u8 timerid, void *context);
int IsUSBConnected(void);
int readMagSens(void);
int IsOnBatt(void);
void GpioWrite(int gpioNumber, s32 *gpioHandle, bool level);
s32 GpioRead(int gpioNumber, s32 *gpioHandle);
s32 ReadSOSGPIO();
s32 ReadMagSensorGPIO();

void ResetSPIFlash(void);
#endif
