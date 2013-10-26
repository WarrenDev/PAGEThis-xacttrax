/** @addtogroup I2C
 *@{
 */
/*H************************************************************************
*/
/** @file    i2c_accelerometer.c
*
*   @brief   I2C interface for accelerometer.
*
*   @details This module contains the driver API for Freescale Semiconductor's  
*            MMA7455L Three Axis Low-g Digital Output Accelerometer.            
*
*   @note    Other help for the reader, including references.
*
*
*//*
*
* CHANGES :
*
* REF NO  DATE     WHO      DETAIL
*         19Jun09  AndyB    Ported from reference design
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*
*H*/

#include <adl_global.h>
#include "i2c.h"
#include "i2c_accelerometer.h"
#include "XactUtilities.h"

// device address
#define I2C_ACCELEROMETER_ADDRESS       0x1D 
/**< I2C address where the Accelerometer is located */

// register address
#define I2C_ACCELEROMETER_REG_XAXIS     0x00
/**< register contains x axis value */
#define I2C_ACCELEROMETER_REG_INTR_DET  0x0A
/**< interrupt detection register */
#define I2C_ACCELEROMETER_REG_ADDR      0x0D
/**< I2C bus address value register */

#define I2C_ACCELEROMETER_REG_DRIFTX_LOW     0x10
/**< Drift X register low*/
#define I2C_ACCELEROMETER_REG_DRIFTX_HIGH    0x11
/**< Drift X register high*/

#define I2C_ACCELEROMETER_REG_DRIFTY_LOW     0x12
/**< Drift X register low */
#define I2C_ACCELEROMETER_REG_DRIFTY_HIGH    0x13
/**< Drift X register high */

#define I2C_ACCELEROMETER_REG_DRIFTZ_LOW     0x14
/**< Drift Z register Low */
#define I2C_ACCELEROMETER_REG_DRIFTZ_HIGH    0x15
/**< Drift Z register High */

#define I2C_ACCELEROMETER_REG_MODE      0x16
/**< Mode register controls level or pulse mode */
#define I2C_ACCELEROMETER_REG_INTR_RST  0x17  

#define I2C_ACCELEROMETER_REG_CONTROL_1 0x18
#define I2C_ACCELEROMETER_THOPT 0x40

#define I2C_ACCELEROMETER_REG_CONTROL_2 0x19
#define I2C_ACCELEROMETER_LDPL 0x1

/**< Interrupt reset register */
#define I2C_ACCELEROMETER_REG_TSHLD     0x1A
/**< Threshold register */
	
// Mode control register
#define I2C_ACCELEROMETER_MODE_LEVEL        2
/**< Bit to select level mdoe */
#define I2C_ACCELEROMETER_MODE_MEASURE        1
/**< Bit to select measure mode */

#define I2C_ACCELEROMETER_GLVL_8G           0
/**< Bit to select 8 G mode */
#define I2C_ACCELEROMETER_GLVL_4G           8
/**< Bit to select 4 g mode */
#define I2C_ACCELEROMETER_GLVL_2G           4
/**< Bit to select 2 g mode */
#define I2C_ACCELEROMETER_DRPD              0x40
/**< Bit to select DRPD mdoe */

// calculation related macros
#define I2C_MAX_FRACTION_BITS               6
/**< Used to calculate fraction portion */
#define I2C_SHIFT_FOR_FIXED1616             (16-I2C_MAX_FRACTION_BITS)
/**< Used to calculate fraction portion */


static int writeI2CReg(int addr, int value);

char I2C_ACCEL_ADDR_STR[] = "0011101";
/*
 * Defines the data type for the callback function which gets called when
 * the vector sum of the acceleration/g-force in X,Y and Z axes crosses the
 * registered vector sum of threshold. 
 */


/** @struct accelerometerInfo_t
 *
 * @par
 * Data structure which holds the current accelerometer readings, threshold
 * vector sum, callback function and contest value set by the user.
 */
typedef struct accelerometerInfo_t
{
    fixed1616  xAxis;  /**< g-force value on X axis, stored in fixed1616 format */
    fixed1616  yAxis;  /**< g-force value on Y axis, stored in fixed1616 format */
    fixed1616  zAxis;  /**< g-force value on Z axis, stored in fixed1616 format */
    fixed1616  threshold; /**< vector sum of threshold value set by the user
                           during registration */
    NaAccelerometerCallbackFn_t cbFn; /**< callback function set by the user
                                      during registration */
    unsigned long context;  /**< value which user expected to see when callback
                             function returns */
    bool sampleReady; /**< Sample is ready to read by the user */
}accelerometerInfo_t;

static accelerometerInfo_t gfInfo;

/** @brief Calculate Acceleration
 *
 * @par This local function converts the g-force value read from 
 * accelerometer to fixed1616 format.
 * 
 * @param ptr
 * @return val
 */
static fixed1616 calculateAcceleration (unsigned char *ptr)
{ 
    unsigned int val;

    val = (ptr[1] << 8) | ptr[0];
    if (val & 0x200)
        val |= 0xFFFFFC00;
    else
        val &= ~0xFFFFFC00;

    val <<= I2C_SHIFT_FOR_FIXED1616;

    return (fixed1616)val;
}

unsigned char naAccelReadThreshold(void)
{
  char m_reg[3] = {'0','0',0};
  unsigned char  rdbuf[3];

  wm_itohexa(&m_reg[0],I2C_ACCELEROMETER_REG_TSHLD,2);
  if (ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"r",m_reg,"1",&rdbuf[0]) != 0)
    return 0xff;

  return rdbuf[0];

}

/* @brief read all the acceleromter x,y,z register values
 * @par accel_calib_data  
 * @return 0 on success
 * @return -1 on failure
 */
int naReadAccelData(ACCEL_CALIBRATION_DATA* accel_calib_data)
{
  char m_reg[3] = {'0','0',0};
  unsigned char  rdbuf[9];
  int ii;

  for (ii=0;ii<9;ii++) {
    wm_itohexa(&m_reg[0],ii,2);
    if (ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"r",m_reg,"1",&rdbuf[ii]) != 0)
      return -1;
  }

  accel_calib_data->x_10_bit = rdbuf[0] | (rdbuf[1] << 8);
  if (rdbuf[1] & 0x1) accel_calib_data->x_10_bit |= 0xfffffc00;      // sign extend if needed.
  accel_calib_data->y_10_bit = rdbuf[2] | (rdbuf[3] << 8);
  if (rdbuf[3] & 0x1) accel_calib_data->y_10_bit |= 0xfffffc00;      // sign extend if needed.
  accel_calib_data->z_10_bit = rdbuf[4] | (rdbuf[5] << 8);
  if (rdbuf[5] & 0x1) accel_calib_data->z_10_bit |= 0xfffffc00;      // sign extend if needed.
  accel_calib_data->x_8_bit  = rdbuf[6];
  if (rdbuf[6] & 0x80) accel_calib_data->x_8_bit |= 0xffffff00;      // sign extend if needed.
  accel_calib_data->y_8_bit  = rdbuf[7];
  if (rdbuf[7] & 0x80) accel_calib_data->y_8_bit |= 0xffffff00;      // sign extend if needed.
  accel_calib_data->z_8_bit  = rdbuf[8];
  if (rdbuf[8] & 0x80) accel_calib_data->z_8_bit |= 0xffffff00;      // sign extend if needed.

  return 0;
}


/* helpfer function to write 1 byte to an address */
static int writeI2CReg(int addr, int value)
{
  char m_reg[3] = {'0','0',0};
  unsigned char wrbuf[4];
  wm_itohexa(&m_reg[0],addr,2);
  wm_itohexa((ascii *)&wrbuf[0],value,2);
  wrbuf[2] = 0x00;
  return ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);
}

/* @brief write the acceleromter "drift" registers. Used for calibration.
 * @par accel_calib_data  
 * @return 0 on success
 * @return -1 on failure
 */
int naWriteAccelData(ACCEL_CALIBRATION_DATA* accel_calib_data)
{
  if (writeI2CReg(I2C_ACCELEROMETER_REG_DRIFTX_LOW, (accel_calib_data->x_offset_value & 0xff))      != 0)
    return -1;
  if (writeI2CReg(I2C_ACCELEROMETER_REG_DRIFTX_HIGH,(accel_calib_data->x_offset_value >> 8) & 0xff) != 0)
    return -1;
  if (writeI2CReg(I2C_ACCELEROMETER_REG_DRIFTY_LOW, (accel_calib_data->y_offset_value & 0xff))      != 0)
    return -1;
  if (writeI2CReg(I2C_ACCELEROMETER_REG_DRIFTY_HIGH,(accel_calib_data->y_offset_value >> 8) & 0xff) != 0)
    return -1;
  if (writeI2CReg(I2C_ACCELEROMETER_REG_DRIFTZ_LOW, (accel_calib_data->z_offset_value & 0xff))      != 0)
    return -1;
  if (writeI2CReg(I2C_ACCELEROMETER_REG_DRIFTZ_HIGH,(accel_calib_data->z_offset_value >> 8) & 0xff) != 0)
    return -1;

  return 0;
}

/** @brief Read the Accelometer
 *
 * @par Reat the last measured value from the accelerometer.
 *
 * @return accel_mag_square, -1 on error
 */
int ReadAccelerometer()
{
  unsigned char rdbuf[9];

  char m_reg[3] = {'0','0',0};
  char size[1];

  u32 temp = I2C_ACCELEROMETER_REG_XAXIS;
  gfInfo.sampleReady = FALSE;
  wm_itohexa(&m_reg[0],temp,2);
  wm_itohexa(size,sizeof(rdbuf),1);
  if(ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"r",m_reg,size,rdbuf) == 0)
    {

      gfInfo.xAxis = calculateAcceleration(rdbuf);
      gfInfo.yAxis = calculateAcceleration(&rdbuf[2]);
      gfInfo.zAxis = calculateAcceleration(&rdbuf[4]);
      gfInfo.sampleReady = TRUE; // now data is ready to read


      fixed1616 x,y,z;
      int gx, gy, gz;
      int accel_mag_square;

      x = (gfInfo.xAxis < 0) ? (gfInfo.xAxis * -1): gfInfo.xAxis;
      y = (gfInfo.yAxis < 0) ? (gfInfo.yAxis * -1): gfInfo.yAxis;
      z = (gfInfo.zAxis < 0) ? (gfInfo.zAxis * -1): gfInfo.zAxis;

      gx = x >> I2C_SHIFT_FOR_FIXED1616;
      gy = y >> I2C_SHIFT_FOR_FIXED1616;
      gz = z >> I2C_SHIFT_FOR_FIXED1616;

      accel_mag_square = (gx * gx) + (gy * gy) + (gz * gz);
      return accel_mag_square;
    }
  else
    {
      gfInfo.sampleReady = FALSE; // we have stale data        
      return -1;
    }
}

/** @brief Accelerometer Epoch
 *
 * @par
 * This API configures the initial settings of the accelerometer. Sets the 
 * suitable mode, resolution and other device specific configuration.
 * 
 * @return void
 */
void naAccelerometerEpoch (void)
{
    unsigned char  rdbuf[4];

    // initialize accelerometer local info
    memset(&gfInfo, 0, sizeof gfInfo);


    char m_reg[3] = {'0','0',0};
    // read the address just to make sure we are accessing the correct part
    u32 temp = I2C_ACCELEROMETER_REG_ADDR;
    wm_itohexa(&m_reg[0],temp,2);
    ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"r",m_reg,"1",rdbuf);

    if ((rdbuf[0] & 0x3F) != I2C_ACCELEROMETER_ADDRESS)
    {
        TRACE((I2C_TRACE_LEVEL,"naAccelerometerEpoch: Invalid device address [0x%X]",  
                    rdbuf[0] & 0x3F));
	DumpMessageUSB("Accel address invalid\r\n",1);
        return;
    }
    else {
        DumpMessage("Accel address good\r\n");
	DumpMessageUSB("Accel address good\r\n",1);
    }

}


/** @brief Set Accelerometer to level detect mode
 *  @return -1 Failed to communicate with device
 */
int naAcclerometerSetLevelMode(void)
{
  u32 temp = I2C_ACCELEROMETER_REG_MODE;
  u32 temp2 = 0;
  int result = -1;
  char m_reg[3] = {'0','0',0};
  unsigned char wrbuf[4];

  wm_itohexa(&m_reg[0],temp,2);
  temp2 = I2C_ACCELEROMETER_GLVL_8G | I2C_ACCELEROMETER_MODE_LEVEL | I2C_ACCELEROMETER_DRPD;             
  wm_itohexa((ascii *)&wrbuf[0],temp2,2);
  wrbuf[2] = 0x00;
  result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);

  return result;
}

/** @brief Set Accelerometer to measure mode
 *  @return -1 Failed to communicate with device
 */
int naAcclerometerSetMeasureMode(void)
{
  u32 temp = I2C_ACCELEROMETER_REG_MODE;
  u32 temp2 = 0;
  int result = -1;
  char m_reg[3] = {'0','0',0};
  unsigned char wrbuf[4];

  wm_itohexa(&m_reg[0],temp,2);
  temp2 = I2C_ACCELEROMETER_GLVL_8G | I2C_ACCELEROMETER_MODE_MEASURE | I2C_ACCELEROMETER_DRPD;             
  wm_itohexa((ascii *)&wrbuf[0],temp2,2);
  wrbuf[2] = 0x00;
  result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);

  return result;
}

int naAcceleromterSetCtrl2(void)
{
    int result = -1;
    unsigned char wrbuf[4];

    char m_reg[3] = {'0','0',0};


    // set threshold value
    u32 temp = I2C_ACCELEROMETER_REG_CONTROL_1;
    wm_itohexa(&m_reg[0],temp,2);

    wm_itohexa((ascii *)&wrbuf[0],I2C_ACCELEROMETER_THOPT,2);
    wrbuf[2] = 0x00;

    result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);

    return result;
}

/** @brief Set Accelermeter Trigger Treshold
 *
 * @par
 * This API sets the trigger threshold and enables the interrupt which can 
 * assert the interrupt line when one of the axis value crosses the given 
 * threshold.
 *
 * @param threshold
 *
 * @return  result 0 on Success,
 * -1  Failed to communicate with the device
 */
int naAccelerometerSetTriggerThreshold (char threshold)
{
    int result = -1;
    unsigned char wrbuf[4];

    char m_reg[3] = {'0','0',0};


    // set threshold value
    u32 temp = I2C_ACCELEROMETER_REG_TSHLD;
    u32 temp2 = 0;
    wm_itohexa(&m_reg[0],temp,2);


    wm_sprintf(g_traceBuf,"threshold = %x\r\n",threshold);
    DumpMessage(g_traceBuf);

    wm_itohexa((ascii *)&wrbuf[0],threshold,2);
    wrbuf[2] = 0x00;
    result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);

    // set to level trigger
    temp = I2C_ACCELEROMETER_REG_MODE;
    wm_itohexa(&m_reg[0],temp,2);
    temp2 = I2C_ACCELEROMETER_GLVL_8G | I2C_ACCELEROMETER_MODE_LEVEL | I2C_ACCELEROMETER_DRPD;             
    wm_itohexa((ascii *)&wrbuf[0],temp2,2);
    wrbuf[2] = 0x00;
    result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);

    return result;	
}

/** @brief Check if the accelerometer has a pending IRQ.
 *
 * @par
 * This will read the accelerometer and see if an IRQ is pending.
 *
 * @return  0 no IRQ
 * @reutnr  1 IRQ
 */
int naAccelerometerCheckIRQ (void)
{
  int result = 0;
  unsigned char  rdbuf[4];
  char m_reg[3] = {'0','0',0};
  u32 temp = I2C_ACCELEROMETER_REG_INTR_DET;

  // check if interrupt is actually triggered
  wm_itohexa(&m_reg[0],temp,2);
  result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"r",m_reg,"1",rdbuf);
  if(result < 0)
    {
      TRACE((I2C_TRACE_LEVEL,"Failed to check acceleration interupt register\n"));
      return 0;
    }

  if ((rdbuf[0] & 3) == 0)
    return 0;
  else 
    return 1;
}

/** @brief Clear Accelerometer interrupt line
 *
 * @par
 * This API clears the interrupt and deasserts the interrupt line.
 *
 * @result  0 on Success,
 * -1  Failed to communicate with the device
 */
int naAccelerometerClearTrigger (void)
{
    int result = -1;
    unsigned char wrbuf[4], rdbuf[4];

    char m_reg[3] = {'0','0',0};
    u32 temp = I2C_ACCELEROMETER_REG_INTR_DET;
    u32 temp2 = 0;

    TRACE((I2C_TRACE_LEVEL,"Trying to clear interrupt\n"));


    // check if interrupt is actually triggered
    wm_itohexa(&m_reg[0],temp,2);
    result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"r",m_reg,"1",rdbuf);
    if(result < 0)
    {
        TRACE((I2C_TRACE_LEVEL,"Failed to check acceleration interupt register\n"));
        return result;
    }

    if ((rdbuf[0] & 3) == 0)
        return result; // no interrupt, just return

    // clear the interrupt, write 1 followed by 0
    temp = I2C_ACCELEROMETER_REG_INTR_RST;
    wm_itohexa(&m_reg[0],temp,2);
    temp2 = rdbuf[0] & 3; 
    wm_itohexa((ascii *)&wrbuf[0],temp2,2);
    wrbuf[2] = 0x00;
    result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);
    if(result < 0)
    {
        TRACE((I2C_TRACE_LEVEL,"Failed to write a '1' to the interupt register\n"));
        return result;
    }

    wrbuf[0] = '0'; 
    wrbuf[1] = '0'; 
    wrbuf[2] = 0x00;
    result = ioCallToI2CDevice(I2C_ACCEL_ADDR_STR,"w",m_reg,(char *)wrbuf,NULL);
    if(result < 0) {
        TRACE((I2C_TRACE_LEVEL,"Failed to write a '0' to clear the interupt register\n"));
        DumpMessage("failed to clear interrupt\r\n");
    }

    return result;	
}
#ifndef NO_MFG_EOS_ENABLED

//#define Debug_Accel 0

#if Debug_Accel
#define dfTRACE(x) TRACE(x)
#else
#define dfTRACE(x)
#endif

#endif /* NO_MFG_EOS_ENABLED */

/**@}*/
