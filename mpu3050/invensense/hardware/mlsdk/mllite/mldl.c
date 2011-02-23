/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mldl.c 4162 2010-11-24 21:35:52Z prao $
 *
 *******************************************************************************/

/** 
 *  @defgroup MLDL 
 *  @brief  Motion Library - Driver Layer.
 *          The Motion Library Driver Layer provides the interface to the 
 *          system drivers that are used by the Motion Library.
 *
 *  @{
 *      @file   mldl.c
 *      @brief  The Motion Library Driver Layer.
**/

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <string.h>

#include "mldl.h"
#include "mldl_cfg.h"
#include "compass.h"
#include "mlsl.h"
#include "mlos.h"
#include "mlinclude.h"
#include "ml.h"
#include "dmpKey.h"
#include "mpu.h"
#include "mlFIFOHW.h"

#include "log.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-mldl"

#define _mldlDebug(x) //{x}

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

#define MAX_LOAD_WRITE_SIZE (MPU_MEM_BANK_SIZE/2) /* 128 */

/*---- structure containing control variables used by MLDL ----*/
static struct mldl_cfg mldlCfg;
static struct ext_slave_descr accel;
static struct ext_slave_descr compass;
static struct mpu3050_platform_data pdata;
static void *sMLSLHandle;
int_fast8_t intTrigger[NUM_OF_INTSOURCES];

/*******************************************************************************
 * Functions for accessing the DMP memory via keys
 ******************************************************************************/

unsigned short (*sGetAddress) (unsigned short key) = NULL;
static const unsigned char *localDmpMemory = NULL;
static       unsigned short localDmpMemorySize = 0;

/**
 *  @internal
 *  @brief Sets the function to use to convert keys to addresses. This 
 *         will changed for each DMP code loaded.
 *  @param func   
 *              Function used to convert keys to addresses.
 *  @endif
 */
void MLDLSetGetAddress( unsigned short (*func)(unsigned short key) )
{
    INVENSENSE_FUNC_START;
    _mldlDebug(MPL_LOGV("setGetAddress %d", (int)func);)
    sGetAddress = func;
}


/**
 *  @internal
 *  @brief  Check if the feature is supported in the currently loaded
 *          DMP code basing on the fact that the key is assigned a 
 *          value or not.
 *  @param  key     the DMP key
 *  @return whether the feature associated with the key is supported
 *          or not.
 */
uint_fast8_t DmpFeatureSupported(unsigned short key)
{
    unsigned short memAddr;

    if ( sGetAddress == NULL ) {
        MPL_LOGE("%s : sGetAddress is NULL\n", __func__);
        return FALSE;
    }
    
    memAddr = sGetAddress( key );
    if ( memAddr >= 0xffff ) {
        MPL_LOGV("MLDLSetMemoryMPU unsupported key\n");
        return FALSE;
    }
    
    return TRUE;
}
 

/**
 *  @internal
 *  @brief  used to get the specified number of bytes from the original 
 *          MPU memory location specified by the key.
 *          Reads the specified number of bytes from the MPU location
 *          that was used to program the MPU specified by the key. Each 
 *          set of code specifies a function that changes keys into 
 *          addresses. This function is set with setGetAddress().
 *
 *  @param  key     The key to use when looking up the address.
 *  @param  length  Number of bytes to read.
 *  @param  buffer  Result for data.
 *
 *  @return ML_SUCCESS if the command is successful, ML_ERROR otherwise. The key
 *          not corresponding to a memory address will result in ML_ERROR.
 *  @endif
 */
tMLError MLDLGetMemoryMPUOriginal( unsigned short key,
                                   unsigned short length, 
                                   unsigned char* buffer )
{
    unsigned short offset;

    if ( sGetAddress == NULL ) {
        return ML_ERROR_NOT_OPENED;
    }

    offset = sGetAddress( key );
    if ( offset >= localDmpMemorySize || (offset+length) > localDmpMemorySize ){
        return ML_ERROR_INVALID_PARAMETER;
    }

    memcpy(buffer, &localDmpMemory[offset], length);

    return ML_SUCCESS;
}

unsigned short MLDLGetAddress(unsigned short key)
{
    unsigned short offset;
    if ( sGetAddress == NULL ) {
        return ML_ERROR_NOT_OPENED;
    }

    offset = sGetAddress( key );
    return offset;
}

/* ---------------------- */
/* -  Static Functions. - */
/* ---------------------- */

/**
 *  @brief  Open the driver layer and resets the internal
 *          gyroscope, accelerometer, and compass data 
 *          structures.
 *  @param  mlslHandle
 *              the serial handle.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLDLOpen(void *mlslHandle)
{
    tMLError result;
    memset(&mldlCfg,  0, sizeof(mldlCfg));
    memset(&accel,    0, sizeof(accel));
    memset(&compass,  0, sizeof(compass));
    memset(&pdata,    0, sizeof(pdata));
    memset(intTrigger,INT_CLEAR,sizeof(intTrigger));

    sMLSLHandle = mlslHandle;

    mldlCfg.accel   = &accel;
    mldlCfg.compass = &compass;
    mldlCfg.pdata   = &pdata;

    result = (tMLError) mpu3050_open(&mldlCfg, sMLSLHandle);
    return result;
}


/**
 *  @brief  Closes/Cleans up the ML Driver Layer.
 *          Put the device in sleep mode.
 *  @return ML_SUCCESS or non-zero error code.
 */
tMLError MLDLClose(void)
{
    INVENSENSE_FUNC_START;
    tMLError result = ML_SUCCESS;
    
    if (!mldlCfg.is_suspended) {
        result = (tMLError) mpu3050_suspend(&mldlCfg, 
                                            sMLSLHandle, 
                                            sMLSLHandle, 
                                            sMLSLHandle,
                                            TRUE, TRUE);
    }
    /* Clear all previous settings */
    memset(&mldlCfg,  0, sizeof(mldlCfg));
    memset(&accel,    0, sizeof(accel));
    memset(&compass,  0, sizeof(compass));
    memset(&pdata,    0, sizeof(pdata));
    sMLSLHandle = NULL;
    return result;
}

/**
 * @brief Starts the DMP running
 *
 * @return ML_SUCCESS or non-zero error code 
 */
tMLError MLDLDmpStart(void)
{
    INVENSENSE_FUNC_START;
    tMLError result = ML_SUCCESS;

    if (mldlCfg.is_suspended) {
        result = mpu3050_resume(&mldlCfg,
                                sMLSLHandle,
                                sMLSLHandle,
                                sMLSLHandle,
                                mldlCfg.pdata->accel.get_slave_descr != 0,
                                mldlCfg.pdata->compass.get_slave_descr != 0);
    }
    return result;
}

/**
 * @brief Stops the DMP running and puts it in low power
 *
 * @return ML_SUCCESS or non-zero error code 
 */
tMLError MLDLDmpStop(void)
{
    INVENSENSE_FUNC_START;
    tMLError result = ML_SUCCESS;

    if (!mldlCfg.is_suspended) {
        result = mpu3050_suspend(&mldlCfg,
                                 sMLSLHandle,
                                 sMLSLHandle,
                                 sMLSLHandle,
                                 mldlCfg.pdata->accel.get_slave_descr != 0,
                                 mldlCfg.pdata->compass.get_slave_descr != 0);
    }
    return result;
}


/**
 *  @brief  Get a pointer to the internal data structure 
 *          storing the configuration for the MPU, the accelerometer 
 *          and the compass in use.
 *  @return a pointer to the data structure of type 'struct mldl_cfg'.
 */
struct mldl_cfg *MLDLGetCfg(void)
{
    return &mldlCfg;
}


/**
 *  @brief   Query the MPU slave address.
 *  @return  The 7-bit mpu slave address.
 */
unsigned char MLDLGetMPUSlaveAddr(void)
{
    INVENSENSE_FUNC_START;
    return mldlCfg.addr;
}

/**
 *  @internal
 * @brief   MLDLCfgDMP configures the Digital Motion Processor internal to 
 *          the MPU. The DMP can be enabled or disabled and the start address 
 *          can be set.
 *
 * @param   enableRun   Enables the DMP processing if set to TRUE.
 * @param   enableFIFO  Enables DMP output to the FIFO if set to TRUE.
 * @param   startAddress start address
 *
 * @return  Zero if the command is successful, an error code otherwise.
*/
tMLError MLDLCtrlDmp( unsigned char enableRun, 
                      unsigned char enableFIFO )
{
    INVENSENSE_FUNC_START;

    mldlCfg.dmp_enable  = enableRun;
    mldlCfg.fifo_enable = enableFIFO;

    return ML_SUCCESS;
}


/**
 * @brief   MLDLCfgInt configures the interrupt function on the specified pin.
 *          The basic interrupt signal characteristics can be set 
 *          (i.e. active high/low, open drain/push pull, etc.) and the 
 *          triggers can be set.
 *          Currently only INTPIN_MPU is supported.
 *
 * @param   triggers        
 *              bitmask of triggers to enable for interrupt.
 *              The available triggers are:
 *              - BIT_MPU_RDY_EN
 *              - BIT_DMP_INT_EN
 *              - BIT_RAW_RDY_EN
 *
 * @return  Zero if the command is successful, an error code otherwise.
*/
tMLError MLDLCfgInt(unsigned char triggers)
{
    tMLError result = ML_SUCCESS;
    if (triggers & !(BIT_MPU_RDY_EN | BIT_DMP_INT_EN | BIT_RAW_RDY_EN) ) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    mldlCfg.int_config = triggers;
    if (!mldlCfg.is_suspended) {
        result = MLSLSerialWriteSingle(
            sMLSLHandle, mldlCfg.addr,
            MPUREG_INT_CFG,
            (mldlCfg.int_config | mldlCfg.pdata->int_config));
    }

    return result;
}


/**
 * @brief   configures the output sampling rate on the MPU.  
 *          Three parameters control the sampling: 
 *
 *          1) Low pass filter bandwidth, and 
 *          2) output sampling divider. 
 *
 *          The output sampling rate is determined by the divider and the low 
 *          pass filter setting. If the low pass filter is set to 
 *          'MPUFILTER_256HZ_NOLPF2', then the sample rate going into the 
 *          divider is 8kHz; for all other settings it is 1kHz.  
 *          The 8-bit divider will divide this frequency to get the resulting 
 *          sample frequency.  
 *          For example, if the filter setting is not 256Hz and the divider is 
 *          set to 7, then the sample rate is as follows: 
 *          sample rate = internal sample rate / div = 1kHz / 8 = 125Hz (or 8ms).
 *          
 *          The low pass filter selection codes control both the cutoff frequency of 
 *          the internal low pass filter and internal analog sampling rate. The 
 *          latter, in turn, affects the final output sampling rate according to the 
 *          sample rate divider settig.
 *              0 -> 256 Hz  cutoff BW, 8 kHz analog sample rate,
 *              1 -> 188 Hz  cutoff BW, 1 kHz analog sample rate, 
 *              2 ->  98 Hz  cutoff BW, 1 kHz analog sample rate,
 *              3 ->  42 Hz  cutoff BW, 1 kHz analog sample rate, 
 *              4 ->  20 Hz  cutoff BW, 1 kHz analog sample rate, 
 *              5 ->  10 Hz  cutoff BW, 1 kHz analog sample rate, 
 *              6 ->   5 Hz  cutoff BW, 1 kHz analog sample rate, 
 *              7 -> 2.1 kHz cutoff BW, 8 kHz analog sample rate.
 *
 * @param   lpf         low pass filter,   0 to 7.
 * @param   divider     Output sampling rate divider, 0 to 255.
 *
 * @return  ML_SUCESS if successful; a non-zero error code otherwise.
**/
tMLError MLDLCfgSamplingMPU( unsigned char lpf, 
                             unsigned char divider )
{
    /*---- do range checking ----*/
    if( lpf >= NUM_MPU_FILTER ) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    mldlCfg.lpf = lpf;
    mldlCfg.divider = divider;

    return ML_SUCCESS;
}



/**
 *  @brief  set the full scale range for the gyros.
 *          The full scale selection codes correspond to:
 *              0 -> 250  dps,
 *              1 -> 500  dps,
 *              2 -> 1000 dps,
 *              3 -> 2000 dps.
 *          Full scale range affect the MPU's measurement
 *          sensitivity.
 *
 *  @param  range   
 *              the gyro full scale range in dps.
 *
 *  @return ML_SUCCESS or non-zero error code.
**/
tMLError MLDLSetFullScaleMPU( float fullScale )
{
    if (fullScale == 250.f)
        mldlCfg.full_scale = MPU_FS_250DPS;
    else if (fullScale == 500.f)
        mldlCfg.full_scale = MPU_FS_500DPS;
    else if (fullScale == 1000.f)
        mldlCfg.full_scale = MPU_FS_1000DPS;
    else if (fullScale == 2000.f)
        mldlCfg.full_scale = MPU_FS_2000DPS;
    else {  // not a valid setting
        MPL_LOGE("Invalid full scale range specification for gyros : %f\n", fullScale);
        MPL_LOGE("\tAvailable values : +/- 250 dps, +/- 500 dps, +/- 1000 dps, +/- 2000 dps\n");
        return ML_ERROR_INVALID_PARAMETER;
    }

    return ML_SUCCESS;
}


/**
 * @brief   This function sets the external sync for the MPU sampling.  
 *          It can be synchronized on the LSB of any of the gyros, any of the 
 *          external accels, or on the temp readings.
 *
 * @param   extSync External sync selection, 0 to 7.
 * @return  Zero if the command is successful; an error code otherwise.
**/
tMLError MLDLSetExternalSyncMPU( unsigned char extSync )
{
    INVENSENSE_FUNC_START;

    /*---- do range checking ----*/
    if( extSync >= NUM_MPU_EXT_SYNC ) {
        return ML_ERROR_INVALID_PARAMETER;
    }
    mldlCfg.ext_sync = extSync;

    return ML_SUCCESS;
}

/** 
 * @brief Sets the power state for the Gyro's.
 *
 * This sets the power state of each gyro to be applied the next time 
 * mpu3050_resume is called.
 * 
 * @param xOn Boolean to turn on the X gyro
 * @param yOn Boolean to turn on the Y gyro
 * @param zOn Boolean to turn on the Z gyro
 * 
 * @return ML_SUCCESS (currently cannot fail)
 */
tMLError MLDLSetGyroPower(unsigned char xOn, 
                          unsigned char yOn, 
                          unsigned char zOn)
{
    unsigned char power = 0;
    if (!xOn) power |= BIT_STBY_XG;
    if (!yOn) power |= BIT_STBY_YG;
    if (!zOn) power |= BIT_STBY_ZG;

    mldlCfg.gyro_power = power;

    return ML_SUCCESS;
}

/**
 * @brief   MLDLClockSource function sets the clock source for the MPU gyro 
 *          processing.  
 *          The source can be any of the following: 
 *          - Internal 8MHz oscillator, 
 *          - PLL with X gyro as reference, 
 *          - PLL with Y gyro as reference, 
 *          - PLL with Z gyro as reference, 
 *          - PLL with external 32.768Mhz reference, or 
 *          - PLL with external 19.2MHz reference
 *          
 *          For best accuracy and timing, it is highly recommended to use one 
 *          of the gyros as the clock source; however this gyro must be 
 *          enabled to use its clock (see 'MLDLPowerMgmtMPU()').
 *
 * @param   clkSource   Clock source selection. 
 *                      Can be one of:
 *                      - CLK_INTERNAL,
 *                      - CLK_PLLGYROX,
 *                      - CLK_PLLGYROY,
 *                      - CLK_PLLGYROZ,
 *                      - CLK_PLLEXT32K, or
 *                      - CLK_PLLEXT19M.
 *
 * @return  Zero if the command is successful; an error code otherwise.
**/
tMLError MLDLClockSource( unsigned char clkSource )
{
    INVENSENSE_FUNC_START;

    /*---- do range checking ----*/
    if( clkSource >= NUM_CLK_SEL ) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    mldlCfg.clk_src = clkSource;

    return ML_SUCCESS;
}


/**
 *  @brief  Set the Temperature Compensation offset.
 *  @param  tc
 *              a pointer to the temperature compensations offset
 *              for the 3 gyro axes.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLDLSetOffsetTC(unsigned char const *tc)
{
    int ii;
    tMLError result;
    if (mldlCfg.is_suspended) {
        for (ii = 0; ii < sizeof(mldlCfg.offset_tc); ii++){
            mldlCfg.offset_tc[ii] = tc[ii];
        }
    } else {
        result = MLSLSerialWriteSingle(sMLSLHandle, mldlCfg.addr,
                                       MPUREG_XG_OFFS_TC, tc[0]);
        ERROR_CHECK(result);
        result = MLSLSerialWriteSingle(sMLSLHandle, mldlCfg.addr,
                                       MPUREG_YG_OFFS_TC, tc[1]);
        ERROR_CHECK(result);
        result = MLSLSerialWriteSingle(sMLSLHandle, mldlCfg.addr,
                                       MPUREG_ZG_OFFS_TC, tc[2]);
        ERROR_CHECK(result);
    }
    return ML_SUCCESS;
}

/**
 *  @internal
 *  @brief  used to get the specified number of bytes in the specified MPU 
 *          memory bank. 
 *          The memory bank is one of the following: 
 *          - MPUMEM_RAM_BANK_0, 
 *          - MPUMEM_RAM_BANK_1, 
 *          - MPUMEM_RAM_BANK_2, or 
 *          - MPUMEM_RAM_BANK_3.
 *
 *  @param  bank    Memory bank to write.
 *  @param  memAddr Starting address for write.
 *  @param  length  Number of bytes to write.
 *  @param  buffer  Result for data.
 *
 *  @return zero if the command is successful, an error code otherwise.
 *  @endif
 */
tMLError
MLDLGetMemoryMPUOneBank( unsigned char  bank,   
                         unsigned char  memAddr, 
                         unsigned short length, 
                         unsigned char* buffer )
{
    tMLError result;

    if ((bank >= MPU_MEM_NUM_RAM_BANKS) || 
        //(memAddr >= MPU_MEM_BANK_SIZE) || always 0, memAddr is an u_char, therefore limited to 255
        ((memAddr + length) > MPU_MEM_BANK_SIZE) || (NULL == buffer)) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    if (mldlCfg.is_suspended) {
        memcpy(buffer,&mldlCfg.ram[bank][memAddr],length);
        result = ML_SUCCESS;
    } else {
        result = MLSLSerialReadMem(sMLSLHandle, mldlCfg.addr, 
                                   ((bank << 8) | memAddr),
                                   length, buffer);
    }

    return result;
}

/**
 *  @internal
 *  @brief  used to set the specified number of bytes in the specified MPU 
 *          memory bank. 
 *          The memory bank is one of the following: 
 *          - MPUMEM_RAM_BANK_0, 
 *          - MPUMEM_RAM_BANK_1, 
 *          - MPUMEM_RAM_BANK_2, or 
 *          - MPUMEM_RAM_BANK_3.
 *
 *  @param  bank    Memory bank to write.
 *  @param  memAddr Starting address for write.
 *  @param  length  Number of bytes to write.
 *  @param  buffer  Result for data.
 *
 *  @return zero if the command is successful, an error code otherwise.
 *  @endif
 */
tMLError MLDLSetMemoryMPUOneBank( unsigned char bank, 
                                  unsigned short memAddr, 
                                  unsigned short length, 
                                  unsigned char const* buffer )
{
    tMLError result;

    if ((bank >= MPU_MEM_NUM_RAM_BANKS) || (memAddr >= MPU_MEM_BANK_SIZE) ||
        ((memAddr + length) > MPU_MEM_BANK_SIZE) || (NULL == buffer)) {
        return ML_ERROR_INVALID_PARAMETER;
    }

    if (mldlCfg.is_suspended) {
        memcpy(&mldlCfg.ram[bank][memAddr],buffer,length);
        result = ML_SUCCESS;
    } else {
        result = MLSLSerialWriteMem(sMLSLHandle, mldlCfg.addr, 
                                    ((bank << 8) | memAddr),
                                    length, buffer);
    }

    return result;
}

/**
 *  @internal 
 *  @brief  used to get the specified number of bytes from the MPU location
 *          specified by the key.
 *          Reads the specified number of bytes from the MPU location
 *          specified by the key. Each set of code specifies a function
 *          that changes keys into addresses. This function is set with 
 *          setGetAddress().
 *
 *  @param  key     The key to use when looking up the address.
 *  @param  length  Number of bytes to read.
 *  @param  buffer  Result for data.
 *
 *  @return ML_SUCCESS if the command is successful, ML_ERROR otherwise. The key
 *          not corresponding to a memory address will result in ML_ERROR.
 *  @endif
 */
tMLError MLDLGetMemoryMPU( unsigned short key,
                           unsigned short length, 
                           unsigned char* buffer )
{
    unsigned char  bank;
    tMLError result;
    unsigned short memAddr;

    if ( sGetAddress == NULL ) {
        return ML_ERROR_NOT_OPENED;
    }

    memAddr = sGetAddress( key );
    if ( memAddr >= 0xffff )
        return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
    bank = memAddr >> 8; // Get Bank
    memAddr &= 0xff;

    while( memAddr + length > MPU_MEM_BANK_SIZE ) {
        // We cross a bank in the middle
        unsigned short sub_length = MPU_MEM_BANK_SIZE - memAddr;
        result = MLDLGetMemoryMPUOneBank(bank, (unsigned char)memAddr, 
                                         sub_length, buffer );
        if (ML_SUCCESS != result)
            return result;
        bank++;
        length -= sub_length;
        buffer += sub_length;
        memAddr = 0;
    }
    result = MLDLGetMemoryMPUOneBank(bank, (unsigned char)memAddr, 
                                      length, buffer );

    return result;
}

/**
 *  @internal
 *  @brief  used to set the specified number of bytes from the MPU location
 *          specified by the key.
 *          Set the specified number of bytes from the MPU location
 *          specified by the key. Each set of DMP code specifies a function
 *          that changes keys into addresses. This function is set with
 *          setGetAddress().
 *
 *  @param  key     The key to use when looking up the address.
 *  @param  length  Number of bytes to write.
 *  @param  buffer  Result for data.
 *
 *  @return ML_SUCCESS if the command is successful, ML_ERROR otherwise. The key
 *          not corresponding to a memory address will result in ML_ERROR.
 *  @endif
 */
tMLError MLDLSetMemoryMPU( unsigned short key, 
                           unsigned short length, 
                           const unsigned char* buffer )
{
    tMLError       result = ML_SUCCESS;
    unsigned short memAddr;
    unsigned char  bank;

    if ( sGetAddress == NULL ) {
        MPL_LOGE("MLDSetMemoryMPU sGetAddress is NULL\n");
        return ML_ERROR_INVALID_MODULE;
    }
    memAddr = sGetAddress( key );

    if ( memAddr >= 0xffff ) {
        MPL_LOGE("MLDLSetMemoryMPU unsupported key\n");
        return ML_ERROR_INVALID_MODULE; // This key not supported
    }

    bank = (unsigned char)(memAddr >> 8);
    memAddr &= 0xff;

    while ( memAddr + length > MPU_MEM_BANK_SIZE ) {
        // We cross a bank in the middle
        unsigned short sub_length = MPU_MEM_BANK_SIZE - memAddr;

        result = MLDLSetMemoryMPUOneBank( bank, memAddr, sub_length, buffer );
        if (ML_SUCCESS != result)
            return result;
        bank++;
        length  -= sub_length;
        buffer  += sub_length;
        memAddr  = 0;
    }
    result = MLDLSetMemoryMPUOneBank( bank, memAddr, length, buffer );

    return result;
}


/**
 *  @brief  Load the DMP with the given code and configuration.
 *  @param  buffer
 *              the DMP data.
 *  @param  length
 *              the length in bytes of the DMP data.
 *  @param  config
 *              the DMP configuration.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError MLDLLoadDMP( const unsigned char *buffer, 
                      unsigned short length, 
                      unsigned short config )
{
    INVENSENSE_FUNC_START;

    tMLError result = ML_SUCCESS;
    unsigned short toWrite;
    unsigned short memAddr = 0;
    localDmpMemory = buffer;
    localDmpMemorySize = length;

    mldlCfg.dmp_cfg1    = (config >> 8 );
    mldlCfg.dmp_cfg2    = (config & 0xff );

    while ( length > 0) {
        toWrite = length;
        if ( toWrite > MAX_LOAD_WRITE_SIZE )
            toWrite = MAX_LOAD_WRITE_SIZE;

        result = MLDLSetMemoryMPUOneBank( memAddr >> 8, memAddr & 0xff, toWrite, buffer );
        ERROR_CHECK(result);

        buffer += toWrite;
        memAddr += toWrite;
        length -= toWrite;
    }

    return result;
}

/**
 *  @internal
 *  @brief  Get the silicon revision ID from.
 *  @return The silicon revision ID 
 *          (0 will be read if mpu3050_open returned an error)
 */
unsigned char MLDLGetSiliconRev(void)
{
    return mldlCfg.silicon_revision;
}

/**
 *  @brief      Enable/Disable the use MPU's VDDIO level shifters.
 *              When enabled the voltage interface with AUX or other external 
 *              accelerometer is using Vlogic instead of VDD (supply).
 *  
 *  @note       Must be called after MLSerialOpen().
 *  @note       Typically be called before MLDmpOpen().
 *              If called after MLDmpOpen(), must be followed by a call to
 *              MLDLApplyLevelShifterBit() to write the setting on the hw.
 * 
 *  @param[in]  enable
 *                  1 to enable, 0 to disable
 *
 *  @return     ML_SUCCESS if successfull, a non-zero error code otherwise.
*/
tMLError MLDLSetLevelShifterBit(unsigned char enable) 
{
    mldlCfg.pdata->level_shifter = enable;
    
    return ML_SUCCESS;
} 

/*******************************************************************************
 *******************************************************************************
 *******************************************************************************
 * @todo these belong with an interface to the kernel driver layer
 *******************************************************************************
 *******************************************************************************
 ******************************************************************************/

/**
 * @brief   MLDLGetIntStatus returns the interrupt status from the specified 
 *          interrupt pin.
 * @param   intPin  
 *              Currently only the value INTPIN_MPU is supported. 
 * @param   status
 *              The available statuses are:
 *              - BIT_MPU_RDY_EN
 *              - BIT_DMP_INT_EN
 *              - BIT_RAW_RDY_EN
 *
 * @return  ML_SUCCESS or a non-zero error code.
 */
tMLError MLDLGetIntStatus( unsigned char intPin, unsigned char* status )
{
    INVENSENSE_FUNC_START;

    tMLError result;

    switch( intPin ) {

        case INTPIN_MPU:
            /*---- return the MPU interrupt status ----*/
            result = MLSLSerialRead(sMLSLHandle, mldlCfg.addr, 
                                    MPUREG_INT_STATUS, 1, status);
            break;

        default:
            result = ML_ERROR_INVALID_PARAMETER;
            break;
    }

    return result;
}

/**
 *  @brief   query the current status of an interrupt source.
 *  @param   srcIndex 
 *              index of the interrupt source.
 *              Currently the only source supported is INTPIN_MPU.
 *
 *  @return  1 if the interrupt has been triggered.
 */
unsigned char MLDLGetIntTrigger(unsigned char srcIndex)
{
    INVENSENSE_FUNC_START;
    return intTrigger[srcIndex];
}

/**
 * @brief clear the 'triggered' status for an interrupt source.
 * @param srcIndex 
 *          index of the interrupt source.
 *          Currently only INTPIN_MPU is supported.
 */
void MLDLClearIntTrigger(unsigned char srcIndex)
{
    INVENSENSE_FUNC_START;
    intTrigger[srcIndex] = 0;
}

/**
 * @brief   MLDLIntHandler function should be called when an interrupt is 
 *          received.  The source parameter identifies which interrupt source 
 *          caused the interrupt. Note that this routine should not be called 
 *          directly from the interrupt service routine.
 *
 * @param   intSource   MPU, AUX1, AUX2, or timer. Can be one of: INTSRC_MPU, INTSRC_AUX1,
 *                      INTSRC_AUX2, or INT_SRC_TIMER.
 *
 * @return  Zero if the command is successful; an error code otherwise.
 */
tMLError MLDLIntHandler(unsigned char intSource)
{
    INVENSENSE_FUNC_START;
    /*---- range check ----*/
    if( intSource >= NUM_OF_INTSOURCES ) {
        return ML_ERROR;
    }

    /*---- save source of interrupt ----*/
    intTrigger[intSource] = INT_TRIGGERED;

#ifdef ML_USE_DMP_SIM
    if(intSource == INTSRC_AUX1 || intSource == INTSRC_TIMER) {
        MLSimHWDataInput();
    }
#endif

    return ML_SUCCESS;
}

/***************************/
/**@}*/ /* end of defgroup */
/***************************/
