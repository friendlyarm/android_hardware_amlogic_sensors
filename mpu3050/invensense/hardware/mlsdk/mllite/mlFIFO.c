/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mlFIFO.c 4175 2010-11-25 01:24:24Z nroyer $
 *
 *******************************************************************************/

/**
 *   @defgroup MLFIFO
 *   @brief Motion Library - FIFO Driver.
 *          The FIFO API Interface.
 *
 *   @{
 *       @file mlFIFO.c
 *       @brief FIFO Interface.
**/
 
#include <string.h>
#include "mlFIFO.h"
#include "mlFIFOHW.h"
#include "dmpKey.h"
#include "mlMathFunc.h"
#include "ml.h"
#include "mldl.h"
#include "mldl_cfg.h"
#include "mlstates.h"
#include "mlsupervisor.h"
#include "mlos.h"
#include "log.h"

static tMLError setFIFOFooter();

#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-fifo"

#define REF_QUATERNION (0)
#define REF_GYROS (REF_QUATERNION+4)
#define REF_ACCEL (REF_GYROS+3)
#define REF_GRAVITY (REF_ACCEL+3)
#define REF_LINEAR_ACCEL (REF_GRAVITY+3)
#define REF_LINEAR_ACCEL_WORLD (REF_LINEAR_ACCEL+3)
#define REF_CONTROL (REF_LINEAR_ACCEL_WORLD+3)
#define REF_BIAS_UNCERT (REF_CONTROL+4)
#define REF_RAW (REF_BIAS_UNCERT+1)
#define REF_RAW_EXTERNAL (REF_RAW+7)
#define REF_TAP (REF_RAW_EXTERNAL+3)

#define REF_QUATERNION_6AXIS (REF_TAP+ML_MAX_NUM_TAP_SAMPLES)
#define REF_DMP_PACKET (REF_QUATERNION_6AXIS+4)
#define REF_GARBAGE (REF_DMP_PACKET+3)
#define REF_LAST (REF_GARBAGE+1)

long FIFOScale[REF_LAST] = { (1L<<30),(1L<<30),(1L<<30),(1L<<30), // Quaternion
                             // 2^(16+30)/((2^30)*((3.14159265358/180)/200)/2)
                             1501974482L, 1501974482L, 1501974482L, // Gyro
                             (1L<<16),(1L<<16),(1L<<16), // Accel
                             (1L<<16),(1L<<16),(1L<<16), // Gravity
                             (1L<<16),(1L<<16),(1L<<16), // Linear Accel
                             (1L<<16),(1L<<16),(1L<<16), // Linear Accel World
                             (1L<<30),(1L<<30),(1L<<30),(1L<<30), // Control
                             (1L<<26), // Bias Uncertainty
                             (1L<<14)*234, // Temperature
                             (1L<<14),(1L<<14),(1L<<14), // Raw Gyro
                             (1L<<14),(1L<<14),(1L<<14), // Raw Accel
                             (1L<<14),(1L<<14),(1L<<14), // Raw External
                             (1L<<30),(1L<<30),(1L<<30),(1L<<30), // Tap
                             (1L<<30),(1L<<30),(1L<<30),(1L<<30) }; // Tap
// The scale factors for tap need to match the number in FIFOScale above.
// FIFOOffsetBase below may also need to be changed if this is not 8
#if ML_MAX_NUM_TAP_SAMPLES != 8
#error
#endif

#define CONFIG_QUAT (0)
#define CONFIG_GYROS (CONFIG_QUAT+1)
#define CONFIG_ACCELS (CONFIG_GYROS+1)
#define CONFIG_GRAV (CONFIG_ACCELS+1)
#define CONFIG_LINEAR_ACCEL (CONFIG_GRAV+1)
#define CONFIG_LINEAR_ACCEL_WORLD (CONFIG_LINEAR_ACCEL+1)
#define CONFIG_CONTROL_DATA (CONFIG_LINEAR_ACCEL_WORLD+1)
#define CONFIG_BIAS_UNCERTAINTY (CONFIG_CONTROL_DATA+1)
#define CONFIG_TEMPERATURE (CONFIG_BIAS_UNCERTAINTY+1)
#define CONFIG_RAW_DATA (CONFIG_TEMPERATURE+1)
#define CONFIG_RAW_EXTERNAL (CONFIG_RAW_DATA+1)
#define CONFIG_DMP_TAP (CONFIG_RAW_EXTERNAL+1)
#define CONFIG_DMP_TAP2 (CONFIG_DMP_TAP+1)
#define CONFIG_DMP_PACKET_NUMBER (CONFIG_DMP_TAP2+1)
#define CONFIG_FOOTER (CONFIG_DMP_PACKET_NUMBER+1)
#define NUMFIFOELEMENTS (CONFIG_FOOTER+1)


const int FIFOOffsetBase[NUMFIFOELEMENTS]={ REF_QUATERNION*4, REF_GYROS*4,
                                      REF_ACCEL*4, REF_GRAVITY*4, REF_LINEAR_ACCEL*4,
                                      REF_LINEAR_ACCEL_WORLD*4, REF_CONTROL*4, REF_BIAS_UNCERT*4,
                                      REF_RAW*4, REF_RAW*4+4, REF_RAW_EXTERNAL*4, 
                                      REF_TAP*4, REF_TAP*4+16, 
                                      REF_DMP_PACKET*4,  REF_GARBAGE*4 };

typedef struct _tFIFOData {
    void (*FIFOProcessCB)(void);
    long decoded[REF_LAST];
    long mlTap[ML_MAX_NUM_TAP_SAMPLES];
    int offsets[REF_LAST*4];
    unsigned short mlFIFORate;
    uint_fast16_t mlFIFOPacketSize;
    uint_fast8_t mlFIFODataConfig[NUMFIFOELEMENTS];
    unsigned char ReferenceCount[REF_LAST];
} tFIFOData;

static tFIFOData FIFOData;

typedef struct {
    // These describe callbacks happening everytime a FIFO block is processed
    int_fast8_t numHighrateProcesses;
    HANDLE mutex;
    tMlxdataFunction highrateProcess[MAX_HIGH_RATE_PROCESSES];
} tMLXCallback;     // MLX_callback_t

tMLXCallback mlxCallback; 

/** 
 * @internal
 * @brief get the FIFO packet size
 * @return the FIFO packet size
 */
uint_fast16_t FIFOGetPacketSize(void) {
    return FIFOData.mlFIFOPacketSize;
}

/**
 *  @brief  Initializes all the internal static variables for 
 *          the FIFO module.
 *  @note   Should be called by the initialization routine such 
 *          as MLDmpOpen().
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise. 
 */
tMLError FIFOParamInit(void)
{
    tMLError result;
    memset( &FIFOData, 0, sizeof( tFIFOData ) );
    FIFOData.decoded[REF_QUATERNION] = 1073741824L; // Set to Identity
    FIFOData.mlFIFORate = 20;
    memset( &mlxCallback, 0, sizeof(tMLXCallback) );
    result = MLOSCreateMutex(&mlxCallback.mutex);
    return result;
}

/**
 *  @brief  Close the FIFO usage.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOClose(void) 
{
    tMLError result;
    result = MLOSDestroyMutex(mlxCallback.mutex);
    memset( &mlxCallback, 0, sizeof(tMLXCallback) );
    return result;
}

/** 
 *  @brief  Reads and processes FIFO data.
 *  @param  numPackets 
 *              Number of FIFO packets to try to read. You should
 *              use a large number here, such as 100, if you want to read all
 *              the full packets in the FIFO, which is typical operation.
 *  @param  processed
 *              the number of FIFO packet processed.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError readAndProcessFIFO( int_fast8_t numPackets, int_fast8_t *processed )
{
    int_fast8_t packet;
    unsigned char footer_n_data[MAX_FIFO_LENGTH+FIFO_FOOTER_SIZE];
    unsigned char *buf = &footer_n_data[FIFO_FOOTER_SIZE];
    tMLError result = ML_SUCCESS;
    uint_fast16_t read;

    if (NULL == processed) 
        return ML_ERROR_INVALID_PARAMETER;

    *processed = 0;
    if ( FIFOData.mlFIFOPacketSize == 0 )
        return result; // Nothing to read

    for (packet=0; packet<numPackets; ++packet) {
        read = MPUGetFIFO((uint_fast16_t)FIFOData.mlFIFOPacketSize, 
                          footer_n_data);
        if (0 == read || read != FIFOData.mlFIFOPacketSize-FIFO_FOOTER_SIZE ) {
            return MLDLGetFIFOStatus();
        }

        result = MLProcessFIFOData(buf);
        ERROR_CHECK(result);

        result = MLAccelCompassSupervisor();
        ERROR_CHECK(result);

        // Callbacks now that we have a buffer of data ready
        result = RunHighRateProcessFuncs();
        ERROR_CHECK(result);

        *processed = *processed + 1;
    }
    return result;
}

/**
 *  @brief  MLSetProcessedDataCallback is used to set a processed data callback
 *          function.  MLSetProcessedDataCallback sets a user defined callback
 *          function that triggers when all the decoding has been finished by
 *          the motion processing engines. It is called before other bigger 
 *          processing engines to allow lower latency for the user.
 *
 *  @pre    MLDmpOpen() with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 *  @param  func    A user defined callback function.
 *
 *  @return ML_SUCCESS if successful, or non-zero error code otherwise.
 */
tMLError MLSetProcessedDataCallback(void (*func)(void) )
{
    INVENSENSE_FUNC_START;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;    

    FIFOData.FIFOProcessCB = func;

    return ML_SUCCESS;
}

/**
 * @internal
 * @brief   Process data from the dmp read via the fifo.  Takes a buffer 
 *          filled with bytes read from the DMP FIFO. 
 *          Currently expects only the 16 bytes of quaternion data. 
 *          Calculates the motion parameters from that data and stores the 
 *          results in an internal data structure.
 * @param[in,out]   dmpData     Pointer to the buffer containing the fifo data.
 * @return  ML_SUCCESS or error code.
**/
tMLError MLProcessFIFOData(const unsigned char* dmpData)
{           
    INVENSENSE_FUNC_START;
    int N,kk;
    unsigned char *p;

    p = (unsigned char *)(&FIFOData.decoded);
    N = FIFOData.mlFIFOPacketSize;
    if ( N > sizeof(FIFOData.decoded) )
        return ML_ERROR;
    memset(&FIFOData.decoded,0,sizeof(FIFOData.decoded));

    for (kk=0; kk<N; ++kk) {
        p[ FIFOData.offsets[kk] ] = *dmpData++;
    }

    // If multiplies are much greater cost than if checks, you could check
    // to see if FIFOScale is non-zero first, or equal to (1L<<30)
    for (kk=0; kk<REF_LAST; ++kk) {
        FIFOData.decoded[kk] = q30_mult(FIFOData.decoded[kk], FIFOScale[kk]);
    }

    memcpy(&FIFOData.decoded[REF_QUATERNION_6AXIS], &FIFOData.decoded[REF_QUATERNION], 4*sizeof(long));

    mlxData.mlFlags[ML_PROCESSED_DATA_READY] = 1;

    return ML_SUCCESS;
}

/** 
 *  @internal
 *  @brief  Turns temperature register values into Temperature and 
 *          sets the temperature element in FIFOData.
 *  @param  reg
 *              the temperature reading.
 */
void decodeTemperature(const unsigned char *reg)
{
    FIFOData.decoded[REF_RAW] = (((long)reg[0]<<8)+((long)reg[1]));
    if (FIFOData.decoded[REF_RAW] > 32768L)
        FIFOData.decoded[REF_RAW] -= 65536L;
    
    FIFOData.decoded[REF_RAW] = q30_mult(FIFOData.decoded[REF_RAW]<<16, FIFOScale[REF_RAW]);
}

/**
 * Decodes Gyro Sensor Data into 16-bit values based upon flags
 * @param[in] regs 8-bit data to decode
 * @param[in] axes lower 3-bits represent whether to decode and process 
 *                 z,y,x axes, with x being LSB
 * @return Number of terms processed out of array
 */
int_fast8_t decodeGyroSensorData(const unsigned char *regs, uint_fast8_t axes )
{
    int_fast8_t i,cnt=0;
    
    // Gyro Raw Data. Note: Raw data does not have any mounting transformation applied
    for (i=0; i<3; ++i) {
        if ( 1 & (axes>>i) ) {
            FIFOData.decoded[REF_RAW+i] = ((int)regs[cnt]<<8)+((int)regs[cnt+1]);
            if (FIFOData.decoded[REF_RAW+i]>32767)
                FIFOData.decoded[REF_RAW+i]-=65536L;
            cnt += 2;
        }
    }
    return cnt;
}

/** Sets accuracy to be one of 0, ML_32_BIT, or ML_16_BIT. Looks up old accuracy if needed.
*/
static uint_fast8_t FIFOSetAccuracy(uint_fast8_t elements, uint_fast8_t accuracy, uint_fast8_t configOffset)
{
    if ( elements ) {
        if ( (accuracy & (ML_16_BIT | ML_32_BIT)) == 0 )
            accuracy = FIFOData.mlFIFODataConfig[configOffset] & (ML_32_BIT | ML_16_BIT);
        else if (accuracy & ML_16_BIT ) 
            if ( (FIFOData.mlFIFODataConfig[configOffset] & ML_32_BIT) == ML_32_BIT )
                accuracy = ML_32_BIT; // 32-bits takes priority
            else
                accuracy = ML_16_BIT;
        else
            accuracy = ML_32_BIT;
    } else {
        accuracy = 0;
    }
    return accuracy;
}

/** Adjusts (len) Reference Counts, at offset (refOffset). If increment is 0, the
* reference counts are subtracted, otherwise they are incremented for each bit set
* in element. The value returned are the elements that should be sent out as data
* through the FIFO.
*/
static uint_fast8_t FIFOSetReference( uint_fast8_t elements, uint_fast8_t increment,
                                      uint_fast8_t refOffset, uint_fast8_t len )
{
    uint_fast8_t kk;

    if ( increment == 0 ) {
        for (kk=0; kk<len; ++kk) {
            if ( ( elements & 1 ) && (FIFOData.ReferenceCount[kk+refOffset] > 0) ) {
                FIFOData.ReferenceCount[kk+refOffset]--;
            }
            elements >>= 1;
        }
    } else {
        for (kk=0; kk<len; ++kk) {
            if ( elements & 1 )
                FIFOData.ReferenceCount[kk+refOffset]++;
            elements >>= 1;
        }
    }
    elements = 0;
    for (kk=0; kk<len; ++kk) {
        if ( FIFOData.ReferenceCount[kk+refOffset] > 0 )
            elements |= (1<<kk);
    }
    return elements;
}

/**
* @param[in] accuracy ML_16_BIT or ML_32_BIT when constructing data to send out the FIFO, 0 when removing
*            from the FIFO.
*/
static tMLError FIFOConstruct3(unsigned char *regs, uint_fast8_t elements, uint_fast8_t accuracy,
    uint_fast8_t refOffset, unsigned short key, uint_fast8_t configOffset)
{
    int_fast8_t kk;
    tMLError result;

    elements = FIFOSetReference( elements, accuracy, refOffset, 3 );
    accuracy = FIFOSetAccuracy( elements, accuracy, configOffset );

    if ( accuracy & ML_16_BIT ) {
        regs[0] = DINAF8 + 2;
    }

    FIFOData.mlFIFODataConfig[configOffset] = elements | accuracy;

    for (kk=0; kk<3; ++kk) {
        if ( (elements & 1) == 0 )
            regs[kk+1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(key, 4, regs);

    return result;
}


/** 
 *  @brief      Returns 1-element vector of temperature
 *  @param[out] data    1-element vector of temperature
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetTemperature(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_RAW] + 2293760L + 13200L*234;
    return ML_SUCCESS;
}


/**
 *  @brief  Get the bias uncertainty tracking value.
 *  @param  data
 *              a buffer to store the bias uncertainty data.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
long FIFOGetBiasUncertainty(void)
{
    return FIFOData.decoded[REF_BIAS_UNCERT] += (1L<<14);
}

/**
 *  @brief  Get the TAP algorithm output from the FIFO.
 *  @param  data
 *              a buffer to store the tap data.
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOGetTap(long* data)
{
    int ii;
    if ( data == NULL )
        return ML_ERROR;

    for (ii = 0; ii < ML_MAX_NUM_TAP_SAMPLES; ii++){
        data[ii] = FIFOData.decoded[REF_TAP+ii];
    }

    return ML_SUCCESS;
}


/** 
 *  @brief      Returns 6-element vector of gyro and accel data
 *  @param[out] data    6-element vector of gyro and accel data
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetSensorData(long *data)
{
    int ii;
    if ( data == NULL )
        return ML_ERROR;

    for (ii = 0; ii < 6; ii++){
        data[ii] = FIFOData.decoded[REF_RAW+1+ii];
    }

    return ML_SUCCESS;
}


/** 
 *  @brief      Returns 3-element vector of external sensor
 *  @param[out] data    3-element vector of external sensor
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetExternalSensorData(long *data)
{
#ifdef M_HW
    int ii;
    if ( data == NULL )
        return ML_ERROR;

    for (ii = 0; ii < 3; ii++){
        data[ii] = FIFOData.decoded[REF_RAW_EXTERNAL+ii];
    }

    return ML_SUCCESS;
#else
    return ML_ERROR;
#endif
}

/** Sends accelerometer data to the FIFO. Accelerometer data is a 3 length vector
 *  of 32-bits. Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 * @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *            bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendAccel(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[4] = { DINAF8 + 1, DINA28, DINA50, DINA78};
    tMLError result;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;
 
    result = FIFOConstruct3( regs, elements, accuracy, REF_ACCEL,
                             KEY_CFG_10, CONFIG_ACCELS  );
    ERROR_CHECK( result );

    FIFOScale[REF_ACCEL+0] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_ACCEL+1] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_ACCEL+2] = 2*mlxData.mlAccelSens;

    return setFIFOFooter();
}


/**
 *  @brief  Send the computed bias uncertainty into the FIFO.
 *          The bias uncertainty can be retrieved from the FIFO via 
 *          FIFOGetBiasUncertainty().
 *
 *  @param  elements
 *              the bias uncertainty components bitmask.
 *              To send all compoents use ML_ALL.
 *  @param  accuracy
 *              The number of bits the bias uncertainty is expressed 
 *              into.
 *              Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *              bit data. 
 *              Set to zero to remove it from the FIFO.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOSendBiasUncertainty(uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char regs;
    uint_fast8_t elements;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference( 1, accuracy, REF_BIAS_UNCERT, 1 );
    if ( elements & 1 ) {
        regs = DINA28;
        FIFOData.mlFIFODataConfig[CONFIG_BIAS_UNCERTAINTY] = ML_ELEMENT_1 | ML_16_BIT;
    } else {
        regs = DINAF8+3;
        FIFOData.mlFIFODataConfig[CONFIG_BIAS_UNCERTAINTY] = 0;
    }
    result = MLDLSetMemoryMPU(KEY_CFG_14, 1, &regs);
    ERROR_CHECK( result );

    return setFIFOFooter();
}

/** Sends control data to the FIFO. Control data is a 4 length vector of 32-bits. 
 *  Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 4 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3, ML_ELEMENT_4 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendControlData(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    int_fast8_t kk;
    tMLError result;
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28, DINA30, DINA38};

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference( elements, accuracy, REF_CONTROL, 4 );
    accuracy = FIFOSetAccuracy( elements, accuracy, CONFIG_CONTROL_DATA );

    if ( accuracy & ML_16_BIT ) {
        regs[0] = DINAF8 + 2;
    }

    FIFOData.mlFIFODataConfig[CONFIG_CONTROL_DATA] = elements | accuracy;

    for (kk=0; kk<4; ++kk) {
        if ( (elements & 1) == 0 )
            regs[kk+1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_1, 5, regs);
    ERROR_CHECK( result );

    return setFIFOFooter();
}


/** 
 * Adds a rolling counter to the fifo packet.  When used with the footer
 * the data comes out the first time:
 * 
 * @code
 * <data0><data1>...<dataN><PacketNum0><PacketNum1>
 * @endcode
 * for every other packet it is
 *
 * @code
 * <FifoFooter0><FifoFooter1><data0><data1>...<dataN><PacketNum0><PacketNum1>
 * @endcode
 *
 * This allows for scanning of the fifo for packets
 * 
 * @return ML_SUCCESS or error code
 */
tMLError FIFOSendDMPPacketNumber(uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    tMLError result;
    unsigned char regs;
    uint_fast8_t elements;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference( 1, accuracy, REF_DMP_PACKET, 1 );
    if ( elements & 1 ) {
        regs = DINA28;
        FIFOData.mlFIFODataConfig[CONFIG_DMP_PACKET_NUMBER] = ML_ELEMENT_1 | ML_16_BIT;
    } else {
        regs = DINAF8+3;
        FIFOData.mlFIFODataConfig[CONFIG_DMP_PACKET_NUMBER] = 0;
    }
    result = MLDLSetMemoryMPU(KEY_CFG_23, 1, &regs);
    ERROR_CHECK( result );

    return setFIFOFooter();
}


/**
 *  @brief  Send the computed gravity vectors into the FIFO.
 *          The gravity vectors can be retrieved from the FIFO via 
 *          FIFOGetGravBody(), to have the gravitation vector expressed
 *          in coordinates relative to the body.
 *
 *  @param  elements
 *              the gravitation vectors components bitmask.
 *              To send all compoents use ML_ALL.
 *  @param  accuracy
 *              The number of bits the gravitation vector is expressed 
 *              into.
 *              Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *              bit data. 
 *              Set to zero to remove it from the FIFO.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOSendGravity(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[4] = { DINAF8 + 1, DINA28, 
                              DINA30, DINA38 };
    tMLError result;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    result = FIFOConstruct3( regs, elements, accuracy, REF_GRAVITY, 
                             KEY_CFG_11, CONFIG_GRAV );
    ERROR_CHECK( result );

    FIFOScale[REF_GRAVITY+0] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_GRAVITY+1] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_GRAVITY+2] = 2*mlxData.mlAccelSens;

    return setFIFOFooter();
}


/** Sends gyro data to the FIFO. Gyro data is a 3 length vector
 *  of 32-bits. Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendGyro(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[4] = { DINAF8 + 1, DINA20, DINA28, DINA30};
    tMLError result;    

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    result = FIFOConstruct3( regs, elements, accuracy, REF_GYROS,
                             KEY_CFG_9, CONFIG_GYROS  );

    return setFIFOFooter();
}
/** Sends linear accelerometer data to the FIFO. Linear accelerometer data is a
 *  3 length vector of 32-bits. It is the acceleration in the body frame
 *  with gravity removed.  Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data. Set to zero to remove it from the FIFO.
 */
tMLError FIFOSendLinearAccel(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[4] = { DINAF8 + 1, DINA28, DINA30, DINA38};
    tMLError result;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    result = FIFOConstruct3( regs, elements, accuracy, REF_LINEAR_ACCEL, 
                             KEY_CFG_12, CONFIG_LINEAR_ACCEL  );
    ERROR_CHECK( result );

    FIFOScale[REF_LINEAR_ACCEL+0] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_LINEAR_ACCEL+1] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_LINEAR_ACCEL+2] = 2*mlxData.mlAccelSens;
   
    return setFIFOFooter();
}

/** Sends linear world accelerometer data to the FIFO. Linear world accelerometer 
 *  data is a 3 length vector of 32-bits. It is the acceleration in the world frame
 *  with gravity removed. Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset.
 *  @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *             bit data.
 */
tMLError FIFOSendLinearAccelWorld(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[6] = { DINAF8 + 1, DINA20, DINA90+11,
                              DINA20, DINA90+8, DINA20 };
    tMLError result;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference( elements, accuracy, REF_LINEAR_ACCEL_WORLD, 3 );
    accuracy = FIFOSetAccuracy( elements, accuracy, CONFIG_LINEAR_ACCEL_WORLD );

    if ( accuracy & ML_16_BIT ) {
        regs[0] = DINAF8 + 2;
    }

    FIFOData.mlFIFODataConfig[CONFIG_LINEAR_ACCEL_WORLD] = elements | accuracy;

    if ( (elements & ML_ELEMENT_1) == 0 )
        regs[1] = DINAA0+3;
    if ( (elements & ML_ELEMENT_2) == 0 )
        regs[3] = DINAA0+3;
    if ( (elements & ML_ELEMENT_3) == 0 )
        regs[5] = DINAA0+3;

    result = MLDLSetMemoryMPU(KEY_CFG_13, 6, regs);
    ERROR_CHECK( result );

    FIFOScale[REF_LINEAR_ACCEL_WORLD+0] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_LINEAR_ACCEL_WORLD+1] = 2*mlxData.mlAccelSens;
    FIFOScale[REF_LINEAR_ACCEL_WORLD+2] = 2*mlxData.mlAccelSens;

    return setFIFOFooter();
}

/** Sends quaternion data to the FIFO. Quaternion data is a 4 length vector
 *   of 32-bits. Should be called once after MLDmpOpen() and before MLDmpStart().
 * @param[in] accuracy Set to ML_32_BIT for 32-bit data, or ML_16_BIT for 16
 *            bit data.
 */
tMLError FIFOSendQuaternion(uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28,
                              DINA30, DINA38};
    uint_fast8_t elements,kk;
    tMLError result;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference( 0xf, accuracy, REF_QUATERNION, 4 );
    accuracy = FIFOSetAccuracy( elements, accuracy, CONFIG_QUAT );

    if ( accuracy & ML_16_BIT ) {
        regs[0] = DINAF8 + 2;
    }

    FIFOData.mlFIFODataConfig[CONFIG_QUAT] = elements | accuracy;

    for (kk=0; kk<4; ++kk) {
        if ( (elements & 1) == 0 )
            regs[kk+1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_8, 5, regs);
    ERROR_CHECK( result );

    return setFIFOFooter();
}

/** Sends raw data to the FIFO. 
 *  Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 7 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 ... ML_ELEMENT_7 or'd together
 *            for a subset. The first element is temperature, the next 3 are gyro data,
 *            and the last 3 accel data.
 */
tMLError FIFOSendRaw(uint_fast8_t elements, uint_fast8_t accuracy)
{
#ifdef M_HW
    unsigned char regs[7] = {DINAA0+3, DINAA0+3, DINAA0+3,
                             DINAA0+3, DINAA0+3, DINAA0+3,
                             DINAA0+3 };

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if ( accuracy & (ML_16_BIT | ML_32_BIT) )
        accuracy = ML_16_BIT;
    else
        accuracy = 0;

    elements = FIFOSetReference( elements, accuracy, REF_RAW, 7 );

    if ( elements & 1 )
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 1 | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 0;
    if ( elements & 0x7e )
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = (0x3f & (elements>>1)) | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = 0;

    if (elements & ML_ELEMENT_1) {
        regs[0] = DINACA;
    }
    if (elements & ML_ELEMENT_2) {
        regs[1] = DINBC4;
    }
    if (elements & ML_ELEMENT_3) {
        regs[2] = DINACC;
    }
    if (elements & ML_ELEMENT_4) {
        regs[3] = DINBC6;
    }
    if (elements & ML_ELEMENT_5) {
        regs[4] = DINBC0;
    }
    if (elements & ML_ELEMENT_6) {
        regs[5] = DINAC8;
    }
    if (elements & ML_ELEMENT_7) {
        regs[6] = DINBC2;
    }
    if ( MLDLSetMemoryMPU(KEY_CFG_15, 7, regs) != ML_SUCCESS )
        return ML_ERROR;

    return setFIFOFooter();

#else
    INVENSENSE_FUNC_START;
    unsigned char regs[13] = {DINAF8+2, DINAF8+3, DINAA0+3,
                              DINAF8+2, DINAA0+3, DINAF8+3,
                              DINAA0+3, DINAF8+2, DINAA0+3,
                              DINAF8+3, DINAA0+3, DINAF8+2,
                              DINAA0+3};

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if ( accuracy & (ML_16_BIT | ML_32_BIT) )
        accuracy = ML_16_BIT;
    else
        accuracy = 0;

    elements = FIFOSetReference( elements, accuracy, REF_RAW, 7 );

    if ( elements & 1 )
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 1 | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] = 0;
    if ( elements & 0x7e )
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = (0x3f & (elements>>1)) | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] = 0;

    if ( elements & 1 )
        regs[0] = DINA20;
    if ( elements & 2 )
        regs[2] = DINA20;
    if ( elements & 4 )
        regs[4] = DINA28;
    if ( elements & 8 )
        regs[6] = DINA28;
    if ( elements & 16 )
        regs[8] = DINA30;
    if ( elements & 32 )
        regs[10] = DINA30;
    if ( elements & 64 )
        regs[12] = DINA38; 

    if ( MLDLSetMemoryMPU(KEY_CFG_15, 13, regs) != ML_SUCCESS )
        return ML_ERROR;

    return setFIFOFooter();
#endif
}

/** Sends raw external data to the FIFO. 
 *  Should be called once after MLDmpOpen() and before MLDmpStart().
 *  @param[in] elements Which of the 3 elements to send. Use ML_ALL for all of them
 *            or ML_ELEMENT_1, ML_ELEMENT_2, ML_ELEMENT_3 or'd together
 *            for a subset. 
 *  @param[in] accuracy ML_16_BIT to send data, 0 to stop sending this data.
 *            Sending and Stop sending are reference counted, so data actually
 *            stops when the reference reaches zero.
 */
tMLError FIFOSendRawExternal(uint_fast8_t elements, uint_fast8_t accuracy)
{
#ifdef M_HW
    unsigned char regs[3] = {DINAA0+3, DINAA0+3,
                             DINAA0+3 };

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    if ( accuracy & (ML_16_BIT | ML_32_BIT) )
        accuracy = ML_16_BIT;
    else
        accuracy = 0;

    elements = FIFOSetReference( elements, accuracy, REF_RAW_EXTERNAL, 3 );

    if ( elements )
        FIFOData.mlFIFODataConfig[CONFIG_RAW_EXTERNAL] = elements | ML_16_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_RAW_EXTERNAL] = 0;

    if (elements & ML_ELEMENT_1) {
        regs[0] = DINBC2;
    }
    if (elements & ML_ELEMENT_2) {
        regs[1] = DINACA;
    }
    if (elements & ML_ELEMENT_3) {
        regs[2] = DINBC4;
    }

    if ( MLDLSetMemoryMPU(KEY_CFG_EXTERNAL, 3, regs) != ML_SUCCESS )
        return ML_ERROR;
    return setFIFOFooter();

#else
    return ML_ERROR; // Feature not supported
#endif
}


/**
 *  @brief  Send the computed tap algorithm output into the FIFO.
 *          The taps can be retrieved from the FIFO via 
 *          FIFOGetTap().
 *
 *  @param  elements
 *              the components bitmask.
 *              To send all compoents use ML_ALL.
 *  @param  accuracy
 *              The number of bits the bias uncertainty is expressed 
 *              into. Use ML_32_BIT for 32-bit data or ML_16_BIT for 
 *              16-bit data.
 *              Set to zero to remove it from the FIFO.
 *
 *  @return ML_SUCCESS if successful, a non-zero error code otherwise.
 */
tMLError FIFOSendTap(uint_fast8_t elements, uint_fast8_t accuracy)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[5] = { DINAF8 + 1, DINA20, DINA28,
                               DINA30, DINA38 };
    unsigned char regs2[4] = { DINA20, DINA28,
                               DINA30, DINA38 };
    tMLError result;
    int_fast8_t kk;

    if ( MLGetState() < ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    elements = FIFOSetReference( elements, accuracy, REF_TAP, 8 );

    if ( elements )
        FIFOData.mlFIFODataConfig[CONFIG_DMP_TAP] = (elements & 0xf) | ML_32_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_DMP_TAP] = 0;

    for (kk=0; kk<4; ++kk) {
        if ( (elements & 1) == 0 )
            regs[kk+1] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_TAP0, 5, regs);
    ERROR_CHECK(result);

    if ( elements )
        FIFOData.mlFIFODataConfig[CONFIG_DMP_TAP2] = (elements & 0xf) | ML_32_BIT;
    else
        FIFOData.mlFIFODataConfig[CONFIG_DMP_TAP2] = 0;

    for (kk=0; kk<4; ++kk) {
        if ( (elements & 1) == 0 )
            regs2[kk] = DINAA0+3;
        elements >>= 1;
    }

    result = MLDLSetMemoryMPU(KEY_CFG_TAP4, 4, regs2);
    ERROR_CHECK(result);

    return setFIFOFooter();
}

/** 
 * @internal
 * Puts footer on FIFO data.
 */
static tMLError setFIFOFooter()
{
#ifdef BIG_ENDIAN
#define ENDIAN 0
#else
#define ENDIAN 1
#endif
    unsigned char regs = DINA30;
    uint_fast8_t tmpCount;
    int_fast8_t i,j;
    int offsetCntr = 0;
    int offset;
    int *FIFOOffsetsPtr = FIFOData.offsets;
    FIFOData.mlFIFOPacketSize = 0;
    for (i=0; i<NUMFIFOELEMENTS; i++) {
        tmpCount = 0;
        offset = FIFOOffsetBase[i];
        for (j=0; j<6; j++) {
            if ((FIFOData.mlFIFODataConfig[i]>>j) & 0x0001) {
                if ( ENDIAN ) { // Little Endian Platform
                    // Special Case for Byte Ordering on Accel Data
                    if ((i == CONFIG_RAW_DATA) && (j > 2)) {
                        tmpCount += 2;
                        switch (MLDLGetCfg()->accel->endian) {
                        case EXT_SLAVE_BIG_ENDIAN:
                            *FIFOOffsetsPtr++ = offset+3;
                            *FIFOOffsetsPtr++ = offset+2;
                            break;
                        case EXT_SLAVE_LITTLE_ENDIAN:
                            *FIFOOffsetsPtr++ = offset+2;
                            *FIFOOffsetsPtr++ = offset+3;
                            break;
                        case EXT_SLAVE_FS8_BIG_ENDIAN:
                            if (j == 3) {
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                                *FIFOOffsetsPtr++ = offset+3;
                            } else if (j == 4) {
                                *FIFOOffsetsPtr++ = offset+3;
                                *FIFOOffsetsPtr++ = offset+7;
                            } else if (j == 5) {
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                            }
                            break;
                        case EXT_SLAVE_FS16_BIG_ENDIAN:
                            if (j == 3) {
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                                *FIFOOffsetsPtr++ = offset+3;
                            } else if (j == 4) {
                                *FIFOOffsetsPtr++ = offset-2;
                                *FIFOOffsetsPtr++ = offset+3;
                            } else if (j == 5) {
                                *FIFOOffsetsPtr++ = offset-2;
                                *FIFOOffsetsPtr++ = offset+3;
                            }
                            break;
                        default:
                            return ML_ERROR; // Bad value on ordering
                        }
                    } else {
                        tmpCount += 2;
                        *FIFOOffsetsPtr++ = offset+3;
                        *FIFOOffsetsPtr++ = offset+2;
                        if ( FIFOData.mlFIFODataConfig[i] & ML_32_BIT) {
                            *FIFOOffsetsPtr++ = offset+1;
                            *FIFOOffsetsPtr++ = offset;
                            tmpCount += 2;
                        }
                    }
                } else {
                    // Big Endian Platform
                    // Special Case for Byte Ordering on Accel Data
                    if ((i == CONFIG_RAW_DATA) && (j > 2)) {
                        tmpCount += 2;
                        switch (MLDLGetCfg()->accel->endian) {
                        case EXT_SLAVE_BIG_ENDIAN:
                            *FIFOOffsetsPtr++ = offset+2;
                            *FIFOOffsetsPtr++ = offset+3;
                            break;
                        case EXT_SLAVE_LITTLE_ENDIAN:
                            *FIFOOffsetsPtr++ = offset+3;
                            *FIFOOffsetsPtr++ = offset+2;
                            break;
                        case EXT_SLAVE_FS8_BIG_ENDIAN:
                            if (j == 3) {
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                                *FIFOOffsetsPtr++ = offset;
                            } else if (j == 4) {
                                *FIFOOffsetsPtr++ = offset;
                                *FIFOOffsetsPtr++ = offset+4;
                            } else if (j == 5) {
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                            }
                            break;
                        case EXT_SLAVE_FS16_BIG_ENDIAN:
                            if (j == 3) {
                                *FIFOOffsetsPtr++ = FIFOOffsetBase[CONFIG_FOOTER]; // Throw this byte away
                                *FIFOOffsetsPtr++ = offset;
                            } else if (j == 4) {
                                *FIFOOffsetsPtr++ = offset-3;
                                *FIFOOffsetsPtr++ = offset;
                            } else if (j == 5) {
                                *FIFOOffsetsPtr++ = offset-3;
                                *FIFOOffsetsPtr++ = offset;
                            }
                            break;
                        default:
                            return ML_ERROR; // Bad value on ordering
                        }
                    } else {
                        tmpCount += 2;
                        *FIFOOffsetsPtr++ = offset;
                        *FIFOOffsetsPtr++ = offset+1;
                        if ( FIFOData.mlFIFODataConfig[i] & ML_32_BIT) {
                            *FIFOOffsetsPtr++ = offset+2;
                            *FIFOOffsetsPtr++ = offset+3;
                            tmpCount += 2;
                        }
                    }
                    
                }
            }
            offset += 4;
        }
        FIFOData.mlFIFOPacketSize += tmpCount;
    }
    if ( FIFOData.mlFIFODataConfig[CONFIG_FOOTER] == 0 && FIFOData.mlFIFOPacketSize > 0 ) {
        // Add footer
        if ( MLDLSetMemoryMPU(KEY_CFG_16, 1, &regs) != ML_SUCCESS )
            return ML_ERROR;
        FIFOData.mlFIFODataConfig[CONFIG_FOOTER] = 0x0001 | ML_16_BIT;
        FIFOData.mlFIFOPacketSize += 2;
    } else if ( FIFOData.mlFIFODataConfig[CONFIG_FOOTER] && (FIFOData.mlFIFOPacketSize == 2 ) ) {
        // Remove Footer
        regs = DINAA0+3;
        if ( MLDLSetMemoryMPU(KEY_CFG_16, 1, &regs) != ML_SUCCESS )
            return ML_ERROR;
        FIFOData.mlFIFODataConfig[CONFIG_FOOTER] = 0;
        FIFOData.mlFIFOPacketSize = 0;
    }

    return ML_SUCCESS;
}

/** 
 * @brief       Returns 3-element vector of accelerometer data in body frame.
 *
 * @param[out]  data    3-element vector of accelerometer data in body frame.
 *                      One gee = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetAccel(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_ACCEL+0];
    data[1] = FIFOData.decoded[REF_ACCEL+1];
    data[2] = FIFOData.decoded[REF_ACCEL+2];
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector derived from 6-axis or 9-axis if 9-axis
 *              was implemented. 6-axis is gyros and accels. 9-axis is gyros, accel and compass.
 *  @param[out] data    4-element quaternion vector. One is scaled to 2^30.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetQuaternion(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_QUATERNION];
    data[1] = FIFOData.decoded[REF_QUATERNION+1];
    data[2] = FIFOData.decoded[REF_QUATERNION+2];
    data[3] = FIFOData.decoded[REF_QUATERNION+3];
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector derived from 6 
 *              axis sensors (gyros and accels).
 *  @param[out] data    
 *                  4-element quaternion vector. One is scaled to 2^30.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetQuaternion6Axis(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_QUATERNION_6AXIS];
    data[1] = FIFOData.decoded[REF_QUATERNION_6AXIS+1];
    data[2] = FIFOData.decoded[REF_QUATERNION_6AXIS+2];
    data[3] = FIFOData.decoded[REF_QUATERNION_6AXIS+3];
    return ML_SUCCESS;
}


/** 
 *  @brief  Returns 3-element vector of gyro data in body frame.
 *  @param[out] data    
 *              3-element vector of gyro data in body frame 
 *              with gravity removed. One degree per second = 2^16.
 *  @return 0 on success or an error code.
 */
tMLError FIFOGetGyro(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_GYROS+0];
    data[1] = FIFOData.decoded[REF_GYROS+1];
    data[2] = FIFOData.decoded[REF_GYROS+2];
    return ML_SUCCESS;
}


/**
 *  @brief  Get the 3-element gravity vector from the FIFO expressed
 *          in coordinates relative to the body frame.
 *  @param  data    
 *              3-element vector of gravity in body frame.
 *  @return 0 on success or an error code.
 */
tMLError FIFOGetGravBody(long *data)
{
    if ( data == NULL )
        return ML_ERROR;

    if ( (FIFOData.mlFIFODataConfig[CONFIG_GRAV] & 7) ) {
        data[0] = FIFOData.decoded[REF_GRAVITY+0];
        data[1] = FIFOData.decoded[REF_GRAVITY+1];
        data[2] = FIFOData.decoded[REF_GRAVITY+2];
    } else {
        // Compute it from Quaternion
        data[0] = q29_mult(FIFOData.decoded[REF_QUATERNION+1],FIFOData.decoded[REF_QUATERNION+3])- q29_mult(FIFOData.decoded[REF_QUATERNION+2],FIFOData.decoded[REF_QUATERNION+0]);
        data[1] = q29_mult(FIFOData.decoded[REF_QUATERNION+2],FIFOData.decoded[REF_QUATERNION+3])+ q29_mult(FIFOData.decoded[REF_QUATERNION+1],FIFOData.decoded[REF_QUATERNION+0]);
        data[2] = q29_mult(FIFOData.decoded[REF_QUATERNION+3],FIFOData.decoded[REF_QUATERNION+3])+ q29_mult(FIFOData.decoded[REF_QUATERNION+0],FIFOData.decoded[REF_QUATERNION+0]) - 1073741824L;
    }
    return ML_SUCCESS;
}


/** 
 *  @brief      Returns 3-element vector of accelerometer data in body frame
 *              with gravity removed.
 *  @param[out] data    3-element vector of accelerometer data in body frame
 *                      with gravity removed. One g = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetLinearAccel(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_LINEAR_ACCEL+0];
    data[1] = FIFOData.decoded[REF_LINEAR_ACCEL+1];
    data[2] = FIFOData.decoded[REF_LINEAR_ACCEL+2];
    return ML_SUCCESS;
}
 
/** 
 *  @brief      Returns 3-element vector of accelerometer data in world frame
 *              with gravity removed.
 *  @param[out] data    3-element vector of accelerometer data in world frame
 *                      with gravity removed. One g = 2^16.
 *  @return     0 on success or an error code.
 */
tMLError FIFOGetLinearAccelWorld(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    if ( ((FIFOData.mlFIFODataConfig[CONFIG_LINEAR_ACCEL_WORLD] & (ML_32_BIT | ML_16_BIT)) == 0) &&
         ((FIFOData.mlFIFODataConfig[CONFIG_ACCELS] & (ML_32_BIT | ML_16_BIT)) != 0) &&
         ((FIFOData.mlFIFODataConfig[CONFIG_QUAT] & (ML_32_BIT | ML_16_BIT)) != 0) )
    {
        long wtemp[4],qi[4],wtemp2[4];
        wtemp[0]=0;
        FIFOGetAccel(&wtemp[1]);
        MLQMult( &FIFOData.decoded[REF_QUATERNION], wtemp, wtemp2 );
        MLQInvert( &FIFOData.decoded[REF_QUATERNION], qi );
        MLQMult( wtemp2, qi, wtemp );
        data[0] = wtemp[1];
        data[1] = wtemp[2];
        data[2] = wtemp[3]-(1L << 16); // Subtract gravity
    } else {
        data[0] = FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+0];
        data[1] = FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+1];
        data[2] = FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+2] - (1L << 16);
    }
    return ML_SUCCESS;
}
 
/** 
 *  @brief      Returns 4-element vector of control data.
 *  @param[out] data    4-element vector of control data.
 *  @return     0 for succes or an error code.
 */
tMLError FIFOGetControlData(long *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_CONTROL+0];
    data[1] = FIFOData.decoded[REF_CONTROL+1];
    data[2] = FIFOData.decoded[REF_CONTROL+2];
    data[3] = FIFOData.decoded[REF_CONTROL+3];
    return ML_SUCCESS;

}


/** 
 *  @brief      Returns 3-element vector of accelerometer data in body frame.
 *  @param[out] data    3-element vector of accelerometer data in body frame in g's.
 *  @return     0 for success or an error code.
 */
tMLError FIFOGetAccelFloat(float *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_ACCEL+0]/65536.0f;
    data[1] = FIFOData.decoded[REF_ACCEL+1]/65536.0f;
    data[2] = FIFOData.decoded[REF_ACCEL+2]/65536.0f;
    return ML_SUCCESS;
}

/** 
 *  @brief      Returns 4-element quaternion vector.
 *  @param[out] data    4-element quaternion vector.
 *  @return     0 on success, an error code otherwise.
 */
tMLError FIFOGetQuaternionFloat(float *data)
{
    if ( data == NULL )
        return ML_ERROR;
    data[0] = FIFOData.decoded[REF_QUATERNION+0]/1073741824.0f;
    data[1] = FIFOData.decoded[REF_QUATERNION+1]/1073741824.0f;
    data[2] = FIFOData.decoded[REF_QUATERNION+2]/1073741824.0f;
    data[3] = FIFOData.decoded[REF_QUATERNION+3]/1073741824.0f;
    return ML_SUCCESS;
}

/**
 * @brief   Command the MPU to put data in the FIFO at a particular rate.
 *
 *          The DMP will add fifo entries every fifoRate + 1 MPU cycles.  For
 *          example if the MPU is running at 200Hz the following values apply:
 *
 *          <TABLE>
 *          <TR><TD>fifoRate</TD><TD>DMP Sample Rate</TD><TD>FIFO update frequency</TD></TR>
 *          <TR><TD>0</TD><TD>200Hz</TD><TD>200Hz</TD></TR>
 *          <TR><TD>1</TD><TD>200Hz</TD><TD>100Hz</TD></TR>
 *          <TR><TD>2</TD><TD>200Hz</TD><TD>50Hz</TD></TR>
 *          <TR><TD>4</TD><TD>200Hz</TD><TD>40Hz</TD></TR>
 *          <TR><TD>9</TD><TD>200Hz</TD><TD>20Hz</TD></TR>
 *          <TR><TD>19</TD><TD>200Hz</TD><TD>10Hz</TD></TR>
 *          </TABLE>
 *
 *  @pre    MLDmpOpen() with MLDmpDefaultOpen() or 
 *          MLDmpPedometerStandAloneOpen() and MLDmpStart() 
 *          must <b>NOT</b> have been called.
 *
 * @param   fifoRate    Divider value - 1.  Output rate is 
 *          (DMP Sample Rate) / (fifoRate + 1).
 *
 * @return  ML_SUCCESS if successful, ML error code on any failure.
 */
tMLError MLSetFIFORate(unsigned short fifoRate)
{
    INVENSENSE_FUNC_START;
    unsigned char regs[2];

    if ( MLGetState() != ML_STATE_DMP_OPENED )
        return ML_ERROR_SM_IMPROPER_STATE;

    FIFOData.mlFIFORate = fifoRate;

    regs[0] = (unsigned char)( (fifoRate>>8) & 0xff);
    regs[1] = (unsigned char)(fifoRate & 0xff);
     
    return MLDLSetMemoryMPU(KEY_D_0_22, 2, regs);
}

/**
 * @brief   Retrieve the current FIFO update divider - 1.
 *          See MLSetFIFORate() for how this value is used.
 * @return  The value of the fifo rate divider or ML_INVALID_FIFO_RATE on error.
**/
unsigned short MLGetFIFORate(void)
{
    return FIFOData.mlFIFORate;
}

/**
 * @brief   Returns the step size for quaternion type data.
 *          Typically the data rate for each FIFO packet.
 * @return  step size for quaternion type data
**/
int_fast16_t GetSampleStepSizeMs(void)
{
    struct mldl_cfg *mldl_cfg = MLDLGetCfg();

    return (FIFOData.mlFIFORate + 1 ) * (1 + mldl_cfg->divider);
}


/** 
 *  @brief  Returns 1 if we have Temperature and Gyro Sensor Data in the FIFO
**/
int_fast8_t haveTempAndGyroSensorData()
{
    int_fast8_t status = ( (FIFOData.mlFIFODataConfig[CONFIG_TEMPERATURE] & 1) == 1) &&
        ( (FIFOData.mlFIFODataConfig[CONFIG_RAW_DATA] & (ML_ELEMENT_1|ML_ELEMENT_2|ML_ELEMENT_3))
        == (ML_ELEMENT_1|ML_ELEMENT_2|ML_ELEMENT_3) );
    return status;
}

/** 
 *  @brief  The gyro data magnitude squared : 
 *          (1 degree per second)^2 = 2^6 = 2^GYRO_MAG_SQR_SHIFT.
 *  @return the computed magnitude squared output of the gyroscope.
 */
unsigned long getGyroMagSqrd()
{
    unsigned long gmag;
    long temp;
    temp = FIFOData.decoded[REF_GYROS+0] >> (16-(GYRO_MAG_SQR_SHIFT/2));
    gmag = temp * temp;
    temp = FIFOData.decoded[REF_GYROS+1] >> (16-(GYRO_MAG_SQR_SHIFT/2));
    gmag += temp * temp;
    temp = FIFOData.decoded[REF_GYROS+2] >> (16-(GYRO_MAG_SQR_SHIFT/2));
    gmag += temp * temp;
    return gmag;
}

/** 
 *  @brief  The gyro data magnitude squared:
 *          (1 g)^2 = 2^16 = 2^ACC_MAG_SQR_SHIFT.
 *  @return the computed magnitude squared output of the accelerometer.
 */
unsigned long getAccMagSqrd()
{
    unsigned long amag;
    long temp;
    temp = FIFOData.decoded[REF_ACCEL+0] >> (16-(ACC_MAG_SQR_SHIFT/2));
    amag = temp * temp;
    temp = FIFOData.decoded[REF_ACCEL+1] >> (16-(ACC_MAG_SQR_SHIFT/2));
    amag += temp * temp;
    temp = FIFOData.decoded[REF_ACCEL+2] >> (16-(ACC_MAG_SQR_SHIFT/2));
    amag += temp * temp;
    return amag;
}

/**
 *  @internal
 */
void overRideQuaternion( float *q )
{
    FIFOData.decoded[REF_QUATERNION+0] = (long)(q[0] * (1L<<30));
    FIFOData.decoded[REF_QUATERNION+1] = (long)(q[1] * (1L<<30));
    FIFOData.decoded[REF_QUATERNION+2] = (long)(q[2] * (1L<<30));
    FIFOData.decoded[REF_QUATERNION+3] = (long)(q[3] * (1L<<30));
    if (FIFOData.mlFIFODataConfig[CONFIG_LINEAR_ACCEL_WORLD] & (ML_32_BIT | ML_16_BIT)) {
        long wtemp[4],qi[4],wtemp2[4];
        MLQInvert( &FIFOData.decoded[REF_QUATERNION_6AXIS], qi );
        wtemp[0]=0;
        wtemp[1]=FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+0];
        wtemp[2]=FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+1];
        wtemp[3]=FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+2];
        MLQMult(qi,wtemp,wtemp2);
        MLQMult(wtemp2, &FIFOData.decoded[REF_QUATERNION_6AXIS], wtemp);
        MLQMult(&FIFOData.decoded[REF_QUATERNION], wtemp, wtemp2);
        MLQInvert(&FIFOData.decoded[REF_QUATERNION], qi);
        MLQMult(wtemp2,qi,wtemp);
        FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+0]=wtemp[1];
        FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+1]=wtemp[2];
        FIFOData.decoded[REF_LINEAR_ACCEL_WORLD+2]=wtemp[3];
    }
}

/**
 * @internal
 * @brief   This registers a function to be called for each set of 
 *          gyro/quaternion/rotation matrix/etc output.
 * @return  error code.
 */
tMLError RegisterHighRateProcess( tMlxdataFunction func )
{
    INVENSENSE_FUNC_START;
    tMLError result;

    result = MLOSLockMutex(mlxCallback.mutex);
    if (ML_SUCCESS != result) {
        return result;
    }

    // Make sure we have not filled up our number of allowable callbacks
    if ( mlxCallback.numHighrateProcesses <= MAX_HIGH_RATE_PROCESSES-1 ) {
        int kk;
        // Make sure we haven't registered this function already
        for (kk=0; kk<mlxCallback.numHighrateProcesses; ++kk) {
            if ( mlxCallback.highrateProcess[kk] == func ) {
                result = ML_ERROR_INVALID_PARAMETER;
                break;
            }
        }

        if (ML_SUCCESS == result) {
            // Add new callback
            mlxCallback.highrateProcess[mlxCallback.numHighrateProcesses] = func;
            mlxCallback.numHighrateProcesses++;
        }
    } else {
        result = ML_ERROR_MEMORY_EXAUSTED;
    }

    MLOSUnlockMutex(mlxCallback.mutex);
    return result;
}

/**
 * @internal
 * @brief   This unregisters a function to be called for each set of 
 *          gyro/quaternion/rotation matrix/etc output.
 * @return  error code.
 */
tMLError UnRegisterHighRateProcess( tMlxdataFunction func )
{
    INVENSENSE_FUNC_START;
    int kk,jj;
    tMLError result;

    result = MLOSLockMutex(mlxCallback.mutex);
    if (ML_SUCCESS != result) {
        return result;
    }

    // Make sure we haven't registered this function already
    result = ML_ERROR_INVALID_PARAMETER;
    for (kk=0; kk<mlxCallback.numHighrateProcesses; ++kk) {
        if ( mlxCallback.highrateProcess[kk] == func ) {
            for (jj=kk+1; jj<mlxCallback.numHighrateProcesses; ++jj) {
                mlxCallback.highrateProcess[jj-1] = mlxCallback.highrateProcess[jj];
            }
            mlxCallback.numHighrateProcesses--;
            result = ML_SUCCESS;
            break;
        }
    }

    MLOSUnlockMutex(mlxCallback.mutex);
    return result;

}

tMLError RunHighRateProcessFuncs(void)
{
    int kk;
    tMLError result,result2;
    
    result = MLOSLockMutex(mlxCallback.mutex);
    if (ML_SUCCESS != result) {
        MPL_LOGE("MLOsLockMutex returned %d\n", result);
        return result;
    }

    // User callbacks take priority over the highrateProcess callback
    if (FIFOData.FIFOProcessCB)
         FIFOData.FIFOProcessCB();

    for (kk=0; kk<mlxCallback.numHighrateProcesses; ++kk) {
        if ( mlxCallback.highrateProcess[kk] ) {
            result2 = mlxCallback.highrateProcess[kk]( &mlxData );
            if (result == ML_SUCCESS )
                result = result2;
        }
    }

    MLOSUnlockMutex(mlxCallback.mutex);
    return result;
}

/*********************/
/** \}*/ /* defgroup */
/*********************/
