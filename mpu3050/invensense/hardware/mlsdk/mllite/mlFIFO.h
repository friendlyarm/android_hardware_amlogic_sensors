/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

#ifndef INVENSENSE_ML_FIFO_H__
#define INVENSENSE_ML_FIFO_H__

#include "mltypes.h"
#include "mlinclude.h"
#include "ml.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**************************************************************************/
    /*  Elements                                                              */
    /**************************************************************************/

#define ML_ELEMENT_1                    0x0001
#define ML_ELEMENT_2                    0x0002
#define ML_ELEMENT_3                    0x0004
#define ML_ELEMENT_4                    0x0008
#define ML_ELEMENT_5                    0x0010
#define ML_ELEMENT_6                    0x0020
#define ML_ELEMENT_7                    0x0040

#define ML_ALL               (0xFF)

    /**************************************************************************/
    /*  Accuracy                                                              */
    /**************************************************************************/

#define ML_16_BIT                       0x40
#define ML_32_BIT                       0x80


    tMLError MLSetFIFORate(unsigned short fifoRate);
    unsigned short MLGetFIFORate(void);
    int_fast16_t   GetSampleStepSizeMs  (void);

    // Register callbacks after a packet of FIFO data is processed
    tMLError RegisterHighRateProcess( tMlxdataFunction func );
    tMLError UnRegisterHighRateProcess( tMlxdataFunction func );
    tMLError RunHighRateProcessFuncs(void);
    
    // Setup FIFO for various output
    tMLError FIFOSendQuaternion( uint_fast8_t accuracy );
    tMLError FIFOSendGyro(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendAccel(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendLinearAccel(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendLinearAccelWorld(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendControlData(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendRaw(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendRawExternal(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendGravity(uint_fast8_t elements, uint_fast8_t accuracy);
    tMLError FIFOSendBiasUncertainty(uint_fast8_t accuracy);
    tMLError FIFOSendDMPPacketNumber(uint_fast8_t accuracy);
    tMLError FIFOSendTap( uint_fast8_t elements, uint_fast8_t accuracy);


    // Get Fixed Point data from FIFO
    tMLError FIFOGetAccel(long *data);
    tMLError FIFOGetQuaternion(long *data);
    tMLError FIFOGetQuaternion6Axis(long *data);
    tMLError FIFOGetGyro(long *data);
    tMLError FIFOGetLinearAccel(long *data);
    tMLError FIFOGetLinearAccelWorld(long *data);
    tMLError FIFOGetControlData(long *data);
    tMLError FIFOGetSensorData(long *data);
    tMLError FIFOGetTemperature(long *data);
    long FIFOGetBiasUncertainty(void);
    tMLError FIFOGetGravBody(long *data);
    tMLError FIFOGetTap(long *data);
    tMLError FIFOGetExternalSensorData(long *data);

    // Get Floating Point data from FIFO
    tMLError FIFOGetAccelFloat(float *data);
    tMLError FIFOGetQuaternionFloat(float *data);

    tMLError MLProcessFIFOData(const unsigned char *dmpData);
    tMLError readAndProcessFIFO( int_fast8_t numPackets, int_fast8_t *processed );

    tMLError MLSetProcessedDataCallback(void (*func)(void) );

    tMLError FIFOParamInit(void);
    tMLError FIFOClose(void);

    int_fast8_t haveTempAndGyroSensorData(void);

    void decodeTemperature(const unsigned char *reg);
    int_fast8_t decodeGyroSensorData(const unsigned char *reg, uint_fast8_t axes );
    unsigned long getGyroMagSqrd(void);
    unsigned long getAccMagSqrd(void);
    void overRideQuaternion( float *q );

    uint_fast16_t FIFOGetPacketSize(void);
#ifdef __cplusplus
}
#endif

#endif // INVENSENSE_ML_FIFO_H__
