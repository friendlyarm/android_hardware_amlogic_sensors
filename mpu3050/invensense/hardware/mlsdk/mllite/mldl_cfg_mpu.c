/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 *
 * $Id: mldl_cfg_mpu.c 4108 2010-11-20 01:34:54Z nroyer $
 *
 ******************************************************************************/

/** 
 *  @addtogroup MLDL
 *  @{
 *      @file   mldl_cfg_mpu3050.c
 *      @brief  The Motion Library Driver Layer.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#include <stddef.h>
#include "mldl_cfg.h"
#include "mlsl.h"
#include "mpu.h"

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */


/* ---------------------- */
/* -  Static Functions. - */
/* ---------------------- */
void mpu_print_cfg(struct mldl_cfg * mldl_cfg)
{
    struct mpu3050_platform_data   *pdata   = mldl_cfg->pdata;
    struct ext_slave_platform_data *accel   = &mldl_cfg->pdata->accel;
    struct ext_slave_platform_data *compass = &mldl_cfg->pdata->compass;

    MPL_LOGD("mldl_cfg.addr             = %02x\n", mldl_cfg->addr);
    MPL_LOGD("mldl_cfg.int_config       = %02x\n", mldl_cfg->int_config);
    MPL_LOGD("mldl_cfg.ext_sync         = %02x\n", mldl_cfg->ext_sync);
    MPL_LOGD("mldl_cfg.full_scale       = %02x\n", mldl_cfg->full_scale);
    MPL_LOGD("mldl_cfg.lpf              = %02x\n", mldl_cfg->lpf);
    MPL_LOGD("mldl_cfg.clk_src          = %02x\n", mldl_cfg->clk_src);
    MPL_LOGD("mldl_cfg.divider          = %02x\n", mldl_cfg->divider);
    MPL_LOGD("mldl_cfg.dmp_enable       = %02x\n", mldl_cfg->dmp_enable);
    MPL_LOGD("mldl_cfg.fifo_enable      = %02x\n", mldl_cfg->fifo_enable);
    MPL_LOGD("mldl_cfg.dmp_cfg1         = %02x\n", mldl_cfg->dmp_cfg1);
    MPL_LOGD("mldl_cfg.dmp_cfg2         = %02x\n", mldl_cfg->dmp_cfg2);
    MPL_LOGD("mldl_cfg.offset_tc[0]     = %02x\n", mldl_cfg->offset_tc[0]);
    MPL_LOGD("mldl_cfg.offset_tc[1]     = %02x\n", mldl_cfg->offset_tc[1]);
    MPL_LOGD("mldl_cfg.offset_tc[2]     = %02x\n", mldl_cfg->offset_tc[2]);
    MPL_LOGD("mldl_cfg.silicon_revision = %02x\n", mldl_cfg->silicon_revision);
    MPL_LOGD("mldl_cfg.product_id       = %02x\n", mldl_cfg->product_id);
    MPL_LOGD("mldl_cfg.trim             = %02x\n", mldl_cfg->trim);

    if (mldl_cfg->accel) {
        MPL_LOGD("slave_accel->suspend      = %02x\n", (int)mldl_cfg->accel->suspend);
        MPL_LOGD("slave_accel->resume       = %02x\n", (int)mldl_cfg->accel->resume);
        MPL_LOGD("slave_accel->read         = %02x\n", (int)mldl_cfg->accel->read);
        MPL_LOGD("slave_accel->type         = %02x\n", mldl_cfg->accel->type);
        MPL_LOGD("slave_accel->reg          = %02x\n", mldl_cfg->accel->reg);
        MPL_LOGD("slave_accel->len          = %02x\n", mldl_cfg->accel->len);
        MPL_LOGD("slave_accel->endian       = %02x\n", mldl_cfg->accel->endian);
        MPL_LOGD("slave_accel->range.mantissa= %02x\n", mldl_cfg->accel->range.mantissa);
        MPL_LOGD("slave_accel->range.fraction= %02x\n", mldl_cfg->accel->range.fraction);
    } else {
        MPL_LOGD("slave_accel               = NULL\n");
    }

    if (mldl_cfg->compass) {
        MPL_LOGD("slave_compass->suspend    = %02x\n", (int)mldl_cfg->compass->suspend);
        MPL_LOGD("slave_compass->resume     = %02x\n", (int)mldl_cfg->compass->resume);
        MPL_LOGD("slave_compass->read       = %02x\n", (int)mldl_cfg->compass->read);
        MPL_LOGD("slave_compass->type       = %02x\n", mldl_cfg->compass->type);
        MPL_LOGD("slave_compass->reg        = %02x\n", mldl_cfg->compass->reg);
        MPL_LOGD("slave_compass->len        = %02x\n", mldl_cfg->compass->len);
        MPL_LOGD("slave_compass->endian     = %02x\n", mldl_cfg->compass->endian);
        MPL_LOGD("slave_compass->range.mantissa= %02x\n", mldl_cfg->compass->range.mantissa);
        MPL_LOGD("slave_compass->range.fraction= %02x\n", mldl_cfg->compass->range.fraction);

    } else {
        MPL_LOGD("slave_compass             = NULL\n");
    }
    MPL_LOGD("accel->get_slave_descr    = %x\n",(unsigned int) accel->get_slave_descr);
    MPL_LOGD("accel->adapt_num          = %02x\n", accel->adapt_num);
    MPL_LOGD("accel->bus                = %02x\n", accel->bus);
    MPL_LOGD("accel->address            = %02x\n", accel->address);
    MPL_LOGD("accel->orientation        = \n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n",
             accel->orientation[0],accel->orientation[1],accel->orientation[2],
             accel->orientation[3],accel->orientation[4],accel->orientation[5],
             accel->orientation[6],accel->orientation[7],accel->orientation[8]);
    MPL_LOGD("compass->get_slave_descr  = %x\n",(unsigned int) compass->get_slave_descr);
    MPL_LOGD("compass->adapt_num        = %02x\n", compass->adapt_num);
    MPL_LOGD("compass->bus              = %02x\n", compass->bus);
    MPL_LOGD("compass->address          = %02x\n", compass->address);
    MPL_LOGD("compass->orientation      = \n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n",
             compass->orientation[0],compass->orientation[1],compass->orientation[2],
             compass->orientation[3],compass->orientation[4],compass->orientation[5],
             compass->orientation[6],compass->orientation[7],compass->orientation[8]);
    
    MPL_LOGD("pdata->int_config         = %02x\n", pdata->int_config);
    MPL_LOGD("pdata->level_shifter      = %02x\n", pdata->level_shifter);
    MPL_LOGD("pdata->orientation        = \n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n"
             "                            %2d %2d %2d\n",
             pdata->orientation[0],pdata->orientation[1],pdata->orientation[2],
             pdata->orientation[3],pdata->orientation[4],pdata->orientation[5],
             pdata->orientation[6],pdata->orientation[7],pdata->orientation[8]);

    MPL_LOGD("Struct sizes: mldl_cfg: %d, "
             "ext_slave_descr:%d, mpu3050_platform_data:%d: RamOffset: %d\n", 
             sizeof(struct mldl_cfg), sizeof(struct ext_slave_descr), 
             sizeof(struct mpu3050_platform_data), 
             offsetof(struct mldl_cfg, ram));
}

/*******************************************************************************
 *******************************************************************************
 * Exported functions
 *******************************************************************************
 ******************************************************************************/

/** 
 * Initializes the pdata structure to defaults.
 *
 * Opens the device to read silicon revision, product id and whoami.  Leaves
 * the device in suspended state for low power.
 * 
 * @param pdata 
 * @param mlsl_handle 
 *
 * @return ML_SUCCESS if silicon revision, product id and woami are supported
 *         by this software.
 */
int mpu3050_open(struct mldl_cfg *mldl_cfg, void *mlsl_handle)
{
    int result;
    result = ioctl((int)mlsl_handle, MPU_GET_MPU_CONFIG, mldl_cfg);

    if (ML_SUCCESS == result && !mldl_cfg->is_suspended) {
        result = mpu3050_suspend(mldl_cfg, mlsl_handle, NULL, NULL,
                                 mldl_cfg->pdata->accel.get_slave_descr != 0,
                                 mldl_cfg->pdata->compass.get_slave_descr != 0);
    }
    return result;
}

int mpu3050_resume(struct mldl_cfg* mldl_cfg, 
                   void *mlsl_handle, 
                   void *accel_handle, 
                   void *compass_handle, 
                   bool resume_accel, bool resume_compass)
{
    int result;
    struct mpu_suspend_resume resume;
    resume.gyro = TRUE;
    resume.accel = resume_accel;
    resume.compass = resume_compass;
    
    //mpu_print_cfg(mldl_cfg);
    result = ioctl((int)mlsl_handle, MPU_SET_MPU_CONFIG, mldl_cfg);
    if (result) {
        return result;
    }
    result = ioctl((int)mlsl_handle, MPU_RESUME, &resume);
    if (ML_SUCCESS == result) {
        mldl_cfg->is_suspended = FALSE;
    }
    return result;
}


int mpu3050_suspend(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
                    void *accel_handle,
                    void *compass_handle,
                    bool accel, bool compass)
{
    int result;
    struct mpu_suspend_resume suspend;
    suspend.gyro = TRUE;
    suspend.accel = accel;
    suspend.compass = compass;
    
    result = ioctl((int)mlsl_handle, MPU_SUSPEND, &suspend);
    if (ML_SUCCESS == result) {
        mldl_cfg->is_suspended = TRUE;
    }
    return result;
}

int mpu3050_read_accel(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
                       unsigned char * data)
{
    int result;
    result = ioctl((int)mlsl_handle, MPU_READ_ACCEL, data);
    return result;
}
int mpu3050_read_compass(struct mldl_cfg *mldl_cfg, void *mlsl_handle,
                         unsigned char * data)
{
    int result;
    result = ioctl((int)mlsl_handle, MPU_READ_COMPASS, data);
    return result;
}


/**
 *@}
 */
