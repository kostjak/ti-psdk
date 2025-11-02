/*
 *
 * Copyright (c) 2020-2024 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifdef QNX
#include <errno.h>
#include <hw/i2c.h>
#include <fcntl.h>
#endif

#include <iss_sensors.h>
#include <iss_sensor_if.h>
#include <iss_sensor_priv.h>
#include <app_remote_service.h>
#include <app_ipc.h>
#include "TI/tivx_mutex.h"

#if defined (SOC_J721E)
#include <ti/board/src/j721e_evm/include/board_cfg.h>
#include <ti/board/src/j721e_evm/include/board_i2c_io_exp.h>
#elif defined (SOC_J721S2)
#include <ti/board/src/j721s2_evm/include/board_cfg.h>
#include <ti/board/src/j721s2_evm/include/board_i2c_io_exp.h>
#elif defined (SOC_J784S4)
#include <ti/board/src/j784s4_evm/include/board_cfg.h>
#include <ti/board/src/j784s4_evm/include/board_i2c_io_exp.h>
#elif defined (SOC_J742S2)
#include <ti/board/src/j742s2_evm/include/board_cfg.h>
#include <ti/board/src/j742s2_evm/include/board_i2c_io_exp.h>
#endif

#if defined(SOC_J721E) || defined(SOC_J721S2) || defined(SOC_J784S4) || defined(SOC_J742S2) || defined(SOC_AM62A) && defined(QNX)
#include <ti/board/src/devices/fpd/ds90ub9702.h>
#endif

#if defined(SOC_AM62A)
#define IO_EXPANDER_ADDR 0x23
#endif

// Note: this is a hack for now
#define CONFIG_I2C2 (1U)


/*******************************************************************************
 *  Globals
 *******************************************************************************
 */

/* Array to store sensor handles registered to the framework. Populated during
*  runtime by sensor drivers calling IssSensor_Register. */
IssSensors_Handle *gIssSensorTable[ISS_SENSORS_MAX_SENSOR_HANDLES];

/* Fusion2 board detect flag */
vx_bool gFusion2Det = vx_false_e;

/* Used to call functions depending on deserializer hubs */
uint8_t gDeserializerHubs = 0;

static I2C_Handle gISS_Sensor_I2cHandle = NULL;
static uint8_t gISS_Sensor_ByteOrder = BOARD_I2C_REG_ADDR_MSB_FIRST;
static uint8_t num_sensors_open = 0;
static IssSensors_Handle *g_pSenHndl[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
static uint8_t g_detectedSensors[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
static int sensor_fd = -1;
I2cParams *desCfgCommon_ptr;

typedef struct
{
    uint8_t nPort;
    uint8_t nVC;
} ub960MapParams;


/*******************************************************************************
 *  Local Functions Declarations
 *******************************************************************************
 */
static void IssSensor_getDeserializerI2cAddr(int8_t desHubInst,
    uint8_t *i2cBus, uint8_t *i2cAddr);
static int32_t checkForHandle(void *handle);
static int32_t setup_io_expander(void);
static int32_t setupI2CInst(uint8_t i2cInst);
static int32_t Serialzier_WriteReg(uint8_t slaveI2cAddr,
    uint16_t regAddr, uint8_t regVal);
static int32_t Serialzier_ReadReg(uint8_t slaveI2cAddr,
    uint16_t regAddr, uint8_t *regVal);
static int32_t Deserializer_ReadReg(uint8_t slaveI2cAddr,
    uint16_t regAddr, uint8_t *regVal);
static int32_t Deserializer_WriteReg(uint8_t   slaveI2cAddr,
    uint16_t  regAddr, uint8_t   regVal);
static int32_t IssSensor_detect_serializer(int8_t desHubInstId, uint16_t *cameras_detected);
static int32_t IssSensor_detect_sensor(uint8_t chId);
static int32_t IssSensor_DeserializerInit(void);
#if !defined(SOC_AM62A)
static int32_t powerOnDeserializers(void);
#endif

static int32_t IssSensor_DeserializerInit_B7();
static int32_t IssSensor_cfgDesScriptCustom(uint8_t  slaveAddr, I2cParams *script, uint8_t  bitOperation);

/*******************************************************************************
 *  Function Definition
 *******************************************************************************
 */
#if defined (MCU_PLUS_SDK)
/**
 * \brief I2C 16-bit register read fuunction
 *
 * This function is used to read the 16-bit data from the i2c
 * device registers
 *
 * \param   handle      [IN]    Low level driver handle
 * \param   slaveAddr   [IN]    I2C slave address
 * \param   regAddr     [IN]    I2C register offset address
 * \param   regData     [OUT]   I2C register data buffer
 * \param   numOfBytes  [IN]    Receive data width
 * \param   byteOrdSel  [IN]    Register address byte order select
 *
 * \return  BOARD_SOK in case of success or appropriate error code.
 */
int32_t Board_i2c16BitRegRd(void   *handle,
                                 uint32_t slaveAddr,
                                 uint16_t regAddr,
                                 uint8_t *regData,
                                 uint8_t numOfBytes,
                                 uint8_t byteOrdSel,
                                 uint32_t i2cTimeout)
{
    int32_t ret = 0;
    I2C_Transaction transaction;
    uint8_t tx[2];

    I2C_Handle i2cHandle = (I2C_Handle)handle;

    /* Initializes the I2C transaction structure with default values */
    I2C_Transaction_init(&transaction);

    transaction.slaveAddress = slaveAddr;
    transaction.writeBuf     = &tx[0];
    transaction.writeCount   = 2;
    transaction.readBuf      = NULL;
    transaction.readCount    = 0;
    transaction.timeout      = i2cTimeout;

    /* 16-bit regAddr data to be sent */
    if(byteOrdSel == BOARD_I2C_REG_ADDR_MSB_FIRST)
    {
        tx[0] = (uint8_t)((regAddr & 0xFF00) >> 8);
        tx[1] = (uint8_t)(regAddr & 0x00FF);
    }
    else
    {
        tx[0] = (uint8_t)(regAddr & 0x00FF);
        tx[1] = (uint8_t)((regAddr & 0xFF00) >> 8);
    }

    ret = I2C_transfer(i2cHandle, &transaction);
    if(ret != I2C_STS_SUCCESS)
    {
        issLogPrintf("Failing while transmitting the rd reg addr with error code - %d\n", ret);
        ret = -1;
        return ret;
    }

    transaction.writeBuf     = NULL;
    transaction.writeCount   = 0;
    transaction.readBuf      = regData;
    transaction.readCount    = numOfBytes;

    ret = I2C_transfer(i2cHandle, &transaction);
    if(ret != I2C_STS_SUCCESS)
    {
        issLogPrintf("Failing while reading the register data by returning - %d\n", ret);
        ret = -1;
        return ret;
    }

    return 0;
}


/**
 * \brief  Get ub9702 i2c address.
 *
 * This function is used to get the I2C address and
 * channel details of UB9702 module.
 * \param   hubInstance [IN]  instance of the csi hub
 * \param   domain      [IN]  I2C instance domain
 * \param   chNum       [IN]  buffer to hold i2c channel number
 * \param   i2cAddr     [IN]  buffer to hold i2c address
 *
 * \return  BOARD_SOK in case of success or appropriate error code.
 */
void Board_fpdUb9702GetI2CAddr(uint8_t *i2cAddr,
                             uint32_t hubInstance)
{

    if(hubInstance == BOARD_FPD_9702_CSI2_DES_HUB1)
    {
        *i2cAddr = 0x3DU;
    }
    else if(hubInstance == BOARD_FPD_9702_CSI2_DES_HUB2)
    {
        *i2cAddr = 0x30U;
    }
    else if(hubInstance == BOARD_FPD_9702_CSI2_DES_HUB3)
    {
        *i2cAddr = 0x32U;
    }
    else
    {
        printf("Enter the correct hub instance");
    }
}
#endif

/**
 *******************************************************************************
 *
 * \brief Powers off deserailzer hubs on Fusion1/2 boards
 *
 * \return  0 if success
 * \return -1 otherwise
 *
 *******************************************************************************
*/
int32_t powerOffDeserializers(void)
{
    int32_t status = -1;
    uint32_t tca6408SlaveAddr = TCA6408_I2C_ADDR_1;
    uint8_t regAddr = TCA6408_CONFIG_REG;
    uint8_t regVal = 0xFF;
    static uint8_t fusionBrdInitDone = 0;

    if(fusionBrdInitDone == 0)
    {
        status = Board_i2c8BitRegWr(gISS_Sensor_I2cHandle, tca6408SlaveAddr, regAddr, &regVal, 1U, SENSOR_I2C_TIMEOUT);
        if(0 == status)
        {
            issLogPrintf("Write 0x%.2x to TCA6408(0x%.2x) register 0x%.2x \n", regVal,
                     tca6408SlaveAddr, regAddr);
        }
        else
        {
            printf(
            "[powerOffDeserializers]: Board_i2c8BitRegWr returned %d while "
            "writing 0x%.2x to TCA6408(0x%.2x) register 0x%.2x \n",
            status, regVal, tca6408SlaveAddr, regAddr);
        }

        appLogWaitMsecs(200);

        fusionBrdInitDone = 0U;
    }
    return status;
}

void getIssSensorI2cInfo(uint8_t *byteOrder, I2C_Handle *i2cHndl)
{
    *byteOrder = gISS_Sensor_ByteOrder;
    *i2cHndl = gISS_Sensor_I2cHandle;
}

int32_t IssSensor_Init()
{
    int32_t status = -1;
    uint32_t cnt;

    for(cnt = 0; cnt < ISS_SENSORS_MAX_SUPPORTED_SENSOR; cnt++)
    {
        g_pSenHndl[cnt] = NULL;
        g_detectedSensors[cnt] = 0xFF;
    }

    status = appRemoteServiceRegister(
        IMAGE_SENSOR_REMOTE_SERVICE_NAME,
        ImageSensor_RemoteServiceHandler
    );

    if(status != 0)
    {
        printf(" REMOTE_SERVICE_SENSOR: ERROR: Unable to register remote service sensor handler\n");
        return status;
    }

    for (cnt = 0U; cnt < ISS_SENSORS_MAX_SENSOR_HANDLES; cnt ++)
    {
        gIssSensorTable[cnt] = NULL;
    }

    /*  Initialization function call of all supported sensors. Sensor
    initialization functions shall register the driver to the framework using
    IssSensor_Register(...) */
    status |= IssSensor_IMX390_Init();
#if defined(QNX) && defined(SOC_AM62A)
    status |= IssSensor_OV2312_Init();
#else /* QNX: I2C reads need update for other sensors */
    status |= IssSensor_IMX390DISCOVERY_NARROW_Init();
    status |= IssSensor_IMX390DISCOVERY_MID_Init();
    status |= IssSensor_IMX390DISCOVERY_WIDE_Init();
    status |= IssSensor_IMX728_Init();
    status |= IssSensor_AR0233_Init();
    status |= IssSensor_AR0820_Init();
    status |= IssSensor_rawtestpat_Init();
    status |= IssSensor_testpat_Init();
    status |= IssSensor_gw_ar0233_Init();
#endif
    return status;
}

int32_t IssSensor_Register(IssSensors_Handle *pSensorHandle)
{
    int32_t status = 0;
    uint32_t cnt = 0U;

    if (NULL == pSensorHandle)
    {
        status = -1;
    }
    else
    {
        /* Find a free entry in the sensor table */
        for (cnt = 0U; cnt < ISS_SENSORS_MAX_SENSOR_HANDLES; cnt++)
        {
            if (NULL == gIssSensorTable[cnt])
            {
                gIssSensorTable[cnt] = pSensorHandle;
                issLogPrintf("Found sensor %s at location %d \n", pSensorHandle->createPrms->name, cnt);
                break;
            }
        }

        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The test framework and test apps cannot reach this portion.
        The test framework does not support the configuration required to trigger this error scenario.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (cnt == ISS_SENSORS_MAX_SENSOR_HANDLES)
        {
            printf(" ISS_SENSOR: Could not register sensor \n");
            status = -1;
        }
        /* LDRA_JUSTIFY_END */
    }

    return (status);
}

IssSensors_Handle *IssSensor_getSensorHandleFromName(char *name)
{
    uint32_t cnt;
    IssSensors_Handle *pSensorHandle = NULL;

    /* Check For Errors */
    if (NULL == name)
    {
        return NULL;
    }

    for (cnt = 0U; cnt < ISS_SENSORS_MAX_SENSOR_HANDLES; cnt++)
    {
        pSensorHandle = gIssSensorTable[cnt];
        if(NULL == pSensorHandle)
        {
            issLogPrintf("pSensorHandle is NULL \n");
            return NULL;
        }
        if(NULL == pSensorHandle->createPrms)
        {
            issLogPrintf("createPrms is NULL \n");
            return NULL;
        }

        if (0 == strncmp(pSensorHandle->createPrms->name, name, ISS_SENSORS_MAX_NAME))
        {
            break;
        }
    }

    return pSensorHandle;
}

int32_t IssSensor_GetSensorInfo(char *name, IssSensor_CreateParams *sensor_prms)
{
    int32_t status = 0;
    IssSensors_Handle *pSensorHandle = NULL;

    pSensorHandle = IssSensor_getSensorHandleFromName(name);
    if(pSensorHandle == NULL)
    {
        printf(
            "[IssSensor_GetSensorInfo] Error: "
            "IssSensor_getSensorHandleFromName returned NULL\n");
        return -1;
    }

    memcpy(sensor_prms, pSensorHandle->createPrms,
                   sizeof(IssSensor_CreateParams));

    return status;
}

int32_t IssSensor_Delete(void* handle)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
    This behaviour is part of the application design. An error print statement can be added in a future release if required.
    <justification end> */
    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;

        memset(pSensorHandle->createPrms, 0, sizeof(IssSensor_CreateParams));
    }
    /* LDRA_JUSTIFY_END */

    return (status);
}

int32_t IssSensor_Config(void* handle, uint32_t chId, uint32_t feat)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        if((chId < pSensorHandle->createPrms->num_channels) && (chId < ISS_SENSORS_MAX_CHANNEL))
        {
            if (NULL != pSensorHandle->sensorFxns->config)
            {
                status = pSensorHandle->sensorFxns->config(chId, handle, feat);
            }
        }
        else
        {
            printf("IssSensor_Config Error : Incorrect channel ID %d \n", chId);
            status = -1;
        }
    }

    return (status);
}

int32_t IssSensor_PowerOn(void* handle, uint32_t chMask)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    num_sensors_open++;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        uint32_t chId = 0;
        pSensorHandle = (IssSensors_Handle *)handle;
        pSensorHandle->sensorIntfPrms->numCamerasStreaming = 0;
        while ((chMask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL))
        {
            if(chMask & 0x1)
            {
                if (NULL != pSensorHandle->sensorFxns->powerOn)
                {
                    status = pSensorHandle->sensorFxns->powerOn(chId, handle);
                }
            }
            chMask = chMask >> 1;
            chId++;
        }
    }

    return (status);

}

int32_t IssSensor_PowerOff(void* handle, uint32_t chMask)
{
    int32_t status;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (status != 0)
    {
        return status;
    }

    uint32_t chId = 0;
    uint32_t chIter = chMask;
    pSensorHandle = (IssSensors_Handle *)handle;

    while ((chIter > 0) && (chId < ISS_SENSORS_MAX_SUPPORTED_SENSOR))
    {
        if (chIter & 0x1)
        {
            if (pSensorHandle->sensorFxns->powerOff != NULL)
            {
                status = pSensorHandle->sensorFxns->powerOff(chId, handle);
            }
            num_sensors_open--;
        }
        chIter = chIter >> 1;
        chId++;
    }

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
    This behaviour is part of the application design. An error print statement can be added in a future release if required.
    <justification end> */
    if (num_sensors_open == 0)
    /* LDRA_JUSTIFY_END */
    {
        #if defined(SOC_AM62A) && defined(QNX)
        if(sensor_fd != -1)
        {
            close(sensor_fd);
        }
        sensor_fd = -1;
        #else
        #if !defined (MCU_PLUS_SDK)
        I2C_close(gISS_Sensor_I2cHandle);
        #endif
        #endif
        gISS_Sensor_I2cHandle = NULL;
    }

    return (status);
}

int32_t IssSensor_Start(void* handle, uint32_t chId)
{
    int32_t status = -1;
    uint32_t numCamerasEnabled = 1;
    IssSensors_Handle *pSensorHandle = (IssSensors_Handle *)NULL;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;

        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start> 
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        The test framework does not support the configuration required to trigger this error scenario.
        Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
        This behaviour is part of the application design. An error print statement can be added in a future release if required.
        <justification end> */
        if(chId < ISS_SENSORS_MAX_CHANNEL)
        /* LDRA_JUSTIFY_END */
        {
#if defined(SOC_AM62A) && defined(QNX)
            if(2312U == pSensorHandle->createPrms->dccId)
            {
                /* OV2312 case: sensor has 2 channels for RGB and IR*/
                if((chId + 1) < ISS_SENSORS_MAX_CHANNEL)
                {
                    g_pSenHndl[chId+1] = pSensorHandle;
                }
                else
                {
                    printf("Error: chId outside of bounds\n");
                    status = -1;
                }
            }
#endif
            if(NULL != pSensorHandle->sensorFxns->streamOn)
            {
                status = pSensorHandle->sensorFxns->streamOn(chId, handle);
            }
        }

        if(status == 0)
        {
            pSensorHandle->sensorIntfPrms->numCamerasStreaming += numCamerasEnabled;
        }
    }

    return (status);
}

int32_t IssSensor_Stop(void* handle, uint32_t chId)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if (0 == status)
    /* LDRA_JUSTIFY_END */
    {
        pSensorHandle = (IssSensors_Handle *)handle;

        if(chId < ISS_SENSORS_MAX_CHANNEL)
        {
            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start> 
            Rationale: The test framework and test apps cannot reach this portion.
            The test framework does not support the configuration required to trigger this error scenario.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */  
            if(NULL != pSensorHandle->sensorFxns->streamOff)
            /* LDRA_JUSTIFY_END */
            {
                status |= pSensorHandle->sensorFxns->streamOff(chId, handle);
            }
            pSensorHandle->sensorIntfPrms->numCamerasStreaming -= 1;
        }
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        The test framework does not support the configuration required to trigger this error scenario.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
        <justification end> */
        else
        {
            printf("IssSensor_Stop Error: Incorrect channel ID %d \n", chId);
            status = -1;
        }
        /* LDRA_JUSTIFY_END */
    }

    g_pSenHndl[chId] = NULL;

    return (status);
}

int32_t IssSensor_GetAeParams(void *handle, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;
    IssSensors_Handle *pSensorHandle = NULL;
 
    /* Check if the handle is valid or not */
    status = checkForHandle(handle);
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
    <justification end> */
    if (status != 0)
    {
        return status;
    }
    /* LDRA_JUSTIFY_END */
 
    pSensorHandle = (IssSensors_Handle *)handle;
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
    <justification end> */
    if (pSensorHandle == NULL)
    {
        printf("IssSensor_SetAeParams Error: sensor handle is NULL for channel ID %d \n", chId);
        return -1;
    }
    /* LDRA_JUSTIFY_END */
 
    if (pSensorHandle->sensorFxns->getAeParams != NULL) {
        status = pSensorHandle->sensorFxns->getAeParams(chId, handle, pExpPrms);
    }
 
    return (status);
}

int32_t IssSensor_SetAeParams(void *handle, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = -1;
    IssSensors_Handle * pSensorHandle = NULL;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        if(pSensorHandle != NULL)
        {
            if(NULL != pSensorHandle->sensorFxns->setAeParams)
            {
                status |= pSensorHandle->sensorFxns->setAeParams(handle, chId, pExpPrms);
            }
            else
            {
                printf("IssSensor_SetAeParams Error : Sensor setAeParams callback is NULL \n");
                memset(pExpPrms, 0x0, sizeof(IssSensor_ExposureParams));
            }
        }
        else
        {
            printf("IssSensor_SetAeParams Error: sensor handle is NULL for channel ID %d \n", chId);
            status = -1;
        }
    }

    return (status);
}

int32_t IssSensor_GetAwbParams(void *handle, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = 0;
    IssSensors_Handle *pSensorHandle = NULL;
 
    /* Check if the handle is valid or not */
    status = checkForHandle(handle);
    if (status != 0)
    {
        return status;
    }
 
    pSensorHandle = (IssSensors_Handle *)handle;
    if (pSensorHandle == NULL)
    {
        printf("IssSensor_SetAwbParams Error : sensor handle is NULL for channel ID %d\n", chId);
        return -1;
    }
 
    if (pSensorHandle->sensorFxns->getAwbParams != NULL)
    {
        status = pSensorHandle->sensorFxns->getAwbParams(chId, handle, pWbPrms);
    }
 
    return (status);
}

int32_t IssSensor_SetAwbParams(void *handle, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = -1;
    IssSensors_Handle * pSensorHandle = NULL;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        //if(chId < pSensorHandle->createPrms->num_channels)
        if(pSensorHandle != NULL)
        {
            if(NULL != pSensorHandle->sensorFxns->setAwbParams)
            {
                status |= pSensorHandle->sensorFxns->setAwbParams(handle, chId, pWbPrms);
            }
            else
            {
                printf("IssSensor_SetAwbParams Error : Sensor setAwbParams callback is NULL \n");
                memset(pWbPrms, 0x0, sizeof(IssSensor_WhiteBalanceParams));
            }
        }
        else
        {
            printf("IssSensor_SetAwbParams Error : sensor handle is NULL for  channel ID %d \n", chId);
            status = -1;
        }
    }

    return (status);
}

int32_t IssSensor_Control(void* handle, uint32_t cmd, void* cmdArgs, void* cmdRetArgs)
{
    int32_t status = VX_FAILURE;
    uint32_t chId = 0xFF;
    IMAGE_SENSOR_CTRLCMD ctrlCmd;
    uint8_t * cmd_ptr = (uint8_t *)cmdArgs;

    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    cmp_ptr cannot be NULL as it is pre-validated by ImageSensor_RemoteServiceHandler()
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if(NULL == cmd_ptr)
    {
        printf("Error : cmd_ptr is NULL \n");
        return VX_FAILURE;
    }
    /* LDRA_JUSTIFY_END */

    cmd_ptr += ISS_SENSORS_MAX_NAME;

    memcpy(&chId, cmd_ptr, sizeof(uint32_t));
    cmd_ptr += sizeof(uint32_t);

    memcpy(&ctrlCmd, cmd_ptr, sizeof(IMAGE_SENSOR_CTRLCMD));
    cmd_ptr += sizeof(IMAGE_SENSOR_CTRLCMD);

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
    Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
    <justification end> */
    switch(ctrlCmd)
    {
    /* LDRA_JUSTIFY_END */
        case IMAGE_SENSOR_CTRLCMD_GETEXPPRG:
            {
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                if(NULL != pSenHndl)
                {
                    status = pSenHndl->sensorFxns->getExpPrg(chId, (void*)pSenHndl, (IssAeDynamicParams *)cmd_ptr);
                }
                /* LDRA_JUSTIFY_START
                <metric start> statement branch <metric end>
                <justification start> 
                Rationale: The component level negative test framework and test applications cannot reach this portion.
                The test framework does not support the configuration required to trigger this error scenario.
                Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
                    However, due to the stated rationale, this is not tested.
                <justification end> */
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }
                /* LDRA_JUSTIFY_END */
            }
            break;
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The component level negative test framework and test applications CAN reach this portion.
                    No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
        Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
                However, due to the stated rationale, this is not tested.
        <justification end> */
        case IMAGE_SENSOR_CTRLCMD_INIT_AEWB_CFG:
            {
                IssSensors_Handle *pSenHndl = g_pSenHndl[chId];
                if (pSenHndl == NULL)
                {
                    printf("Error : sensor handle is NULL for channel %d.\n",chId);
                    status = VX_FAILURE;
                    break;
                }

                if (pSenHndl->sensorFxns->initAewbConfig)
                {
                    pSenHndl->sensorFxns->initAewbConfig(chId, (void*)pSenHndl);
                    status = VX_SUCCESS;
                }
            }
            break;
        /* LDRA_JUSTIFY_END */
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The component level negative test framework and test applications CAN reach this portion.
                    No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
        Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
                However, due to the stated rationale, this is not tested.
        <justification end> */
        case IMAGE_SENSOR_CTRLCMD_GETWBCFG:
            {
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                if(NULL != pSenHndl)
                {
                    if(NULL != pSenHndl->sensorFxns)
                    {
                        if(NULL != pSenHndl->sensorFxns->getWbCfg)
                        {
                            status = pSenHndl->sensorFxns->getWbCfg(chId, (void*)pSenHndl, (IssAwbDynamicParams *)cmd_ptr);
                        }else
                        {
                            /*Sensor driver does not support WB update API*/
                            memset(cmd_ptr, 0, sizeof(IssAwbDynamicParams));
                        }
                    }
                }
                else
                {
                    memset(cmd_ptr, -1, sizeof(IssAwbDynamicParams));
                    status = VX_FAILURE;
                }
            }
            break;
        /* LDRA_JUSTIFY_END */
        case IMAGE_SENSOR_CTRLCMD_SETEXPGAIN:
            {
                IssSensor_ExposureParams aePrms;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                /* LDRA_JUSTIFY_START
                <metric start> branch <metric end>
                <justification start> 
                Rationale: The test framework and test apps cannot reach this portion.
                The test framework does not support the configuration required to trigger this error scenario.
                Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
                However, due to the stated rationale, this is not tested.
                <justification end> */
                if(NULL != pSenHndl)
                /* LDRA_JUSTIFY_END */
                {
                    memcpy(&aePrms, cmd_ptr, sizeof(IssSensor_ExposureParams));
                    status = IssSensor_SetAeParams(pSenHndl, chId, &aePrms);
                }
                /* LDRA_JUSTIFY_START
                <metric start> statement branch <metric end>
                <justification start> 
                Rationale: The test framework and test apps cannot reach this portion.
                The test framework does not support the configuration required to trigger this error scenario.
                Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
                However, due to the stated rationale, this is not tested.
                <justification end> */
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }
                /* LDRA_JUSTIFY_END */
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_GETEXPGAIN:
            status = 0;
            {
                IssSensor_ExposureParams aePrms;
                memset(&aePrms, 0, sizeof(IssSensor_ExposureParams));
 
                IssSensors_Handle *pSenHndl = g_pSenHndl[chId];
                if (pSenHndl != NULL)
                {
                    status = IssSensor_GetAeParams(pSenHndl, chId, &aePrms);
                    memcpy(cmd_ptr, &aePrms, sizeof(IssSensor_ExposureParams));
                }
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_SETWBGAIN:
            {
                IssSensor_WhiteBalanceParams awbPrms;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                if(NULL != pSenHndl)
                {
                    memcpy(&awbPrms, cmd_ptr, sizeof(IssSensor_WhiteBalanceParams));
                    status = IssSensor_SetAwbParams(pSenHndl, chId, &awbPrms);
                }
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_GETWBGAIN:
            status = 0;
            {
                IssSensor_WhiteBalanceParams awbPrms;
                memset(&awbPrms, 0, sizeof(IssSensor_WhiteBalanceParams));
                
                IssSensors_Handle *pSenHndl = g_pSenHndl[chId];
                if (pSenHndl != NULL)
                {
                    status = IssSensor_GetAwbParams(pSenHndl, chId, &awbPrms);
                    memcpy(cmd_ptr, &awbPrms, sizeof(IssSensor_WhiteBalanceParams));
                }
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_DEBUG:
            /*Reserved for future use*/
            status = 0;
            issLogPrintf("IMAGE_SENSOR_CTRLCMD_DEBUG \n");
            {
                uint32_t * ptr32 = (uint32_t * )cmd_ptr;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                uint32_t rw_flag = *ptr32++; /*ReadWrite Flag = 1 for Write*/
                uint32_t devType = *ptr32++; /*Device Type*/
                uint32_t regAddr32 = *ptr32++; /*Register Address*/
                uint32_t regVal32 = *ptr32++; /*Register Value*/
                uint32_t slaveI2cAddr = 0x0;
                uint8_t regAddr = 0xBC;
                uint8_t regVal = 0xDE;

                if(0U == devType)
                {
                    uint8_t  ch_num;
                    uint32_t ub960InstanceId = chId;
                    uint8_t  i2cSlaveAddr8;
                    IssSensor_getDeserializerI2cAddr(ub960InstanceId, &ch_num, &i2cSlaveAddr8);
                    slaveI2cAddr = i2cSlaveAddr8;
                }
                else if(1U == devType)
                {
                    slaveI2cAddr = pSenHndl->createPrms->i2cAddrSer[chId];
                }
                else if(2U == devType)
                {
                    slaveI2cAddr = pSenHndl->createPrms->i2cAddrSensor[chId];
                }
                else if (255U == devType)
                {
                    slaveI2cAddr = *ptr32;
                }
                else
                {
                    printf("IMAGE_SENSOR_CTRLCMD_DEBUG Error : Unsupported devType %d \n", devType);
                    return -1;
                }

                if(2U == devType)
                {
                    I2cParams sensorI2cParams;
                    sensorI2cParams.nDelay = 1U;
                    sensorI2cParams.nRegAddr = (uint16_t)(regAddr32);
                    sensorI2cParams.nRegValue = (uint16_t)(regVal32);
                    status = pSenHndl->sensorFxns->readWriteReg(chId, pSenHndl, rw_flag, &sensorI2cParams);
                    if(0 != status)
                    {
                        printf("IMAGE_SENSOR_CTRLCMD_DEBUG : Error reading from register 0x%.2x on the image sensor %s \n", regAddr, pSenHndl->createPrms->name);
                    }
                    /* LDRA_JUSTIFY_START
                    <metric start> statement branch <metric end>
                    <justification start> 
                    Rationale: The test framework and test apps cannot reach this portion.
                    The test framework does not support the configuration required to trigger this scenario.
                    Effect on this unit: If the control reaches here, our code base is expected to read the register values and continue execution as expected.
                    However, due to the stated rationale, this is not tested.
                    <justification end> */
                    else
                    {
                        ptr32 = (uint32_t * )cmdArgs;
                        *ptr32 = (uint32_t)sensorI2cParams.nRegValue;
                    }
                    /* LDRA_JUSTIFY_END */
                }
                else
                {
                    if(0U == rw_flag)
                    {
                        regAddr = (uint8_t)(regAddr32);
                        status = Board_i2c8BitRegRd(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr, &regVal, 1U, SENSOR_I2C_TIMEOUT);
                        if(0 != status)
                        {
                            printf("IMAGE_SENSOR_CTRLCMD_DEBUG : Error reading from register 0x%.2x on slave device 0x%.2x \n", regAddr, slaveI2cAddr);
                        }
                        issLogPrintf("IMAGE_SENSOR_CTRLCMD_DEBUG : Read 0x%.2x from register 0x%.2x on slave device 0x%.2x \n", regVal, regAddr, slaveI2cAddr);
                    }
                    else if (1U == rw_flag)
                    {
                        regAddr = (uint8_t)(regAddr32);
                        regVal =  (uint8_t)(regVal32);
                        status = Board_i2c8BitRegWr(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr, &regVal, 1U, SENSOR_I2C_TIMEOUT);
                        /* LDRA_JUSTIFY_START
                        <metric start> branch <metric end>
                        <justification start> 
                        Rationale: The test framework and test apps cannot reach this portion.
                        The test framework does not support the configuration required to trigger this error scenario.
                        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
                        However, due to the stated rationale, this is not tested.
                        <justification end> */
                        if(0 != status)
                        /* LDRA_JUSTIFY_END */
                        {
                            printf("IMAGE_SENSOR_CTRLCMD_DEBUG : Error writing 0x%.2x to register 0x%.2x on slave device 0x%.2x \n", regVal, regAddr, slaveI2cAddr);
                        }
                    }
                    else
                    {
                        status = -1;
                        printf("IMAGE_SENSOR_CTRLCMD_DEBUG Error : Invalid rwflag = %d \n", rw_flag);
                    }

                    if(0 == status)
                    {
                        ptr32 = (uint32_t * )cmdArgs;
                        *ptr32 = (uint32_t)regVal;
                    }
                }
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_READ_SENSOR_REG:
            {
                uint32_t * ptr32 = (uint32_t * )cmd_ptr;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                I2cParams reg_read;
                reg_read.nDelay = 0;
                reg_read.nRegAddr = (uint16_t)(*ptr32);
                ptr32++;
                reg_read.nRegValue = 0xFF;
                status = pSenHndl->sensorFxns->readWriteReg(chId, pSenHndl, 0, &reg_read);
                *ptr32 = reg_read.nRegValue;
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_WRITE_SENSOR_REG:
            {
                uint32_t * ptr32 = (uint32_t * )cmd_ptr;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                I2cParams reg_write;
                reg_write.nDelay = 0;
                reg_write.nRegAddr = (uint16_t)(*ptr32);
                ptr32++;
                reg_write.nRegValue = (uint16_t)(*ptr32);
                status = pSenHndl->sensorFxns->readWriteReg(chId, pSenHndl, 1, &reg_write);
                *ptr32 = reg_write.nRegValue;
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_READ_SENSOR_REG_IMMEDIATE_I2C:
            {
                uint32_t *ptr32       = (uint32_t *)cmd_ptr;
                uint32_t i2cSlaveAddr = *ptr32++;
                uint16_t regAddr      = (uint16_t)(*ptr32++);
                uint8_t  regVal       = 0;
 
                I2C_Handle     sensorI2cHandle    = NULL;
                static uint8_t sensorI2cByteOrder = 255U;
                getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
                if (sensorI2cHandle == NULL)
                {
                    printf("Sensor I2C Handle is NULL \n");
                    break;
                }
                status = Board_i2c16BitRegRd(sensorI2cHandle, i2cSlaveAddr, regAddr, &regVal, 1U, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
                if (status != 0)
                {
                    printf("Error : I2C Timeout while reading from register 0x%x \n", regAddr);
                }
                *ptr32 = regVal;
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_WRITE_SENSOR_REG_IMMEDIATE_I2C:
            {
                uint32_t *ptr32       = (uint32_t *)cmd_ptr;
                uint32_t i2cSlaveAddr = *ptr32++;
                uint16_t regAddr      = (uint16_t)*ptr32++;
                uint8_t  regVal       = (uint8_t)*ptr32;
 
                I2C_Handle     sensorI2cHandle    = NULL;
                static uint8_t sensorI2cByteOrder = 255U;
                getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
                if (sensorI2cHandle == NULL)
                {
                    printf("Sensor I2C Handle is NULL \n");
                    break;
                }
                status = Board_i2c16BitRegWr(sensorI2cHandle, i2cSlaveAddr, regAddr, &regVal, 1U, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
                if (status != 0)
                {
                    printf("Error : I2C Timeout while writing 0x%x to register 0x%x \n", regVal, regAddr);
                }
            }
            break;
        default:
            status = -1;
            printf("IMAGE_SENSOR_CTRLCMD_DEBUG Error : Unknown control command %d \n", ctrlCmd);
            break;
    }

    return (status);
}

int32_t IssSensor_cfgDesScript(I2cParams *script, int8_t desHubInstId)
{
    uint16_t regAddr;
    uint8_t  regValue;
    uint16_t delayMilliSec;
    uint32_t cnt;
    int32_t status = 0;
    uint8_t  desI2cBusNum = 0;
    uint8_t  desI2cAddr;

    /*Assumption for Fusion1 board - maximum two UB960s*/
    if((desHubInstId < BOARD_CSI_INST_0) || (gDeserializerHubs < desHubInstId))
    {
        printf("Error : Invalid desHubInstId %d \n", desHubInstId);
        return -1;
    }

    IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum,
        &desI2cAddr);

    issLogPrintf("Deserializer config start \n");
    if(NULL != script)
    {
        cnt = 0;
        regAddr  = script[cnt].nRegAddr;
        regValue = script[cnt].nRegValue;
        delayMilliSec = script[cnt].nDelay;

        while(regAddr != 0xFFFF)
        {
            status |= Deserializer_WriteReg(desI2cAddr, regAddr, regValue);
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start> 
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            The test framework does not support the configuration required to trigger this error scenario.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
                However, due to the stated rationale, this is not tested.
            <justification end> */
            if (0 != status)
            {
                printf(" Deserializer Error: Reg Write Failed for regAddr 0x%.2x, cnt = %d\n", regAddr, cnt);
                break;
            }
            /* LDRA_JUSTIFY_END */

            if(delayMilliSec > 0)
            {
                appLogWaitMsecs(delayMilliSec);
            }

            cnt++;
            regAddr  = script[cnt].nRegAddr;
            regValue = script[cnt].nRegValue;
            delayMilliSec = script[cnt].nDelay;
        }
    }
    issLogPrintf("End of deserializer config \n");
    return (status);
}

int32_t ub953_cfgScript(uint8_t  i2cInstId, uint8_t  slaveAddr, I2cParams *script)
{
    uint16_t regAddr;
    uint8_t  regValue;
    uint16_t delayMilliSec;
    uint32_t cnt;
    int32_t status = 0;

    issLogPrintf("ub953 config start : slaveAddr = 0x%.2x \n", slaveAddr);
    if(NULL != script)
    {
        cnt = 0;
        regAddr  = script[0].nRegAddr;

        while(regAddr != 0xFFFF)
        {
            regAddr  = script[cnt].nRegAddr;
            regValue = script[cnt].nRegValue;
            delayMilliSec = script[cnt].nDelay;
            /* Convert Registers address and value into 8bit array */
            status |= Serialzier_WriteReg(slaveAddr, regAddr, regValue);

            if (0 != status)
            {
                printf(" UB953 Error: Reg Write Failed for regAddr 0x%.2x\n", regAddr);
                break;
            }
            cnt++;
            if(delayMilliSec > 0)
            {
                appLogWaitMsecs(delayMilliSec);
            }
        }
    }
    issLogPrintf(" End of UB953 config \n");
    return (status);
}

/* To do masked write, set delay to 0x00FFFFFF where the 2 MSB is the delay, the value is the masking value
 bitOperation: 0 = OR; 1 = AND
*/
static int32_t IssSensor_cfgDesScriptCustom(uint8_t  desI2cAddr, I2cParams *script, uint8_t  bitOperation)
{
    int32_t status = 0;

    uint16_t regAddr;
    uint8_t regVal;
    uint32_t delayMilliSec;
    uint32_t cnt;

    if(NULL != script)
    {
        cnt = 0;
        regAddr  = script[cnt].nRegAddr;
        regVal = script[cnt].nRegValue;
        delayMilliSec = script[cnt].nDelay;

        while(regAddr != 0xFFFF)
        {
            if((delayMilliSec & 0x00FFFFFF) == 0x00FFFFFF)
            {
                /* masked write */
                issLogPrintf("%d , masking with 0x%02x, writing back with 0x%02x\n", cnt+1, script[cnt].nRegValue, regVal);

                regVal = 0x00;
                status |= Deserializer_ReadReg(desI2cAddr, regAddr, &regVal);
                if(bitOperation == 0)
                {
                    regVal = regVal | script[cnt].nRegValue;
                }
                else if(bitOperation == 1)
                {
                    regVal = regVal & script[cnt].nRegValue;
                }
                else
                {
                    printf("IssSensor_cfgDesScriptCustom: Invalid bit operation: %d. Defaulting to OR \n", bitOperation);
                    bitOperation = 0;
                }
                status |=  Deserializer_WriteReg(desI2cAddr, regAddr, regVal);

                delayMilliSec = delayMilliSec >> 24;
            }
            else
            {
                issLogPrintf("%d Writing 0x%02x to 0x%02x\n", cnt+1, regVal, regAddr);
                Deserializer_WriteReg(desI2cAddr, regAddr, regVal);
            }

            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start> 
            Rationale: The component level negative test framework and test applications CAN reach this portion.
                        No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
            Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
                    However, due to the stated rationale, this is not tested.
            <justification end> */
            if(delayMilliSec > 0)
            /* LDRA_JUSTIFY_END */
            {
                issLogPrintf("\tdelaying for 0x%02x\n", delayMilliSec);
                appLogWaitMsecs(delayMilliSec);
            }

            cnt++;
            regAddr  = script[cnt].nRegAddr;
            regVal = script[cnt].nRegValue;
            delayMilliSec = script[cnt].nDelay;
        }
    }
    issLogPrintf(" End of deserializer config \n");
    return (status);
}

#if defined(SOC_AM62A) && defined(QNX)
int32_t updateUB960VCMapConfig()
{
    uint8_t port=0, vc=0, status=0;
    uint8_t ub960InstanceId = 0;
    uint8_t ub960I2cInstId;
    uint8_t ub960I2cAddr;
    ub960MapParams vc_map;
    uint8_t portSelRegValues[4]= {0x01, 0x12, 0x24, 0x38};

    Board_fpdU960GetI2CAddr(&ub960I2cInstId, &ub960I2cAddr, ub960InstanceId);
    for(int cnt=0;cnt<ISS_SENSORS_MAX_SUPPORTED_SENSOR;cnt++)
    {
        if(g_detectedSensors[cnt] == 1) /*OV2312*/
        {
            vc_map.nPort = port;
            vc_map.nVC = vc | ((vc+1)<<2);

            status |= Deserializer_WriteReg(ub960I2cAddr, 0x4C, portSelRegValues[port]);
            if (0 != status)
            {
                issLogPrintf("UB960 Error: Reg Write Failed for regAddr %x, cnt = %d\n", 0x4C, portSelRegValues[port]);
                break;
            }
            appLogWaitMsecs(100);

            status |= Deserializer_WriteReg(ub960I2cAddr, 0x72,vc_map.nVC);
            if (0 != status)
            {
                issLogPrintf("UB960 Error: Reg Write Failed for regAddr %x, cnt = %d\n", 0x72, vc_map.nVC);
                break;
            }
            appLogWaitMsecs(100);
            port++;
            vc= vc+2;
        }
        else if(g_detectedSensors[cnt] == 0) /*IMX390*/
		{
            vc_map.nPort = port;
            vc_map.nVC = vc;

            status |= Deserializer_WriteReg(ub960I2cAddr, 0x4C,portSelRegValues[port]);
            if (0 != status)
            {
                issLogPrintf("UB960 Error: Reg Write Failed for regAddr %x, cnt = %d\n", 0x4C, portSelRegValues[port]);
                break;
            }
            appLogWaitMsecs(100);

            status |= Deserializer_WriteReg(ub960I2cAddr, 0x72,vc_map.nVC);
            if (0 != status)
            {
                issLogPrintf("UB960 Error: Reg Write Failed for regAddr %x, cnt = %d\n",  0x72,vc_map.nVC);
                break;
            }
            appLogWaitMsecs(100);
            port++;
            vc++;
        }
    }
    return 0;
}
#endif

int32_t enableUB960Broadcast(int8_t desHubInstId)
{
    int32_t status = 0;
    I2cParams enableUB960BroadcastScript[] =
    {
        {0x4C, 0x0F, 0x10},
        {0xFFFF,0x00, 0x00}
    };

    status = IssSensor_cfgDesScript(enableUB960BroadcastScript, desHubInstId);

    return status;
}

int32_t disableUB960Broadcast(int8_t desHubInstId)
{
    int32_t status = 0;
    I2cParams disableUB960BroadcastScript[] =
    {
        {0x4C, DES_PORT_SEL_0, 0x10},
        {0x5C, (SER_0_I2C_ALIAS<< 1U), 0x10},
        {0x66, (SENSOR_0_I2C_ALIAS << 1U), 0x10},

        {0x4C, DES_PORT_SEL_1, 0x10},
        {0x5C, (SER_1_I2C_ALIAS<< 1U), 0x10},
        {0x66, (SENSOR_1_I2C_ALIAS << 1U), 0x10},

        {0x4C, DES_PORT_SEL_2, 0x10},
        {0x5C, (SER_2_I2C_ALIAS<< 1U), 0x10},
        {0x66, (SENSOR_2_I2C_ALIAS << 1U), 0x10},

        {0x4C, DES_PORT_SEL_3, 0x10},
        {0x5C, (SER_3_I2C_ALIAS<< 1U), 0x10},
        {0x66, (SENSOR_3_I2C_ALIAS << 1U), 0x10},

        {0xFFFF,0x00, 0x00}
    };

    if(0U == desHubInstId)
    {
        disableUB960BroadcastScript[1].nRegValue = (SER_0_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[2].nRegValue = (SENSOR_0_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[4].nRegValue = (SER_1_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[5].nRegValue = (SENSOR_1_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[7].nRegValue = (SER_2_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[8].nRegValue = (SENSOR_2_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[10].nRegValue = (SER_3_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[11].nRegValue = (SENSOR_3_I2C_ALIAS<<1U);
    } else if(1U == desHubInstId)
    {
        disableUB960BroadcastScript[1].nRegValue = (SER_4_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[2].nRegValue = (SENSOR_4_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[4].nRegValue = (SER_5_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[5].nRegValue = (SENSOR_5_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[7].nRegValue = (SER_6_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[8].nRegValue = (SENSOR_6_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[10].nRegValue = (SER_7_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[11].nRegValue = (SENSOR_7_I2C_ALIAS<<1U);
    } 
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
    Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
    <justification end> */
    else if(2U == desHubInstId)
    /* LDRA_JUSTIFY_END */
    {
        disableUB960BroadcastScript[1].nRegValue = (SER_8_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[2].nRegValue = (SENSOR_8_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[4].nRegValue = (SER_9_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[5].nRegValue = (SENSOR_9_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[7].nRegValue = (SER_10_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[8].nRegValue = (SENSOR_10_I2C_ALIAS<<1U);

        disableUB960BroadcastScript[10].nRegValue = (SER_11_I2C_ALIAS<<1U);
        disableUB960BroadcastScript[11].nRegValue = (SENSOR_11_I2C_ALIAS<<1U);
    }
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    else
    {
        printf("enableUB960Broadcast: Error: Invalid instance ID \n");
        return -1;
    }
    /* LDRA_JUSTIFY_END */

    status = IssSensor_cfgDesScript(disableUB960BroadcastScript, desHubInstId);

    return status;
}

int8_t getDeserializerInstIdFromChId(uint32_t chId)
{
    int8_t desHubInstId = -1;
    /* Assumptions :
    *  max 2 UB960  instances (Fusion1 Board)
    *  max 3 UB9702 instances (Fusion2 Board)
    *  up to 4 cameras per instance of UB960/UB9702
    */

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this scenario.
    Effect on this unit: If the control reaches here, our code base is expected to continue execution as expected.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if(chId < 4U)
    /* LDRA_JUSTIFY_END */
    {
        desHubInstId = BOARD_CSI_INST_0;
    }
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
    This behaviour is part of the application design. An error print statement can be added in a future release if required.
    <justification end> */
    else if(chId < 8U)
    {
        desHubInstId = BOARD_CSI_INST_1;
    }
    /* LDRA_JUSTIFY_END */

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this scenario.
    Effect on this unit: If the control reaches here, our code base is expected to continue execution as expected.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    else if((gFusion2Det == vx_true_e) && (chId < 12U))
    /* LDRA_JUSTIFY_END */
    {
        desHubInstId = BOARD_CSI_INST_2;
    }
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    else
    {
        printf("Error : Invalid chId 0x%.2x \n", chId);
        desHubInstId = -1;
    }
    /* LDRA_JUSTIFY_END */

    return desHubInstId;
}

/* LDRA_JUSTIFY_START
<metric start> statement branch <metric end>
<justification start> 
Rationale: The component level negative test framework and test applications CAN reach this portion.
            No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
        However, due to the stated rationale, this is not tested.
<justification end> */
int32_t enableUB960Streaming(uint32_t chId)
{
    int32_t status = -1;
    int8_t desHubInstId = getDeserializerInstIdFromChId(chId);
    uint8_t desI2cAddr;
    uint8_t desI2cBusNum = 0;
    uint8_t regVal = 0x0;

    I2cParams ub960DesCSI2Enable[SENSOR_4_I2C_ALIAS] = {
        /* CSI-TX Port Config */
        {0x32, 0x03, 0x20}, /* Read CSI-TX Port 0, Write CSI-TX Ports 0 and 1 */
        {0x20, 0xF0, 0x10}, /* Disable FPD-Link RX forwarding */
        {0x33, 0x03, 0x20}, /* 4 CSI-2 Lanes; Continous clock mode, CSI-2 output enabled */
        {0x20, 0x00, 0x10}, /* Enable FPD-Link RX forwarding (all ports) to CSI-TX port 0 */

        {0xFFFF,0x00, 0x00} /* End script */
    };

    status = IssSensor_cfgDesScript(ub960DesCSI2Enable, desHubInstId);

    return status;
}
/* LDRA_JUSTIFY_END */

int32_t disableUB960Streaming(uint32_t chId)
{
    int32_t status = -1;
    int8_t desHubInstId = getDeserializerInstIdFromChId(chId);

    I2cParams ub960DesCSI2Disable[3u] = {
        {0x4C, 0x00, 0x10},
        {0x33, 0x02, 0x10},
        {0xFFFF, 0x00, 0x0} //End of script
    };

    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if(desHubInstId < 0)
    {
        printf("Error : Invalid desHubInstId \n");
    }
    else
    /* LDRA_JUSTIFY_END */
    {
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
    Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
    <justification end> */
    switch(chId)
        {
    /* LDRA_JUSTIFY_END */
        case 0:
        case 4:
        case 8:
            ub960DesCSI2Disable[0].nRegValue = DES_PORT_SEL_0;
            break;
        case 1:
        case 5:
        case 9:
            ub960DesCSI2Disable[0].nRegValue = DES_PORT_SEL_1;
            break;
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The test framework and test apps cannot reach this portion.
        The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
        Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
        <justification end> */
        case 2:
        case 6:
        /* LDRA_JUSTIFY_END */
        case 10:
            ub960DesCSI2Disable[0].nRegValue = DES_PORT_SEL_2;
            break;
        case 3:
        case 7:
        case 11:
            ub960DesCSI2Disable[0].nRegValue = DES_PORT_SEL_3;
            break;
        default:
            printf("Error : disableUB960Streaming: Invalid channel ID 0x%.2x \n", chId);
            status = -1;
            break;
        }
        status = IssSensor_cfgDesScript(ub960DesCSI2Disable, desHubInstId);
    }

    return status;
}

// Selects a specific port on the UB960.
// This function is unaware of broadcast mode, check for broadcast mode
// before calling, otherwise broadcast mode will be clobbered.
// Returns -1 on errors, 0 on success.
int32_t UB960_SelectPort(uint32_t chId)
{
    int32_t status = -1;
    int8_t desHubInstId = getDeserializerInstIdFromChId(chId);
    uint8_t desI2cAddr;
    uint8_t desI2cBusNum = 0;
    uint8_t portSelReg = 0x4C;
    uint8_t portSelVal = 1 << (chId%4) | (chId%4) << 4; // Select the proper port.

    
    IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum,
        &desI2cAddr);

    if(desI2cAddr != 0) 
    {
        status = 0;
    }
    if(status == 0) 
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    {
        status = Deserializer_WriteReg(desI2cAddr, portSelReg, portSelVal); 
    }
    /* LDRA_JUSTIFY_END */
    return status;
}

int32_t UB960_SetSerAlias(uint32_t chId, uint8_t ser_alias_i2c_addr_7bit)
{
    int32_t status = -1;
    int8_t desHubInstId = getDeserializerInstIdFromChId(chId);
    uint8_t desI2cAddr;
    uint8_t  desI2cBusNum = 0;
    uint8_t portSelReg = 0x4C;
    uint8_t portSelVal;

    I2cParams ub960setSerAlias[2u] = {
        {0x5C, 0x0, 0x1},
        {0xFFFF, 0x0, 0x0}
    };

    IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum,
        &desI2cAddr);

    Deserializer_ReadReg(desI2cAddr, portSelReg, &portSelVal);
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
    Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
    <justification end> */
    if(0x0F != portSelVal)
    /* LDRA_JUSTIFY_END */
    {
        status = UB960_SelectPort(chId);
    }

    if(status == 0) {
        ub960setSerAlias[0].nRegValue = ser_alias_i2c_addr_7bit << 1;
        status = IssSensor_cfgDesScript(ub960setSerAlias, desHubInstId);
    }

    return status;
}

int32_t UB960_SetAlias(uint32_t chId, uint8_t slot, uint8_t phy_i2c_addr_7bit, uint8_t alias_i2c_addr_7bit)
{
    int32_t status = -1;
    int8_t desHubInstId = getDeserializerInstIdFromChId(chId);
    uint8_t desI2cAddr;
    uint8_t  desI2cBusNum = 0;
    uint8_t portSelReg = 0x4C;
    uint8_t portSelVal;

    uint8_t alias_register_pairs[8][2] = {
        {0x5D, 0x65}, 
        {0x5E, 0x66},
        {0x5F, 0x67},
        {0x60, 0x68},
        {0x61, 0x69},
        {0x62, 0x70},
        {0x63, 0x71},
        {0x64, 0x72}
    };

    I2cParams ub960setAlias[3u] = {
        {0x0, 0x0, 0x1},
        {0x0, 0x0, 0x1},
        {0xFFFF, 0x0, 0x0}
    };

    ub960setAlias[0].nRegAddr = alias_register_pairs[slot][0];
    ub960setAlias[1].nRegAddr = alias_register_pairs[slot][1]; // Select which alias we are setting.

    printf("Configuring Alias on UB960 CHID: %i, aliasing 0x%.2x to 0x%.2x\n", chId, phy_i2c_addr_7bit, alias_i2c_addr_7bit);

    IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum,
        &desI2cAddr);

    Deserializer_ReadReg(desI2cAddr, portSelReg, &portSelVal);
    if(0x0F != portSelVal)
    {
        status = UB960_SelectPort(chId);
    }

    if(status == 0) {
        ub960setAlias[0].nRegValue = phy_i2c_addr_7bit<<1U;
        ub960setAlias[1].nRegValue = alias_i2c_addr_7bit << 1U;
        status = IssSensor_cfgDesScript(ub960setAlias, desHubInstId);
    }

    return status;
}

int32_t UB960_SetSensorAlias(uint32_t chId, uint8_t sensor_phy_i2c_addr_7bit, uint8_t ser_alias_i2c_addr_7bit)
{
    int32_t status = -1;
    int8_t desHubInstId = getDeserializerInstIdFromChId(chId);
    uint8_t pageSelReg = 0x4C;
    uint8_t pageSelVal = 0xFF;
    uint8_t desI2cAddr;
    uint8_t  desI2cBusNum;
    uint8_t  sensor_alias[ISS_SENSORS_MAX_SUPPORTED_SENSOR] =
        {SENSOR_0_I2C_ALIAS, SENSOR_1_I2C_ALIAS, SENSOR_2_I2C_ALIAS, SENSOR_3_I2C_ALIAS,
        SENSOR_4_I2C_ALIAS, SENSOR_5_I2C_ALIAS, SENSOR_6_I2C_ALIAS, SENSOR_7_I2C_ALIAS,
        SENSOR_8_I2C_ALIAS, SENSOR_9_I2C_ALIAS, SENSOR_10_I2C_ALIAS, SENSOR_11_I2C_ALIAS};

    I2cParams ub960setSensorAlias[5u] = {
        {0x4C, 0x00, 0x1},
        {0x5E, 0x00, 0x1},
        {0x66, 0x00, 0x1},
        {0x5C, 0x00, 0x1},
        {0xFFFF, 0x00, 0x0} //End of script
    };

    if(desHubInstId < 0)
    {
        printf("Error : Invalid desHubInstId %d\n", desHubInstId);
    }
    else
    {
        IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum,
            &desI2cAddr);

        Deserializer_ReadReg(desI2cAddr, pageSelReg, &pageSelVal);
        if(0x0F == pageSelVal)
        {
            /*Broadcast Enabled. No need to set page select register*/
            ub960setSensorAlias[0].nRegValue = 0x0F;
        }
        else
        {
            /*Broadcast not enabled. Must set page select as per channel ID*/
            switch(chId)
            {
                case 0:
                case 4:
                case 8:
                    ub960setSensorAlias[0].nRegValue = DES_PORT_SEL_0;
                    break;
                case 1:
                case 5:
                case 9:
                    ub960setSensorAlias[0].nRegValue = DES_PORT_SEL_1;
                    break;
                case 2:
                case 6:
                case 10:
                    ub960setSensorAlias[0].nRegValue = DES_PORT_SEL_2;
                    break;
                case 3:
                case 7:
                case 11:
                    ub960setSensorAlias[0].nRegValue = DES_PORT_SEL_3;
                    break;
                default:
                    printf("Error : UB960_SetSensorAlias: Invalid channel ID 0x%.2x \n", chId);
                    status = -1;
                    break;
            }
        }

        ub960setSensorAlias[1].nRegValue = sensor_phy_i2c_addr_7bit << 1U;
        ub960setSensorAlias[2].nRegValue = sensor_alias[chId]<<1U;
        ub960setSensorAlias[3].nRegValue = ser_alias_i2c_addr_7bit<<1U;
        status = IssSensor_cfgDesScript(ub960setSensorAlias, desHubInstId);
    }

    return status;
}

void IssSensor_CloseHandle()
{
    #if !defined(MCU_PLUS_SDK)
    I2C_close(gISS_Sensor_I2cHandle);
    #endif

    gISS_Sensor_I2cHandle = NULL;
}

static int32_t IssSensor_registerHandle(void* handle, uint32_t chMask)
{
    int32_t status = -1;
    uint32_t chId = 0;

    while ((chMask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL))
    {
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start> 
        Rationale: The test framework and test apps cannot reach this portion.
        The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
        Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
        <justification end> */
        if((chMask & 0x1) == 0x1)
        /* LDRA_JUSTIFY_END */
        {
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start> 
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            The test framework does not support the configuration required to trigger this error scenario.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
                However, due to the stated rationale, this is not tested.
            <justification end> */
            if(g_pSenHndl[chId] != NULL)
            {
                printf("Warning : channel ID %d already has a registered sensor handle \n", chId);
            }
            /* LDRA_JUSTIFY_END */
            issLogPrintf("Registered handle at g_pSenHndl[%d] for chId = %d \n", chId, chId);
            g_pSenHndl[chId] = handle;
        }
        chMask = chMask >> 1;
        chId++;
    }

    status = 0;
    return status;
}

int32_t ImageSensor_RemoteServiceHandler(char *service_name, uint32_t cmd,
    void *prm, uint32_t prm_size, uint32_t flags)
{
    int32_t status = -1;
    uint16_t cmdMemoryNeeded = 0;
    uint8_t * cmd_param = (uint8_t * )prm;
    uint8_t chId;
    uint32_t sensor_features_requested = 0;
    uint32_t channel_mask = 0;
    //uint32_t channel_mask_supported = 0;
    char * sensor_name = NULL;
    IssSensors_Handle * pSenHndl = NULL;
    IssSensor_CreateParams * pSenParams = (IssSensor_CreateParams * )NULL;
    uint32_t bit_count = 0;
    uint32_t tmpchMask = 0;

    switch(cmd)
    {
        case IM_SENSOR_CMD_ENUMERATE:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_ENUMERATE \n");
            cmdMemoryNeeded = ISS_SENSORS_MAX_NAME * ISS_SENSORS_MAX_SENSOR_HANDLES;
            if(prm_size < cmdMemoryNeeded)
            {
                printf("Error : Insufficient prm size %d, need at least %d \n", prm_size, cmdMemoryNeeded);
                return -1;
            }
#if !defined(B7_IMPLEMENTATION)
            IssSensor_DeserializerInit();
#endif
            {
                uint8_t count;
                for(count = 0; count < ISS_SENSORS_MAX_SENSOR_HANDLES; count++)
                {
                    pSenHndl = gIssSensorTable[count];
                    if(NULL !=pSenHndl)
                    {
                        memcpy(cmd_param + (count*ISS_SENSORS_MAX_NAME), pSenHndl->createPrms->name, ISS_SENSORS_MAX_NAME);
                    }
                }
            }
            status = 0;

            break;
        case IM_SENSOR_CMD_QUERY:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_QUERY \n");
            sensor_name = (char*)(cmd_param);
            issLogPrintf("Received Query for %s \n", sensor_name);

            /*Copy sensor properties at prm, after sensor name*/
            pSenParams = (IssSensor_CreateParams * )(cmd_param+ISS_SENSORS_MAX_NAME);
            status = IssSensor_GetSensorInfo(sensor_name, pSenParams);
            break;
        case IM_SENSOR_CMD_PWRON:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_PWRON \n");
            sensor_name = (char*)(cmd_param);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            issLogPrintf("IM_SENSOR_CMD_PWRON : channel_mask = 0x%.2x \n", channel_mask);

            pSenHndl = IssSensor_getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                status = IssSensor_registerHandle(pSenHndl, channel_mask);
#if defined(B7_IMPLEMENTATION)
                if(status == -1)
                {
                    printf("ERROR : IssSensor_registerHandle returned -1");
                }
                else
                {
                    IssSensor_DeserializerInit();
                    status = IssSensor_PowerOn((void*)pSenHndl, channel_mask);
                }
#else
                status = IssSensor_PowerOn((void*)pSenHndl, channel_mask);
#endif
            }
            break;
        case IM_SENSOR_CMD_CONFIG:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_CONFIG \n");
            sensor_name = (char*)(cmd_param);
            memcpy(&sensor_features_requested, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            issLogPrintf("Application requested features = 0x%.2x \n ", sensor_features_requested);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME+sizeof(uint32_t)), sizeof(uint32_t));
            pSenHndl = IssSensor_getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                status = 0;
                int32_t probeStatus;

                if(1U == pSenHndl->sensorIntfPrms->sensorBroadcast)
                {
                    /* enable broadcast mode only if more than one camera connected to des hub */
                    tmpchMask = (channel_mask & 0x00F);
                    bit_count = (tmpchMask & 1) + ((tmpchMask>>1) & 1) + ((tmpchMask>>2) & 1) + ((tmpchMask>>3) & 1);
                    if(bit_count > 1)
                    {
                        issLogPrintf("Configuring all cameras on UB9xx_0 Des in broadcast mode \n");
                        status |= enableUB960Broadcast(0);
                        status |= IssSensor_Config((void*)pSenHndl, 0, sensor_features_requested);
                        channel_mask &= 0xFF0;
                    }

                    bit_count = 0;
                    tmpchMask = ((channel_mask >> 4) & 0x00F);
                    bit_count = (tmpchMask & 1) + ((tmpchMask>>1) & 1) + ((tmpchMask>>2) & 1) + ((tmpchMask>>3) & 1);
                    if(bit_count > 1)
                    {
                        issLogPrintf("Configuring all cameras on UB9xx_1 Des in broadcast mode \n");
                        status |= enableUB960Broadcast(1);
                        status |= IssSensor_Config((void*)pSenHndl, 4, sensor_features_requested);
                        channel_mask &= 0xF0F;
                    }

                    bit_count = 0;
                    tmpchMask = ((channel_mask >> 8) & 0x00F);
                    bit_count = (tmpchMask & 1) + ((tmpchMask>>1) & 1) + ((tmpchMask>>2) & 1) + ((tmpchMask>>3) & 1);
                    if(bit_count > 1)
                    {
                        issLogPrintf("Configuring all cameras on UB9xx_2 Des in broadcast mode \n");
                        status |= enableUB960Broadcast(2);
                        status |= IssSensor_Config((void*)pSenHndl, 8, sensor_features_requested);
                        channel_mask &= 0x0FF;
                    }
                }

                issLogPrintf("Disabling broadcast mode \n");
                for(int i = 0; i < gDeserializerHubs; i++)
                {
                    disableUB960Broadcast(i);
                }


                issLogPrintf("IM_SENSOR_CMD_CONFIG channel_mask = %.2x\n", channel_mask);

                if(0 != channel_mask)
                {
                    chId = 0;
                    while( (channel_mask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL) )
                    {
                        if((channel_mask & 0x1) == 0x1)
                        {
                            issLogPrintf("Configuring camera # %d \n", chId);
                            status |= IssSensor_Config((void*)pSenHndl, chId, sensor_features_requested);
                        }
                        chId++;

                        channel_mask = channel_mask >> 1U;
#if defined(SOC_AM62A) && defined(QNX)
                        if(strcmp(sensor_name, "OV2312-UB953_LI") == 0)
                        {
                            channel_mask = channel_mask >> 1U;
                        }
#endif
                    }
                }
            }
            issLogPrintf("IM_SENSOR_CMD_CONFIG returning status = %d\n", status);

            break;
        case IM_SENSOR_CMD_STREAM_ON:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_STREAM_ON \n");
            sensor_name = (char*)(cmd_param);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            pSenHndl = IssSensor_getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                issLogPrintf("IM_SENSOR_CMD_STREAM_ON:  channel_mask = 0x%.2x\n", channel_mask);
                status = 0;

                //channel_mask_supported = (1<<pSenHndl->createPrms->num_channels) - 1;
                //channel_mask &= channel_mask_supported;
                /* Broadcast stream on to sensor is unstable. Setting this to 2 so it never goes in, but leaving the code to be fixed */
                if(2U == pSenHndl->sensorIntfPrms->sensorBroadcast)
                {
                    if((channel_mask & 0x00F) != 0)
                    {
                        issLogPrintf("Starting all cameras on UB9xx_0 Des in broadcast mode \n");
                        status |= enableUB960Broadcast(0);
                        status |= IssSensor_Start((void*)pSenHndl, 0);
                        channel_mask &= 0xFF0;
                    }

                    if((channel_mask & 0x0F0) != 0)
                    {
                        issLogPrintf("Starting all cameras on UB9xx_1 Des in broadcast mode \n");
                        status |= enableUB960Broadcast(1);
                        status |= IssSensor_Start((void*)pSenHndl, 4);
                        channel_mask &= 0xF0F;
                    }

                    if((channel_mask & 0xF00) != 0)
                    {
                        issLogPrintf("Starting all cameras on UB9xx_2 Des in broadcast mode \n");
                        status |= enableUB960Broadcast(2);
                        status |= IssSensor_Start((void*)pSenHndl, 8);
                        channel_mask &= 0x0FF;
                    }
                }

                /*Disable broadcast after enabling streaming
                so that every camera can have independent 2A control */
                for(int i = 0; i < gDeserializerHubs; i++)
                {
                    disableUB960Broadcast(i);
                }

#if defined(SOC_AM62A) && defined(QNX)
                updateUB960VCMapConfig();
#endif

                chId = 0;

                while( (channel_mask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL) )
                {
                    if(channel_mask & 0x1)
                    {
                        status |= IssSensor_Start((void*)pSenHndl, chId);
                        if(status < 0)
                        {
                            printf("Error : Failed to start sensor at channel Id %d \n", chId);
                        }
                        if( NULL ==g_pSenHndl[chId])
                        {
                            printf("Error : sensor handle at channel Id %d = NULL \n", chId);
                        }
                    }

                    chId++;

                    channel_mask = channel_mask >> 1U;
#if defined(SOC_AM62A) && defined(QNX)
                    if(strcmp(sensor_name, "OV2312-UB953_LI") == 0)
                    {
                        channel_mask = channel_mask >> 1U;
                    }
#endif
                }
            }

            break;
        case IM_SENSOR_CMD_STREAM_OFF:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_STREAM_OFF \n");

            sensor_name = (char*)(cmd_param);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            pSenHndl = IssSensor_getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                issLogPrintf("IM_SENSOR_CMD_STREAM_ON:  channel_mask = 0x%.2x\n", channel_mask);
                status = 0;

                chId = 0;

                while( (channel_mask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL) )
                {
                    if(channel_mask & 0x1)
                    {
                        status = IssSensor_Stop((void*)pSenHndl, chId);
                        if(status < 0)
                        {
                            printf("Warning : Failed to stop sensor at channel Id %d \n", chId);
                        }
                        if( NULL !=g_pSenHndl[chId])
                        {
                            printf("Warning : sensor handle at channel Id %d is not NULL \n", chId);
                        }
                    }

                    chId++;

                    channel_mask = channel_mask >> 1U;

#if defined(SOC_AM62A) && defined(QNX)
                    if(strcmp(sensor_name, "OV2312-UB953_LI")==0)
                    {
                        channel_mask = channel_mask >> 1U;
                    }
#endif
                }
            }
            break;
        case IM_SENSOR_CMD_PWROFF:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_PWROFF \n");
            sensor_name = (char*)(cmd_param);
            memcpy(&channel_mask, (cmd_param + ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            issLogPrintf("IM_SENSOR_CMD_PWROFF : channel_mask = 0x%x \n", channel_mask);
            pSenHndl = IssSensor_getSensorHandleFromName(sensor_name);
            if (pSenHndl != NULL)
            {
                status = IssSensor_PowerOff((void*)pSenHndl, channel_mask);
            }
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start> 
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            The test framework does not support the configuration required to trigger this error scenario.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
                However, due to the stated rationale, this is not tested.
            <justification end> */
            else
            {
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            /* LDRA_JUSTIFY_END */
            IssSensor_CloseHandle();
            break;
        case IM_SENSOR_CMD_CTL:
            status = IssSensor_Control(NULL, 0, (void*)cmd_param, NULL);
            break;
        case IM_SENSOR_CMD_DETECT:
            {
                uint8_t channel_id = 0;
                int32_t probeStatus;
                uint16_t serializers_detected = 0;
                memcpy(&channel_mask, cmd_param, sizeof(uint32_t));

                probeStatus = IssSensor_detect_serializer(BOARD_CSI_INST_0, &serializers_detected);
                if(probeStatus != 0)
                {
                    issLogPrintf("IssSensor_detect_serializer returned 0x%.2x \n", probeStatus);
                }

                probeStatus = IssSensor_detect_serializer(BOARD_CSI_INST_1, &serializers_detected);
                if(probeStatus != 0)
                {
                    issLogPrintf("IssSensor_detect_serializer returned 0x%.2x \n", probeStatus);
                }

                if(gFusion2Det)
                {
                    probeStatus = IssSensor_detect_serializer(BOARD_CSI_INST_2, &serializers_detected);
                    if(probeStatus != 0)
                    {
                        issLogPrintf("IssSensor_detect_serializer returned 0x%.2x \n", probeStatus);
                    }
                }

                while(channel_mask > 0)
                {
                    if(channel_mask & 0x1)
                    {
                        int32_t detectedSensor;
                        int32_t tmp = (1<<channel_id);
                        if(tmp&serializers_detected)
                        {
                            if(g_detectedSensors[channel_id] == 0xFF)
                            {
                                /*Sensor has not been detected at this port yet. Try to detect */
                                detectedSensor = IssSensor_detect_sensor(channel_id);
                                if(detectedSensor < 0)
                                {
                                    issLogPrintf("IM_SENSOR_CMD_DETECT found serializer but no sensor at chId %d \n", chId);
                                    detectedSensor = 0xFF;
                                }
                                else
                                {
                                    issLogPrintf("IM_SENSOR_CMD_DETECT : Found sensor %d at port %d \n", detectedSensor, channel_id);
                                }
                                g_detectedSensors[channel_id] = (uint8_t)detectedSensor;
                                cmd_param[channel_id] = detectedSensor;
                            }
                            else
                            {
                                /* Sensor has previously been detected at this port. It might be getting used.
                                *  Trying to probe can overwrite the configuration
                                *  Return the previously detected ID instead of probing again */
                                issLogPrintf("IM_SENSOR_CMD_DETECT : Previously found sensor %d at port %d \n", g_detectedSensors[chId], chId);
                                cmd_param[channel_id] = (uint8_t)g_detectedSensors[channel_id];
                            }
                        }else
                        {
                            issLogPrintf("IM_SENSOR_CMD_DETECT No serializer found at port %d \n", channel_id);
                            cmd_param[channel_id] = 0xFF;
                        }
                    }
                    else
                    {
                        cmd_param[channel_id] = 0xFF;
                    }
                    channel_mask = channel_mask >> 1;
                    channel_id++;
                }
                status = 0;
            }
            break;
        default:
            issLogPrintf("ImageSensor_RemoteServiceHandler : Unsupported command : %d\n", cmd);
            status = -1;
    }

    return status;
}

int32_t IssSensor_DeInit()
{
    int32_t status = -1;
    status = appRemoteServiceUnRegister(IMAGE_SENSOR_REMOTE_SERVICE_NAME);
    if(status!=0)
    {
        printf(" REMOTE_SERVICE_SENSOR: ERROR: Unable to unregister remote service sensor handler\n");
    }
    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

/**
 *******************************************************************************
 *
 * \brief Get deserializer I2C address and channel/bus number
 * 
 * \param[in]   desHubInst    Deserializer hub instance. (Fusion1: 0, 1)
 *                              (Fusion2: 0, 1, 2)
 * \param[out]  i2cBus           Channel/bus number of desHubInstId
 * \param[out]  i2cAddr         I2C address of desHubInstId
 *
 * \ingroup group_vision_function_imaging_sensordrv
 *******************************************************************************
*/
static void IssSensor_getDeserializerI2cAddr(int8_t desHubInst,
    uint8_t *i2cBus, uint8_t *i2cAddr)
{
    #if defined (MCU_PLUS_SDK)
    if(gFusion2Det)
    {
        Board_fpdUb9702GetI2CAddr(i2cAddr, desHubInst);
    }
    else{
        Board_fpdUb960GetI2CAddr(i2cAddr, desHubInst);
    }
    #else
    if(gFusion2Det)
    {
        Board_fpdUb9702GetI2CAddr(desHubInst, NULL, i2cBus, i2cAddr);
    }
    else
    {
        Board_fpdU960GetI2CAddr(i2cBus, i2cAddr, desHubInst);
    }
    #endif
}

static int32_t checkForHandle(void* handle)
{
    int32_t found = -1;
    uint32_t cnt;

    /* Find a free entry in the sensor table */
    for (cnt = 0U; cnt < ISS_SENSORS_MAX_SENSOR_HANDLES; cnt ++)
    {
        if (handle == (void*)gIssSensorTable[cnt])
        {
            found = 0;
            break;
        }
    }

    return (found);
}

#if defined(SOC_AM62A)
static int32_t setup_io_expander(void)
{
    int32_t status = -1;
    int32_t ioexp_fd = -1;
    int32_t i = 0;
    I2C_Handle ioexp_I2cHandle = (I2C_Handle) &ioexp_fd;
    ioexp_fd = open("/dev/i2c1", O_RDWR);

    if(ioexp_fd == -1)
    {
        status = -1;
    }
    else
    {
        /* set i2c bus speed  - i2c_fast */
        uint32_t i2c_speed = 400000;
        status = devctl(ioexp_fd, DCMD_I2C_SET_BUS_SPEED, &i2c_speed, sizeof(i2c_speed), NULL);
        if (status != 0)
        {
            issLogPrintf("io-expander error: QNX I2C set bus speed failed\n");
        }

        uint8_t cmd[] = {0x08, 0x09, 0x0a,
                         0x06,  /* Set P23 of the io-expander as Low */
                         0x0e}; /* Configure P23 of the io-expander as output */

        uint8_t i2cData[] = {0x00, 0x00, 0x00,
                             0xf7,  /* Set P23 of the io-expander as Low */
                             0xe7}; /* Configure P23 of the io-expander as output */

        for (i = 0; i <5; i++)
        {
            status = Board_i2c8BitRegWr(ioexp_I2cHandle, IO_EXPANDER_ADDR, cmd[i], &i2cData[i], 1, 0);
            if (status != 0)
            {
                issLogPrintf("io-expander error: Reg Write Failed for regAddr %x\n", cmd[i]);
            }
        }
    }
    return status;
}
#endif

/**
 *******************************************************************************
 *
 * \brief Initialize I2C on bus with Fusion board
 * 
 * \param[in]  i2cInst  I2C instance/bus Fusion board is on
 *
 * \return  0 if success
 * \return -1 otherwise
 *
 *******************************************************************************
*/
static int32_t setupI2CInst(uint8_t i2cInst)
{
    int32_t status = -1;

#if defined(SOC_AM62A) && defined(QNX)
        /* open i2c driver */
        sensor_fd = open("/dev/i2c2", O_RDWR);
        if(sensor_fd == -1){
            status = -1;
        }else{
            /* set i2c bus speed - i2c_fast */
            uint32_t i2c_speed = 400000;
            status = devctl(sensor_fd, DCMD_I2C_SET_BUS_SPEED, &i2c_speed, sizeof(i2c_speed), NULL);
            if (status != 0)
            {
                issLogPrintf("%s: QNX i2c bus speed failed(%d)\n", __func__, status);
            }

            /* configure i2c switch to select CSI MIPI connector */
            int i2cData = 0x2;
            iov_t siov[2];
            i2c_send_t hdr;

            hdr.slave.addr = 0x71;
            hdr.slave.fmt = I2C_ADDRFMT_7BIT;
            hdr.len = 1;
            hdr.stop = 1;

            SETIOV(&siov[0], &hdr, sizeof(hdr));
            SETIOV(&siov[1], &i2cData, sizeof(i2cData));

            status = devctlv(sensor_fd, DCMD_I2C_SEND, 2, 0, siov, NULL, NULL);
            if (status != EOK)
            {
                issLogPrintf("%s: QNX i2c write failed(%d)\n", __func__, status);
            }
            gISS_Sensor_I2cHandle = (I2C_Handle)&sensor_fd;
        }
        status = setup_io_expander();
        if (status != EOK)
        {
            issLogPrintf("%s: IO expander config failed(%d)\n", __func__, status);
        }
#else
    #if !defined(MCU_PLUS_SDK)
    I2C_Params i2cParams;

    /* Initializes the I2C Parameters */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    /* Configures the I2C instance with the passed parameters*/
    if(NULL == gISS_Sensor_I2cHandle)
    {
       gISS_Sensor_I2cHandle = I2C_open(i2cInst, &i2cParams);
    }
    #else
    gISS_Sensor_I2cHandle = I2C_getHandle(CONFIG_I2C2);
    #endif
#endif

    if(gISS_Sensor_I2cHandle == NULL)
    {
        printf("[setupI2CInst]: Failed to initialze I2C bus %d!!!\n", i2cInst);
        status = -1;
    }
    else
    {
        status = 0;
    }

    return status;
}

static int32_t Serialzier_WriteReg(uint8_t slaveI2cAddr,
                                   uint16_t regAddr,
                                   uint8_t regVal)
{
    int32_t status = -1;
    status = Board_i2c8BitRegWr(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr,
                                &regVal, 1U, SENSOR_I2C_TIMEOUT);
    if(0 != status)
    {
        printf(
            "Error writing 0x%.2x to serializer (slaveAddr: 0x%.2x) register 0x%.2x!\n",
            regVal, slaveI2cAddr, regAddr);
    }
    return status;
}

static int32_t Serialzier_ReadReg(uint8_t slaveI2cAddr,
                                uint16_t regAddr,
                                uint8_t *regVal)
{
    int32_t status = -1;
    status = Board_i2c8BitRegRd(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr,
                                regVal, 1U, SENSOR_I2C_TIMEOUT);
    if(0 != status)
    {
        printf(
            "Error reading from serializer(0x%.2x) register 0x%.2x!\n",
            slaveI2cAddr, regAddr);
    }
    return status;
}

static int32_t Deserializer_ReadReg(uint8_t slaveI2cAddr,
                                    uint16_t regAddr,
                                    uint8_t *regVal)
{
    int32_t status = -1;
    status = Board_i2c8BitRegRd(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr,
                                regVal, 1U, SENSOR_I2C_TIMEOUT);
    if(0 != status)
    {
        printf(
            "Error reading from de-serializer(0x%.2x) register 0x%.2x!\n",
            slaveI2cAddr, regAddr);
    }
    return status;
}

static int32_t Deserializer_WriteReg(uint8_t   slaveI2cAddr,
                                     uint16_t  regAddr,
                                     uint8_t   regVal)
{
    int32_t status = -1;
    status = Board_i2c8BitRegWr(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr,
                                &regVal, 1U, SENSOR_I2C_TIMEOUT);
    if(0 != status)
    {
        printf(
            "Error writing 0x%.2x to de-serializer(0x%.2x) register 0x%.2x!\n",
            regVal, slaveI2cAddr, regAddr);
    }
    return status;
}

/* Detects which ports of a de-serializer hub instance have a connected serializer
by checking LOCK_STS and PORT_PASS registers for each de-serializer port.
Returns a 4-bit mask - 0 = Ser Not Detected, 1 = Ser Detected */
static int32_t IssSensor_detect_serializer(int8_t desHubInstId, uint16_t *cameras_detected)
{
    uint8_t mask = 0x0;
    uint8_t pageSelectOrig = 0x0;
    uint8_t regVal = 0x0;
    uint8_t found = 0x0;
    int32_t status = -1;
    uint8_t  desI2cInstId = 0;
    uint8_t  desI2cAddr;

    uint8_t portSelCfg[] = {DES_PORT_SEL_0, DES_PORT_SEL_1,
                            DES_PORT_SEL_2, DES_PORT_SEL_3};

    IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cInstId,
        &desI2cAddr);

    status = Deserializer_ReadReg(desI2cAddr, 0x4C, &pageSelectOrig);
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
    <justification end> */
    if(status != 0)
    /* LDRA_JUSTIFY_END */
    {
        return status;
    }
    
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications CAN reach this portion.
                No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
    Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
            However, due to the stated rationale, this is not tested.
    <justification end> */
    for(int i = 0; i < (sizeof(portSelCfg) / sizeof(portSelCfg[0])); i++)
    {
        /* Select port on deserializer */
        status = Deserializer_WriteReg(desI2cAddr, 0x4C, portSelCfg[i]);
        if(status != 0)
        {
            return status;
        }

        /* Read RX_PORT_STS1(0x4D) */
        status = Deserializer_ReadReg(desI2cAddr, 0x4D, &regVal);
        if(status != 0)
        {
            return status;
        }

        /* Extract PORT_PASS(0x4D[1]) and LOCK_STS(0x4D[0]) */
        found = regVal & 0x3;
        if(found == 0x3)
        {
            mask |=  (portSelCfg[i] & 0x0F);
        }
    }
    /* LDRA_JUSTIFY_END */

    status = Deserializer_WriteReg(desI2cAddr, 0x4C, pageSelectOrig);

    *cameras_detected |= (mask << (desHubInstId * 4U));

    return status;
}

/* Probes all the sensors in gIssSensorTable */
/* If a sensor is detected, the index of the sensor in gIssSensorTable is copied to sensor_id_found */
static int32_t IssSensor_detect_sensor(uint8_t chId)
{
    int32_t probeStatus = -1;
    int32_t i = 0;
    IssSensors_Handle *pSenHndl;
    int32_t sensor_id_found = -1;

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The test framework does not support the configuration required to trigger this error scenario.
    Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
    This behaviour is part of the application design. An error print statement can be added in a future release if required.
    <justification end> */
    while ((sensor_id_found < 0) && (i < ISS_SENSORS_MAX_SENSOR_HANDLES))
    /* LDRA_JUSTIFY_END */
    {
        pSenHndl = gIssSensorTable[i];
        if(NULL !=pSenHndl)
        {
            probeStatus = pSenHndl->sensorFxns->probe(chId, pSenHndl);

            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start> 
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            The test framework does not support the configuration required to trigger this error scenario.
            Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
            This behaviour is part of the application design. An error print statement can be added in a future release if required.
            <justification end> */
            if(probeStatus == 0 )
            {
                /*Sensor found at index i*/
                issLogPrintf("Found sensor %s at port # %d\n", pSenHndl->createPrms->name, chId);
                sensor_id_found = i;
            }
            /* LDRA_JUSTIFY_END */
        }
        i++;
    }

    return sensor_id_found;
}

static void IssSensor_GetPortSelectRegisterValue(uint32_t chId, uint8_t *portSel, uint8_t *portSelInd)
{
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    doubt
    Rationale: The component level negative test framework and test applications CAN reach this portion.
                No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
    Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
            However, due to the stated rationale, this is not tested.
    <justification end> */
    switch(chId)
    {
    /* LDRA_JUSTIFY_END */
        case 0:
        case 4:
        case 8:
            *portSel = DES_PORT_SEL_0;
            *portSelInd = 0x04;
            break;
        case 1:
        case 5:
        case 9:
            *portSel = DES_PORT_SEL_1;
            *portSelInd = 0x08;
            break;
        case 2:
        case 6:
        case 10:
            *portSel = DES_PORT_SEL_2;
            *portSelInd = 0x0C;
            break;
        case 3:
        case 7:
        case 11:
            *portSel = DES_PORT_SEL_3;
            *portSelInd = 0x10;
            break;
        case 0xFF:
            /* Read port 0; Write all ports (broadcast mode) */
            *portSel = 0x0F;

            /*  Simultaneous Writes To FPD-Link RX Ports 0-3 Analog (FPD_RX_PORT_ANALOG) Registers */
            *portSelInd = 0x18;
            break;
        default:
            printf("Error : Invalid channel ID 0x%x \n", chId);
            return;
    }
}

static int32_t IssSensor_DesSelPort(uint8_t chId)
{
    /* write code to select port based on chID */
    int32_t status = -1;
    
    uint8_t  portSel = 0;
    uint8_t  portSelInd = 0;
    int8_t desHubInstId = -1;

    static I2cParams portSelScript[] = {
        {0x4C, 0x00, 0x00},  // read/write port registers
        {0xB0, 0x00, 0x00},   // read/write port indirect registers

        {0xFFFF,0x00, 0x00} /* End script */
    };


    IssSensor_GetPortSelectRegisterValue(chId, &portSel, &portSelInd);
    if(portSel == 0)
    {
        printf("IssSensor_DesSelPort : IssSensor_GetPortSelectRegisterValue incorrect channel ID = %d\n", chId);
        status = -1;
        return status;
    }

    /* Set read/write for port registers */
    portSelScript[0].nRegValue = portSel;

    /* Set read/write for port indirect registers */
    portSelScript[1].nRegValue = portSelInd;

    desHubInstId = getDeserializerInstIdFromChId(chId);
    IssSensor_cfgDesScript(portSelScript, desHubInstId);

    // printf("Port select: 0x4C = 0x%02x \t 0xB0 = 0x%02x for chID = %d\n", portSel, portSelInd, chId);

    return status;
}

#if defined(B7_IMPLEMENTATION)
static int32_t configure_serializer(uint8_t chId, IssSensor_ModuleParams moduleInfo, uint8_t serAddr)
{
    int32_t status = -1;

    static I2cParams ub971InitScript[] = {
        {0x4B, 0x02, 0x00}, /* disable BC alternate mode auto detect */
        {0x49, 0x06, 0x00}, /* decrease link detect timer on 971 BC-RX */
        {0x0A, 0x12, 0x00}, /* Speed up I2C bus watchdog timer to ~50us */
        {0xFFFF,0x00, 0x00} /* End script */
    };

    static I2cParams ub953InitScript[] = {
        {0x0A, 0x12, 0x00},  /* Speed up I2C bus watchdog timer to ~50us */
        {0xFFFF,0x00, 0x00} /* End script */
    };

    
    if(moduleInfo.fpdLink == IMAGE_SERDES_FPD_LINK_3)
    {
        status = ub953_cfgScript(0, serAddr, ub953InitScript);
        if(status != 0)
        {
            printf("configure_serializer : [FPD3] Failed to configure serializer for port %d\n", chId);
        }
    }
    else if(moduleInfo.fpdLink == IMAGE_SERDES_FPD_LINK_4)
    {
        if(gFusion2Det == vx_true_e)
        {
            status = ub953_cfgScript(0, serAddr, ub971InitScript);
            if(status != 0)
            {
                printf("configure_serializer : [FPD4] Failed to configure serializer for port %d\n", chId);
            }
        }
    }
    else
    {
        printf("configure_serializer: Incorrect FPD version: %d\n", moduleInfo.fpdLink);
    }

    return status;
}

static int32_t configure_deserializer_port(uint8_t  desI2cAddr, IMAGE_SERDES_FPD_LINK fpdLink, uint32_t chId)
{
    int32_t status = -1;

    chId = chId % 4;

    I2cParams ub9702_ub953portModeScript[] = {
        {0xE4, 0x02, 0x00},// Set FPD functional mode to FPD3 Async CSI Mode
        {0x58, 0x46, 0x00FFFFFF},// BC_FREQ_SELECT=(PLL_FREQ/3200) Mbps  
        {0xB1, 0xA8, 0x00},// write to indirect registers
        {0xB2, 0x80, 0x00},// set aeq_lock_mode = 1

        /* FPD4 BC CONFIG START */
        {0xB1, 0x04, 0x00},
        {0xB2, 0x40, 0x00},// remove HiZ of NMOS drv of spare driver
        {0xB1, 0x1B, 0x00},
        {0xB2, 0x28, 0x00},// remove HiZ of PMOS drv of spare driver & disable 1 CMR ladder
        {0xB1, 0x0D, 0x00},
        {0xB2, 0x7F, 0x01},// enable the FPD3 spare driver time.sleep(.1)
        /* FPD4 BC CONFIG END */

        {0x0C, 0x1 << chId, 0x00FFFFFF},// readback already enabled ports
        {0xA0, 0x10 | chId, 0x00FFFFFF},// Set VC channel

        {0xFFFF,0x00, 0x00} /* End script */
    };

    I2cParams ub9702_ub971portModeScript[] = {
        {0xE4, 0x00, 0x00}, // Set FPD functional mode to FPD4 Sync Mode
        {0x58, 0x06, 0x00FFFFFF}, /* Masked write */
        
        /* FPD4 BC CONFIG START */
        {0xB1, 0x04, 0x00},
        {0xB2, 0x00, 0x00},// set FPD PBC drv into FPD IV mode
        {0xB1, 0x1B, 0x00},
        {0xB2, 0x00, 0x00},// set FPD PBC drv into FPD IV mode
        /* FPD4 BC CONFIG END */

        {0xB1, 0x21, 0x00},
        {0xB2, 0x2F, 0x00},// set 960 AEQ timer to 400us/step
        {0x3C, 0x0F, 0x00},// disable lock lost feature

        {0x0C, 0x1 << chId, 0x00FFFFFF}, /* Masked write */
        {0xA0, 0x10 | chId, 0x00FFFFFF}, /* Set VC channel */
        {0x3C, 0x9F, 0x00}, // enable lock lost feature and enable Exclusive Forwarding
        {0xC7, 0x10, 0x00}, // CSI Transmit Port2 Enable
        
        {0xFFFF,0x00, 0x00} /* End script */
    };

    IssSensor_DesSelPort(chId);
    
    if(fpdLink == IMAGE_SERDES_FPD_LINK_3)
    {
        issLogPrintf("\t Configuring port %d to FPD3 Mode\n ", chId);
        status = IssSensor_cfgDesScriptCustom(desI2cAddr, ub9702_ub953portModeScript, 0);
    }
    else if(fpdLink == IMAGE_SERDES_FPD_LINK_4)
    {
        issLogPrintf("\t Configuring port %d to FPD4 Mode\n ", chId);

        status = IssSensor_cfgDesScriptCustom(desI2cAddr, ub9702_ub971portModeScript, 0);
    }
    else
    {
        printf(" error %s: %d\n", __func__, __LINE__);
    }

    return status;
}

static int32_t disable_deserializer_port(uint8_t  desI2cAddr, IMAGE_SERDES_FPD_LINK fpdLink, uint32_t chId)
{
    int32_t status = -1;

    uint8_t rxPortEn[4] = {0xFE, 0xFD, 0xFB, 0xF7};

    static I2cParams ub9702portDisableScript[] = {
        {0x0C, 0x00, 0x00FFFFFF},  // reg_0x0C_mask = {0: 0xFE, 1: 0xFD, 2: 0xFB, 3: 0xF7} (read_0x0C & reg_0x0C_mask[1])
        {0x87, 0x1B, 0x00},  // disable FPD Rx and FPD BC CMR
        {0x58, 0xEF, 0x00FFFFFF},  // reg_0x58 & 0xEF  disable FPD BC Tx
        {0x80, 0x15, 0x00}, // disable PI
        {0xB1, 0x4D, 0x00}, // 
        {0xB2, 0x03, 0x00}, // disable AEQ
        {0xB1, 0x85, 0x00}, // 
        {0xB2, 0x09, 0x00}, // PI disabled and oDAC disabled
        {0xB1, 0x34, 0x00}, // 
        {0xB2, 0x20, 0x00}, // AEQ configured for disabled link
        {0xB1, 0x79, 0x00}, // 
        {0xB2, 0x45, 0x00}, // disable AEQ clock and DFE 
        {0xB1, 0xA5, 0x00}, // 
        {0xB2, 0x82, 0x00}, // pwrdown FPD3 CDR

        {0xFFFF,0x00, 0x00} /* End script */
    };

    ub9702portDisableScript[0].nRegValue = rxPortEn[chId % 4];

    // printf(" \t\tdesI2cAddr: 0x%02x rxPortEn[chId %% 4] = 0x%02x\n", desI2cAddr, rxPortEn[chId % 4]);

    status = IssSensor_cfgDesScriptCustom(desI2cAddr, ub9702portDisableScript, 1);

    return status;
}

static int32_t checkerlockrecover_des(uint8_t  desI2cAddr, uint32_t chId)
{
    int32_t status = -1;

    uint8_t rx_AEQ_limit = 0x10; // Assumption that max AEQ should be under 16 
    uint8_t prev_AEQ = 0xFF;

    uint8_t rx_AEQ, rx_lock;

    for(int retry = 0; retry < ISS_SENSORS_MAX_SUPPORTED_SENSOR; retry++)
    {
        /* lock status */
        Deserializer_ReadReg(desI2cAddr, 0x4D, &rx_lock);
        rx_lock = rx_lock & 0x01;

        /* read/write indirect registers */
        Deserializer_WriteReg(desI2cAddr, 0xB1, 0x2C);

        Deserializer_ReadReg(desI2cAddr, 0xB2, &rx_AEQ);

        if(rx_lock == 0)
        {
            static I2cParams ub9702AeqPart1[] = {
                {0xE4, 0x10, 0x01}, // time.sleep(0.0001)
                {0xE4, 0x00, 0x00}, // Toggle Recovery Bit
                
                {0xB1, 0x28, 0x00}, // Restart AEQ by changing max to 0 --> 0x23
                {0xB2, 0x00, 0x02}, // time.sleep(0.02)
                {0xB2, 0x23, 0x02}, // AEQ Restart

                {0xFFFF,0x00, 0x00} /* End script */
            };

            IssSensor_cfgDesScriptCustom(desI2cAddr, ub9702AeqPart1, 0);

            printf("Port not locked at 1st check recovery loop\n\n");
            printf("Port Lock = 0x%02x Retry=%d\n\n", rx_lock, retry);

            if(retry==3)
            {
                printf("!!ERROR!! Max number of retry for FPD-Link IV lock!!ERROR!!\n\n");
                printf("Please check connections and hardware\n\n");

                return -1;
            }
            else if( (rx_lock == 1) && (rx_AEQ >= rx_AEQ_limit) )
            {
                if(rx_AEQ != prev_AEQ)
                {
                    static I2cParams ub9702AeqPart2[] = {
                        {0xE4, 0x10, 0x01}, /* time.sleep(0.0001) */
                        {0xE4, 0x00, 0x00}, // Toggle Recovery Bit
                        {0xB1, 0x28, 0x00}, // Restart AEQ by changing max to 0 --> 0x23
                        {0xB2, 0x00, 0x02}, /* time.sleep(0.02) */
                        {0xB2, 0x23, 0x00}, // AEQ Restart time.sleep(0.02)

                        {0xFFFF,0x00, 0x00} /* End script */
                    };

                    IssSensor_cfgDesScriptCustom(desI2cAddr, ub9702AeqPart2, 0);
                    printf("Port high AEQ at 1st check recovery loop\n\n");
                    printf("Port Lock = 0x%02x RX_AEQ=02%02x Retry=%d\n\n", rx_lock, rx_AEQ, retry);
                    prev_AEQ = rx_AEQ;

                    if (retry==3)
                    {
                        printf("!!ERROR!! Max number of retry for FPD-Link IV unstable AEQ !!ERROR!!\n\n");
                        printf("Please check connections and hardware\n\n");
                        return -1;
                    }
                }
                else
                {
                    printf("Lossy Cable Detected Lock =0x%02x,   RX_AEQ (0x%02x) > RX_AEQ_Limit(0x%02x),   Retry=%d \n\n", rx_lock, rx_AEQ, rx_AEQ_limit, retry);
                    printf("// Port Will continue with initiation sequence but high AEQ \n\n");
                    break;
                }

            }
            else
            {
                printf("Port FPD-Link IV Locked and AEQ Normal before setting AEQ window in en_AEQ_LMS \n\n");
            }
        }
    }

    return status;
}

static int32_t deserializer_aeq_optimization(uint8_t  desI2cAddr, uint32_t chId)
{
    int32_t status = -1;

    uint8_t read_aeq_init = 0x00;

    I2cParams UB9702aeqOptimizationScript[] = {
        {0xB1, 0x27, 0x00},  // read/write indirect registers
        {0xB2, 0x00, 0x00},  // read/write indirect registers
        {0xB1, 0x28, 0x00},  // Set AEQ Min
        {0xB2, 0x00, 0x00},  // Set AEQ Max
        {0xB1, 0x2B, 0x00},  // Set AEQ offset
        {0xB2, 0x00, 0x00},  //Set Offset to 0
        {0xB1, 0x9E, 0x00},  // read/write indirect registers
        {0xB2, 0x00, 0x00},  // enable AEQ tap2
        {0xB1, 0xF1, 0x00},  // read/write indirect registers
        {0xB2, 0x00, 0x00},  // Set VGA Gain 1 Gain 2 override to 0
        {0xB1, 0x77, 0x00},  // read/write indirect registers
        {0xB2, 0x80, 0x00},  // Set VGA Initial Sweep Gain to 0
        {0xB1, 0x74, 0x00},  // read/write indirect registers
        {0xB2, 0x00, 0x00},  // Set VGA_Adapt (VGA Gain) override to 0 (thermometer encoded)
        {0xB1, 0x2E, 0x00},  // read/write indirect registers
        {0xB2, 0x40, 0x00},  // Enable VGA_SWEEP
        {0xB1, 0xF0, 0x00},  // read/write indirect registers
        {0xB2, 0x00, 0x00},  // Disable VGA_SWEEP_GAIN_OV, disable VGA_TUNE_OV

        {0xB1, 0x72, 0x00},  // read/write indirect registers
        {0xB2, 0x2B, 0x00},  // Set VGA HIGH Threshold to 43
        {0xB1, 0x73, 0x00},  // read/write indirect registers
        {0xB2, 0x12, 0x00},  // Set VGA LOW Threshold to 18
        {0xB1, 0x87, 0x00},  // read/write indirect registers
        {0xB2, 0x20, 0x00},  // set vga_sweep_th to 32
        {0xB1, 0x21, 0x00},  // read/write indirect registers
        {0xB2, 0xEF, 0x00},  // set AEQ timer to 400us/step and parity threshold to 7

        // {0x4C, 0x01, 0x00},  // read/write port0 registers
        // {0xB0, 0x04, 0x00},  // read/write port0 indirect registers

        {0xD4,0x10, 0x00}, // Hold State Machine in reset
        {0x01,0x21, 0x01},  // Soft reset and release GPIO hold time.sleep(0.01)
        {0xD4,0x00, 0x02}, // Release SM in reset time.sleep(0.015) //wait time for stable lock

        // {0x4C, 0x01, 0x00},  // read/write port0 registers
        // {0xB0, 0x04, 0x00},  // read/write port0 indirect registers

        {0xB1, 0x90, 0x00},  // read/write indirect registers
        {0xB2, 0x40, 0x00},  // enable DFE LMS
        {0xB1, 0x71, 0x00},  // read/write indirect registers
        {0xB2, 0x20, 0x01},  // Disable VGA Gain1 override time.sleep(0.0001)
        {0xB2, 0x00, 0x00},  // Disable VGA Gain2 override

        {0xFFFF,0x00, 0x00} /* End script */
    };

    /* read/write indirect registers */
    Deserializer_WriteReg(desI2cAddr, 0xB1, 0x2C);

    /* initial FPD-Link IV AEQ value */
    Deserializer_ReadReg(desI2cAddr, 0xB2, &read_aeq_init);

    /* read/write indirect registers */
    UB9702aeqOptimizationScript[1].nRegValue = read_aeq_init;

    /* Set AEQ Max */
    UB9702aeqOptimizationScript[3].nRegValue = read_aeq_init+1;

    IssSensor_cfgDesScriptCustom(desI2cAddr, UB9702aeqOptimizationScript, 0);

    status = 0;
    return status;
}

static int32_t ub953_temp_ramp(uint8_t  desI2cAddr, uint8_t  serI2cAddr, uint32_t chId)
{
    int32_t status = -1;

    uint8_t read_aeq_init = 0x00;
    uint8_t reg_0x58 = 0x00;
    uint8_t temp_code_95x = 0x00;
    uint8_t vdac_ori = 0x00;
    uint8_t vdac = 0x00;
    uint8_t vco_tweak = 0x00;

    Deserializer_ReadReg(desI2cAddr, 0x58, &reg_0x58);
    reg_0x58 = reg_0x58 | 0x40; // Enable I2C Passthrough
    Deserializer_WriteReg(desI2cAddr, 0x58, reg_0x58);

    Deserializer_ReadReg(desI2cAddr, 0x53, &temp_code_95x);

    Serialzier_WriteReg(serI2cAddr, 0xB0, 0x04);
    Serialzier_WriteReg(serI2cAddr, 0xB1, 0x4B);
    Serialzier_ReadReg(serI2cAddr, 0xB2, &vdac_ori);
    vdac = vdac_ori | 0x20;

    Serialzier_WriteReg(serI2cAddr, 0xB1, 0x4C);
    Serialzier_ReadReg(serI2cAddr, 0xB2, &vco_tweak);
    vco_tweak = (vco_tweak & 0x8F) | 0x70;
    Deserializer_WriteReg(serI2cAddr,0xB2, vco_tweak);

    Serialzier_WriteReg(serI2cAddr, 0xB1, 0x4B);

    switch (temp_code_95x)
    {
        case 0:
        case 1:
            vdac = vdac - 1;
            break;
        case 2:
        case 3:
            /* vdac does not change */
            break;
        case 4:
        case 5:
        case 6:
            vdac = vdac + 1;
            break;
        case 7:
            vdac = vdac + 3;
            break;
        default:
            printf(" Warning %s: %d\n", __func__, __LINE__);
            break;
    }

    Serialzier_WriteReg(serI2cAddr, 0xB2, vdac);

    reg_0x58 = reg_0x58 | 0x20; // Enable all auto ACK I2C Passthrough
    Deserializer_WriteReg(desI2cAddr, 0x58, reg_0x58);

    Serialzier_WriteReg(serI2cAddr, 0x01, 0x01); // Soft Rest to apply PLL updates
    appLogWaitMsecs(20); /* wait 20 ms */

    reg_0x58 = reg_0x58 & 0x1F; // Disable auto ACK and I2C passthrough
    Deserializer_WriteReg(desI2cAddr, 0x58, reg_0x58);

    return status;
}

static int32_t deserializer_enable_csitx(uint8_t  desI2cAddr)
{
    int32_t status = -1;

    uint8_t regVal = 0x00;

    I2cParams UB9702csiTxEnableScript[] = {
        /* CSI-TX Port Config */
        {0x32, 0x03, 0x20}, /* Read CSI-TX Port 0, Write CSI-TX Ports 0 and 1 */
        {0x20, 0xF0, 0x20}, /* Disable FPD-Link RX forwarding */
        {0x33, 0x03, 0x20}, /* 4 CSI-2 Lanes; Continous clock mode, CSI-2 output enabled */
        {0x20, 0x00, 0x10}, /* Enable FPD-Link RX forwarding (all ports) to CSI-TX port 0 */

        {0xFFFF,0x00, 0x00} /* End script */
    };

    
    status = IssSensor_cfgDesScriptCustom(desI2cAddr, UB9702csiTxEnableScript, 0);
    
    Deserializer_ReadReg(desI2cAddr, 0x20, &regVal);

    regVal = 0x00;
    Deserializer_ReadReg(desI2cAddr, 0x33, &regVal);

    return status;
}

static int32_t deserializer_setBackChannel50mbps(uint8_t  desI2cAddr)
{
    int32_t status = -1;

    I2cParams UB9702setBackChannel50mbpsScript[] = {
        /* Set back channel to 50Mbps */
        {0xB0, 0x16, 0x20}, // PLL_CTRL indirect registers
        {0xB1, 0x8A, 0x20},
        {0xB2, 0x00, 0x20}, // Set RX PLL MASH divider order for 8GHz
        {0xB1, 0x87, 0x20},
        {0xB2, 0xA0, 0x20}, // Set RX PLL NCOUNT setting for 8GHz
        {0xB1, 0x83, 0x20},
        {0xB2, 0x00, 0x20}, // Set RX PLL MASH numerator setting [23:16] for 8GHz
        {0xB1, 0x84, 0x20},
        {0xB2, 0x00, 0x20}, // Set RX PLL MASH numerator setting [15:8] for 8GHz
        {0xB1, 0x85, 0x20},
        {0xB2, 0x00, 0x20}, // Set RX PLL MASH numerator setting [7:0] for 8GHz

        {0xFFFF,0x00, 0x00} /* End script */
    };

    status = IssSensor_cfgDesScriptCustom(desI2cAddr, UB9702setBackChannel50mbpsScript, 0);

    return status;
}

static int32_t deserializer_setCsiSpeed(uint8_t  desI2cAddr, IMAGE_SERDES_FPD_LINK fpdLink)
{
    int32_t status = -1;

    I2cParams UB9702setCsiSpeed1500mbpsScript[] = {
        /* Set CSI Transmitter to 1500Mbps */
        {0x1F, 0x00, 0x20},
        {0xC9, 0x0F, 0x20},
        {0xB0, 0x1C, 0x20}, // CSI-2_ANALOG indirect registers
        {0xB1, 0x92, 0x20},
        {0xB2, 0x80, 0x20},

        {0xFFFF,0x00, 0x00} /* End script */
    };

    I2cParams UB9702setCsiSpeed2500mbpsScript[] = {
        /* Set CSI Transmitter to 2500Mbps */
        {0x1F, 0x10, 0x20},
        {0xC9, 0x19, 0x20},
        {0xB0, 0x1C, 0x20}, // CSI-2_ANALOG indirect registers
        {0xB1, 0x92, 0x20},
        {0xB2, 0x80, 0x20},

        {0xFFFF,0x00, 0x00} /* End script */
    };

    if (fpdLink == IMAGE_SERDES_FPD_LINK_3)
    {
        printf(" setting CSI speed to 1500Mbps\n");
        status = IssSensor_cfgDesScriptCustom(desI2cAddr, UB9702setCsiSpeed1500mbpsScript, 0);
    }
    else
    {
        printf(" setting CSI speed to 2500Mbps\n");
        status = IssSensor_cfgDesScriptCustom(desI2cAddr, UB9702setCsiSpeed2500mbpsScript, 0);
    }

    return status;
}

/* chMask will be bit mask of cameras being programmed */
static int32_t IssSensor_DeserializerInit_B7()
{
    int32_t status = -1;

    uint8_t  desI2cBusNum = 0;
    uint8_t  desI2cAddr;
    int8_t desHubInstId = -1;
    uint8_t  regVal;
    uint32_t chId = 0;
    IssSensors_Handle *pSenHndl;
    IssSensor_CreateParams *sensorInfo;
    IssSensor_ModuleParams *moduleInfo;

    uint8_t portSel[4] = {DES_PORT_SEL_0, DES_PORT_SEL_1, DES_PORT_SEL_2, 
        DES_PORT_SEL_3};

    uint8_t i = 0;
    uint8_t tmpRd = 0x00;
    uint8_t reg_0x4D = 0x00;

    /* g_pSenHndl should have already been populated in IM_SENSOR_CMD_PWRON
    (this function gets called in config which is after power on) */
    chId = 0;
    while(chId < ISS_SENSORS_MAX_SUPPORTED_SENSOR)
    {
        desHubInstId = getDeserializerInstIdFromChId(chId);
        IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum, &desI2cAddr);

        /* Select port on deserializer (direct and indirect registers) */
        IssSensor_DesSelPort(chId);

        if(g_pSenHndl[chId] != NULL)
        {
            pSenHndl = g_pSenHndl[chId];
            assert(NULL != pSenHndl);

            sensorInfo = pSenHndl->createPrms;
            assert(NULL != sensorInfo);

            moduleInfo = &(sensorInfo->moduleInfo);
            assert(NULL != moduleInfo);

            /* --------------- BEGIN 971 serializer programming --------------- */
            /* Configure serializer alias with freeze and auto ACK in deserializer */
            Deserializer_WriteReg(desI2cAddr, 0x5B, moduleInfo->serI2cAddr+1);
            Deserializer_WriteReg(desI2cAddr, 0x5C, (sensorInfo->i2cAddrSer[chId] << 1)+1);

            Deserializer_ReadReg(desI2cAddr, 0x58, &regVal);
            
            /* Enable I2C Passthrough with auto ACK */
            regVal = 0x00;
            Deserializer_ReadReg(desI2cAddr, 0x58, &regVal);
            regVal = regVal | 0x60;
            Deserializer_WriteReg(desI2cAddr, 0x58, regVal);

            configure_serializer(chId, *moduleInfo, sensorInfo->i2cAddrSer[chId]);
            
            /* Disable auto ACK and I2C passthrough */
            regVal = regVal & 0x1F;
            Deserializer_WriteReg(desI2cAddr, 0x58, regVal);
            /* --------------- END 971 serializer programming --------------- */
            
            if(moduleInfo->fpdLink == IMAGE_SERDES_FPD_LINK_4)
            {
                Deserializer_WriteReg(desI2cAddr, 0xD4, 0x10); /* Hold State Machine in reset */
                Deserializer_WriteReg(desI2cAddr, 0xB1, 0x28);
                Deserializer_WriteReg(desI2cAddr, 0xB2, 0x00); /* set AEQ max to 0 */
            }

            configure_deserializer_port(desI2cAddr, moduleInfo->fpdLink, chId);
        }
        else
        {
            issLogPrintf("\t Disabling port %d\n", chId);
            disable_deserializer_port(desI2cAddr, moduleInfo->fpdLink, chId);
        }
        chId++;
    }

    for(int hubInst = 0; hubInst < gDeserializerHubs; hubInst++)
    {
        desI2cAddr = 0x00;
        IssSensor_getDeserializerI2cAddr(hubInst, &desI2cBusNum, &desI2cAddr);

        Deserializer_WriteReg(desI2cAddr, 0x01, 0x21); /* Soft reset and release GPIO hold */
        appLogWaitMsecs(10); /* delay 10ms */
        if(moduleInfo->fpdLink == IMAGE_SERDES_FPD_LINK_3)
        {
            appLogWaitMsecs(105); /* delay 105ms for stable lock */
        }
    }

    chId = 0;
    while(chId < ISS_SENSORS_MAX_SUPPORTED_SENSOR)
    {
        desHubInstId = getDeserializerInstIdFromChId(chId);
        IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum, &desI2cAddr);

        /* Select port on deserializer (direct and indirect registers) */
        IssSensor_DesSelPort(chId);

        if(g_pSenHndl[chId] != NULL)
        {
            pSenHndl = g_pSenHndl[chId];
            assert(NULL != pSenHndl);

            sensorInfo = pSenHndl->createPrms;
            assert(NULL != sensorInfo);

            moduleInfo = &(sensorInfo->moduleInfo);
            assert(NULL != moduleInfo);

            if(moduleInfo->fpdLink == IMAGE_SERDES_FPD_LINK_4)
            {
                Deserializer_WriteReg(desI2cAddr, 0xD4, 0x00); /* Release State Machine in reset */
                appLogWaitMsecs(5); /* delay 5ms */

                Deserializer_WriteReg(desI2cAddr, 0xB1, 0x28); /* AEQ restart */
                Deserializer_WriteReg(desI2cAddr, 0xB2, 0x23); /* AEQ restart */
                appLogWaitMsecs(20); /* delay 20ms for stable lock FPD-Link IV */

                checkerlockrecover_des(desI2cAddr, chId);
            }
        }
        chId++;
    }

    chId = 0;
    while(chId < ISS_SENSORS_MAX_SUPPORTED_SENSOR)
    {
        desHubInstId = getDeserializerInstIdFromChId(chId);
        IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum, &desI2cAddr);

        /* Select port on deserializer (direct and indirect registers) */
        IssSensor_DesSelPort(chId);

        if(g_pSenHndl[chId] != NULL)
        {
            pSenHndl = g_pSenHndl[chId];
            assert(NULL != pSenHndl);

            sensorInfo = pSenHndl->createPrms;
            assert(NULL != sensorInfo);

            moduleInfo = &(sensorInfo->moduleInfo);
            assert(NULL != moduleInfo);

            if(moduleInfo->fpdLink == IMAGE_SERDES_FPD_LINK_4)
            {
                deserializer_aeq_optimization(desI2cAddr, chId);
            }
            else if(moduleInfo->fpdLink == IMAGE_SERDES_FPD_LINK_3)
            {
                ub953_temp_ramp(desI2cAddr, sensorInfo->i2cAddrSer[chId], chId);
            }
            else
            {
                printf(" error %s: %d\n", __func__, __LINE__);
            }
        }
        chId++;
    }

    chId = 0;
    while(chId < ISS_SENSORS_MAX_SUPPORTED_SENSOR)
    {
        desHubInstId = getDeserializerInstIdFromChId(chId);
        IssSensor_getDeserializerI2cAddr(desHubInstId, &desI2cBusNum, &desI2cAddr);

        /* Select port on deserializer (direct and indirect registers) */
        IssSensor_DesSelPort(chId);

        if(g_pSenHndl[chId] != NULL)
        {
            pSenHndl = g_pSenHndl[chId];
            assert(NULL != pSenHndl);

            sensorInfo = pSenHndl->createPrms;
            assert(NULL != sensorInfo);

            moduleInfo = &(sensorInfo->moduleInfo);
            assert(NULL != moduleInfo);

            /* Configure serializer alias without freeze and remove auto ACK in deserializer */
            Deserializer_WriteReg(desI2cAddr, 0x5B, moduleInfo->serI2cAddr);
            Deserializer_WriteReg(desI2cAddr, 0x5C, (sensorInfo->i2cAddrSer[chId] << 1));

            /* Enable I2C Passthrough */
            regVal = 0x00;
            Deserializer_ReadReg(desI2cAddr, 0x58, &regVal);
            regVal = regVal | 0x40;
            Deserializer_WriteReg(desI2cAddr, 0x58, regVal);

            Deserializer_ReadReg(desI2cAddr, 0x4D, 0x00); // Clear status register of errors.
            Deserializer_ReadReg(desI2cAddr, 0x4E, 0x00); // Clear status register of errors.
            Deserializer_ReadReg(desI2cAddr, 0x55, 0x00); // Clear status register of errors.
            Deserializer_ReadReg(desI2cAddr, 0x56, 0x00); // Clear status register of errors. 
            Deserializer_ReadReg(desI2cAddr, 0x7A, 0x00); // Clear status register of errors.  
            Deserializer_ReadReg(desI2cAddr, 0x7B, 0x00); // Clear status register of errors.
        }
        else
        {
            // printf("\tskipping 4 for chid: %d \n", chId);
        }
        chId++;
    }
    appLogWaitMsecs(5);

    for(int deshubInst = 0; deshubInst < gDeserializerHubs; deshubInst++)
    {
        IssSensor_getDeserializerI2cAddr(deshubInst, &desI2cBusNum, &desI2cAddr);
        deserializer_setBackChannel50mbps(desI2cAddr);
        deserializer_setCsiSpeed(desI2cAddr, moduleInfo->fpdLink);
    }

    status = 0;
    return status;
}
#endif

/* Checks which cameras are connected to the specified Deserializer */
/* Returns a 4-bit mask - 0 = Camera Not Detected, 1 = Camera Detected */
static int32_t IssSensor_DeserializerInit()
{
    int32_t status = -1;
    uint8_t  desI2cBusNum = 0;
    uint8_t  desI2cAddr;
    tivx_mutex desCfgLock;
    vx_bool bDet = vx_false_e; /* default to Fusion1 configuration */

    tivxMutexCreate(&desCfgLock);
    tivxMutexLock(desCfgLock);

    /* TODO: !defined(MCU_PLUS_SDK) should be removed when PDK-15177 fixed  */
    #if !defined(SOC_AM62A) && !defined(MCU_PLUS_SDK)
    /* Check for the Fusion2 board */
    bDet = Board_detectBoard(BOARD_ID_FUSION2);
    #endif
    if(bDet == vx_true_e)
    {
        printf("Fusion2 Board Detected, using Fusion2 configuration\n");
        gFusion2Det = vx_true_e;
        desCfgCommon_ptr = ub9702DesCfg_Common;
        gDeserializerHubs = 3U;
    }
    else
    {
        issLogPrintf("Fusion1 Board configuration\n");
        gFusion2Det = vx_false_e;
        desCfgCommon_ptr = ub960DesCfg_Common;
        gDeserializerHubs = 2U;
    }

    IssSensor_getDeserializerI2cAddr(BOARD_CSI_INST_0, &desI2cBusNum,
        &desI2cAddr);

    status = setupI2CInst(desI2cBusNum);
    if(status!=0)
    {
        printf(" I2C ERROR \n");
        tivxMutexUnlock(desCfgLock);
        tivxMutexDelete(&desCfgLock);
        return status;
    }

#if !defined(SOC_AM62A)
    status = powerOnDeserializers();
    if(status != 0)
    {
        printf(
            "IssSensor_DeserializerInit Error: powerOnDeserializers "
            "returned %d \n", status);
    }
#endif

    /* Deserializer non-port specific configuration */
    for(int hubInst = 0; hubInst < gDeserializerHubs; hubInst++)
    {
#if defined(B7_IMPLEMENTATION)
        if(gFusion2Det == vx_true_e)
        {
            IssSensor_getDeserializerI2cAddr(hubInst, &desI2cBusNum, &desI2cAddr);
    
            /* Disable lock lost feature */
            Deserializer_WriteReg(desI2cAddr, 0x3C, 0x0F);
        }
        else
        {
            status = IssSensor_cfgDesScript(desCfgCommon_ptr, hubInst);
        }
#else
        status = IssSensor_cfgDesScript(desCfgCommon_ptr, hubInst);
#endif
    }

#if defined(B7_IMPLEMENTATION)
    if(gFusion2Det == vx_true_e)
    {
        status = IssSensor_DeserializerInit_B7();
        if(status != 0)
        {
            printf("Error : IssSensor_DeserializerInit_B7 returned %d while configuring DES \n", status);
        }
    }
#endif

    tivxMutexUnlock(desCfgLock);
    tivxMutexDelete(&desCfgLock);

    return 0;
}

#if !defined(SOC_AM62A)
/**
 *******************************************************************************
 *
 * \brief Powers on deserailzer hubs on Fusion1/2 boards
 *
 * \return  0 if success
 * \return -1 otherwise
 
 *******************************************************************************
*/
static int32_t powerOnDeserializers(void)
{
    int32_t status = -1;
    uint8_t regAddr;

    #if !defined(MCU_PLUS_SDK)
    uint8_t regVals[] = {0xFF, 0xFE};
    uint32_t tca6408SlaveAddr = TCA6408_I2C_ADDR_1;
    regAddr = TCA6408_CONFIG_REG;

    for(int i = 0; i < (sizeof(regVals)/sizeof(regVals[0])); i++)
    {
        status = Board_i2c8BitRegWr(gISS_Sensor_I2cHandle, tca6408SlaveAddr, regAddr, &regVals[i], 1U, SENSOR_I2C_TIMEOUT);
        if(0 == status)
        {
            issLogPrintf("write 0x%.2x to TCA6408 register 0x%.2x \n", regVals[i], regAddr);
        }
        else
        {
            printf("Error writing to TCA6408 register 0x%.2x \n", regAddr);
        }
        appLogWaitMsecs(200);
    }
    #else
    uint32_t tca9543I2cSlaveAddr = 0x70;
    uint8_t regVal = 0x01; /* Enable Channel 0 */

    /* Main I2C2 interacts with CSI0_I2C2 via Channel 0 of the TCA9543APWR,
    *  a 2-Channel I2C Bus Switch. A value of 0x01 has to be written to the
    *  TCA9543APWR to enable I2C communication through Channel 0. The address
    *  of TCA9543APWR is 0x70 on J7AEN (defined by hardware).
    */

    status = Board_i2c8BitRegWrSingle(
        gISS_Sensor_I2cHandle,
        tca9543I2cSlaveAddr,
        &regVal,
        0x20);

    if(status == 0)
    {
        issLogPrintf(
            "Write 0x%.2x to TCA9543APWR (0x%.2x) successful!!!\n",
            regVal,
            tca9543I2cSlaveAddr);
    }
    else
    {
        printf(
            "Error writing 0x%.2x to TCA9543APWR (0x%.2x)!!!\n",
            regVal,
            tca9543I2cSlaveAddr);
    }
    #endif

    return status;
}
#endif