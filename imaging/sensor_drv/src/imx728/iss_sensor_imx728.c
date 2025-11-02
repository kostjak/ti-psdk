/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Sony Semiconductor Solutions Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the “Software”), to deal in
 * the Software without restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 *
 * Copyright (c) 2024 Texas Instruments Incorporated
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
#include "iss_sensor_imx728.h"

// #define IMX728_ENABLE_EXTERNAL_SYNC

#include "imx728_ub971_sensor_config_wdr.h"
#include "imx728_serdes_config.h"

#if defined (SOC_J721E)
#include <ti/board/src/j721e_evm/include/board_cfg.h>
#include <ti/board/src/j721e_evm/include/board_i2c_io_exp.h>
#elif defined (SOC_J721S2)
#include <ti/board/src/j721s2_evm/include/board_cfg.h>
#include <ti/board/src/j721s2_evm/include/board_i2c_io_exp.h>
#elif defined (SOC_J784S4)
#include <ti/board/src/j784s4_evm/include/board_cfg.h>
#include <ti/board/src/j784s4_evm/include/board_i2c_io_exp.h>
#endif

static IssSensor_CreateParams  imx728CreatePrms = {
    SENSOR_SONY_IMX728_UB971_SONY,     /*sensor name*/
    0x6,                               /*i2cInstId*/
    {
        SENSOR_0_I2C_ALIAS, SENSOR_1_I2C_ALIAS, SENSOR_2_I2C_ALIAS, SENSOR_3_I2C_ALIAS,
        SENSOR_4_I2C_ALIAS, SENSOR_5_I2C_ALIAS, SENSOR_6_I2C_ALIAS, SENSOR_7_I2C_ALIAS
    },/*i2cAddrSensor*/
    {
        SER_0_I2C_ALIAS, SER_1_I2C_ALIAS, SER_2_I2C_ALIAS, SER_3_I2C_ALIAS,
        SER_4_I2C_ALIAS, SER_5_I2C_ALIAS, SER_6_I2C_ALIAS, SER_7_I2C_ALIAS
    },/*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            IMX728_OUT_WIDTH,                                 /*width*/
            IMX728_OUT_HEIGHT-IMX728_META_HEIGHT_AFTER,       /*height*/
            1,                                       /*num_exposures*/
            vx_false_e,                              /*line_interleaved*/
            {
                {TIVX_RAW_IMAGE_16_BIT, 11},         /*dataFormat and MSB [0]*/
            },
            0,                                       /*meta_height_before*/
            IMX728_META_HEIGHT_AFTER,                /*meta_height_after*/
        },
        ISS_SENSOR_IMX728_FEATURES,    /*features*/
        ALGORITHMS_ISS_AEWB_MODE_AEWB, /*aewbMode*/
        30,                            /*fps*/
        4,                             /*numDataLanes*/
        {1, 2, 3, 4},                  /*dataLanesMap*/
        {0, 0, 0, 0},                  /*dataLanesPolarity*/
        CSIRX_LANE_BAND_SPEED_1350_TO_1500_MBPS, /*CSI Clock*/
    },
#if defined(B7_IMPLEMENTATION)
    {   /* moduleInfo */
        IMAGE_SERDES_FPD_LINK_4,           /* fpdLink */
        IMX728_SONY_I2C_ADDR<<1,        /* senI2cAddr */
    #ifdef IMX728_D3
        0x30,                           /* serI2cAddr */
    #else
        0x32,                           /* serI2cAddr */
    #endif
    },
#endif
    8,                                 /*numChan*/
    728,                               /*dccId*/
};

static IssSensorFxns imx728SensorFxns = {
    IMX728_Probe,
    IMX728_Config,
    IMX728_StreamOn,
    IMX728_StreamOff,
    IMX728_PowerOn,
    IMX728_PowerOff,
    IMX728_GetExpParams,
    IMX728_SetAeParams,
    IMX728_GetDccParams,
    IMX728_InitAewbConfig,
    IMX728_GetIspConfig,
    IMX728_ReadWriteReg,
    IMX728_GetExpPrgFxn,
    IMX728_deinit,
    NULL, /* getWbCfg */
    NULL, /* setAwbParams */
};

static IssSensorIntfParams imx728SensorIntfPrms = {
    0,          /* sensorBroadcast */
    0,          /* enableFsin */
    0,          /* numCameraStreaming */
};

IssSensors_Handle imx728SensorHandle = {
    1,                             /*isUsed*/
    &imx728CreatePrms,             /*CreatePrms*/
    &imx728SensorFxns,             /*SensorFxns*/
    &imx728SensorIntfPrms,         /*SensorIntfPrms*/
};

#ifdef IMX728_D3
static uint8_t i2cAddrMisc[ISS_SENSORS_MAX_CHANNEL] = {
    MISC_0_I2C_ALIAS, MISC_1_I2C_ALIAS, MISC_2_I2C_ALIAS, MISC_3_I2C_ALIAS,
    MISC_4_I2C_ALIAS, MISC_5_I2C_ALIAS, MISC_6_I2C_ALIAS, MISC_7_I2C_ALIAS
#if defined(SOC_J784S4) || defined(SOC_J742S2)
    , MISC_8_I2C_ALIAS, MISC_9_I2C_ALIAS, MISC_10_I2C_ALIAS, MISC_11_I2C_ALIAS
#endif
    };
#endif

/*
    * \brief DCC Parameters of IMX728
    */
IssCapture_CmplxIoLaneCfg imx728Csi2CmplxIoLaneCfg;

static uint16_t sp1hGainRegValueOld[ISS_SENSORS_MAX_CHANNEL];
static uint32_t exposureTimeLinesOld[ISS_SENSORS_MAX_CHANNEL];
static uint8_t gisFPD4enabled[2] = {0, 0};

int32_t IssSensor_IMX728_Init()
{
    int32_t status;

    status = IssSensor_Register(&imx728SensorHandle);
    if (status != 0)
    {
        printf("IssSensor_IMX728_Init failed \n");
    }

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
    */

static int32_t IMX728_Probe(uint32_t chId, void *pSensorHdl)
{
    /* not supported */
    return -1;
}

static uint16_t IMX728_GetPortSelectRegisterValue(uint32_t chId)
{
    switch (chId)
    {
        case 0:
        case 4:
            return 0x01;
        case 1:
        case 5:
            return 0x12;
        case 2:
        case 6:
            return 0x24;
        case 3:
        case 7:
            return 0x38;
        case 0xFF:
            return 0x0F;
        default:
            printf("Error : Invalid channel ID 0x%x \n", chId);
            return 0;
    }
}

static int32_t IMX728_SetSerializerPhyI2cAddr(uint32_t chId)
{
    I2cParams script[3u] = {
        {0x4C, 0x01, 0xFF},
#ifdef IMX728_D3
        {0x5D, 0x18 << 1 /* 0x30 */, 0xFF},
#else
        {0x5D, 0x32, 0xFF},
#endif
        {0xFFFF, 0x00, 0x0} //End of script
    };

    int8_t  desInstanceId = getDeserializerInstIdFromChId(chId);

    if (desInstanceId < 0)
    {
        printf("Error : Invalid desInstanceId %d\n", desInstanceId);
        return -1;
    }

    script[0].nRegValue = IMX728_GetPortSelectRegisterValue(chId);
    if (script[0].nRegValue == 0)
    {
        return -1;
    }

    int32_t status = IssSensor_cfgDesScript(script, desInstanceId);
    if (status != 0)
    {
        return status;
    }

    return 0;
}

static int32_t IMX728_PowerOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    int8_t  desInstanceId = getDeserializerInstIdFromChId(chId);
    if(desInstanceId < 0)
    {
        printf("Error : Invalid desInstanceId %d\n", desInstanceId);
        return -1;
    }

#if !defined(B7_IMPLEMENTATION)
    for(int i = 0; i < 2; i++)
    {
        if(i == desInstanceId)
        {
            if(gisFPD4enabled[i] == 0)
            {
                status = IssSensor_cfgDesScript(ub9702_971_DesCfg_Common, i);
                if(status!=0)
                {
                    printf("Error : IssSensor_cfgDesScript returned %d while configuring DES %d \n", status, desInstanceId);
                }
                gisFPD4enabled[i] = 1;
            }
        }
    }
#endif

#ifdef IMX728_D3
    /* i2cAddrMisc[0] = MISC_0_I2C_ALIAS = 0x50 */
    uint8_t pcalI2cAliasAddr = i2cAddrMisc[chId];

    /* PCAL9538A @ 7-bit address 0x70 */

    I2cParams script[4u] = {
        {0x4C, 0x01, 0xFF},
        {0x5F, IMX728_PCAL_7BIT_ADDR<<1, 0xFF},
        {0x67, pcalI2cAliasAddr<<1, 0xFF},
        {0xFFFF, 0x00, 0x0} //End of script
    };

    script[0].nRegValue = IMX728_GetPortSelectRegisterValue(chId);
    if (script[0].nRegValue == 0)
    {
        return -1;
    }

    status = IssSensor_cfgDesScript(script, desInstanceId);
    if (status != 0)
    {
        printf("\n\n IMX728_Probe: IssSensor_cfgDesScript FAILED \n\n");
        return status;
    }
    
    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = 255U;
    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    uint8_t regVal = 0xFE;
    status = Board_i2c8BitRegWr(sensorI2cHandle, pcalI2cAliasAddr, 0x03, &regVal, 1, SENSOR_I2C_TIMEOUT);
    regVal = 0x01;
    status = Board_i2c8BitRegWr(sensorI2cHandle, pcalI2cAliasAddr, 0x01, &regVal, 1, SENSOR_I2C_TIMEOUT);

    // printf("\n IMX728_Probe: DEBUG 0\n");
        
    if(status != 0) {
        printf("IMX728 Error: Configuring PCAL for Reset line of ch %d returned %d \n", chId, status);
        return status;
    }

    printf("\n IMX728_Probe: PCAL RESET SUCCESSFUL DEBUG 1 \n\n");

    appLogWaitMsecs(200);
#endif /* IMX728_D3 */

    printf("IMX728_PowerOn : chId = %d\n", chId);

    IssSensors_Handle *pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams *pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    uint32_t i2cInstId = pCreatePrms->i2cInstId;

    /* Set alias of sensor and serializer */
    status = UB960_SetSensorAlias(chId, IMX728_SONY_I2C_ADDR, pCreatePrms->i2cAddrSer[chId]);
    if (status != 0)
    {
        printf("IMX728_Config Error : des_SetSensorAlias for chId %d returned %d \n", chId, status);
        return status;
    }

    /* Register physical address of serializer to deserializer*/
    status = IMX728_SetSerializerPhyI2cAddr(chId);
    if (status != 0)
    {
        printf("IMX728_Config Error : IMX728_SetSerializerPhyI2cAddr for chId %d returned %d \n",
                chId, status);
        return status;
    }

    status = ub953_cfgScript(i2cInstId, pCreatePrms->i2cAddrSer[chId], ub953SerCfg_SonyIMX728);

    if (status != 0)
    {
        printf("IMX728_Config : UB953 config failed for camera # %d \n", chId);
        return status;
    }

    appLogWaitMsecs(20);

    return status;
}

static int32_t IMX728_PowerOff(uint32_t chId, void *pSensorHdl)
{
    printf("IMX728_PowerOff : chId = %d\n", chId);

    IssSensors_Handle *pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams *pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    uint32_t i2cInstId = pCreatePrms->i2cInstId;
    int32_t status = ub953_cfgScript(i2cInstId, pCreatePrms->i2cAddrSer[chId], powerOff_SonyIMX728);

    if (status != 0)
    {
        printf("IMX728_PowerOff : UB9XX failed for camera # %d \n", chId);
        return status;
    }

    return 0;
}

#if !defined(OPTIMIZED_SENSOR_CONFIG)
static int32_t IMX728_RunSensorConfigScript(uint32_t i2cInstId, uint8_t sensorI2cAddr, const I2cParams *sensorCfg, uint16_t sensor_cfg_script_len)
{
    int32_t status = 0;
    uint16_t regAddr;
    uint8_t regValue;
    uint16_t delayMilliSec;
    uint32_t regCnt;

    if(NULL != sensorCfg)
    {
        regCnt = 0;
        regAddr  = sensorCfg[regCnt].nRegAddr;
        regValue = sensorCfg[regCnt].nRegValue;
        delayMilliSec = sensorCfg[regCnt].nDelay;

        printf(" Configuring IMX728 imager 0x%x.. Please wait till it finishes \n", sensorI2cAddr);
        while(regCnt<sensor_cfg_script_len)
        {
            status |= IMX728_WriteReg(i2cInstId, sensorI2cAddr, regAddr, &regValue, 1u, 0);

            if (0 != status)
            {
                printf(" \n \n IMX728: Sensor Reg Write Failed for regAddr 0x%x \n \n", regAddr);
            }

            if(delayMilliSec > 0)
            {
               appLogWaitMsecs(delayMilliSec);
            }

            regCnt++;
            regAddr  = sensorCfg[regCnt].nRegAddr;
            regValue = sensorCfg[regCnt].nRegValue;
            delayMilliSec = sensorCfg[regCnt].nDelay;
        }
        /*Wait 100ms after the init is done*/
        appLogWaitMsecs(100);
        //printf(" IMX728 config done \n ");
    }
    else
    {
        printf(" IMX728 config script is NULL \n");
    }
    return status;
}
#else
static int32_t IMX728_RunSensorConfigScript(uint32_t i2cInstId,
                                               uint8_t i2cAddrSensor,
                                               const I2cParams *sensorConfig,
                                               uint32_t sensorConfigScriptSize)
{
    int32_t status = 0;

    if (sensorConfig == NULL)
    {
        printf("IMX728: Sensor Config Script is NULL.\n");
        return 0;
    }

    uint32_t       regCounter = 0;
    static uint8_t regValueBuffer[64]; // The type of the number of bytes in Board_i2c16BitRegWr is uint8_t

    while (regCounter < sensorConfigScriptSize)
    {
        uint16_t startRegAddr  = sensorConfig[regCounter].nRegAddr;
        uint16_t delayMilliSec = sensorConfig[regCounter].nDelay;

        uint16_t regAddr = startRegAddr;

        uint8_t * pRegValueBuffer = regValueBuffer;
        *pRegValueBuffer++ = sensorConfig[regCounter].nRegValue;
        uint16_t writeByteCounter = 1;

        /* Burst transfer if delayMIlliSec is 0 && the address of the next register is consecutive */
        while (delayMilliSec == 0 && writeByteCounter < 64 && regCounter + 1 < sensorConfigScriptSize
                && sensorConfig[regCounter + 1].nRegAddr - regAddr == 1)
        {
            writeByteCounter++;
            regCounter++;
            regAddr = sensorConfig[regCounter].nRegAddr;
            *pRegValueBuffer++ = sensorConfig[regCounter].nRegValue;
            delayMilliSec = sensorConfig[regCounter].nDelay;
        }

        status |= IMX728_WriteReg(i2cInstId, i2cAddrSensor,
                                  startRegAddr, regValueBuffer, (uint8_t)writeByteCounter, 0);

        if (status != 0)
        {
            printf(" \n IMX728: Sensor Register Write Failed for Addr 0x%04x \n \n", regAddr);
        }

        if (delayMilliSec > 0)
        {
            appLogWaitMsecs(delayMilliSec);
        }

        regCounter++;
    }

    return status;
}
#endif

#ifdef IMX728_ENABLE_EXTERNAL_SYNC
static int32_t IMX728_EnableFsync(uint32_t chId)
{
    static I2cParams script[10u] = {
        {0x4C, 0x00, 0x1},
        {0x6E, 0x8A, 0x00}, // BC_GPIO_CTL0: FrameSync signal to GPIO0
        {0x10, 0x91, 0x00}, // GPIO0 FrameSync signal; Output Source Select: Device Status; Output: Enabled
        {0x58, 0x5E, 0x00}, // Back Channel FREQ SELECT: 50 Mbps
        {0x19, 0x15, 0x00}, // FS_HIGH_TIME_1
        {0x1A, 0xB3, 0x00}, // FS_HIGH_TIME_0
        {0x1B, 0xC3, 0x00}, // FS_LOW_TIME_1
        {0x1C, 0x50, 0x00}, // FS_LOW_TIME_0
        {0x18, 0x01, 0x00}, // Enable Frame Sync

        {0xFFFF, 0x00, 0x0} //End of script
    };

    int8_t desInstanceId = getDeserializerInstIdFromChId(chId);
    if (desInstanceId < 0)
    {
        printf("Error : Invalid desInstanceId %d\n", desInstanceId);
        return -1;
    }

    script[0].nRegValue = IMX728_GetPortSelectRegisterValue(chId);
    if (script[0].nRegValue == 0)
    {
        return -1;
    }

    int32_t status = IssSensor_cfgDesScript(script, desInstanceId);
    if (status != 0)
    {
        return status;
    }

    return 0;
}
#endif

static int32_t IMX728_Transit_To_Standby(uint32_t chId, uint8_t i2cAddrSensor, uint32_t i2cInstId)
{
    uint8_t regVal = IMX728_CK_INCK_FREQ_EN;
    int32_t status = IMX728_WriteReg(i2cInstId, i2cAddrSensor, IMX728_CK_INCK_FREQ_I2C_EN_ADDR, &regVal, 1, 0);
    if (status != 0)
    {
        printf("IMX728_Config : IMX728 configuration failed for camera # %d \n", chId);
        return status;
    }
    regVal = IMX728_CK_INCK_FREQ_18MHZ;
    status = IMX728_WriteReg(i2cInstId, i2cAddrSensor, IMX728_CK_INCK_FREQ_I2C_ADDR, &regVal, 1, 0);
    if (status != 0)
    {
        printf("IMX728_Config : IMX728 configuration failed for camera # %d \n", chId);
        return status;
    }
    regVal = IMX728_CK_SLEEP_H_TO_STANDBY;
    status = IMX728_WriteReg(i2cInstId, i2cAddrSensor, IMX728_CK_SLEEP_H_ADDR, &regVal, 1, 0);
    if (status != 0)
    {
        printf("IMX728_Config : IMX728 configuration failed for camera # %d \n", chId);
        return status;
    }
    appLogWaitMsecs(30);

    return status;
}


static int32_t IMX728_Config(uint32_t chId, void *pSensorHdl, uint32_t sensorFeaturesRequested)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams * pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    if (sensorFeaturesRequested != (sensorFeaturesRequested & ISS_SENSOR_IMX728_FEATURES))
    {
        printf("IMX728_Config : Error. feature set 0x%x is not supported \n", sensorFeaturesRequested);
        return -1;
    }

#ifdef IMX728_ENABLE_EXTERNAL_SYNC
    /* Enable SYNC */
    status = IMX728_EnableFsync(chId);
    if (status != 0)
    {
        printf("IMX728_Config Error : IMX728_EnableFsync for chId %d returned %d \n",
               chId, status);
        return status;
    }
#endif

    uint8_t  i2cAddrSensor = pCreatePrms->i2cAddrSensor[chId];
    uint32_t i2cInstId     = pCreatePrms->i2cInstId;

    status = IMX728_Transit_To_Standby(chId, i2cAddrSensor, i2cInstId);

    /* Read status of serial nor flash */
    uint8_t nvStatus = 0x00;
    status = IMX728_ReadReg(i2cInstId, i2cAddrSensor,
                               IMX728_NV_STATUS_REG_ADDR, &nvStatus, 1u, 1u);
    if (status != 0)
    {
        printf("IMX728_Config: Failed to read status of serial nor flash\n");
        return status;
    }

    if (nvStatus == 0)
    {
        /* Configure Sensor */
        printf(" Configuring IMX728 imager 0x%x without serial nor flash.. Please wait till it finishes \n",
                i2cAddrSensor);
        
        uint32_t sensorConfigSize = sizeof(imx728Ub971ConfigWdr) / sizeof(I2cParams);
        status = IMX728_RunSensorConfigScript(i2cInstId, i2cAddrSensor, imx728Ub971ConfigWdr, sensorConfigSize);

        appLogWaitMsecs(100);
    }
    else
    {
        printf("IMX728_Config : Serial nor flash is not supported in this driver.\n");
        return -1;
    }

    if (status != 0)
    {
        printf("IMX728_Config : IMX728 configuration failed for camera # %d \n", chId);
        return status;
    }

    printf(" IMX728 configuration done\n");

    return (status);
}

static int32_t IMX728_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    IssSensors_Handle *pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams *pCreatePrms;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;
    int8_t desInstanceId = getDeserializerInstIdFromChId(chId);
    uint8_t regVal;

    if (desInstanceId < 0)
    {
        printf("Invalid desInstanceId \n");
        return -1;
    }

    assert(pSenHandle != NULL);
    pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    i2cInstId     = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    uint32_t sensorConfigSize = sizeof(imx728BeforeStreamingConfig) / sizeof(I2cParams);
    status = IMX728_RunSensorConfigScript(i2cInstId, sensorI2cAddr, imx728BeforeStreamingConfig, sensorConfigSize);

    if (status != 0)
    {
        printf("IMX728_StreamOn: IMX728 configuration failed before streaming.\n");
        return -1;
    }

    appLogWaitMsecs(1);

    /* Write Streaming Settings */
    regVal = 0x5C;
    status |= IMX728_WriteReg(i2cInstId, sensorI2cAddr, IMX728_CK_STRM_H_ADDR, &regVal, 1u, 0);
    regVal = 0xA3;
    status |= IMX728_WriteReg(i2cInstId, sensorI2cAddr, IMX728_CK_STRM_H_ADDR, &regVal, 1u, 0);
    printf("IMX728 Streaming On\n");

    appLogWaitMsecs(35);

    sensorConfigSize = sizeof(imx728AfterStreamingConfig) / sizeof(I2cParams);
    status = IMX728_RunSensorConfigScript(i2cInstId, sensorI2cAddr, imx728AfterStreamingConfig, sensorConfigSize);

    if (status != 0)
    {
        printf("IMX728_StreamOn: IMX728 configuration failed after streaming.\n");
        return -1;
    }

    appLogWaitMsecs(10);

    status |= enableUB960Streaming(chId);

    return (status);
}

static int32_t IMX728_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle *pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams *pCreatePrms;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;
    uint8_t regVal;

    assert(pSenHandle != NULL);
    pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    i2cInstId     = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    regVal = 0xFF;
    status |= IMX728_WriteReg(i2cInstId, sensorI2cAddr, IMX728_CK_STRM_H_ADDR, &regVal, 1u, 1);

    appLogWaitMsecs(10);
    status |= disableUB960Streaming(chId);

    gisFPD4enabled[0] = 0;
    gisFPD4enabled[1] = 0;

    return status;
}

inline static int32_t IMX728_SetExposureTime(uint32_t i2cInstId,
                                                uint8_t sensorI2cAddr,
                                                uint32_t exposureTimeLines)
{
    int32_t status = -1;
    uint16_t regAddrSp1h  = IMX728_SP1_EXPOSURE_TIME_FID0_ADDR;
    uint16_t regAddrSp2h  = IMX728_SP2_EXPOSURE_TIME_FID0_ADDR;

    status = IMX728_WriteReg(i2cInstId,
                                sensorI2cAddr,
                                regAddrSp1h,
                                (uint8_t*)&exposureTimeLines,
                                4u,
                                0);
    if (status != 0)
    {
        printf("Error writing 0x%x to IMX728 register 0x%x \n",
                regAddrSp1h, exposureTimeLines);
    }

    status = IMX728_WriteReg(i2cInstId,
                                sensorI2cAddr,
                                regAddrSp2h,
                                (uint8_t*)&exposureTimeLines,
                                4u,
                                1);
    if (status != 0)
    {
        printf("Error writing 0x%x to IMX728 register 0x%x \n",
                regAddrSp2h, exposureTimeLines);
    }

    return status;
}

inline static int32_t IMX728_SetAnalogGain(uint32_t i2cInstId,
                                                uint8_t sensorI2cAddr,
                                                uint16_t sp1hAgain)
{
    int32_t status = -1;
    uint16_t regAddr = IMX728_FME_ISP_GAIN_FID0_ADDR;

    /* FID0 */
    status = IMX728_WriteReg(i2cInstId,
                                sensorI2cAddr,
                                regAddr,
                                (uint8_t*)&sp1hAgain, 2u, 0);
    if (status != 0)
    {
        printf("Error writing 0x%x to IMX728 register 0x%x \n", regAddr, sp1hAgain);
    }


    return status;
}

static int32_t IMX728_SetAeParams(void *pSensorHdl,
                                  uint32_t chId,
                                  IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = -1;

    IssSensors_Handle *pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams *pCreatePrms;
    uint16_t sp1hAgain = 0;
    uint32_t exposureTimeLines;
    uint32_t vmax;
    uint8_t regUpdateValue;
    uint16_t cnt;
    uint32_t i2cInstId;
    uint8_t  sensorI2cAddr;

    assert(pSenHandle != NULL);
    pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    i2cInstId     = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    /* Exposure time */
    status = IMX728_ReadReg(i2cInstId, sensorI2cAddr,
                            IMX728_VMAX_REG_ADDR, (uint8_t*)&vmax, 4u, 0);

    if (status != 0)
    {
        printf("Error read IMX728 register 0x%x \n", IMX728_VMAX_REG_ADDR);
        return -1;
    }
    exposureTimeLines = ((pExpPrms->exposureTime[ISS_SENSOR_EXPOSURE_LONG]
                            / 1000 * vmax * pCreatePrms->sensorInfo.fps) + 500000)  / 1000000;

    /* Analog Gain */
    for (cnt = 0; cnt < ISS_IMX728_GAIN_TBL_SIZE; cnt ++)
    {
        if (pExpPrms->analogGain[ISS_SENSOR_EXPOSURE_LONG] <= gIMX728GainsTable[cnt][0])
        {
            sp1hAgain = gIMX728GainsTable[cnt][1];
            break;
        }
    }

    if (sp1hAgain == sp1hGainRegValueOld[chId] &&
            exposureTimeLines == exposureTimeLinesOld[chId])
    {
        /* Reduce I2C transactions.
        Do not write to the sensor if register value does not change */
        return 0;
    }

    status = IMX728_SetExposureTime(i2cInstId, sensorI2cAddr, exposureTimeLines);
    if (status != 0)
    {
        return status;
    }
    status = IMX728_SetAnalogGain(i2cInstId, sensorI2cAddr, sp1hAgain);
    if (status != 0)
    {
        return status;
    }

    /* Run Register Update */
    regUpdateValue = IMX728_REG_UPDATE;
    status = IMX728_WriteReg(i2cInstId, sensorI2cAddr,
                             IMX728_REG_UPDATE_ADDR, &regUpdateValue, 1u, 0);
    
    if (status != 0)
    {
        printf("Error write IMX728 register 0x%x \n", IMX728_REG_UPDATE_ADDR);
        return -1;
    }

    appLogWaitMsecs(1);
    
    sp1hGainRegValueOld[chId]  = sp1hAgain;
    exposureTimeLinesOld[chId] = exposureTimeLines;

    return (status);
}

static int32_t IMX728_GetDccParams(uint32_t chId,
                                        void *pSensorHdl,
                                        IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t IMX728_GetExpParams(uint32_t chId,
                                        void *pSensorHdl,
                                        IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;

    assert(pExpPrms != NULL);
    pExpPrms->expRatio = ISS_SENSOR_IMX728_DEFAULT_EXP_RATIO;

    return (status);
}

static void IMX728_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    return;
}

static void IMX728_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    return;
}

static void IMX728_deinit (uint32_t chId, void *pSensorHdl)
{
    return;
}

static int32_t IMX728_ReadWriteReg (uint32_t chId, void *pSensorHdl,
                                    uint32_t readWriteFlag, I2cParams *pReg)
{
    int32_t status = 0;

    uint8_t  regValue8  = 0;
    uint16_t regValue16 = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;

    assert(pSenHandle != NULL);
    pCreatePrms = pSenHandle->createPrms;

    assert(pReg != NULL);

    if (readWriteFlag == 1U)
    {
        /*write*/
        regValue8 = pReg->nRegValue;
        status = IMX728_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue8, 1u, 0);
    }
    else if (readWriteFlag == 2U)
    {
        /* Read 2 bytes */
        status = IMX728_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, (uint8_t*)&regValue16, 2u, 0);
        if (status == 0)
        {
            pReg->nRegValue = regValue16;
        }
    }
    else if (readWriteFlag == 3U)
    {
        /* Write 2 bytes */
        regValue16 = pReg->nRegValue;
        status = IMX728_WriteReg(pCreatePrms->i2cInstId,
                pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, (uint8_t*)&regValue16, 2u, 0);
    }
    else
    {
        /*read*/
        status = IMX728_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue8, 1u, 0);

        if (status == 0)
        {
            pReg->nRegValue = regValue8;
        }
    }
    return (status);
}

static int32_t IMX728_ReadReg(uint8_t     i2cInstId,
                                uint8_t     i2cAddr,
                                uint16_t    regAddr,
                                uint8_t     *regVal,
                                uint32_t    numRegs,
                                uint8_t     skip_error)
{
    int32_t  status = -1;
    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = 255U;
    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    if (sensorI2cHandle == NULL)
    {
        printf("Sensor I2C Handle is NULL \n");
        return -1;
    }
    status = Board_i2c16BitRegRd(sensorI2cHandle, i2cAddr, regAddr, regVal, numRegs, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
    if (status != 0 && skip_error == 0)
    {
        printf("Error : I2C Timeout while reading from IMX728 register 0x%x on I2C slave 0x%x \n",
                regAddr, i2cAddr);
    } else if (skip_error != 0)
    {
        status = 0;
    }
    return (status);
}

static int32_t IMX728_WriteReg(uint8_t    i2cInstId,
                               uint8_t    i2cAddr,
                               uint16_t   regAddr,
                               uint8_t    *regVal,
                               uint32_t   numRegs,
                               uint8_t    skip_error) {
    int32_t    status = -1;
    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = 255U;
    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    if (sensorI2cHandle == NULL)
    {
        printf("Sensor I2C Handle is NULL \n");
        return -1;
    }
    status = Board_i2c16BitRegWr(sensorI2cHandle, i2cAddr, regAddr, regVal, numRegs, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
    if (status != 0 && skip_error == 0)
    {
        printf("Error : I2C Timeout while writing 0x%x to IMX728 register 0x%04x \n",
                *regVal, regAddr);
    } else if (skip_error != 0)
    {
        status = 0;
    }

    return (status);
}

static int32_t IMX728_GetExpPrgFxn(uint32_t chId,
                                    void *pSensorHdl,
                                    IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t status = 0;
    uint8_t count = 0;

    p_ae_dynPrms->targetBrightnessRange.min = 40;
    p_ae_dynPrms->targetBrightnessRange.max = 50;
    p_ae_dynPrms->targetBrightness = 45;
    p_ae_dynPrms->threshold = 1;
    p_ae_dynPrms->enableBlc = 1;

    p_ae_dynPrms->exposureTimeStepSize         = 1000000;  // nsec
    p_ae_dynPrms->exposureTimeRange[count].min = 10000000; // nsec
    p_ae_dynPrms->exposureTimeRange[count].max = 10000000; // nsec
    p_ae_dynPrms->analogGainRange[count].min   = 1024;
    p_ae_dynPrms->analogGainRange[count].max   = 16229;
    p_ae_dynPrms->digitalGainRange[count].min  = 256;
    p_ae_dynPrms->digitalGainRange[count].max  = 256;
    count++;

    p_ae_dynPrms->numAeDynParams = count;
    return (status);
}

