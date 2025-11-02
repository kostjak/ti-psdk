/*
 *
 * Copyright (c) 2025 Texas Instruments Incorporated
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

#include <utils/ipc/include/app_ipc.h>
#include <utils/remote_service/include/app_remote_service.h>
#include <utils/iss/include/app_iss.h>
#include <utils/console_io/include/app_log.h>

#ifdef VPAC3L
#include <dcc_viss_ov2312.h>
#endif

#include <dcc_viss_imx390.h>
#include <dcc_viss_imx390_wdr.h>

#include <dcc_viss_imx390_discovery_mid.h>
#include <dcc_viss_imx390_discovery_mid_wdr.h>
#include <dcc_viss_imx390_discovery_narrow.h>
#include <dcc_viss_imx390_discovery_narrow_wdr.h>
#include <dcc_viss_imx390_discovery_wide.h>
#include <dcc_viss_imx390_discovery_wide_wdr.h>

#include <dcc_viss_imx728_wdr.h>

#include <dcc_viss_ar0233.h>
#include <dcc_viss_ar0233_wdr.h>

#include <dcc_viss_ar0820.h>
#include <dcc_viss_ar0820_wdr.h>

#include <dcc_viss_ub9xx_raw_test_pattern.h>

#define _ENABLE_2A_

#ifdef VPAC3L
static uint8_t  dcc_viss_ov2312[DCC_VISS_OV2312_DCC_CFG_NUM_ELEM] = DCC_VISS_OV2312DCC_CFG;
#endif

static uint8_t  dcc_viss_imx390[DCC_VISS_IMX390_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390DCC_CFG;
static uint8_t  dcc_viss_imx390_wdr[DCC_VISS_IMX390_WDR_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390_WDRDCC_CFG;

static uint8_t  dcc_viss_imx390_discovery_mid[DCC_VISS_IMX390_DISCOVERY_MID_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390_DISCOVERY_MIDDCC_CFG;
static uint8_t  dcc_viss_imx390_discovery_mid_wdr[DCC_VISS_IMX390_DISCOVERY_MID_WDR_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390_DISCOVERY_MID_WDRDCC_CFG;
static uint8_t  dcc_viss_imx390_discovery_narrow[DCC_VISS_IMX390_DISCOVERY_NARROW_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390_DISCOVERY_NARROWDCC_CFG;
static uint8_t  dcc_viss_imx390_discovery_narrow_wdr[DCC_VISS_IMX390_DISCOVERY_NARROW_WDR_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390_DISCOVERY_NARROW_WDRDCC_CFG;
static uint8_t  dcc_viss_imx390_discovery_wide[DCC_VISS_IMX390_DISCOVERY_WIDE_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390_DISCOVERY_WIDEDCC_CFG;
static uint8_t  dcc_viss_imx390_discovery_wide_wdr[DCC_VISS_IMX390_DISCOVERY_WIDE_WDR_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX390_DISCOVERY_WIDE_WDRDCC_CFG;

static uint8_t  dcc_viss_imx728_wdr[DCC_VISS_IMX728_WDR_DCC_CFG_NUM_ELEM] = DCC_VISS_IMX728_WDRDCC_CFG;

static uint8_t  dcc_viss_ar0233_linear[DCC_VISS_AR0233_DCC_CFG_NUM_ELEM] = DCC_VISS_AR0233DCC_CFG;
static uint8_t  dcc_viss_ar0233_wdr[DCC_VISS_AR0233_WDR_DCC_CFG_NUM_ELEM] = DCC_VISS_AR0233_WDRDCC_CFG;

static uint8_t  dcc_viss_ar0820_linear[DCC_VISS_AR0820_DCC_CFG_NUM_ELEM] = DCC_VISS_AR0820DCC_CFG;
static uint8_t  dcc_viss_ar0820_wdr[DCC_VISS_AR0820_WDR_DCC_CFG_NUM_ELEM] = DCC_VISS_AR0820_WDRDCC_CFG;

static uint8_t  dcc_viss_ub9xx_raw_test_pattern_linear[DCC_VISS_UB9XX_RAW_TEST_PATTERN_DCC_CFG_NUM_ELEM] = DCC_VISS_UB9XX_RAW_TEST_PATTERNDCC_CFG;

#include <dcc_ldc_imx390.h>
#include <dcc_ldc_imx390_wdr.h>

#include <dcc_ldc_imx390_discovery_mid.h>
#include <dcc_ldc_imx390_discovery_mid_wdr.h>
#include <dcc_ldc_imx390_discovery_narrow.h>
#include <dcc_ldc_imx390_discovery_narrow_wdr.h>
#include <dcc_ldc_imx390_discovery_wide.h>
#include <dcc_ldc_imx390_discovery_wide_wdr.h>

static uint8_t  dcc_ldc_imx390[DCC_LDC_IMX390_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390DCC_CFG;
static uint8_t  dcc_ldc_imx390_wdr[DCC_LDC_IMX390_WDR_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390_WDRDCC_CFG;

static uint8_t  dcc_ldc_imx390_discovery_mid[DCC_LDC_IMX390_DISCOVERY_MID_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390_DISCOVERY_MIDDCC_CFG;
static uint8_t  dcc_ldc_imx390_discovery_mid_wdr[DCC_LDC_IMX390_DISCOVERY_MID_WDR_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390_DISCOVERY_MID_WDRDCC_CFG;
static uint8_t  dcc_ldc_imx390_discovery_narrow[DCC_LDC_IMX390_DISCOVERY_NARROW_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390_DISCOVERY_NARROWDCC_CFG;
static uint8_t  dcc_ldc_imx390_discovery_narrow_wdr[DCC_LDC_IMX390_DISCOVERY_NARROW_WDR_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390_DISCOVERY_NARROW_WDRDCC_CFG;
static uint8_t  dcc_ldc_imx390_discovery_wide[DCC_LDC_IMX390_DISCOVERY_WIDE_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390_DISCOVERY_WIDEDCC_CFG;
static uint8_t  dcc_ldc_imx390_discovery_wide_wdr[DCC_LDC_IMX390_DISCOVERY_WIDE_WDR_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX390_DISCOVERY_WIDE_WDRDCC_CFG;

#include <dcc_ldc_imx728_wdr.h>
static uint8_t  dcc_ldc_imx728_wdr[DCC_LDC_IMX728_WDR_DCC_CFG_NUM_ELEM] = DCC_LDC_IMX728_WDRDCC_CFG;

#include <dcc_ldc_ar0233.h>
#include <dcc_ldc_ar0233_wdr.h>
static uint8_t  dcc_ldc_ar0233_linear[DCC_LDC_AR0233_DCC_CFG_NUM_ELEM] = DCC_LDC_AR0233DCC_CFG;
static uint8_t  dcc_ldc_ar0233_wdr[DCC_LDC_AR0233_WDR_DCC_CFG_NUM_ELEM] = DCC_LDC_AR0233_WDRDCC_CFG;

#include <dcc_ldc_ar0820.h>
#include <dcc_ldc_ar0820_wdr.h>
static uint8_t  dcc_ldc_ar0820_linear[DCC_LDC_AR0820_DCC_CFG_NUM_ELEM] = DCC_LDC_AR0820DCC_CFG;
static uint8_t  dcc_ldc_ar0820_wdr[DCC_LDC_AR0820_WDR_DCC_CFG_NUM_ELEM] = DCC_LDC_AR0820_WDRDCC_CFG;

#include <dcc_ldc_ub9xx_raw_test_pattern.h>
static uint8_t  dcc_ldc_ub9xx_raw_test_pattern_linear[DCC_LDC_UB9XX_RAW_TEST_PATTERN_DCC_CFG_NUM_ELEM] = DCC_LDC_UB9XX_RAW_TEST_PATTERNDCC_CFG;

#include <dcc_ldc_ub96x_uyvytestpat.h>
static uint8_t  dcc_ldc_ub96x_uyvytestpat[DCC_LDC_UB96X_UYVYTESTPAT_DCC_CFG_NUM_ELEM] = DCC_LDC_UB96X_UYVYTESTPATDCC_CFG;

#include <dcc_ldc_gw_ar0233.h>
static uint8_t  dcc_ldc_gw_ar0233[DCC_LDC_GW_AR0233_DCC_CFG_NUM_ELEM] = DCC_LDC_GW_AR0233DCC_CFG;

#ifdef VPAC3L
#include <dcc_ldc_ov2312.h>
static uint8_t  dcc_ldc_ov2312[DCC_LDC_OV2312_DCC_CFG_NUM_ELEM] = DCC_LDC_OV2312DCC_CFG;
#endif

#ifdef _ENABLE_2A_
#include <dcc_2a_imx390.h>
#include <dcc_2a_imx390_wdr.h>

#include <dcc_2a_imx390_discovery_mid.h>
#include <dcc_2a_imx390_discovery_mid_wdr.h>
#include <dcc_2a_imx390_discovery_narrow.h>
#include <dcc_2a_imx390_discovery_narrow_wdr.h>
#include <dcc_2a_imx390_discovery_wide.h>
#include <dcc_2a_imx390_discovery_wide_wdr.h>

#include <dcc_2a_imx728_wdr.h>

#include <dcc_2a_ar0233.h>
#include <dcc_2a_ar0233_wdr.h>

#include <dcc_2a_ar0820.h>
#include <dcc_2a_ar0820_wdr.h>

#ifdef VPAC3L
#include <dcc_2a_ov2312.h>
#endif

#include <dcc_2a_ub9xx_raw_test_pattern.h>

static uint8_t  dcc_2a_imx390[DCC_2A_IMX390_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390DCC_CFG;
static uint8_t  dcc_2a_imx390_wdr[DCC_2A_IMX390_WDR_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390_WDRDCC_CFG;

static uint8_t  dcc_2a_imx390_discovery_mid[DCC_2A_IMX390_DISCOVERY_MID_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390_DISCOVERY_MIDDCC_CFG;
static uint8_t  dcc_2a_imx390_discovery_mid_wdr[DCC_2A_IMX390_DISCOVERY_MID_WDR_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390_DISCOVERY_MID_WDRDCC_CFG;
static uint8_t  dcc_2a_imx390_discovery_narrow[DCC_2A_IMX390_DISCOVERY_NARROW_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390_DISCOVERY_NARROWDCC_CFG;
static uint8_t  dcc_2a_imx390_discovery_narrow_wdr[DCC_2A_IMX390_DISCOVERY_NARROW_WDR_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390_DISCOVERY_NARROW_WDRDCC_CFG;
static uint8_t  dcc_2a_imx390_discovery_wide[DCC_2A_IMX390_DISCOVERY_WIDE_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390_DISCOVERY_WIDEDCC_CFG;
static uint8_t  dcc_2a_imx390_discovery_wide_wdr[DCC_2A_IMX390_DISCOVERY_WIDE_WDR_DCC_CFG_NUM_ELEM] = DCC_2A_IMX390_DISCOVERY_WIDE_WDRDCC_CFG;

static uint8_t  dcc_2a_imx728_wdr[DCC_2A_IMX728_WDR_DCC_CFG_NUM_ELEM] = DCC_2A_IMX728_WDRDCC_CFG;

static uint8_t  dcc_2a_ar0233_linear[DCC_2A_AR0233_DCC_CFG_NUM_ELEM] = DCC_2A_AR0233DCC_CFG;
static uint8_t  dcc_2a_ar0233_wdr[DCC_2A_AR0233_WDR_DCC_CFG_NUM_ELEM] = DCC_2A_AR0233_WDRDCC_CFG;

static uint8_t  dcc_2a_ar0820_linear[DCC_2A_AR0820_DCC_CFG_NUM_ELEM] = DCC_2A_AR0820DCC_CFG;
static uint8_t  dcc_2a_ar0820_wdr[DCC_2A_AR0820_WDR_DCC_CFG_NUM_ELEM] = DCC_2A_AR0820_WDRDCC_CFG;

#ifdef VPAC3L
static uint8_t  dcc_2a_ov2312[DCC_2A_OV2312_DCC_CFG_NUM_ELEM] = DCC_2A_OV2312DCC_CFG;
#endif

static uint8_t  dcc_2a_ub9xx_raw_test_pattern_linear[DCC_2A_UB9XX_RAW_TEST_PATTERN_DCC_CFG_NUM_ELEM] = DCC_2A_UB9XX_RAW_TEST_PATTERNDCC_CFG;

#endif

int32_t appIssGetDCCSizeVISS(char * sensor_name, uint32_t wdr_mode)
{
    int32_t size = -1;
    switch(wdr_mode)
    {
        case 0:
            /*Linear mode*/
            #ifdef VPAC3L
            if(0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                size = DCC_VISS_OV2312_DCC_CFG_NUM_ELEM;
            else
            #endif
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                size = DCC_VISS_IMX390_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                size = DCC_VISS_IMX390_DISCOVERY_MID_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)) {
                    printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                    size = DCC_VISS_IMX390_DISCOVERY_NARROW_DCC_CFG_NUM_ELEM;
            } else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                size = DCC_VISS_IMX390_DISCOVERY_WIDE_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                size = DCC_VISS_AR0233_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                size = DCC_VISS_AR0820_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, UB9XX_RAW_TESTPAT))
                size = DCC_VISS_UB9XX_RAW_TEST_PATTERN_DCC_CFG_NUM_ELEM;
            else
                size = -1;
            break;
        case 1:
            /*WDR mode*/
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                size = DCC_VISS_IMX390_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                size = DCC_VISS_IMX390_DISCOVERY_MID_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                size = DCC_VISS_IMX390_DISCOVERY_NARROW_WDR_DCC_CFG_NUM_ELEM;
            }
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                size = DCC_VISS_IMX390_DISCOVERY_WIDE_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX728_UB971_SONY))
                size = DCC_VISS_IMX728_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                size = DCC_VISS_AR0233_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                size = DCC_VISS_AR0820_WDR_DCC_CFG_NUM_ELEM;
#ifdef VPAC3L
            else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                size = DCC_VISS_OV2312_DCC_CFG_NUM_ELEM;
#endif
            else
                size = -1;
            break;
        default:
            /*Unsupported mode*/
            size = -1;
    }
    return size;
}

int32_t appIssGetDCCBuffVISS(char * sensor_name, uint32_t wdr_mode,  uint8_t * dcc_buf, int32_t num_bytes)
{
    switch(wdr_mode)
    {
        case 0:
            /*Linear mode*/
            #ifdef VPAC3L
            if(0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                memcpy(dcc_buf, dcc_viss_ov2312, num_bytes);
            else
            #endif
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                memcpy(dcc_buf, dcc_viss_imx390, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                memcpy(dcc_buf, dcc_viss_imx390_discovery_mid, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                memcpy(dcc_buf, dcc_viss_imx390_discovery_narrow, num_bytes);
            }
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                memcpy(dcc_buf, dcc_viss_imx390_discovery_wide, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                memcpy(dcc_buf, dcc_viss_ar0233_linear, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                memcpy(dcc_buf, dcc_viss_ar0820_linear, num_bytes);
            else if (0 == strcmp(sensor_name, UB9XX_RAW_TESTPAT))
                memcpy(dcc_buf, dcc_viss_ub9xx_raw_test_pattern_linear, num_bytes);
            else
                return -1;
            break;
        case 1:
            /*WDR mode*/
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                memcpy(dcc_buf, dcc_viss_imx390_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                memcpy(dcc_buf, dcc_viss_imx390_discovery_mid_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                memcpy(dcc_buf, dcc_viss_imx390_discovery_narrow_wdr, num_bytes);
            }
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                memcpy(dcc_buf, dcc_viss_imx390_discovery_wide_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX728_UB971_SONY))
                memcpy(dcc_buf, dcc_viss_imx728_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                memcpy(dcc_buf, dcc_viss_ar0233_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                memcpy(dcc_buf, dcc_viss_ar0820_wdr, num_bytes);
#ifdef VPAC3L
            else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                memcpy(dcc_buf, dcc_viss_ov2312, num_bytes);
#endif
            else
                return -1;
            break;
        default:
            /*Unsupported mode*/
            return -1;
    }

    return 0;
}

int32_t appIssGetDCCSize2A(char * sensor_name, uint32_t wdr_mode)
{
    int32_t size = -1;
    switch(wdr_mode)
    {
        case 0:
            /*Linear mode*/
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                size = DCC_2A_IMX390_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                size = DCC_2A_IMX390_DISCOVERY_MID_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                size = DCC_2A_IMX390_DISCOVERY_NARROW_DCC_CFG_NUM_ELEM;
            }
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                size = DCC_2A_IMX390_DISCOVERY_WIDE_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                size = DCC_2A_AR0233_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                size = DCC_2A_AR0820_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, UB9XX_RAW_TESTPAT))
                size = DCC_2A_UB9XX_RAW_TEST_PATTERN_DCC_CFG_NUM_ELEM;
#ifdef VPAC3L
            else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                size = DCC_2A_OV2312_DCC_CFG_NUM_ELEM;
#endif
            else
                size = -1;
            break;
        case 1:
            /*WDR mode*/
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                size = DCC_2A_IMX390_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                size = DCC_2A_IMX390_DISCOVERY_MID_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)) {
                printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                size = DCC_2A_IMX390_DISCOVERY_NARROW_WDR_DCC_CFG_NUM_ELEM;
            }
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                size = DCC_2A_IMX390_DISCOVERY_WIDE_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX728_UB971_SONY))
                size = DCC_2A_IMX728_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                size = DCC_2A_AR0233_WDR_DCC_CFG_NUM_ELEM;
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                size = DCC_2A_AR0820_WDR_DCC_CFG_NUM_ELEM;
#ifdef VPAC3L
            else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                size = DCC_2A_OV2312_DCC_CFG_NUM_ELEM;
#endif
            else
                size = -1;
            break;
        default:
            /*Unsupported mode*/
            size = -1;
    }
    return size;
}

int32_t appIssGetDCCBuff2A(char * sensor_name, uint32_t wdr_mode,  uint8_t * dcc_buf, int32_t num_bytes)
{
    switch(wdr_mode)
    {
        case 0:
            /*Linear mode*/
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                memcpy(dcc_buf, dcc_2a_imx390, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                memcpy(dcc_buf, dcc_2a_imx390_discovery_mid, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                memcpy(dcc_buf, dcc_2a_imx390_discovery_narrow, num_bytes);
            }
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                memcpy(dcc_buf, dcc_2a_imx390_discovery_wide, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                memcpy(dcc_buf, dcc_2a_ar0233_linear, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                memcpy(dcc_buf, dcc_2a_ar0820_linear, num_bytes);
            else if (0 == strcmp(sensor_name, UB9XX_RAW_TESTPAT))
                memcpy(dcc_buf, dcc_2a_ub9xx_raw_test_pattern_linear, num_bytes);
#ifdef VPAC3L
            else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                memcpy(dcc_buf, dcc_2a_ov2312, num_bytes);
#endif
            else
                return -1;
            break;
        case 1:
            /*WDR mode*/
            if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                memcpy(dcc_buf, dcc_2a_imx390_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                memcpy(dcc_buf, dcc_2a_imx390_discovery_mid_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)) {
                printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                memcpy(dcc_buf, dcc_2a_imx390_discovery_narrow_wdr, num_bytes);
            }
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                memcpy(dcc_buf, dcc_2a_imx390_discovery_wide_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX728_UB971_SONY))
                memcpy(dcc_buf, dcc_2a_imx728_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                memcpy(dcc_buf, dcc_2a_ar0233_wdr, num_bytes);
            else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                memcpy(dcc_buf, dcc_2a_ar0820_wdr, num_bytes);
#ifdef VPAC3L
            else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                memcpy(dcc_buf, dcc_2a_ov2312, num_bytes);
#endif
            else
                return -1;
            break;
        default:
            /*Unsupported mode*/
            return -1;
    }

    return 0;
}

int32_t appIssGetDCCSizeLDC(char * sensor_name, uint32_t wdr_mode)
{
    int32_t size = -1;

    if (0 == strcmp(sensor_name, UB96X_TESTPATTERN_UYVY))
    {
        size = DCC_LDC_UB96X_UYVYTESTPAT_DCC_CFG_NUM_ELEM;
    }
    else if (0 == strcmp(sensor_name, GW_AR0233_UYVY))
    {
        size = DCC_LDC_GW_AR0233_DCC_CFG_NUM_ELEM;
    }
    else
    {
        switch(wdr_mode)
        {
            case 0:
                /*Linear mode*/
                if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                    size = DCC_LDC_IMX390_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                    size = DCC_LDC_IMX390_DISCOVERY_MID_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                    printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                    size = DCC_LDC_IMX390_DISCOVERY_NARROW_DCC_CFG_NUM_ELEM;
                }
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                    size = DCC_LDC_IMX390_DISCOVERY_WIDE_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                    size = DCC_LDC_AR0233_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                    size = DCC_LDC_AR0820_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, UB9XX_RAW_TESTPAT))
                    size = DCC_LDC_UB9XX_RAW_TEST_PATTERN_DCC_CFG_NUM_ELEM;
#ifdef VPAC3L
                else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                    size = DCC_LDC_OV2312_DCC_CFG_NUM_ELEM;
#endif
                else
                    size = -1;
                break;
            case 1:
                /*WDR mode*/
                if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                    size = DCC_LDC_IMX390_WDR_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                    size = DCC_LDC_IMX390_DISCOVERY_MID_WDR_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                    printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                    size = DCC_LDC_IMX390_DISCOVERY_NARROW_WDR_DCC_CFG_NUM_ELEM;
                }
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                    size = DCC_LDC_IMX390_DISCOVERY_WIDE_WDR_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX728_UB971_SONY))
                    size = DCC_LDC_IMX728_WDR_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                    size = DCC_LDC_AR0233_WDR_DCC_CFG_NUM_ELEM;
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                    size = DCC_LDC_AR0820_WDR_DCC_CFG_NUM_ELEM;
#ifdef VPAC3L
                else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                    size = DCC_LDC_OV2312_DCC_CFG_NUM_ELEM;
#endif
                else
                    size = -1;
                break;
            default:
                /*Unsupported mode*/
                size = -1;
        }

    }

    return size;
}

int32_t appIssGetDCCBuffLDC(char * sensor_name, uint32_t wdr_mode,  uint8_t * dcc_buf, int32_t num_bytes)
{
    if (0 == strcmp(sensor_name, UB96X_TESTPATTERN_UYVY))
    {
        memcpy(dcc_buf, dcc_ldc_ub96x_uyvytestpat, num_bytes);
    }
    else if (0 == strcmp(sensor_name, GW_AR0233_UYVY))
    {
        memcpy(dcc_buf, dcc_ldc_gw_ar0233, num_bytes);
    }
    else
    {
        switch(wdr_mode)
        {
            case 0:
                /*Linear mode*/
                if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                    memcpy(dcc_buf, dcc_ldc_imx390, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                    memcpy(dcc_buf, dcc_ldc_imx390_discovery_mid, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                    printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                    memcpy(dcc_buf, dcc_ldc_imx390_discovery_narrow, num_bytes);
                }
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                    memcpy(dcc_buf, dcc_ldc_imx390_discovery_wide, num_bytes);                    
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                    memcpy(dcc_buf, dcc_ldc_ar0233_linear, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                    memcpy(dcc_buf, dcc_ldc_ar0820_linear, num_bytes);
                else if (0 == strcmp(sensor_name, UB9XX_RAW_TESTPAT))
                    memcpy(dcc_buf, dcc_ldc_ub9xx_raw_test_pattern_linear, num_bytes);
#ifdef VPAC3L
                else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                    memcpy(dcc_buf, dcc_ldc_ov2312, num_bytes);
#endif
                else
                    return -1;
                break;
            case 1:
                /*WDR mode*/
                if(0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_D3))
                    memcpy(dcc_buf, dcc_ldc_imx390_wdr, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_MID))
                    memcpy(dcc_buf, dcc_ldc_imx390_discovery_mid_wdr, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_NARROW)){
                    printf("********************imx390 Discovery %s [%d]********************\n", __func__, __LINE__);
                    memcpy(dcc_buf, dcc_ldc_imx390_discovery_narrow_wdr, num_bytes);
                }
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX390_UB953_DISCOVERY_WIDE))
                    memcpy(dcc_buf, dcc_ldc_imx390_discovery_wide_wdr, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_SONY_IMX728_UB971_SONY))
                    memcpy(dcc_buf, dcc_ldc_imx728_wdr, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0233_UB953_MARS))
                    memcpy(dcc_buf, dcc_ldc_ar0233_wdr, num_bytes);
                else if (0 == strcmp(sensor_name, SENSOR_ONSEMI_AR0820_UB953_LI))
                    memcpy(dcc_buf, dcc_ldc_ar0820_wdr, num_bytes);
#ifdef VPAC3L
                else if (0 == strcmp(sensor_name, SENSOR_OV2312_UB953_LI))
                    memcpy(dcc_buf, dcc_ldc_ov2312, num_bytes);
#endif
                else
                    return -1;
                break;
            default:
                /*Unsupported mode*/
                return -1;
        }
    }

    return 0;
}

int32_t appIssGetResizeParams(uint16_t in_width, uint16_t in_height, uint16_t tgt_width, uint16_t tgt_height, uint16_t * out_width, uint16_t * out_height)
{
    uint32_t tmp = 0;
    uint32_t rsz_ratio = 1024;
    uint32_t rsz_ratio_x = (tgt_width*1024)/in_width;
    uint32_t rsz_ratio_y = (tgt_height*1024)/in_height;

    /*Assuming downscale. Will pick the ratio which has a bigger downscale factor*/
    /*X and Y dimensions will be scaled by same factor to maintain aspect ratio*/
    rsz_ratio = rsz_ratio_x < rsz_ratio_y ? rsz_ratio_x : rsz_ratio_y;
    tmp = (in_width*rsz_ratio)/1024;
    *out_width = (tmp>>3)<<3;/*Align to 8x*/
    tmp = (in_height*rsz_ratio)/1024;
    *out_height = (tmp>>3)<<3;/*Align to 8x*/

    return 0;
}

/*
Checks if the plugin is relevant to VISS. Returns
    0, if False
    1, if True
*/
uint8_t is_viss_plugin(uint32_t plugin_id)
{
    switch (plugin_id)
    {
        case DCC_ID_H3A_AEWB_CFG:
        case DCC_ID_H3A_MUX_LUTS:
        case DCC_ID_RFE_DECOMPAND:
        case DCC_ID_IPIPE_RGB_RGB_1:
        case DCC_ID_NSF4:
        case DCC_ID_BLACK_CLAMP:
        case DCC_ID_IPIPE_CFA:
        case DCC_ID_VISS_GLBCE:
        case DCC_ID_VISS_LSC:
        case DCC_ID_VISS_DPC:
        case DCC_ID_VISS_YEE:
        case DCC_ID_RAWFE_WB1_VS:
        case DCC_ID_VISS_GAMMA:
#ifdef VPAC3
        case DCC_ID_VISS_CFAI3_A:
        case DCC_ID_VISS_CFAI3_B:
        case DCC_ID_VISS_CAC:
        case DCC_ID_VISS_RAWHIST:
        case DCC_ID_VISS_CCMV:
#endif
#ifdef VPAC3L
        case DCC_ID_VISS_CFAI3_A:
        case DCC_ID_VISS_CAC:
        case DCC_ID_VISS_RAWHIST:
        case DCC_ID_VISS_DPC_EXT:
        case DCC_ID_VISS_LSC_EXT:
        case DCC_ID_VISS_PCID:
#endif
            return 1U;
            break;
        default:
            return 0;
    }
    return 0;
}

/*
Checks if the plugin is relevant to AEWB. Returns
    0, if False
    1, if True
*/
uint8_t is_aewb_plugin(uint32_t plugin_id)
{
    switch (plugin_id)
    {
        case DCC_ID_H3A_AEWB_CFG:
        case DCC_ID_AAA_ALG_AWB_TI3:
            return 1U;
            break;
        default:
            return 0;
    }
    return 0;
}

/*
Checks if the plugin is relevant to LDC. Returns
    0, if False
    1, if True
*/
uint8_t is_ldc_plugin(uint32_t plugin_id)
{
    switch (plugin_id)
    {
        case DCC_ID_MESH_LDC_J7:
            return 1U;
            break;
        default:
            return 0;
    }
    return 0;
}

int32_t appSplitVpacDcc(uint8_t *dcc_buf_in, uint32_t prmSize,
                        uint8_t ** dcc_buf_viss, uint32_t *dcc_buf_viss_num_bytes,
                        uint8_t ** dcc_buf_aewb, uint32_t *dcc_buf_aewb_num_bytes,
                        uint8_t ** dcc_buf_ldc, uint32_t *dcc_buf_ldc_num_bytes)
{
    int32_t status = 0;
    uint8_t *tmp_buf;
    uint32_t offset_2a = 0;
    uint32_t offset_viss = 0;
    uint32_t offset_ldc = 0;
    uint8_t *pluginBuf;
    dcc_component_header_type     *header_data;
    uint32_t offset = 0;

    tmp_buf = malloc(prmSize);
    if(NULL == tmp_buf)
    {
        printf("Failed to allocate %d bytes for dcc_buf_2a \n", prmSize);
        return -1;
    }
    else
    {
        *dcc_buf_aewb = tmp_buf;
        tmp_buf = NULL;
    }

    tmp_buf = malloc(prmSize);
    if(NULL == tmp_buf)
    {
        printf("Failed to allocate %d bytes for dcc_buf_viss \n", prmSize);
        free(*dcc_buf_aewb);
        return -1;
    }
    else
    {
        *dcc_buf_viss = tmp_buf;
        tmp_buf = NULL;
    }

    tmp_buf = malloc(prmSize);
    if(NULL == tmp_buf)
    {
        printf("Failed to allocate %d bytes for dcc_buf_ldc \n", prmSize);
        free(*dcc_buf_aewb);
        free(*dcc_buf_viss);
        return -1;
    }
    else
    {
        *dcc_buf_ldc = tmp_buf;
        tmp_buf = NULL;
    }

    pluginBuf = dcc_buf_in;

    while (offset < prmSize)
    {
        header_data = (dcc_component_header_type*)pluginBuf;

        if(1U == is_viss_plugin(header_data->dcc_descriptor_id))
        {
            memcpy(*dcc_buf_viss + offset_viss, pluginBuf, header_data->total_file_sz);
            offset_viss += header_data->total_file_sz;
        }

        if(1U == is_aewb_plugin(header_data->dcc_descriptor_id))
        {
            memcpy(*dcc_buf_aewb + offset_2a, pluginBuf, header_data->total_file_sz);
            offset_2a += header_data->total_file_sz;
        }

        if(1U == is_ldc_plugin(header_data->dcc_descriptor_id))
        {
            memcpy(*dcc_buf_ldc + offset_ldc, pluginBuf, header_data->total_file_sz);
            offset_ldc += header_data->total_file_sz;
        }

        pluginBuf += header_data->total_file_sz;
        offset += header_data->total_file_sz;
    }

    *dcc_buf_viss_num_bytes = offset_viss;
    *dcc_buf_ldc_num_bytes = offset_ldc;
    *dcc_buf_aewb_num_bytes = offset_2a;
    return status;
}

