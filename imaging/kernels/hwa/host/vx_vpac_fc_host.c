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
#ifdef BUILD_VPAC_FC

#include "TI/tivx.h"
#include "TI/hwa_vpac_viss.h"
#include "tivx_hwa_kernels.h"
#include "tivx_kernel_vpac_viss.h"
#include "TI/hwa_vpac_msc.h"
#include "tivx_kernel_vpac_msc.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_hwa_host_priv.h"
#include "TI/hwa_vpac_fc.h"
#include "tivx_kernel_vpac_fc.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static vx_status VX_CALLBACK tivxAddKernelVpacFcValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[]);
static vx_status VX_CALLBACK tivxAddKernelVpacFcInitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params);
static vx_status VX_CALLBACK tivxAddKernelVpacFcDeinitialize(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num_params);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static vx_kernel vx_vpac_fc_kernel = NULL;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static vx_status VX_CALLBACK tivxAddKernelVpacFcValidate(vx_node node,
            const vx_reference parameters[ ],
            vx_uint32 num,
            vx_meta_format metas[])
{
    vx_status status = (vx_status)VX_SUCCESS;

    vx_user_data_object configuration = NULL;
    vx_char configuration_name[VX_MAX_REFERENCE_NAME] = {0};
    vx_size configuration_size = 0;
    tivx_vpac_fc_viss_msc_params_t tivxFcParams;

    tivx_raw_image viss_raw = NULL;
    vx_uint32 viss_raw_w = 0, viss_raw_h = 0;

    vx_image viss_out0 = NULL;
    vx_image viss_out1 = NULL;
    vx_image viss_out2 = NULL;
    vx_image viss_out3 = NULL;

    vx_image msc_out0 = NULL;
    vx_df_image msc_out0_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out0_w = 0, msc_out0_h = 0;

    vx_image msc_out1 = NULL;
    vx_df_image msc_out1_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out1_w = 0, msc_out1_h = 0;

    vx_image msc_out2 = NULL;
    vx_df_image msc_out2_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out2_w = 0, msc_out2_h = 0;

    vx_image msc_out3 = NULL;
    vx_df_image msc_out3_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out3_w = 0, msc_out3_h = 0;

    vx_image msc_out4 = NULL;
    vx_df_image msc_out4_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out4_w = 0, msc_out4_h = 0;

    vx_image msc_out5 = NULL;
    vx_df_image msc_out5_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out5_w = 0, msc_out5_h = 0;

    vx_image msc_out6 = NULL;
    vx_df_image msc_out6_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out6_w = 0, msc_out6_h = 0;

    vx_image msc_out7 = NULL;
    vx_df_image msc_out7_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out7_w = 0, msc_out7_h = 0;

    vx_image msc_out8 = NULL;
    vx_df_image msc_out8_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out8_w = 0, msc_out8_h = 0;

    vx_image msc_out9 = NULL;
    vx_df_image msc_out9_fmt = VX_DF_IMAGE_VIRT;
    vx_uint32 msc_out9_w = 0, msc_out9_h = 0;

    vx_user_data_object ae_awb_result = NULL;
    vx_char ae_awb_result_name[VX_MAX_REFERENCE_NAME] = {0};
    vx_size ae_awb_result_size = 0;
    tivx_ae_awb_params_t ae_awb_params;
    vx_user_data_object dcc_buffer = NULL;
    vx_user_data_object h3a_out = NULL;
    vx_distribution hist0_out = NULL;
    vx_distribution hist1_out = NULL;
    vx_distribution raw_hist_out = NULL;
    vx_char h3a_out_name[VX_MAX_REFERENCE_NAME] = {0};
    vx_size h3a_out_size = 0;

    vx_char dcc_buffer_name[VX_MAX_REFERENCE_NAME] = {0};
    vx_size dcc_buffer_size = 0;

    if ((num != TIVX_KERNEL_VPAC_FC_MAX_PARAMS) ||
        (NULL == parameters[TIVX_KERNEL_VPAC_FC_CONFIGURATION_IDX]) ||
        (NULL == parameters[TIVX_KERNEL_VPAC_FC_VISS_RAW_IDX]))
    {
        status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }

    if ((vx_status)VX_SUCCESS == status)
    {
        configuration = (vx_user_data_object)parameters[TIVX_KERNEL_VPAC_FC_CONFIGURATION_IDX];
        ae_awb_result = (vx_user_data_object)parameters[TIVX_KERNEL_VPAC_FC_VISS_AE_AWB_RESULT_IDX];
        dcc_buffer = (vx_user_data_object)parameters[TIVX_KERNEL_VPAC_FC_DCC_BUF_IDX];
        viss_raw = (tivx_raw_image)parameters[TIVX_KERNEL_VPAC_FC_VISS_RAW_IDX];
        viss_out0 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_VISS_OUT0_IDX];
        viss_out1 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_VISS_OUT1_IDX];
        viss_out2 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_VISS_OUT2_IDX];
        viss_out3 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_VISS_OUT3_IDX];
        h3a_out = (vx_user_data_object)parameters[TIVX_KERNEL_VPAC_FC_VISS_H3A_AEW_AF_IDX];
        hist0_out = (vx_distribution)parameters[TIVX_KERNEL_VPAC_FC_VISS_HISTOGRAM0_IDX];
        hist1_out = (vx_distribution)parameters[TIVX_KERNEL_VPAC_FC_VISS_HISTOGRAM1_IDX];
        raw_hist_out = (vx_distribution)parameters[TIVX_KERNEL_VPAC_FC_VISS_RAW_HISTOGRAM_IDX];
        msc_out0 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT0_IMG_IDX];
        msc_out1 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT1_IMG_IDX];
        msc_out2 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT2_IMG_IDX];
        msc_out3 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT3_IMG_IDX];
        msc_out4 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT4_IMG_IDX];
        msc_out5 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT5_IMG_IDX];
        msc_out6 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT6_IMG_IDX];
        msc_out7 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT7_IMG_IDX];
        msc_out8 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT8_IMG_IDX];
        msc_out9 = (vx_image)parameters[TIVX_KERNEL_VPAC_FC_MSC_SCALE_OUT9_IMG_IDX];
    }

    /* PARAMETER ATTRIBUTE FETCH */
    if ((vx_status)VX_SUCCESS == status)
    {
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, (vx_enum)VX_USER_DATA_OBJECT_NAME, &configuration_name, sizeof(configuration_name)));
        tivxCheckStatus(&status, vxQueryUserDataObject(configuration, (vx_enum)VX_USER_DATA_OBJECT_SIZE, &configuration_size, sizeof(configuration_size)));

        tivxCheckStatus(&status, tivxQueryRawImage(viss_raw, (vx_enum)TIVX_RAW_IMAGE_WIDTH, &viss_raw_w, sizeof(viss_raw_w)));
        tivxCheckStatus(&status, tivxQueryRawImage(viss_raw, (vx_enum)TIVX_RAW_IMAGE_HEIGHT, &viss_raw_h, sizeof(viss_raw_h)));

         /* Disable viss_out0-3 as they are not supported */
        if (NULL != viss_out0 || NULL != viss_out1 || NULL != viss_out2 || NULL != viss_out3)
        {
            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'viss_out0', 'viss_out1', 'viss_out2', and 'viss_out3' are not supported and should be NULL\n");
        }

        if (NULL != msc_out0)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out0, (vx_enum)VX_IMAGE_FORMAT, &msc_out0_fmt, sizeof(msc_out0_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out0, (vx_enum)VX_IMAGE_WIDTH, &msc_out0_w, sizeof(msc_out0_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out0, (vx_enum)VX_IMAGE_HEIGHT, &msc_out0_h, sizeof(msc_out0_h)));
        }

        if (NULL != msc_out1)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out1, (vx_enum)VX_IMAGE_FORMAT, &msc_out1_fmt, sizeof(msc_out1_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out1, (vx_enum)VX_IMAGE_WIDTH, &msc_out1_w, sizeof(msc_out1_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out1, (vx_enum)VX_IMAGE_HEIGHT, &msc_out1_h, sizeof(msc_out1_h)));
        }

        if (NULL != msc_out2)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out2, (vx_enum)VX_IMAGE_FORMAT, &msc_out2_fmt, sizeof(msc_out2_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out2, (vx_enum)VX_IMAGE_WIDTH, &msc_out2_w, sizeof(msc_out2_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out2, (vx_enum)VX_IMAGE_HEIGHT, &msc_out2_h, sizeof(msc_out2_h)));
        }

        if (NULL != msc_out3)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out3, (vx_enum)VX_IMAGE_FORMAT, &msc_out3_fmt, sizeof(msc_out3_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out3, (vx_enum)VX_IMAGE_WIDTH, &msc_out3_w, sizeof(msc_out3_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out3, (vx_enum)VX_IMAGE_HEIGHT, &msc_out3_h, sizeof(msc_out3_h)));
        }

        if (NULL != msc_out4)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out4, (vx_enum)VX_IMAGE_FORMAT, &msc_out4_fmt, sizeof(msc_out4_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out4, (vx_enum)VX_IMAGE_WIDTH, &msc_out4_w, sizeof(msc_out4_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out4, (vx_enum)VX_IMAGE_HEIGHT, &msc_out4_h, sizeof(msc_out4_h)));
        }
        if (NULL != msc_out5)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out5, (vx_enum)VX_IMAGE_FORMAT, &msc_out5_fmt, sizeof(msc_out5_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out5, (vx_enum)VX_IMAGE_WIDTH, &msc_out5_w, sizeof(msc_out5_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out5, (vx_enum)VX_IMAGE_HEIGHT, &msc_out5_h, sizeof(msc_out5_h)));
        }

        if (NULL != msc_out6)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out6, (vx_enum)VX_IMAGE_FORMAT, &msc_out6_fmt, sizeof(msc_out6_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out6, (vx_enum)VX_IMAGE_WIDTH, &msc_out6_w, sizeof(msc_out6_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out6, (vx_enum)VX_IMAGE_HEIGHT, &msc_out6_h, sizeof(msc_out6_h)));
        }

        if (NULL != msc_out7)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out7, (vx_enum)VX_IMAGE_FORMAT, &msc_out7_fmt, sizeof(msc_out7_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out7, (vx_enum)VX_IMAGE_WIDTH, &msc_out7_w, sizeof(msc_out7_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out7, (vx_enum)VX_IMAGE_HEIGHT, &msc_out7_h, sizeof(msc_out7_h)));
        }

        if (NULL != msc_out8)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out8, (vx_enum)VX_IMAGE_FORMAT, &msc_out8_fmt, sizeof(msc_out8_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out8, (vx_enum)VX_IMAGE_WIDTH, &msc_out8_w, sizeof(msc_out8_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out8, (vx_enum)VX_IMAGE_HEIGHT, &msc_out8_h, sizeof(msc_out8_h)));
        }

        if (NULL != msc_out9)
        {
            tivxCheckStatus(&status, vxQueryImage(msc_out9, (vx_enum)VX_IMAGE_FORMAT, &msc_out9_fmt, sizeof(msc_out9_fmt)));
            tivxCheckStatus(&status, vxQueryImage(msc_out9, (vx_enum)VX_IMAGE_WIDTH, &msc_out9_w, sizeof(msc_out9_w)));
            tivxCheckStatus(&status, vxQueryImage(msc_out9, (vx_enum)VX_IMAGE_HEIGHT, &msc_out9_h, sizeof(msc_out9_h)));
        }
        if (NULL != ae_awb_result)
        {
            tivxCheckStatus(&status, vxQueryUserDataObject(ae_awb_result, (vx_enum)VX_USER_DATA_OBJECT_SIZE, &ae_awb_result_size, sizeof(ae_awb_result_size)));
            tivxCheckStatus(&status, vxQueryUserDataObject(ae_awb_result, (vx_enum)VX_USER_DATA_OBJECT_NAME, &ae_awb_result_name, sizeof(ae_awb_result_name)));
        }

        if (NULL != dcc_buffer)
        {
            tivxCheckStatus(&status, vxQueryUserDataObject(dcc_buffer, (vx_enum)VX_USER_DATA_OBJECT_SIZE, &dcc_buffer_size, sizeof(dcc_buffer_size)));
            tivxCheckStatus(&status, vxQueryUserDataObject(dcc_buffer, (vx_enum)VX_USER_DATA_OBJECT_NAME, &dcc_buffer_name, sizeof(dcc_buffer_name)));
        }
        if (NULL != h3a_out)
        {
            tivxCheckStatus(&status, vxQueryUserDataObject(h3a_out, (vx_enum)VX_USER_DATA_OBJECT_SIZE, &h3a_out_size, sizeof(h3a_out_size)));
            tivxCheckStatus(&status, vxQueryUserDataObject(h3a_out, (vx_enum)VX_USER_DATA_OBJECT_NAME, &h3a_out_name, sizeof(h3a_out_name)));
        }
    }

    /* PARAMETER CHECKING */
    if ((vx_status)VX_SUCCESS == status)
    {
         /* Validate configuration user data object */
        if (NULL != configuration)
        {
            if ((configuration_size != sizeof(tivx_vpac_fc_viss_msc_params_t)) ||
                (strncmp(configuration_name, "tivx_vpac_fc_viss_msc_params_t", sizeof(configuration_name)) != 0))
            {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "'configuration' should be a user_data_object of type:\n tivx_vpac_fc_viss_msc_params_t\n");
            }
        }

        if ((vx_status)VX_SUCCESS == status)
        {
            status = vxCopyUserDataObject(configuration, 0, sizeof(tivx_vpac_fc_viss_msc_params_t), &tivxFcParams, (vx_enum)VX_READ_ONLY, (vx_enum)VX_MEMORY_TYPE_HOST);
            if ((vx_status)VX_SUCCESS != status)
            {
            VX_PRINT(VX_ZONE_ERROR, "'configuration' user data object has not been populated by application\n");
            }
        }
        if ((vx_status)VX_SUCCESS == status)
        {
            if (NULL != ae_awb_result)
            {
                if ((ae_awb_result_size != sizeof(tivx_ae_awb_params_t)) ||
                    (strncmp(ae_awb_result_name, "tivx_ae_awb_params_t", sizeof(ae_awb_result_name)) != 0))
                {
                    status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                    VX_PRINT(VX_ZONE_ERROR, "'ae_awb_result' should be a user_data_object of type:\n tivx_ae_awb_params_t \n");
                }
            }
        }

        if ((vx_status)VX_SUCCESS == status)
        {
            if (NULL != ae_awb_result)
            {
                status = vxCopyUserDataObject(ae_awb_result, 0, sizeof(tivx_ae_awb_params_t), &ae_awb_params, (vx_enum)VX_READ_ONLY, (vx_enum)VX_MEMORY_TYPE_HOST);
                if ((vx_status)VX_SUCCESS != status)
                {
                    VX_PRINT(VX_ZONE_ERROR, "'ae_awb_result' user data object has not been populated by application\n");
                }
            }
        }
    }

     /* Validate raw image dimensions */
    if ((vx_status)VX_SUCCESS == status)
    {
        if (NULL != viss_raw)
        {
            if ((viss_raw_w == 0) || (viss_raw_h == 0))
            {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "'viss_raw' dimensions should be non-zero\n");
            }
        }
    }

     /* Disable histogram outputs as they are not supported */
    if ((vx_status)VX_SUCCESS == status)
    {
        if (NULL != hist0_out || NULL != hist1_out || NULL != raw_hist_out)
        {
            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'hist0_out', 'hist1_out', and 'raw_hist_out' are not supported for flexconnect kernel at the moment and should be NULL\n");
        }
    }

#if !defined (VPAC3) && !defined (VPAC3L)
    if ((vx_status)VX_SUCCESS == status)
    {
        if (NULL != hist1_out)
        {
            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'hist1_out' should be NULL for this SoC \n");
        }
        if (NULL != raw_hist_out)
        {
            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "'raw_hist_out' should be NULL for this SoC \n");
        }
    }
#endif

    if ((vx_status)VX_SUCCESS == status)
    {
        if (NULL != h3a_out)
        {
            if ((h3a_out_size != sizeof(tivx_h3a_data_t)) ||
            (strncmp(h3a_out_name, "tivx_h3a_data_t", sizeof(h3a_out_name)) != 0))
            {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "'h3a_out' should be a user_data_object of type:\n tivx_h3a_data_t \n");
            }
        }
    }

    if ((vx_status)VX_SUCCESS == status)
    {
        if (NULL != dcc_buffer)
        {
            if ((dcc_buffer_size < 1U) ||
            (strncmp(dcc_buffer_name, "dcc_viss", sizeof(dcc_buffer_name)) != 0))
            {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "'dcc_buffer' should be a user_data_object of name:\n dcc_viss \n");
            }
        }
    }

    if ((vx_status)VX_SUCCESS == status)
    {
         /* Validate the format of each MSC output image */
        vx_df_image valid_formats[] = {VX_DF_IMAGE_U8, VX_DF_IMAGE_U16, TIVX_DF_IMAGE_P12, VX_DF_IMAGE_NV12, TIVX_DF_IMAGE_NV12_P12, VX_DF_IMAGE_UYVY, VX_DF_IMAGE_YUYV};
        vx_uint32 num_valid_formats = sizeof(valid_formats) / sizeof(valid_formats[0]);

        for (vx_uint32 i = 0; i <= 9; i++)
        {
            vx_image msc_out = NULL;
            vx_df_image msc_out_fmt;

            switch (i)
            {
                case 0: msc_out = msc_out0; break;
                case 1: msc_out = msc_out1; break;
                case 2: msc_out = msc_out2; break;
                case 3: msc_out = msc_out3; break;
                case 4: msc_out = msc_out4; break;
                case 5: msc_out = msc_out5; break;
                case 6: msc_out = msc_out6; break;
                case 7: msc_out = msc_out7; break;
                case 8: msc_out = msc_out8; break;
                case 9: msc_out = msc_out9; break;
            }

            if (NULL != msc_out)
            {
                tivxCheckStatus(&status, vxQueryImage(msc_out, (vx_enum)VX_IMAGE_FORMAT, &msc_out_fmt, sizeof(msc_out_fmt)));

                vx_bool is_valid_format = vx_false_e;
                for (vx_uint32 j = 0; j < num_valid_formats; j++)
                {
                    if (msc_out_fmt == valid_formats[j])
                    {
                        is_valid_format = vx_true_e;
                        break;
                    }
                }

                if (!is_valid_format)
                {
                    status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                    VX_PRINT(VX_ZONE_ERROR, "'msc_out%d' should be an image of type:\n VX_DF_IMAGE_U8, VX_DF_IMAGE_U16, or TIVX_DF_IMAGE_P12\n", i);
                }
            }
        }
    }

        /* All MSC outputs should be one single format, cannot support mixed formats in outputs */
        if ((vx_status)VX_SUCCESS == status)
        {
            vx_df_image reference_format = VX_DF_IMAGE_VIRT;

            if (NULL != msc_out0)
            {
            reference_format = msc_out0_fmt;
            }

            for (vx_uint32 i = 1; i <= 9; i++)
            {
                vx_image msc_out = NULL;
                vx_df_image msc_out_fmt = VX_DF_IMAGE_VIRT;

                switch (i)
                {
                case 1: msc_out = msc_out1; msc_out_fmt = msc_out1_fmt; break;
                case 2: msc_out = msc_out2; msc_out_fmt = msc_out2_fmt; break;
                case 3: msc_out = msc_out3; msc_out_fmt = msc_out3_fmt; break;
                case 4: msc_out = msc_out4; msc_out_fmt = msc_out4_fmt; break;
                case 5: msc_out = msc_out5; msc_out_fmt = msc_out5_fmt; break;
                case 6: msc_out = msc_out6; msc_out_fmt = msc_out6_fmt; break;
                case 7: msc_out = msc_out7; msc_out_fmt = msc_out7_fmt; break;
                case 8: msc_out = msc_out8; msc_out_fmt = msc_out8_fmt; break;
                case 9: msc_out = msc_out9; msc_out_fmt = msc_out9_fmt; break;
                }

                if (NULL != msc_out && msc_out_fmt != reference_format)
                {
                    status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                    VX_PRINT(VX_ZONE_ERROR, "All MSC outputs should have the same format, except for NULL outputs\n");
                    break;
                }
            }
        }

#if defined (VPAC3) || defined (VPAC3L)
        /* Verify mapping of msc_out with msc_out_msc_in_map
        *  1. If outputs are of format VX_DF_IMAGE_NV12, TIVX_DF_IMAGE_NV12_P12, VX_DF_IMAGE_UYVY, VX_DF_IMAGE_YUYV then; alternate msc outputs would be disabled but 1 msc out will map to 2 params.msc_out_msc_in_map
        *  2. If outputs are of format VX_DF_IMAGE_U8, VX_DF_IMAGE_U16, TIVX_DF_IMAGE_P12 then; direct 1:1 mapping of msc_out to params.msc_out_msc_in_map
        */
        if ((vx_status)VX_SUCCESS == status)
        {
            /*  Verify mapping of msc_out with msc_out_msc_in_map for condition 1 */
            for (vx_uint32 i = 0; i <= 9; i += 2)
            {
                vx_image msc_out = NULL;
                vx_image msc_out_next = NULL;
                vx_df_image msc_out_fmt = VX_DF_IMAGE_VIRT;

                switch (i)
                {
                case 0: msc_out = msc_out0; msc_out_next = msc_out1; break;
                case 2: msc_out = msc_out2; msc_out_next = msc_out3; break;
                case 4: msc_out = msc_out4; msc_out_next = msc_out5; break;
                case 6: msc_out = msc_out6; msc_out_next = msc_out7; break;
                case 8: msc_out = msc_out8; msc_out_next = msc_out9; break;
                }

                if (NULL != msc_out)
                {
                    tivxCheckStatus(&status, vxQueryImage(msc_out, (vx_enum)VX_IMAGE_FORMAT, &msc_out_fmt, sizeof(msc_out_fmt)));

                    if ((msc_out_fmt == VX_DF_IMAGE_NV12) || 
                    (msc_out_fmt == TIVX_DF_IMAGE_NV12_P12) || 
                    (msc_out_fmt == VX_DF_IMAGE_UYVY) || 
                    (msc_out_fmt == VX_DF_IMAGE_YUYV))
                    {
                        if (NULL != msc_out_next)
                        {
                            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                            VX_PRINT(VX_ZONE_ERROR, "Alternate MSC outputs should be disabled for formats NV12, NV12_P12, UYVY, or YUYV\n");
                        }

                         /* Check msc_out_msc_in_map logic for condition 1 */
                        if (tivxFcParams.msc_out_msc_in_map[i] != tivxFcParams.msc_out_msc_in_map[i + 1])
                        {
                            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                            VX_PRINT(VX_ZONE_ERROR, "For condition 1, msc_out_msc_in_map[%d] and msc_out_msc_in_map[%d] should map to the same input\n", i, i + 1);
                        }
                    }
                     /* Verify mapping of msc_out with msc_out_msc_in_map for condition 2 */
                    if ((msc_out_fmt == VX_DF_IMAGE_U8) || 
                    (msc_out_fmt == VX_DF_IMAGE_U16) || 
                    (msc_out_fmt == TIVX_DF_IMAGE_P12))
                    {
                        if (NULL != msc_out_next)
                        {
                            vx_df_image msc_out_next_fmt = VX_DF_IMAGE_VIRT;
                            tivxCheckStatus(&status, vxQueryImage(msc_out_next, (vx_enum)VX_IMAGE_FORMAT, &msc_out_next_fmt, sizeof(msc_out_next_fmt)));

                            if (msc_out_fmt != msc_out_next_fmt)
                            {
                            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                            VX_PRINT(VX_ZONE_ERROR, "All MSC outputs should have the same format for condition 2\n");
                            }
                        }
                    }
                }
            }
        }
#endif

#if defined(VPAC3L)
        /* For VPAC3L devices only, if msc_in_thread_viss_out_map is enabled with TIVX_VPAC_FC_VISS_OUT2 or TIVX_VPAC_FC_VISS_OUT3 , the MSC out format can only be VX_DF_IMAGE_U8, TIVX_DF_IMAGE_NV12_P12, TIVX_DF_IMAGE_P12  */
        if ((vx_status)VX_SUCCESS == status)
        {
            for (vx_uint32 i = 0; i < TIVX_KERNEL_VPAC_FC_MAX_MSC_INPUT_THREADS; i++)
            {
                if ((tivxFcParams.msc_in_thread_viss_out_map[i] == TIVX_VPAC_FC_VISS_OUT2) ||
                    (tivxFcParams.msc_in_thread_viss_out_map[i] == TIVX_VPAC_FC_VISS_OUT3))
                {
                    if ((msc_out0_fmt != VX_DF_IMAGE_U8) && (msc_out0_fmt != TIVX_DF_IMAGE_NV12_P12) && (msc_out0_fmt != TIVX_DF_IMAGE_P12))
                    {
                        status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                        VX_PRINT(VX_ZONE_ERROR, "For VPAC3L devices, msc_in_thread_viss_out_map[%d] should be set to VX_DF_IMAGE_U8, TIVX_DF_IMAGE_NV12_P12, or TIVX_DF_IMAGE_P12\n", i);
                    }
                }
            }
        }

        /* For VPAC3L devices only allow MSC0 to be used in msc_out_msc_in_map */
        if ((vx_status)VX_SUCCESS == status)
        {
            for (vx_uint32 i = 0; i < TIVX_VPAC_MSC_MAX_OUTPUT; i++)
            {
                if (tivxFcParams.msc_out_msc_in_map[i] != TIVX_VPAC_FC_MSC0 &&
                    tivxFcParams.msc_out_msc_in_map[i] != TIVX_VPAC_FC_MSC_TH_INVALID)
                {
                    status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                    VX_PRINT(VX_ZONE_ERROR, "For VPAC3L devices, msc_out_msc_in_map[%d] should be set to TIVX_VPAC_FC_MSC0 or TIVX_VPAC_FC_MSC_TH_INVALID\n", i);
                }
            }
        }
#endif

#if !defined (VPAC3) && !defined (VPAC3L)
        /* Verify mapping of msc_out with msc_out_msc_in_map
        *   1. For VPAC1 only U8 or U16 is supported at the moment, therefore only msc_out0, msc_out2 ... msc_out8 should be enabled. Only even positions of msc_out_msc_in_map
        */
        /*  Verify mapping of msc_out with msc_out_msc_in_map for VPAC1 */
        if ((vx_status)VX_SUCCESS == status)
        {
            for (vx_uint32 i = 0; i <= 9; i++)
            {
            vx_image msc_out = NULL;

            switch (i)
            {
            case 0: msc_out = msc_out0; break;
            case 2: msc_out = msc_out2; break;
            case 4: msc_out = msc_out4; break;
            case 6: msc_out = msc_out6; break;
            case 8: msc_out = msc_out8; break;
            default: msc_out = NULL; break;
            }

            if (NULL != msc_out)
            {
                vx_df_image msc_out_fmt = VX_DF_IMAGE_VIRT;
                tivxCheckStatus(&status, vxQueryImage(msc_out, (vx_enum)VX_IMAGE_FORMAT, &msc_out_fmt, sizeof(msc_out_fmt)));

                if ((msc_out_fmt != VX_DF_IMAGE_U8) && (msc_out_fmt != VX_DF_IMAGE_U16))
                {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "For VPAC1, only VX_DF_IMAGE_U8 or VX_DF_IMAGE_U16 formats are supported for MSC outputs\n");
                }

                if (tivxFcParams.msc_out_msc_in_map[i] == TIVX_VPAC_FC_MSC_TH_INVALID)
                {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "For VPAC1, even positions in msc_out_msc_in_map must not be set to TIVX_VPAC_FC_MSC_TH_INVALID\n");
                }
            }
            else
            {
                if (i % 2 == 0)
                {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "For VPAC1, only even MSC outputs (msc_out0, msc_out2, ..., msc_out8) should be enabled\n");
                }

                if (tivxFcParams.msc_out_msc_in_map[i] != TIVX_VPAC_FC_MSC_TH_INVALID)
                {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "For VPAC1, odd positions in msc_out_msc_in_map must be set to TIVX_VPAC_FC_MSC_TH_INVALID\n");
                }
            }
            }
        }
#endif

#if defined (VPAC3) || defined (VPAC3L)
        /* At the moment Flexconnect doesn't support mixed combination of MSC0 and MSC1, therefore msc_out_msc_in_map[] should only have all TIVX_VPAC_FC_MSC0 or all TIVX_VPAC_FC_MSC1 or TIVX_VPAC_FC_MSC_TH_INVALID  */
        /*  Check if msc_out_msc_in_map[] contains only TIVX_VPAC_FC_MSC0, TIVX_VPAC_FC_MSC1, or TIVX_VPAC_FC_MSC_TH_INVALID */
        vx_bool is_msc0 = vx_true_e;
        vx_bool is_msc1 = vx_true_e;

        if ((vx_status)VX_SUCCESS == status)
        {
            for (vx_uint32 i = 0; i < TIVX_VPAC_MSC_MAX_OUTPUT; i++)
            {
            if (tivxFcParams.msc_out_msc_in_map[i] != TIVX_VPAC_FC_MSC0 &&
                tivxFcParams.msc_out_msc_in_map[i] != TIVX_VPAC_FC_MSC1 &&
                tivxFcParams.msc_out_msc_in_map[i] != TIVX_VPAC_FC_MSC_TH_INVALID)
            {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "msc_out_msc_in_map[%d] contains an invalid value\n", i);
                break;
            }

            if (tivxFcParams.msc_out_msc_in_map[i] == TIVX_VPAC_FC_MSC0)
            {
                is_msc1 = vx_false_e;
            }
            else if (tivxFcParams.msc_out_msc_in_map[i] == TIVX_VPAC_FC_MSC1)
            {
                is_msc0 = vx_false_e;
            }
            }
        }

        if ((vx_status)VX_SUCCESS == status)
        {
            if (!is_msc0 && !is_msc1)
            {
            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
            VX_PRINT(VX_ZONE_ERROR, "msc_out_msc_in_map[] should only contain all TIVX_VPAC_FC_MSC0 or all TIVX_VPAC_FC_MSC1 or TIVX_VPAC_FC_MSC_TH_INVALID\n");
            }
        }

        /* If MSC0 is enabled, msc_in_thread_viss_out_map[0] and [1]  should only have the VISS_OUT0, 1, 2, 3 values, for MSC1 only msc_in_thread_viss_out_map[2] and msc_in_thread_viss_out_map[3] should have the values
        *   1. If MSC0 is used [2] and [3] should be disabled and [0] and [1] should be disabled while using MSC1
        */
        /*  Validate msc_in_thread_viss_out_map for MSC0 and MSC1 according to condition 1 */
        if ((vx_status)VX_SUCCESS == status)
        {
            for (vx_uint32 i = 0; i < TIVX_KERNEL_VPAC_FC_MAX_MSC_INPUT_THREADS; i++)
            {
                if (vx_true_e == is_msc0)  /* MSC0 */
                {
                    if ((i == 0 || i == 1) && (tivxFcParams.msc_in_thread_viss_out_map[i] == TIVX_VPAC_FC_MSC_CH_INVALID))
                    {
                        status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                        VX_PRINT(VX_ZONE_ERROR, "For MSC0, msc_in_thread_viss_out_map[%d] should have non-invalid values\n", i);
                    }
                }
                else  /* MSC1 */
                {
                    if ((i == 2 || i == 3) && (tivxFcParams.msc_in_thread_viss_out_map[i] == TIVX_VPAC_FC_MSC_CH_INVALID))
                    {
                        status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                        VX_PRINT(VX_ZONE_ERROR, "For MSC1, msc_in_thread_viss_out_map[%d] should have non-invalid values\n", i);
                    }
                }
            }
        }
#endif

#if !defined (VPAC3) && !defined (VPAC3L)
        /* For VPAC1 msc_in_thread_viss_out_map[1] and [3] should always be disabled */
        if ((vx_status)VX_SUCCESS == status)
        {
            for (vx_uint32 i = 0; i < TIVX_KERNEL_VPAC_FC_MAX_MSC_INPUT_THREADS; i++)
            {
            if ((i == 1 || i == 3) && (tivxFcParams.msc_in_thread_viss_out_map[i] != TIVX_VPAC_FC_MSC_CH_INVALID))
            {
                status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
                VX_PRINT(VX_ZONE_ERROR, "For VPAC1, msc_in_thread_viss_out_map[%d] should always be set to TIVX_VPAC_FC_MSC_CH_INVALID\n", i);
            }
            }
        }
#endif

    return status;
}

static vx_status VX_CALLBACK tivxAddKernelVpacFcInitialize(vx_node node,
            const vx_reference parameters[],
            vx_uint32 num_params)
{
    vx_status status = (vx_status)VX_SUCCESS;

    if ((num_params != TIVX_KERNEL_VPAC_FC_MAX_PARAMS) ||
        (NULL == parameters[TIVX_KERNEL_VPAC_FC_CONFIGURATION_IDX]) ||
        (NULL == parameters[TIVX_KERNEL_VPAC_FC_VISS_RAW_IDX]))
    {
        status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
        VX_PRINT(VX_ZONE_ERROR, "One or more REQUIRED parameters are set to NULL\n");
    }

    if ((vx_status)VX_SUCCESS == status)
    {
        tivxLogRtTraceKernelInstanceAddEvent(node, TIVX_KERNEL_VPAC_FC_RT_TRACE_OFFSET_HWA, "FC_HWA");
    }

    return status;
}

static vx_status VX_CALLBACK tivxAddKernelVpacFcDeinitialize(vx_node node,
            const vx_reference parameters[],
            vx_uint32 num_params)
{
    vx_status status = (vx_status)VX_SUCCESS;

    tivxLogRtTraceKernelInstanceRemoveEvent(node, TIVX_KERNEL_VPAC_FC_RT_TRACE_OFFSET_HWA);

    return status;
}

vx_status tivxAddKernelVpacFc(vx_context context)
{
    vx_kernel kernel;
    vx_status status;
    uint32_t index;
    vx_enum kernel_id;

    status = vxAllocateUserKernelId(context, &kernel_id);
    if(status != (vx_status)VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Unable to allocate user kernel ID\n");
    }

    if (status == (vx_status)VX_SUCCESS)
    {
        kernel = vxAddUserKernel(
                    context,
                    TIVX_KERNEL_VPAC_FC_NAME,
                    kernel_id,
                    NULL,
                    TIVX_KERNEL_VPAC_FC_MAX_PARAMS,
                    tivxAddKernelVpacFcValidate,
                    tivxAddKernelVpacFcInitialize,
                    tivxAddKernelVpacFcDeinitialize);

        status = vxGetStatus((vx_reference)kernel);
    }
    if (status == (vx_status)VX_SUCCESS)
    {
        index = 0;

        {
            /* Configuration */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_INPUT,
                        VX_TYPE_USER_DATA_OBJECT,
                        (vx_enum)VX_PARAMETER_STATE_REQUIRED
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* AE/AEWB Result */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_INPUT,
                        VX_TYPE_USER_DATA_OBJECT,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* DCC Buffer */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_INPUT,
                        VX_TYPE_USER_DATA_OBJECT,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* VISS Input RAW Images */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_INPUT,
                        TIVX_TYPE_RAW_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_REQUIRED
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* VISS OUTPUT 0 */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* VISS OUTPUT 1 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* VISS OUTPUT 2 */

            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* VISS OUTPUT 3 */

            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* H3A Output */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        VX_TYPE_USER_DATA_OBJECT,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* Histogram0 Output */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_DISTRIBUTION,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* Histogram1 Output */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_DISTRIBUTION,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* Raw Histogram Output */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_DISTRIBUTION,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }        
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 0 */
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_REQUIRED
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 1 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 2 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 3 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 4 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 5 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 6 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 7 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 8 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* MSC OUT 9 */            
            status = vxAddParameterToKernel(kernel,
                        index,
                        (vx_enum)VX_OUTPUT,
                        (vx_enum)VX_TYPE_IMAGE,
                        (vx_enum)VX_PARAMETER_STATE_OPTIONAL
            );
            index++;
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            /* add supported target's */
            tivxAddKernelTarget(kernel, TIVX_TARGET_VPAC_FC);
            #if defined(SOC_J784S4) || defined(SOC_J742S2)
            tivxAddKernelTarget(kernel, TIVX_TARGET_VPAC2_FC);
            #endif
        }
        if (status == (vx_status)VX_SUCCESS)
        {
            status = vxFinalizeKernel(kernel);
        }
        if (status != (vx_status)VX_SUCCESS)
        {
            vxReleaseKernel(&kernel);
            kernel = NULL;
        }
    }
    else
    {
        kernel = NULL;
    }
    vx_vpac_fc_kernel = kernel;

    return status;
}

vx_status tivxRemoveKernelVpacFc(vx_context context)
{
    vx_status status;
    vx_kernel kernel = vx_vpac_fc_kernel;

    status = vxRemoveKernel(kernel);
    vx_vpac_fc_kernel = NULL;

    return status;
}

void tivx_vpac_fc_params_init(tivx_vpac_fc_viss_msc_params_t *tivxFcParams)
{
    if (NULL != tivxFcParams)
    {

        uint32_t i;
        memset(tivxFcParams, 0x0, sizeof(tivx_vpac_fc_viss_msc_params_t));

#if defined (VPAC3) || defined (VPAC3L)
        /* Initialize msc_in_thread_viss_out_map */
        tivxFcParams->msc_in_thread_viss_out_map[0] = TIVX_VPAC_FC_VISS_OUT0;
        tivxFcParams->msc_in_thread_viss_out_map[1] = TIVX_VPAC_FC_VISS_OUT1;
        tivxFcParams->msc_in_thread_viss_out_map[0] = TIVX_VPAC_FC_MSC_CH_INVALID;
        tivxFcParams->msc_in_thread_viss_out_map[1] = TIVX_VPAC_FC_MSC_CH_INVALID;

        /* Initialize msc_out_msc_in_map */
        for(i = 0; i < TIVX_VPAC_MSC_MAX_OUTPUT; i++)
        {
            tivxFcParams->msc_out_msc_in_map[i] = TIVX_VPAC_FC_MSC0;
        }
#else
        /* Initialize msc_in_thread_viss_out_map */
        tivxFcParams->msc_in_thread_viss_out_map[0] = TIVX_VPAC_FC_VISS_OUT0;
        tivxFcParams->msc_in_thread_viss_out_map[1] = TIVX_VPAC_FC_MSC_CH_INVALID;
        tivxFcParams->msc_in_thread_viss_out_map[2] = TIVX_VPAC_FC_MSC_CH_INVALID;
        tivxFcParams->msc_in_thread_viss_out_map[3] = TIVX_VPAC_FC_MSC_CH_INVALID;

        /* Initialize msc_out_msc_in_map */
        for(i = 0; i < TIVX_VPAC_MSC_MAX_OUTPUT; i++)
        {
            if (i % 2 == 0)
            {
                tivxFcParams->msc_out_msc_in_map[i] = TIVX_VPAC_FC_MSC0;
            }
            else
            {
                tivxFcParams->msc_out_msc_in_map[i] = TIVX_VPAC_FC_MSC_TH_INVALID;
            }
        }
#endif
    }
}
#endif






