/*
 *
 * Copyright (c) 2019 Texas Instruments Incorporated
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
 * Redistributions must preserve existing copyright prmnotices and reproduce this license
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "TI/tivx.h"
#include "TI/hwa_vpac_nf.h"
#include "tivx_hwa_kernels.h"
#include "tivx_kernel_vpac_nf_generic.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"
#include "tivx_hwa_vpac_nf_priv.h"
#include "TI/tivx_event.h"
#include "TI/tivx_mutex.h"

#include "vhwa/include/vhwa_m2mNf.h"
#include "utils/perf_stats/include/app_perf_stats.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint32_t                            isAlloc;
    tivx_vpac_nf_common_params_t        nfGenericParams;
    Vhwa_M2mNfCreatePrms                createPrms;
    Vhwa_M2mNfConfig                    nf_cfg;
    Nf_WgtTableConfig                   wgtTbl;
    Fvid2_Handle                        handle;
    tivx_event                          waitForProcessCmpl;
    /* Structure for error event parameters
     * Used to register callback for the given set of events.*/
    Nf_ErrEventParams                   errEvtPrms;

    Fvid2_Frame                         inFrm;
    Fvid2_Frame                         outFrm;
    Fvid2_CbParams                      cbPrms;
    Fvid2_FrameList                     inFrmList;
    Fvid2_FrameList                     outFrmList;

    /* Stores the status of error interrupt */
    uint32_t                            err_stat;
    /* Structure for watchdog timer error event parameters
     * Used to register callback for the given set of events.*/
    Nf_WdTimerErrEventParams            wdTimererrEvtPrms;
    /* Stores the status of watchdog timer error interrupt */
    uint32_t                            wdTimerErrStatus;

    /*! Instance ID of the NF driver */
    uint32_t                            nf_drv_inst_id;
    /*! HWA Performance ID */
    app_perf_hwa_id_t                   hwa_perf_id;
    /* Store a 32-bit bitmask used to specify which error events 
     * should be reported */
    uint32_t                            enableErrorEvents;
    /* Timestamp for the most recent frame processed. */
    uint64_t                            timestamp;
} tivxVpacNfGenericObj;

typedef struct
{
    tivx_mutex lock;
    tivxVpacNfGenericObj nfGenericObj[VHWA_M2M_NF_MAX_HANDLES];
} tivxVpacNfGenericInstObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static vx_status VX_CALLBACK tivxVpacNfGenericProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxVpacNfGenericCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxVpacNfGenericDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxVpacNfGenericControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);

static tivxVpacNfGenericObj *tivxVpacNfGenericAllocObject(
       tivxVpacNfGenericInstObj *instObj);
static void tivxVpacNfGenericFreeObject(
       tivxVpacNfGenericInstObj *instObj, tivxVpacNfGenericObj *nf_generic_obj);
static void tivxVpacNfSetFmt(Fvid2_Format *fmt,
    const tivx_obj_desc_image_t *img_desc);
static vx_status tivxVpacNfGenericSetHtsLimitCmd(
    tivxVpacNfGenericObj *nf_generic_obj,
    const tivx_obj_desc_user_data_object_t *usr_data_obj);
static vx_status tivxVpacNfGenericSetCoeff(tivxVpacNfGenericObj *nf_generic_obj,
    const tivx_obj_desc_user_data_object_t *usr_data_obj);
static vx_status tivxVpacNfGenericGetErrStatusCmd(const tivxVpacNfGenericObj *nf_generic_obj,
    tivx_obj_desc_scalar_t *scalar_obj_desc);
static vx_status tivxVpacNfGenericGetPsaStatusCmd(const tivxVpacNfGenericObj *nf_generic_obj,
    tivx_obj_desc_user_data_object_t *usr_data_obj);

int32_t tivxVpacNfGenericFrameComplCb(Fvid2_Handle handle, void *appData);
void tivxVpacNfGenericErrorCb(Fvid2_Handle handle, uint32_t errEvents, void *appData);
static void tivxVpacNfGenericWdTimerErrorCb(Fvid2_Handle handle, uint32_t wdTimerErrEvents, void *appData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static tivx_target_kernel vx_vpac_nf_generic_target_kernel = NULL;

tivxVpacNfGenericInstObj gTivxVpacNfGenericInstObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void tivxAddTargetKernelVpacNfGeneric(void)
{
    vx_status status = (vx_status)VX_FAILURE;
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */  
    if (self_cpu == (vx_enum)TIVX_CPU_ID_VPAC1)
    /* LDRA_JUSTIFY_END */
    {
        strncpy(target_name, TIVX_TARGET_VPAC_NF, TIVX_TARGET_MAX_NAME);
        status = (vx_status)VX_SUCCESS;
    }
    #if defined(SOC_J784S4) || defined(SOC_J742S2)
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */  
    else if (self_cpu == (vx_enum)TIVX_CPU_ID_VPAC2)
    {
        strncpy(target_name, TIVX_TARGET_VPAC2_NF, TIVX_TARGET_MAX_NAME);
        status = (vx_status)VX_SUCCESS;
    }
    /* LDRA_JUSTIFY_END */
    #endif
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */  
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid CPU ID\n");
        status = (vx_status)VX_FAILURE;
    }
    /* LDRA_JUSTIFY_END */

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */  
    if (status == (vx_status)VX_SUCCESS)
    {
    /* LDRA_JUSTIFY_END */
        vx_vpac_nf_generic_target_kernel = tivxAddTargetKernelByName(
                    TIVX_KERNEL_VPAC_NF_GENERIC_NAME,
                    target_name,
                    tivxVpacNfGenericProcess,
                    tivxVpacNfGenericCreate,
                    tivxVpacNfGenericDelete,
                    tivxVpacNfGenericControl,
                    NULL);
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (NULL != vx_vpac_nf_generic_target_kernel)
        /* LDRA_JUSTIFY_END */
        {
            /* Allocate lock mutex */
            status = tivxMutexCreate(&gTivxVpacNfGenericInstObj.lock);
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if ((vx_status)VX_SUCCESS != status)
            {
                VX_PRINT(VX_ZONE_ERROR,
                    "Failed to create Mutex\n");
            }
            else
            /* LDRA_JUSTIFY_END */
            {
                memset(&gTivxVpacNfGenericInstObj.nfGenericObj, 0x0,
                    sizeof(tivxVpacNfGenericObj) * VHWA_M2M_NF_MAX_HANDLES);
            }
        }
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        else
        {
            status = (vx_status)VX_FAILURE;

            /* TODO: how to handle this condition */
            VX_PRINT(VX_ZONE_ERROR,
                "Failed to Add NF Generic TargetKernel\n");
        }
        /* LDRA_JUSTIFY_END */ 
    }
}
/* LDRA_JUSTIFY_START
<metric start> statement branch <metric end>
<justification start> 
Rationale: The component level test framework and test applications cannot reach this portion.
This failure case is out of scope for the imaging test framework.
Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
This behaviour is part of the application design. However, due to the stated rationale, this is not tested.
<justification end> */
void tivxRemoveTargetKernelVpacNfGeneric(void)
{
    vx_status status = (vx_status)VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_vpac_nf_generic_target_kernel);
    if (status == (vx_status)VX_SUCCESS)
    {
        vx_vpac_nf_generic_target_kernel = NULL;
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to Remove Nf TargetKernel\n");
    }
    if (NULL != gTivxVpacNfGenericInstObj.lock)
    {
        tivxMutexDelete(&gTivxVpacNfGenericInstObj.lock);
    }
}
/* LDRA_JUSTIFY_END */

/* ========================================================================== */
/*                              OPENVX Callbacks                              */
/* ========================================================================== */

static vx_status VX_CALLBACK tivxVpacNfGenericProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                    status = (vx_status)VX_SUCCESS;
    int32_t                      fvid2_status = FVID2_SOK;
    uint32_t                     size;
    int32_t                      k;
    int32_t                      m;
    int16_t                      temp_lut[25];
    int16_t                     *pConv;
    void                        *conv_target_ptr;
    tivxVpacNfGenericObj        *nf_generic_obj = NULL;
    tivx_obj_desc_convolution_t *conv;
    tivx_obj_desc_image_t       *src;
    tivx_obj_desc_image_t       *dst;
    Fvid2_FrameList             *inFrmList;
    Fvid2_FrameList             *outFrmList;
    uint64_t                    cur_time;
    tivx_obj_desc_t             *out_base_desc = NULL;

    status = tivxCheckNullParams(obj_desc, num_params,
                TIVX_KERNEL_VPAC_NF_GENERIC_MAX_PARAMS);

    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> Rationale: The test framework and test apps cannot reach this portion. Passing only valid types to tivxAddKernelVpacNfGenericValidate function. 
                        Error cases pre-validating at tiovx.
    Effect on this unit: None;Unused feature, cannot be enabled, control cannot reach to this condition.
    <justification end> */
    if ((vx_status)VX_SUCCESS != status)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid Descriptor\n");
    }
    /* LDRA_JUSTIFY_END */

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */

    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&nf_generic_obj, &size);

        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if ((vx_status)VX_SUCCESS != status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Null Desc\n");
        }
        /* LDRA_JUSTIFY_END */

        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        else if (sizeof(tivxVpacNfGenericObj) != size)
        {
            VX_PRINT(VX_ZONE_ERROR, "Incorrect object size\n");
            status = (vx_status)VX_FAILURE;
        }
        else
        /* LDRA_JUSTIFY_END */
        {
            /* do nothing */
        }
    }

    /* LDRA_JUSTIFY_START
    <metric start>  branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        inFrmList = &nf_generic_obj->inFrmList;
        outFrmList = &nf_generic_obj->outFrmList;
        src = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_VPAC_NF_GENERIC_INPUT_IDX];
        conv = (tivx_obj_desc_convolution_t *)obj_desc[TIVX_KERNEL_VPAC_NF_GENERIC_CONV_IDX];
        dst = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_VPAC_NF_GENERIC_OUTPUT_IDX];

        out_base_desc = (tivx_obj_desc_t *)dst;
        nf_generic_obj->timestamp = out_base_desc->timestamp;

        conv_target_ptr = tivxMemShared2TargetPtr(&conv->mem_ptr);

        tivxCheckStatus(&status, tivxMemBufferMap(conv_target_ptr, conv->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));

        pConv = conv_target_ptr;

        /* Centers the given matrix in the 5x5 */
        {
            for (m = -2; m < 3; m++)
            {
                for (k = -2; k < 3; k++)
                {
                    if ( (m < (-((int32_t)conv->rows/2))) || (m > ((int32_t)conv->rows/2)))
                    {
                        /* Force to zero */
                        temp_lut[((m + 2) * 5) + (k + 2)] = 0;
                    }
                    else if ( (k < (-((int32_t)conv->columns/2))) || (k > ((int32_t)conv->columns/2)))
                    {
                        /* Force to zero */
                        temp_lut[((m + 2) * 5) + (k + 2)] = 0;
                    }
                    else
                    {
                        temp_lut[((m + 2) * 5) + (k + 2)] = pConv[((m + ((int32_t)conv->rows/2)) * (int32_t)conv->columns) +
                                                                       (k + ((int32_t)conv->columns/2))];
                    }
                }
            }
        }

        /* Since it is convolution, flip the matrix (decide later to do this or not) */
        for (k = 0; k < 12; k++) {
            nf_generic_obj->wgtTbl.genFilterCoeffs[k] = (int32_t) temp_lut[25 - 1 - k];
        }
        for (k = 13; k < 25; k++) {
            nf_generic_obj->wgtTbl.genFilterCoeffs[k - 1] = (int32_t) temp_lut[25 - 1 - k];
        }

        nf_generic_obj->nf_cfg.nfCfg.centralPixelWeight = temp_lut[12];
        nf_generic_obj->wgtTbl.filterMode      = NF_FILTER_MODE_GENERIC_2D_FILTER;

        /* Update NF params */
        fvid2_status = Fvid2_control(nf_generic_obj->handle,
            IOCTL_VHWA_M2M_NF_SET_PARAMS, &nf_generic_obj->nf_cfg, NULL);
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */  
        if (FVID2_SOK != fvid2_status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Set parameters request failed\n");
            status = (vx_status)VX_FAILURE;
        }
        else
        /* LDRA_JUSTIFY_END */
        {
            status = (vx_status)VX_SUCCESS;
        }
    }
    /* LDRA_JUSTIFY_START
    <metric start>  branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        /* Set NF coeff */
        fvid2_status = Fvid2_control(nf_generic_obj->handle, IOCTL_VHWA_M2M_NF_SET_FILTER_COEFF,
            &nf_generic_obj->wgtTbl, NULL);
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */  
        if (FVID2_SOK != fvid2_status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Set coeff request failed\n");
            status = (vx_status)VX_FAILURE;
        }
        /* LDRA_JUSTIFY_END */
    }

    /* LDRA_JUSTIFY_START
    <metric start>  branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        /* Initialize NF Input Frame List */
        inFrmList->frames[0U] =
            &nf_generic_obj->inFrm;
        inFrmList->numFrames = 1U;

        nf_generic_obj->inFrm.addr[0U] = tivxMemShared2PhysPtr(
            src->mem_ptr[0].shared_ptr,
            (int32_t)src->mem_ptr[0].mem_heap_region);

        /* Initialize NF Output Frame List */
        outFrmList->frames[0U] = &nf_generic_obj->outFrm;
        outFrmList->numFrames = 1U;

        nf_generic_obj->outFrm.addr[0U] = tivxMemShared2PhysPtr(
            dst->mem_ptr[0].shared_ptr,
            (int32_t)dst->mem_ptr[0].mem_heap_region);
            
        cur_time = tivxPlatformGetTimeInUsecs();    

        /* Submit NF Request*/
        fvid2_status = Fvid2_processRequest(nf_generic_obj->handle, inFrmList,
            outFrmList, FVID2_TIMEOUT_FOREVER);
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */ 
        if (FVID2_SOK != fvid2_status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to Submit Request\n");
            status = (vx_status)VX_FAILURE;
        }
        /* LDRA_JUSTIFY_END */
    }

    /* LDRA_JUSTIFY_START
    <metric start>  branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        /* Wait for Frame Completion */
        tivxEventWait(nf_generic_obj->waitForProcessCmpl, VX_TIMEOUT_WAIT_FOREVER);

        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start> Rationale:
        This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the error.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if((0u == nf_generic_obj->wdTimerErrStatus) && (0u == nf_generic_obj->err_stat))
        /* LDRA_JUSTIFY_END */
        {
            fvid2_status = Fvid2_getProcessedRequest(nf_generic_obj->handle,
                inFrmList, outFrmList, 0);
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */ 
            if (FVID2_SOK != fvid2_status)
            {
                VX_PRINT(VX_ZONE_ERROR, "Failed to Get Processed Request\n");
                status = (vx_status)VX_FAILURE;
            }
            /* LDRA_JUSTIFY_END */
        }
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> Rationale:
        This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the error.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        else
        {
            if(0u != nf_generic_obj->wdTimerErrStatus)
            {
                VX_PRINT(VX_ZONE_ERROR, "HTS stall: Watchdog timer error interrupt triggered \n");
                status = (vx_status)VX_ERROR_TIMEOUT;
            }
            if(0u != nf_generic_obj->err_stat)
            {
                VX_PRINT(VX_ZONE_ERROR, "Error interrupt: NF error interrupt triggered \n");
                status = (vx_status)VX_FAILURE;
            }
        }
        /* LDRA_JUSTIFY_END */
    }

    /* LDRA_JUSTIFY_START
    <metric start>  branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        cur_time = tivxPlatformGetTimeInUsecs() - cur_time;

        appPerfStatsHwaUpdateLoad(nf_generic_obj->hwa_perf_id,
            (uint32_t)cur_time,
            dst->imagepatch_addr[0U].dim_x*dst->imagepatch_addr[0U].dim_y /* pixels processed */
            );
    }

    /* LDRA_JUSTIFY_START
    <metric start>  branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        tivxCheckStatus(&status, tivxMemBufferUnmap(conv_target_ptr, conv->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
    }

    return status;
}

static vx_status VX_CALLBACK tivxVpacNfGenericCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                         status = (vx_status)VX_SUCCESS;
    int32_t                           fvid2_status = FVID2_SOK;
    int32_t                           k;
    int32_t                           m;
    int16_t                           temp_lut[25];
    int16_t                          *pConv;
    tivx_obj_desc_convolution_t      *conv;
    void                             *conv_target_ptr;
    tivxVpacNfGenericObj             *nf_generic_obj = NULL;
    Vhwa_M2mNfConfig                 *nf_cfg = NULL;
    tivx_vpac_nf_common_params_t     *params = NULL;
    tivx_obj_desc_user_data_object_t *params_array = NULL;
    tivx_obj_desc_image_t            *src;
    tivx_obj_desc_image_t            *dst;
    void                             *params_array_target_ptr = NULL;

    status = tivxCheckNullParams(obj_desc, num_params,
                TIVX_KERNEL_VPAC_NF_GENERIC_MAX_PARAMS);

    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS != status)
    {
        VX_PRINT(VX_ZONE_ERROR, "Required input parameter set to NULL\n");
    }
    /* LDRA_JUSTIFY_END */

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        nf_generic_obj = tivxVpacNfGenericAllocObject(&gTivxVpacNfGenericInstObj);
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (NULL != nf_generic_obj)
        /* LDRA_JUSTIFY_END */
        {
            params_array = (tivx_obj_desc_user_data_object_t *)
                obj_desc[TIVX_KERNEL_VPAC_NF_GENERIC_CONFIGURATION_IDX];
            src = (tivx_obj_desc_image_t *)
                obj_desc[TIVX_KERNEL_VPAC_NF_GENERIC_INPUT_IDX];
            dst = (tivx_obj_desc_image_t *)
                obj_desc[TIVX_KERNEL_VPAC_NF_GENERIC_OUTPUT_IDX];
        }
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */ 
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to Alloc Nf Generic Object\n");
            status = (vx_status)VX_ERROR_NO_RESOURCES;
        }
        /* LDRA_JUSTIFY_END */
    }
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        Vhwa_M2mNfCreatePrmsInit(&nf_generic_obj->createPrms);

        nf_cfg = &nf_generic_obj->nf_cfg;

        params_array_target_ptr = tivxMemShared2TargetPtr(&params_array->mem_ptr);

        tivxCheckStatus(&status, tivxMemBufferMap(params_array_target_ptr, params_array->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));

        params = (tivx_vpac_nf_common_params_t *)params_array_target_ptr;

        nf_generic_obj->enableErrorEvents = params->enable_error_events;
        nf_generic_obj->createPrms.enablePsa = params->enable_psa;
        /* Initialize NF Config with defaults */
        Nf_ConfigInit(&nf_cfg->nfCfg);


        status = tivxEventCreate(&nf_generic_obj->waitForProcessCmpl);

        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start> 
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if ((vx_status)VX_SUCCESS == status)
        /* LDRA_JUSTIFY_END */ 
        {
            nf_generic_obj->cbPrms.cbFxn   = tivxVpacNfGenericFrameComplCb;
            nf_generic_obj->cbPrms.appData = nf_generic_obj;

            nf_generic_obj->handle = Fvid2_create(FVID2_VHWA_M2M_NF_DRV_ID,
                nf_generic_obj->nf_drv_inst_id, (void *)&nf_generic_obj->createPrms,
                NULL, &nf_generic_obj->cbPrms);
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL == nf_generic_obj->handle)
            {
                VX_PRINT(VX_ZONE_ERROR, "Failed to Alloc Nf Generic Object\n");
                status = (vx_status)VX_FAILURE;
            }
            /* LDRA_JUSTIFY_END */ 
        }
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to allocate Event\n");
        }
        /* LDRA_JUSTIFY_END */ 
    }

    /* Register Error Callback */
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        nf_generic_obj->errEvtPrms.errEvents = VHWA_NF_RD_ERR | VHWA_NF_WR_ERR;
        nf_generic_obj->errEvtPrms.cbFxn     = tivxVpacNfGenericErrorCb;
        nf_generic_obj->errEvtPrms.appData   = nf_generic_obj;

        fvid2_status = Fvid2_control(nf_generic_obj->handle,
            IOCTL_VHWA_M2M_NF_REGISTER_ERR_CB, &nf_generic_obj->errEvtPrms, NULL);
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */  
        if (FVID2_SOK != fvid2_status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to Register Error Callback\n");
            status = (vx_status)VX_FAILURE;
        }
        /* LDRA_JUSTIFY_END */
    }
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        nf_generic_obj->wdTimererrEvtPrms.WdTimererrEvents = VHWA_NF_WDTIMER_ERR;
        nf_generic_obj->wdTimererrEvtPrms.cbFxn     = tivxVpacNfGenericWdTimerErrorCb;
        nf_generic_obj->wdTimererrEvtPrms.appData   = nf_generic_obj;
        fvid2_status = Fvid2_control(nf_generic_obj->handle,
            VHWA_M2M_IOCTL_NF_REGISTER_WDTIMER_ERR_CB, &nf_generic_obj->wdTimererrEvtPrms, NULL);
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */ 
        if (FVID2_SOK != fvid2_status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Fvid2_control Failed: Register Watchdog timer Error Callback\n");
            status = (vx_status)VX_FAILURE;
        }
        /* LDRA_JUSTIFY_END */
    }
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        conv = (tivx_obj_desc_convolution_t *)obj_desc[TIVX_KERNEL_VPAC_NF_GENERIC_CONV_IDX];
        conv_target_ptr = tivxMemShared2TargetPtr(&conv->mem_ptr);
        tivxCheckStatus(&status, tivxMemBufferMap(conv_target_ptr, conv->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
        pConv = conv_target_ptr;

        /* Centers the given matrix in the 5x5 */
        {
            for (m = -2; m < 3; m++)
            {
                for (k = -2; k < 3; k++)
                {
                    if ( (m < (-((int32_t)conv->rows/2))) || (m > ((int32_t)conv->rows/2)))
                    {
                        /* Force to zero */
                        temp_lut[((m + 2) * 5) + (k + 2)] = 0;
                    }
                    else if ( (k < (-((int32_t)conv->columns/2))) || (k > ((int32_t)conv->columns/2)))
                    {
                        /* Force to zero */
                        temp_lut[((m + 2) * 5) + (k + 2)] = 0;
                    }
                    else
                    {
                        temp_lut[((m + 2) * 5) + (k + 2)] = pConv[((m + ((int32_t)conv->rows/2)) * (int32_t)conv->columns) +
                                                                       (k + ((int32_t)conv->columns/2))];
                    }
                }
            }
        }

        /* Since it is convolution, flip the matrix (decide later to do this or not) */
        for (k = 0; k < 12; k++) {
            nf_generic_obj->wgtTbl.genFilterCoeffs[k] = (int32_t) temp_lut[25 - 1 - k];
        }
        for (k = 13; k < 25; k++) {
            nf_generic_obj->wgtTbl.genFilterCoeffs[k - 1] = (int32_t) temp_lut[25 - 1 - k];
        }

        /* Set NF Config parameters - centralPixelWeight set in tivxVpacNfGenericProcess */
        nf_cfg->nfCfg.filterMode = NF_FILTER_MODE_GENERIC_2D_FILTER;
        nf_cfg->nfCfg.tableMode = 0u;
        nf_cfg->nfCfg.skipMode = params->output_pixel_skip;
        nf_cfg->nfCfg.interleaveMode = params->input_interleaved;
        nf_cfg->nfCfg.outputShift = params->output_downshift;
        nf_cfg->nfCfg.outputOffset = params->output_offset;
        nf_cfg->nfCfg.numSubTables = 0u;
        nf_cfg->nfCfg.subTableIdx = 0u;
        nf_cfg->nfCfg.centralPixelWeight = temp_lut[12];
        nf_generic_obj->wgtTbl.filterMode      = NF_FILTER_MODE_GENERIC_2D_FILTER;

        tivxVpacNfSetFmt(&nf_cfg->inFmt, src);
        tivxVpacNfSetFmt(&nf_cfg->outFmt, dst);

        /* Save the parameters in the object variable,
           This is used to compare with config in process request to check if
           VPAC NF parameters needs to be reconfigured */
        memcpy(&nf_generic_obj->nfGenericParams, params, sizeof(tivx_vpac_nf_common_params_t));

        fvid2_status = Fvid2_control(nf_generic_obj->handle,
            IOCTL_VHWA_M2M_NF_SET_PARAMS, &nf_generic_obj->nf_cfg, NULL);
        if (FVID2_SOK != fvid2_status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Set parameters request failed\n");
            status = (vx_status)VX_FAILURE;
        }

        /* Set NF coeff */
        fvid2_status = Fvid2_control(nf_generic_obj->handle, IOCTL_VHWA_M2M_NF_SET_FILTER_COEFF,
            &nf_generic_obj->wgtTbl, NULL);
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (FVID2_SOK != fvid2_status)
        {
            VX_PRINT(VX_ZONE_ERROR, "Set coeffs request failed\n");
            status = (vx_status)VX_FAILURE;
        }
        /* LDRA_JUSTIFY_END */ 

        tivxCheckStatus(&status, tivxMemBufferUnmap(params_array_target_ptr, params_array->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
        tivxCheckStatus(&status, tivxMemBufferUnmap(conv_target_ptr, conv->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
    }

    if ((vx_status)VX_SUCCESS == status)
    {
        tivxSetTargetKernelInstanceContext(kernel, nf_generic_obj,
            sizeof(tivxVpacNfGenericObj));
    }
    else
    {
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (NULL != nf_generic_obj)
        /* LDRA_JUSTIFY_END */
        {
            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL != nf_generic_obj->handle)
            /* LDRA_JUSTIFY_END */
            {
                Fvid2_delete(nf_generic_obj->handle, NULL);
                nf_generic_obj->handle = NULL;
            }
            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL != nf_generic_obj->waitForProcessCmpl)
            /* LDRA_JUSTIFY_END */
            {
                tivxEventDelete(&nf_generic_obj->waitForProcessCmpl);
            }

            tivxVpacNfGenericFreeObject(&gTivxVpacNfGenericInstObj, nf_generic_obj);
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxVpacNfGenericDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                status = (vx_status)VX_SUCCESS;
    uint32_t                 size;
    tivxVpacNfGenericObj    *nf_generic_obj = NULL;

    status = tivxCheckNullParams(obj_desc, num_params,
                TIVX_KERNEL_VPAC_NF_GENERIC_MAX_PARAMS);
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&nf_generic_obj, &size);

        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (((vx_status)VX_SUCCESS == status) && (NULL != nf_generic_obj) &&
            (sizeof(tivxVpacNfGenericObj) == size))
        /* LDRA_JUSTIFY_END */
        {
            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, then our code base is expected to prevent undefined behaviour due to dereferencing a NULL pointer.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL != nf_generic_obj->handle)
            /* LDRA_JUSTIFY_END */
            {
                Fvid2_delete(nf_generic_obj->handle, NULL);
                nf_generic_obj->handle = NULL;
            }
             /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, then our code base is expected to prevent undefined behaviour due to dereferencing a NULL pointer.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL != nf_generic_obj->waitForProcessCmpl)
            /* LDRA_JUSTIFY_END */
            {
                tivxEventDelete(&nf_generic_obj->waitForProcessCmpl);
            }

            tivxVpacNfGenericFreeObject(&gTivxVpacNfGenericInstObj, nf_generic_obj);
        }
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid Target Instance Context\n");
        }
        /* LDRA_JUSTIFY_END */
    }
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid Descriptor\n");
    }
    /* LDRA_JUSTIFY_END */
    return status;
}

static vx_status VX_CALLBACK tivxVpacNfGenericControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                         status = (vx_status)VX_SUCCESS;
    uint32_t                          size;
    tivxVpacNfGenericObj             *nf_generic_obj = NULL;

    status = tivxGetTargetKernelInstanceContext(kernel,
        (void **)&nf_generic_obj, &size);

    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS != status)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to get Target Kernel Instance Context\n");
    }
    /* LDRA_JUSTIFY_END */

    /* LDRA_JUSTIFY_END */ 
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    else if ((NULL == nf_generic_obj) ||
        (sizeof(tivxVpacNfGenericObj) != size))
    {
        VX_PRINT(VX_ZONE_ERROR, "Wrong Size for Nf Generic Obj\n");
        status = (vx_status)VX_FAILURE;
    }
    else
    /* LDRA_JUSTIFY_END */
    {
        /* do nothing */
    }

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        switch (node_cmd_id)
        {
            case TIVX_VPAC_NF_CMD_SET_HTS_LIMIT:
            {
                status = tivxVpacNfGenericSetHtsLimitCmd(nf_generic_obj,
                    (tivx_obj_desc_user_data_object_t *)obj_desc[0U]);
                break;
            }
            case TIVX_VPAC_NF_CMD_SET_COEFF:
            {
                status = tivxVpacNfGenericSetCoeff(nf_generic_obj,
                    (tivx_obj_desc_user_data_object_t *)obj_desc[0U]);
                break;
            }
            case TIVX_VPAC_NF_CMD_GET_ERR_STATUS:
            {
                status = tivxVpacNfGenericGetErrStatusCmd(nf_generic_obj,
                    (tivx_obj_desc_scalar_t *)obj_desc[0U]);
                break;
            }
            case TIVX_VPAC_NF_CMD_GET_PSA_STATUS:
            {
                status = tivxVpacNfGenericGetPsaStatusCmd(nf_generic_obj,
                    (tivx_obj_desc_user_data_object_t *)obj_desc[0U]);
                break;
            }
            default:
            {
                VX_PRINT(VX_ZONE_ERROR, "Invalid Node Command Id\n");
                status = (vx_status)VX_FAILURE;
                break;
            }
        }
    }

    return (status);
}

/* ========================================================================== */
/*                          Local Functions                                   */
/* ========================================================================== */

static tivxVpacNfGenericObj *tivxVpacNfGenericAllocObject(
       tivxVpacNfGenericInstObj *instObj)
{
    uint32_t        cnt;
    tivxVpacNfGenericObj *nf_generic_obj = NULL;
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    /* Lock instance mutex */
    tivxMutexLock(instObj->lock);
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale:The test framework and test apps cannot reach this portion. The current kernel test app cannot support all 4 NF handles
    Effect on this unit: It will utilise all the 4 NF handles which are validated in the NF driver.
    <justification end> */ 
    for (cnt = 0U; cnt < VHWA_M2M_NF_MAX_HANDLES; cnt ++)
    /* LDRA_JUSTIFY_END */
    {
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The test framework and test apps cannot reach this portion.
        The test framework does not support the configuration required to trigger this scenario.
        Effect on this unit: If the control reaches here, the code is expected to allocate another handle for NF.
        However, due to the stated rationale, this is not tested.
        <justification end> */ 
        if (0U == instObj->nfGenericObj[cnt].isAlloc)
        /* LDRA_JUSTIFY_END */
        {
            nf_generic_obj = &instObj->nfGenericObj[cnt];
            memset(nf_generic_obj, 0x0, sizeof(tivxVpacNfGenericObj));
            instObj->nfGenericObj[cnt].isAlloc = 1U;

            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, the code base is NOT expected to accumulate and return an error.
            This behaviour is part of the application design. However, due to the stated rationale, this is not tested.
            <justification end> */
            if (self_cpu == (vx_enum)TIVX_CPU_ID_VPAC1)
            /* LDRA_JUSTIFY_END */
            {
                instObj->nfGenericObj[cnt].nf_drv_inst_id = VHWA_M2M_NF_DRV_INST_ID;
                instObj->nfGenericObj[cnt].hwa_perf_id    = APP_PERF_HWA_VPAC1_NF;
            }
            #if defined(SOC_J784S4) || defined(SOC_J742S2)
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, the code base is NOT expected to accumulate and return an error.
            This behaviour is part of the application design. However, due to the stated rationale, this is not tested.
            <justification end> */  
            else if (self_cpu == (vx_enum)TIVX_CPU_ID_VPAC2)
            {
                instObj->nfGenericObj[cnt].nf_drv_inst_id = VHWA_M2M_VPAC_1_NF_DRV_INST_ID_0;
                instObj->nfGenericObj[cnt].hwa_perf_id    = APP_PERF_HWA_VPAC2_NF;
            }
            #endif
            break;
            /* LDRA_JUSTIFY_END */
        }
    }

    /* Release instance mutex */
    tivxMutexUnlock(instObj->lock);

    return (nf_generic_obj);
}

static void tivxVpacNfGenericFreeObject(tivxVpacNfGenericInstObj *instObj,
    tivxVpacNfGenericObj *nf_generic_obj)
{
    uint32_t cnt;

    /* Lock instance mutex */
    tivxMutexLock(instObj->lock);
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale:The test framework and test apps cannot reach this portion. The current kernel test app cannot support all 4 NF handles
    Effect on this unit: It will free all the 4 NF handles which are validated in the NF driver.
    <justification end> */ 
    for (cnt = 0U; cnt < VHWA_M2M_NF_MAX_HANDLES; cnt ++)
    /* LDRA_JUSTIFY_END */
    {
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> 
        Rationale: The test framework and test apps cannot reach this portion.
        The test framework does not support the configuration required to trigger this scenario.
        Effect on this unit: If the control reaches here, the code is expected to free the allocated handle for NF.
        However, due to the stated rationale, this is not tested.
        <justification end> */ 
        if (nf_generic_obj == &instObj->nfGenericObj[cnt])
        {
            nf_generic_obj->isAlloc = 0U;
            break;
        }
        /* LDRA_JUSTIFY_END */
    }

    /* Release instance mutex */
    tivxMutexUnlock(instObj->lock);
}

static void tivxVpacNfSetFmt(Fvid2_Format *fmt,
    const tivx_obj_desc_image_t *img_desc)
{
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If control reaches here, the code base is expected to prevent undefined behaviour by avoiding dereferencing a NULL pointer.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if (NULL != img_desc)
    /* LDRA_JUSTIFY_END */
    {
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        The parameters are pre-validated by the host kernel before the control reaches here.
        Effect on this unit: If the control reaches here, our code base is expected to print the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        switch (img_desc->format)
        {
        /* LDRA_JUSTIFY_END */
            case (vx_df_image)VX_DF_IMAGE_U8:
            {
                fmt->dataFormat = FVID2_DF_LUMA_ONLY;
                fmt->ccsFormat = FVID2_CCSF_BITS8_PACKED;
                break;
            }
            case (vx_df_image)VX_DF_IMAGE_U16:
            {
                fmt->dataFormat = FVID2_DF_LUMA_ONLY;
                fmt->ccsFormat = FVID2_CCSF_BITS12_UNPACKED16;
                break;
            }
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications CAN reach this portion.
            No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
            Effect on this unit: If the control reaches here, our code base is expected to set input/output dataFormat and ccsFormat accordingly.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            case (vx_df_image)VX_DF_IMAGE_S16:
            {
                fmt->dataFormat = FVID2_DF_LUMA_ONLY;
                fmt->ccsFormat = FVID2_CCSF_BITS12_UNPACKED16;
                break;
            }
            /* LDRA_JUSTIFY_END */
            case (vx_df_image)TIVX_DF_IMAGE_P12:
            {
                fmt->dataFormat = FVID2_DF_LUMA_ONLY;
                fmt->ccsFormat = FVID2_CCSF_BITS12_PACKED;
                break;
            }
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            The parameter is expected to be pre-validated from a software layer above imaging. Therefore, this failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to print the error.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            default:
            {
                VX_PRINT(VX_ZONE_ERROR, "Invalid Vx Image Format\n");
                break;
            }
            /* LDRA_JUSTIFY_END */
        }

        fmt->width     = img_desc->imagepatch_addr[0U].dim_x;
        fmt->height    = img_desc->imagepatch_addr[0U].dim_y;
        fmt->pitch[0U] = (uint32_t)img_desc->imagepatch_addr[0U].stride_y;
    }
}

/* ========================================================================== */
/*                    Control Command Implementation                          */
/* ========================================================================== */

static vx_status tivxVpacNfGenericSetHtsLimitCmd(
    tivxVpacNfGenericObj *nf_generic_obj,
    const tivx_obj_desc_user_data_object_t *usr_data_obj)
{
    vx_status                                status = (vx_status)VX_SUCCESS;
    int32_t                                  fvid2_status = FVID2_SOK;
    Vhwa_HtsLimiter                          hts_limit;
    tivx_vpac_nf_hts_bw_limit_params_t      *app_hts_prms;
    void                                    *target_ptr;

    if(NULL == usr_data_obj)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid Argument\n");
        status = (vx_status)VX_FAILURE;
    }

    if((vx_status)VX_SUCCESS == status)
    {
        target_ptr = tivxMemShared2TargetPtr(&usr_data_obj->mem_ptr);

        tivxCheckStatus(&status, tivxMemBufferMap(target_ptr, usr_data_obj->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));

        if (sizeof(tivx_vpac_nf_hts_bw_limit_params_t) ==
                usr_data_obj->mem_size)
        {
            app_hts_prms = (tivx_vpac_nf_hts_bw_limit_params_t *)target_ptr;

            hts_limit.enableBwLimit = app_hts_prms->enable_hts_bw_limit;
            hts_limit.cycleCnt = app_hts_prms->cycle_cnt;
            hts_limit.tokenCnt = app_hts_prms->token_cnt;

            fvid2_status = Fvid2_control(nf_generic_obj->handle,
                        IOCTL_VHWA_M2M_NF_SET_HTS_LIMIT, &hts_limit, NULL);
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */ 
            if (FVID2_SOK != fvid2_status)
            {
                VX_PRINT(VX_ZONE_ERROR, "Set HTS limit request failed\n");
                status = (vx_status)VX_FAILURE;
            }
            /* LDRA_JUSTIFY_END */ 
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid Argument\n");
            status = (vx_status)VX_FAILURE;
        }
        tivxCheckStatus(&status, tivxMemBufferUnmap(target_ptr, usr_data_obj->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Null Argument\n");
    }

    return (status);
}

static vx_status tivxVpacNfGenericSetCoeff(tivxVpacNfGenericObj *nf_generic_obj,
    const tivx_obj_desc_user_data_object_t *usr_data_obj)
{
    vx_status                         status = (vx_status)VX_SUCCESS;
    int32_t                           fvid2_status = FVID2_SOK;
    Nf_WgtTableConfig                *wgtTbl = NULL;
    void                             *target_ptr;

    if (NULL != usr_data_obj)
    {
        target_ptr = tivxMemShared2TargetPtr(&usr_data_obj->mem_ptr);

        tivxCheckStatus(&status, tivxMemBufferMap(target_ptr, usr_data_obj->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));

        if (sizeof(Nf_WgtTableConfig) ==
                usr_data_obj->mem_size)
        {
            wgtTbl = (Nf_WgtTableConfig *)target_ptr;
            fvid2_status = Fvid2_control(nf_generic_obj->handle, IOCTL_VHWA_M2M_NF_SET_FILTER_COEFF,
                           wgtTbl, NULL);
            
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */ 
            if (FVID2_SOK != fvid2_status)
            {
                VX_PRINT(VX_ZONE_ERROR, "Set coeff request failed\n");
                    status = (vx_status)VX_FAILURE;
            }
            /* LDRA_JUSTIFY_END */ 
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid Argument\n");
            status = (vx_status)VX_FAILURE;
        }
        tivxCheckStatus(&status, tivxMemBufferUnmap(target_ptr, usr_data_obj->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Null Argument\n");
        status = (vx_status)VX_FAILURE;
    }

    return (status);
}

static vx_status tivxVpacNfGenericGetErrStatusCmd(const tivxVpacNfGenericObj *nf_generic_obj,
    tivx_obj_desc_scalar_t *scalar_obj_desc)
{
    vx_status                           status = (vx_status)VX_SUCCESS;
    uint32_t                            combined_err_stat;

    if (NULL != scalar_obj_desc)
    {
        combined_err_stat = nf_generic_obj->err_stat & 0x3FFFFFFF;
         /* LDRA_JUSTIFY_START
        <metric start> branch statement <metric end>
        <justification start> Rationale:
        This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the error.
        Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
        This behaviour is part of the application design. However, due to the stated rationale, this is not tested.
        <justification end> */
        if(0u != (nf_generic_obj->wdTimerErrStatus & VHWA_NF_WDTIMER_ERR))
        {
            combined_err_stat |= TIVX_VPAC_NF_WDTIMER_ERR;
        }
        /* LDRA_JUSTIFY_END */ 
        scalar_obj_desc->data.u32 = combined_err_stat;
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Null argument\n");
        status = (vx_status)VX_FAILURE;
    }

    return (status);
}

static vx_status tivxVpacNfGenericGetPsaStatusCmd(const tivxVpacNfGenericObj *nf_generic_obj,
    tivx_obj_desc_user_data_object_t *usr_data_obj)
{
    vx_status                         status = (vx_status)VX_SUCCESS;
    Vhwa_M2mNfPsaSign                 psa;
    int32_t                           fvid2_status;
    void                              *target_ptr;
    tivx_vpac_nf_psa_timestamp_data_t *psa_status_ptr;
    
    if (NULL != usr_data_obj)
    {
        if (sizeof(tivx_vpac_nf_psa_timestamp_data_t) ==
                usr_data_obj->mem_size)
        {
            fvid2_status = Fvid2_control(nf_generic_obj->handle,
                IOCTL_VHWA_M2M_NF_GET_PSA_SIGN, &psa, NULL);
            
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (FVID2_SOK != fvid2_status)
            {
                VX_PRINT(VX_ZONE_ERROR,
                    "tivxVpacNfGenericGetPsa: Fvid2_control Failed\n");
                status = (vx_status)VX_FAILURE;
            }
            /* LDRA_JUSTIFY_END */ 
            
            /* Map the user data object to access the memory */
            target_ptr = tivxMemShared2TargetPtr(&usr_data_obj->mem_ptr);

            tivxCheckStatus(&status, tivxMemBufferMap(target_ptr, usr_data_obj->mem_size,
                (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_WRITE_ONLY));

            psa_status_ptr = (tivx_vpac_nf_psa_timestamp_data_t *)target_ptr;

            /* Copy driver data (PSA values) into the structure */
            tivx_obj_desc_memcpy(&psa_status_ptr->psa_values, psa.psaSign, sizeof(psa.psaSign));

            /* Add timestamp to the structure */
            psa_status_ptr->timestamp = nf_generic_obj->timestamp;

            /* Unmap the memory after copying */
            tivxCheckStatus(&status, tivxMemBufferUnmap(target_ptr, usr_data_obj->mem_size,
                (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_WRITE_ONLY));
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Incorrect Data Object Size \n");
            status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
        }
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Null argument. Expected pointer to tivx_obj_desc_user_data_object_t\n");
        status = (vx_status)VX_FAILURE;
    }
    return (status);
}

/* ========================================================================== */
/*                              Driver Callbacks                              */
/* ========================================================================== */

int32_t tivxVpacNfGenericFrameComplCb(Fvid2_Handle handle, void *appData)
{
    tivxVpacNfGenericObj *nf_generic_obj = (tivxVpacNfGenericObj *)appData;
    /* LDRA_JUSTIFY_START
    <metric start> branch statement <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The parameter is expected to be pre-validated from the caller.
    Effect on this unit: If the control reaches here, the code base is NOT expected to accumulate and return an error.
    This behaviour is part of the application design. An error accumulation statement can be added in a future release.
    <justification end> */
    if (NULL != nf_generic_obj)
    {
        tivxEventPost(nf_generic_obj->waitForProcessCmpl);
    }
    /* LDRA_JUSTIFY_END */

    return FVID2_SOK;
}

/* LDRA_JUSTIFY_START
<metric start> branch statement <metric end>
<justification start>
Rationale: This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the API.
Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
However, due to the stated rationale, this is not tested.
<justification end> */
void tivxVpacNfGenericErrorCb(Fvid2_Handle handle, uint32_t errEvents, void *appData)
{
    tivxVpacNfGenericObj *nf_generic_obj = (tivxVpacNfGenericObj *)appData;

    if (NULL != nf_generic_obj)
    {
        nf_generic_obj->err_stat = nf_generic_obj->enableErrorEvents & errEvents;
        
        if(0 != nf_generic_obj->err_stat)
        {
            tivxEventPost(nf_generic_obj->waitForProcessCmpl);
        }
    }
}
/* LDRA_JUSTIFY_END */

/* LDRA_JUSTIFY_START
<metric start> branch statement <metric end>
<justification start> Rationale:
This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the API.
Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
However, due to the stated rationale, this is not tested.
<justification end> */
static void tivxVpacNfGenericWdTimerErrorCb(Fvid2_Handle handle, uint32_t wdTimerErrEvents, void *appData)
{
    tivxVpacNfGenericObj *nf_generic_obj = (tivxVpacNfGenericObj *)appData;
    if (NULL != nf_generic_obj)
    {
        nf_generic_obj->wdTimerErrStatus = nf_generic_obj->enableErrorEvents & wdTimerErrEvents;

        if(0u != nf_generic_obj->wdTimerErrStatus)
        {
            tivxEventPost(nf_generic_obj->waitForProcessCmpl);
        }
    }
}
/* LDRA_JUSTIFY_END */

BUILD_ASSERT(((VHWA_NF_RD_ERR == TIVX_VPAC_NF_RD_ERR)? 1 : 0));
BUILD_ASSERT(((VHWA_NF_WR_ERR == TIVX_VPAC_NF_WR_ERR)? 1 : 0));
BUILD_ASSERT(sizeof(Vhwa_M2mNfPsaSign) == sizeof(((tivx_vpac_nf_psa_timestamp_data_t *)0)->psa_values));
