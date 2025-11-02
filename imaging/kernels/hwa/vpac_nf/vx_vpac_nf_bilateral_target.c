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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "TI/tivx.h"
#include "TI/hwa_vpac_nf.h"
#include "tivx_hwa_kernels.h"
#include "tivx_kernel_vpac_nf_bilateral.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"
#include "tivx_hwa_vpac_nf_priv.h"
#include "TI/tivx_event.h"
#include "TI/tivx_mutex.h"
#include <math.h>

#include "vhwa/include/vhwa_m2mNf.h"
#include "utils/perf_stats/include/app_perf_stats.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define LUT_ROWS 5

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint32_t                            isAlloc;
    tivx_vpac_nf_bilateral_params_t     nfBilateralParams;
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
} tivxVpacNfBilateralObj;

typedef struct
{
    tivx_mutex lock;
    tivxVpacNfBilateralObj nfBilateralObj[VHWA_M2M_NF_MAX_HANDLES];

} tivxVpacNfBilateralInstObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static vx_status VX_CALLBACK tivxVpacNfBilateralProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxVpacNfBilateralCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxVpacNfBilateralDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxVpacNfBilateralControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static tivxVpacNfBilateralObj *tivxVpacNfBilateralAllocObject(
       tivxVpacNfBilateralInstObj *instObj);
static void tivxVpacNfBilateralFreeObject(
       tivxVpacNfBilateralInstObj *instObj, tivxVpacNfBilateralObj *nf_bilateral_obj);
static void tivxVpacNfSetFmt(Fvid2_Format *fmt,
    const tivx_obj_desc_image_t *img_desc);
static void tivxVpacNfBilateralGenerateLut(uint8_t subRangeBits, const vx_float64 *sigma_s,
    const vx_float64 *sigma_r, uint32_t *i_lut);
static uint32_t tivxVpacNfBilateralGenerateLutCoeffs(uint8_t mode,uint8_t inp_bitw,
    uint8_t filtSize, vx_float64 sigma_s, vx_float64 sigma_r, vx_float64 *f_wt_lut, uint8_t out_bitw,
    uint32_t *i_wt_lut_spatial, uint32_t *i_wt_lut_full);
static void tivxVpacNfBilateralInterleaveTables(uint32_t * const *i_lut, uint8_t numTables,
    uint32_t rangeLutEntries);
static uint32_t getSubRangeBits(uint16_t i);
static vx_status tivxVpacNfBilateralSetHtsLimitCmd(
    tivxVpacNfBilateralObj *nf_bilateral_obj,
    const tivx_obj_desc_user_data_object_t *usr_data_obj);
static vx_status tivxVpacNfBilateralSetCoeff(tivxVpacNfBilateralObj *nf_bilateral_obj,
    const tivx_obj_desc_user_data_object_t *usr_data_obj);
static vx_status tivxVpacNfBilateralGetErrStatusCmd(const tivxVpacNfBilateralObj *nf_bilateral_obj,
    tivx_obj_desc_scalar_t *scalar_obj_desc);
static vx_status tivxVpacNfBilateralGetPsaStatusCmd(const tivxVpacNfBilateralObj *nf_bilateral_obj,
    tivx_obj_desc_user_data_object_t *usr_data_obj);

int32_t tivxVpacNfBilateralFrameComplCb(Fvid2_Handle handle, void *appData);
void tivxVpacNfBilateralErrorCb(Fvid2_Handle handle, uint32_t errEvents, void *appData);
static void tivxVpacNfBilateralWdTimerErrorCb(Fvid2_Handle handle, uint32_t wdTimerErrEvents, void *appData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static tivx_target_kernel vx_vpac_nf_bilateral_target_kernel = NULL;

tivxVpacNfBilateralInstObj gTivxVpacNfBilateralInstObj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void tivxAddTargetKernelVpacNfBilateral(void)
{
    vx_status status = (vx_status)VX_FAILURE;
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    /* LDRA_JUSTIFY_START
    <metric start>statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */ 
    if (self_cpu == (vx_enum)TIVX_CPU_ID_VPAC1)
    {
        strncpy(target_name, TIVX_TARGET_VPAC_NF, TIVX_TARGET_MAX_NAME);
        status = (vx_status)VX_SUCCESS;
    }
    /* LDRA_JUSTIFY_END */
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
    #endif
    /* LDRA_JUSTIFY_END */
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
        vx_vpac_nf_bilateral_target_kernel = tivxAddTargetKernelByName(
                    TIVX_KERNEL_VPAC_NF_BILATERAL_NAME,
                    target_name,
                    tivxVpacNfBilateralProcess,
                    tivxVpacNfBilateralCreate,
                    tivxVpacNfBilateralDelete,
                    tivxVpacNfBilateralControl,
                    NULL);
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (NULL != vx_vpac_nf_bilateral_target_kernel)
        /* LDRA_JUSTIFY_END */
        {
            /* Allocate lock mutex */
            status = tivxMutexCreate(&gTivxVpacNfBilateralInstObj.lock);
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start> Rationale: The component level negative test framework and test applications cannot reach this portion.
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
                memset(&gTivxVpacNfBilateralInstObj.nfBilateralObj, 0x0,
                    sizeof(tivxVpacNfBilateralObj) * VHWA_M2M_NF_MAX_HANDLES);
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
                "Failed to Add NF Bilateral TargetKernel\n");
        }
        /* LDRA_JUSTIFY_END */ 
    }
}
/* LDRA_JUSTIFY
<metric start> statement branch <metric end>
<function start> void tivxRemoveTargetKernelVpacNfBilateral.* <function end>
<justification start> 
Rationale: The component level test framework and test applications cannot reach this portion.
This failure case is out of scope for the imaging test framework.
Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
This behaviour is part of the application design. However, due to the stated rationale, this is not tested.
<justification end> */
void tivxRemoveTargetKernelVpacNfBilateral(void)
{
    vx_status status = (vx_status)VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_vpac_nf_bilateral_target_kernel);
    if (status == (vx_status)VX_SUCCESS)
    {
        vx_vpac_nf_bilateral_target_kernel = NULL;
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to Remove Nf TargetKernel\n");
    }
    if (NULL != gTivxVpacNfBilateralInstObj.lock)
    {
        tivxMutexDelete(&gTivxVpacNfBilateralInstObj.lock);
    }
}

/* ========================================================================== */
/*                              OPENVX Callbacks                              */
/* ========================================================================== */

static vx_status VX_CALLBACK tivxVpacNfBilateralProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                         status = (vx_status)VX_SUCCESS;
    int32_t                           fvid2_status = FVID2_SOK;
    uint32_t                          size;
    tivxVpacNfBilateralObj           *nf_bilateral_obj = NULL;
    tivx_obj_desc_image_t            *src;
    tivx_obj_desc_image_t            *dst;
    Fvid2_FrameList                  *inFrmList;
    Fvid2_FrameList                  *outFrmList;
    uint64_t                         cur_time;
    tivx_obj_desc_t         *out_base_desc = NULL;


    status = tivxCheckNullParams(obj_desc, num_params,
                TIVX_KERNEL_VPAC_NF_BILATERAL_MAX_PARAMS);
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to print the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS != status)
    {
        VX_PRINT(VX_ZONE_ERROR, "Invalid Descriptor\n");
    }
    /* LDRA_JUSTIFY_END */

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> Rationale: The test framework and test apps cannot reach this portion. Passing only valid types to tivxAddKernelVpacNfBilateralValidate function. 
                        Error cases pre-validating at tiovx.
    Effect on this unit: None;Unused feature, cannot be enabled, control cannot reach to the false condition.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&nf_bilateral_obj, &size);

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
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        else if (sizeof(tivxVpacNfBilateralObj) != size)
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
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        inFrmList = &nf_bilateral_obj->inFrmList;
        outFrmList = &nf_bilateral_obj->outFrmList;
        src = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_VPAC_NF_BILATERAL_INPUT_IDX];
        dst = (tivx_obj_desc_image_t *)obj_desc[TIVX_KERNEL_VPAC_NF_BILATERAL_OUTPUT_IDX];

        out_base_desc = (tivx_obj_desc_t *)dst;
        nf_bilateral_obj->timestamp = out_base_desc->timestamp;
        /* Initialize NF Input Frame List */
        inFrmList->frames[0U] =
            &nf_bilateral_obj->inFrm;
        inFrmList->numFrames = 1U;

        nf_bilateral_obj->inFrm.addr[0U] = tivxMemShared2PhysPtr(
            src->mem_ptr[0].shared_ptr,
            (int32_t)src->mem_ptr[0].mem_heap_region);

        /* Initialize NF Output Frame List */
        outFrmList->frames[0U] = &nf_bilateral_obj->outFrm;
        outFrmList->numFrames = 1U;

        nf_bilateral_obj->outFrm.addr[0U] = tivxMemShared2PhysPtr(
            dst->mem_ptr[0].shared_ptr,
            (int32_t)dst->mem_ptr[0].mem_heap_region);

        cur_time = tivxPlatformGetTimeInUsecs();

        /* Submit NF Request*/
        fvid2_status = Fvid2_processRequest(nf_bilateral_obj->handle, inFrmList,
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
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        /* Wait for Frame Completion */
        tivxEventWait(nf_bilateral_obj->waitForProcessCmpl, VX_TIMEOUT_WAIT_FOREVER);
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start> Rationale:
        This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the error.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if((0u == nf_bilateral_obj->wdTimerErrStatus) && (0u == nf_bilateral_obj->err_stat))
        /* LDRA_JUSTIFY_END */
        {
            fvid2_status = Fvid2_getProcessedRequest(nf_bilateral_obj->handle,
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
            if(0u != nf_bilateral_obj->wdTimerErrStatus)
            {
                VX_PRINT(VX_ZONE_ERROR, "HTS stall: Watchdog timer error interrupt triggered \n");
                status = (vx_status)VX_ERROR_TIMEOUT;
            }
            if(0u != nf_bilateral_obj->err_stat)
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
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        cur_time = tivxPlatformGetTimeInUsecs() - cur_time;

        appPerfStatsHwaUpdateLoad(nf_bilateral_obj->hwa_perf_id,
            (uint32_t)cur_time,
            dst->imagepatch_addr[0U].dim_x*dst->imagepatch_addr[0U].dim_y /* pixels processed */
            );
    }

    return status;
}

static vx_status VX_CALLBACK tivxVpacNfBilateralCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                         status = (vx_status)VX_SUCCESS;
    int32_t                           fvid2_status = FVID2_SOK;
    tivxVpacNfBilateralObj           *nf_bilateral_obj = NULL;
    Vhwa_M2mNfConfig                 *nf_cfg = NULL;
    tivx_vpac_nf_bilateral_params_t  *params = NULL;
    tivx_vpac_nf_bilateral_sigmas_t  *sigmas = NULL;
    tivx_obj_desc_user_data_object_t *params_array = NULL;
    tivx_obj_desc_user_data_object_t *sigmas_array = NULL;
    tivx_obj_desc_image_t            *src;
    tivx_obj_desc_image_t            *dst;
    void                             *params_array_target_ptr = NULL;
    void                             *sigmas_array_target_ptr = NULL;

    status = tivxCheckNullParams(obj_desc, num_params,
                TIVX_KERNEL_VPAC_NF_BILATERAL_MAX_PARAMS);
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
        VX_PRINT(VX_ZONE_ERROR, "Required input parameter set to NULL\n");
        status = (vx_status)VX_FAILURE;
    }
    /* LDRA_JUSTIFY_END */
    
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        nf_bilateral_obj = tivxVpacNfBilateralAllocObject(&gTivxVpacNfBilateralInstObj);
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (NULL != nf_bilateral_obj)
        /* LDRA_JUSTIFY_END */
        {
            params_array = (tivx_obj_desc_user_data_object_t *)
                obj_desc[TIVX_KERNEL_VPAC_NF_BILATERAL_CONFIGURATION_IDX];
            sigmas_array = (tivx_obj_desc_user_data_object_t *)
                obj_desc[TIVX_KERNEL_VPAC_NF_BILATERAL_SIGMAS_IDX];
            src = (tivx_obj_desc_image_t *)
                obj_desc[TIVX_KERNEL_VPAC_NF_BILATERAL_INPUT_IDX];
            dst = (tivx_obj_desc_image_t *)
                obj_desc[TIVX_KERNEL_VPAC_NF_BILATERAL_OUTPUT_IDX];
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
            VX_PRINT(VX_ZONE_ERROR, "Failed to Alloc Nf Bilateral Object\n");
            status = (vx_status)VX_ERROR_NO_RESOURCES;
        }
        /* LDRA_JUSTIFY_END */
    }
   
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> Rationale: The component level negative test framework and test applications cannot reach this portion.
    This failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        Vhwa_M2mNfCreatePrmsInit(&nf_bilateral_obj->createPrms);

        nf_cfg = &nf_bilateral_obj->nf_cfg;

        params_array_target_ptr = tivxMemShared2TargetPtr(&params_array->mem_ptr);
        sigmas_array_target_ptr = tivxMemShared2TargetPtr(&sigmas_array->mem_ptr);

        tivxCheckStatus(&status, tivxMemBufferMap(params_array_target_ptr, params_array->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
        tivxCheckStatus(&status, tivxMemBufferMap(sigmas_array_target_ptr, sigmas_array->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));

        params = (tivx_vpac_nf_bilateral_params_t *)params_array_target_ptr;
        sigmas = (tivx_vpac_nf_bilateral_sigmas_t *)sigmas_array_target_ptr;

        /* Initialize NF Config with defaults */
        Nf_ConfigInit(&nf_cfg->nfCfg);
        nf_bilateral_obj->createPrms.enablePsa = params->params.enable_psa;

        status = tivxEventCreate(&nf_bilateral_obj->waitForProcessCmpl);
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
            nf_bilateral_obj->cbPrms.cbFxn   = tivxVpacNfBilateralFrameComplCb;
            nf_bilateral_obj->cbPrms.appData = nf_bilateral_obj;

            nf_bilateral_obj->handle = Fvid2_create(FVID2_VHWA_M2M_NF_DRV_ID,
                nf_bilateral_obj->nf_drv_inst_id, (void *)&nf_bilateral_obj->createPrms,
                NULL, &nf_bilateral_obj->cbPrms);
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL == nf_bilateral_obj->handle)
            {
                VX_PRINT(VX_ZONE_ERROR, "Failed to Alloc Nf Bilateral Object\n");
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
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        nf_bilateral_obj->errEvtPrms.errEvents = VHWA_NF_RD_ERR | VHWA_NF_WR_ERR;
        nf_bilateral_obj->errEvtPrms.cbFxn     = tivxVpacNfBilateralErrorCb;
        nf_bilateral_obj->errEvtPrms.appData   = nf_bilateral_obj;

        fvid2_status = Fvid2_control(nf_bilateral_obj->handle,
            IOCTL_VHWA_M2M_NF_REGISTER_ERR_CB, &nf_bilateral_obj->errEvtPrms, NULL);
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start> Rationale: The component level negative test framework and test applications cannot reach this portion.
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
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        nf_bilateral_obj->wdTimererrEvtPrms.WdTimererrEvents = VHWA_NF_WDTIMER_ERR;
        nf_bilateral_obj->wdTimererrEvtPrms.cbFxn     = tivxVpacNfBilateralWdTimerErrorCb;
        nf_bilateral_obj->wdTimererrEvtPrms.appData   = nf_bilateral_obj;
        fvid2_status = Fvid2_control(nf_bilateral_obj->handle,
            VHWA_M2M_IOCTL_NF_REGISTER_WDTIMER_ERR_CB, &nf_bilateral_obj->wdTimererrEvtPrms, NULL);
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
    Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        /* Set NF Config parameters - centralPixelWeight set in tivxVpacNfBilateralProcess */
        nf_cfg->nfCfg.filterMode = NF_FILTER_MODE_BILATERAL;
        nf_cfg->nfCfg.tableMode = params->adaptive_mode;
        nf_cfg->nfCfg.skipMode = params->params.output_pixel_skip;
        nf_cfg->nfCfg.interleaveMode = params->params.input_interleaved;
        nf_cfg->nfCfg.outputShift = params->params.output_downshift;
        nf_cfg->nfCfg.outputOffset = params->params.output_offset;
        nf_cfg->nfCfg.numSubTables = getSubRangeBits(sigmas->num_sigmas);
        nf_cfg->nfCfg.subTableIdx = params->sub_table_select;
        nf_cfg->nfCfg.centralPixelWeight = 255;
        nf_bilateral_obj->wgtTbl.filterMode = NF_FILTER_MODE_BILATERAL;
        nf_bilateral_obj->enableErrorEvents = params->params.enable_error_events;

        tivxVpacNfBilateralGenerateLut((uint8_t)getSubRangeBits(sigmas->num_sigmas), sigmas->sigma_space, sigmas->sigma_range,
            nf_bilateral_obj->wgtTbl.blFilterLut);

        tivxVpacNfSetFmt(&nf_cfg->inFmt, src);
        tivxVpacNfSetFmt(&nf_cfg->outFmt, dst);

        /* Save the parameters in the object variable,
           This is used to compare with config in process request to check if
           VPAC NF parameters needs to be reconfigured */

        memcpy(&nf_bilateral_obj->nfBilateralParams, params, sizeof(tivx_vpac_nf_bilateral_params_t));

        fvid2_status = Fvid2_control(nf_bilateral_obj->handle,
            IOCTL_VHWA_M2M_NF_SET_PARAMS, &nf_bilateral_obj->nf_cfg, NULL);
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
        /* LDRA_JUSTIFY_END */

        /* Set NF coeff */
        fvid2_status = Fvid2_control(nf_bilateral_obj->handle, IOCTL_VHWA_M2M_NF_SET_FILTER_COEFF,
            &nf_bilateral_obj->wgtTbl, NULL);
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
        tivxCheckStatus(&status, tivxMemBufferUnmap(sigmas_array_target_ptr, sigmas_array->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY));
    }

    if ((vx_status)VX_SUCCESS == status)
    {
        tivxSetTargetKernelInstanceContext(kernel, nf_bilateral_obj,
            sizeof(tivxVpacNfBilateralObj));
    }
    else
    {
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        The parameter is expected to be pre-validated from a software layer above imaging.
        Therefore, this failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (NULL != nf_bilateral_obj)
        /* LDRA_JUSTIFY_END */
        {
            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
            However, due to the stated rationale, this is not tested..
            <justification end> */
            if (NULL != nf_bilateral_obj->handle)
            /* LDRA_JUSTIFY_END */
            {
                Fvid2_delete(nf_bilateral_obj->handle, NULL);
                nf_bilateral_obj->handle = NULL;
            }
            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, our code base is expected to perform clean-up operations.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL != nf_bilateral_obj->waitForProcessCmpl)
            /* LDRA_JUSTIFY_END */
            {
                tivxEventDelete(&nf_bilateral_obj->waitForProcessCmpl);
            }

            tivxVpacNfBilateralFreeObject(&gTivxVpacNfBilateralInstObj, nf_bilateral_obj);
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxVpacNfBilateralDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                status = (vx_status)VX_SUCCESS;
    uint32_t                 size;
    tivxVpacNfBilateralObj    *nf_bilateral_obj = NULL;

    status = tivxCheckNullParams(obj_desc, num_params,
                TIVX_KERNEL_VPAC_NF_BILATERAL_MAX_PARAMS);
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
            (void **)&nf_bilateral_obj, &size);
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start>
        Rationale: The component level negative test framework and test applications cannot reach this portion.
        This failure case is out of scope for the imaging test framework.
        Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
        However, due to the stated rationale, this is not tested.
        <justification end> */
        if (((vx_status)VX_SUCCESS == status) && (NULL != nf_bilateral_obj) &&
            (sizeof(tivxVpacNfBilateralObj) == size))
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
            if (NULL != nf_bilateral_obj->handle)
            /* LDRA_JUSTIFY_END */
            {
                Fvid2_delete(nf_bilateral_obj->handle, NULL);
                nf_bilateral_obj->handle = NULL;
            }
           /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start>
            Rationale: The component level negative test framework and test applications cannot reach this portion.
            This failure case is out of scope for the imaging test framework.
            Effect on this unit: If the control reaches here, then our code base is expected to prevent undefined behaviour due to dereferencing a NULL pointer.
            However, due to the stated rationale, this is not tested.
            <justification end> */
            if (NULL != nf_bilateral_obj->waitForProcessCmpl)
            /* LDRA_JUSTIFY_END */
            {
                tivxEventDelete(&nf_bilateral_obj->waitForProcessCmpl);
            }

            tivxVpacNfBilateralFreeObject(&gTivxVpacNfBilateralInstObj, nf_bilateral_obj);
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
            status = (vx_status)VX_FAILURE;
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

static vx_status VX_CALLBACK tivxVpacNfBilateralControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id, tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status                         status = (vx_status)VX_SUCCESS;
    uint32_t                          size;
    tivxVpacNfBilateralObj             *nf_bilateral_obj = NULL;

    status = tivxGetTargetKernelInstanceContext(kernel,
        (void **)&nf_bilateral_obj, &size);
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
        VX_PRINT(VX_ZONE_ERROR, "Failed to get Target Kernel Instance Context\n");
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
    else if ((NULL == nf_bilateral_obj) ||
        (sizeof(tivxVpacNfBilateralObj) != size))
    {
        VX_PRINT(VX_ZONE_ERROR, "Wrong Size for Nf Bilateral Obj\n");
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
    Effect on this unit: If the control reaches here, our code base is expected to accumulate the error status and return the same to the application.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    if ((vx_status)VX_SUCCESS == status)
    /* LDRA_JUSTIFY_END */
    {
        switch (node_cmd_id)
        {
            case TIVX_VPAC_NF_CMD_SET_HTS_LIMIT:
            {
                status = tivxVpacNfBilateralSetHtsLimitCmd(nf_bilateral_obj,
                    (tivx_obj_desc_user_data_object_t *)obj_desc[0U]);
                break;
            }
            case TIVX_VPAC_NF_CMD_SET_COEFF:
            {
                status = tivxVpacNfBilateralSetCoeff(nf_bilateral_obj,
                    (tivx_obj_desc_user_data_object_t *)obj_desc[0U]);
                break;
            }
            case TIVX_VPAC_NF_CMD_GET_ERR_STATUS:
            {
                status = tivxVpacNfBilateralGetErrStatusCmd(nf_bilateral_obj,
                    (tivx_obj_desc_scalar_t *)obj_desc[0U]);
                break;
            }
            case TIVX_VPAC_NF_CMD_GET_PSA_STATUS:
            {
                status = tivxVpacNfBilateralGetPsaStatusCmd(nf_bilateral_obj,
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

static tivxVpacNfBilateralObj *tivxVpacNfBilateralAllocObject(
       tivxVpacNfBilateralInstObj *instObj)
{
    uint32_t        cnt;
    tivxVpacNfBilateralObj *nf_bilateral_obj = NULL;
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    /* Lock instance mutex */
    tivxMutexLock(instObj->lock);
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale:The test framework and test apps cannot reach this portion. The current kernel test app cannot support all 4 NF handles
    Effect on this unit: It will utilise all the 4 NF handles which are validated in the NF driver.
    <justification end> */
    for (cnt = 0U; cnt < VHWA_M2M_NF_MAX_HANDLES; cnt ++)
    /* LDRA_JUSTIFY_END */

    {
        if (0U == instObj->nfBilateralObj[cnt].isAlloc)
        {
            nf_bilateral_obj = &instObj->nfBilateralObj[cnt];
            memset(nf_bilateral_obj, 0x0, sizeof(tivxVpacNfBilateralObj));
            instObj->nfBilateralObj[cnt].isAlloc = 1U;

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
                instObj->nfBilateralObj[cnt].nf_drv_inst_id = VHWA_M2M_NF_DRV_INST_ID;
                instObj->nfBilateralObj[cnt].hwa_perf_id    = APP_PERF_HWA_VPAC1_NF;
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
                instObj->nfBilateralObj[cnt].nf_drv_inst_id = VHWA_M2M_VPAC_1_NF_DRV_INST_ID_0;
                instObj->nfBilateralObj[cnt].hwa_perf_id    = APP_PERF_HWA_VPAC2_NF;
            }
            /* LDRA_JUSTIFY_END */
            #endif
            break;
        }
    }

    /* Release instance mutex */
    tivxMutexUnlock(instObj->lock);

    return (nf_bilateral_obj);
}

static void tivxVpacNfBilateralFreeObject(tivxVpacNfBilateralInstObj *instObj,
    tivxVpacNfBilateralObj *nf_bilateral_obj)
{
    uint32_t cnt;

    /* Lock instance mutex */
    tivxMutexLock(instObj->lock);
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale: The test framework and test apps cannot reach this portion.
    The test framework does not support the configuration required to trigger this scenario.
    Effect on this unit: If the control reaches here, the code is expected to free the allocated handle for NF.
    However, due to the stated rationale, this is not tested.
    <justification end> */
    for (cnt = 0U; cnt < VHWA_M2M_NF_MAX_HANDLES; cnt ++)
    /* LDRA_JUSTIFY_END */
    {
        if (nf_bilateral_obj == &instObj->nfBilateralObj[cnt])
        {
            nf_bilateral_obj->isAlloc = 0U;
            break;
        }
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
        <metric start> branch <metric end>
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

static void tivxVpacNfBilateralGenerateLut(uint8_t subRangeBits, const vx_float64 *sigma_s,
    const vx_float64 *sigma_r, uint32_t *i_lut)
{
    uint32_t numTables = (uint32_t)1U << (uint32_t)subRangeBits;
    uint8_t tableNum;
    uint32_t rangeLutEntries = (uint32_t)256 >> (uint32_t)subRangeBits;
    uint32_t f_lut_size = LUT_ROWS * 256 * sizeof(vx_float64);
    vx_float64   *f_lut = tivxMemAlloc(f_lut_size, (vx_enum)TIVX_MEM_EXTERNAL);

    for (tableNum = 0; tableNum < numTables; tableNum++)
    {
        vx_float64 s_sigma = sigma_s[tableNum];
        vx_float64 r_sigma = sigma_r[tableNum];

        /*-----------------------------------------------------------------*/
        /* Generate fixed point LUT values, with index to LUT being        */
        /* pixel differences.                                              */
        /*-----------------------------------------------------------------*/
        tivxVpacNfBilateralGenerateLutCoeffs
            (
            0,
            8U - (uint8_t)subRangeBits,
            5,
            s_sigma,
            r_sigma,
            f_lut,
            8,
            NULL,
            &i_lut[tableNum * (uint32_t)LUT_ROWS * rangeLutEntries]
            );
    }

    tivxMemFree(f_lut, f_lut_size, (vx_enum)TIVX_MEM_EXTERNAL);

    if (numTables > 1U)
    {
        tivxVpacNfBilateralInterleaveTables(&i_lut, (uint8_t)numTables, rangeLutEntries);
    }
}

static uint32_t tivxVpacNfBilateralGenerateLutCoeffs(uint8_t mode,uint8_t inp_bitw,
    uint8_t filtSize, vx_float64 sigma_s, vx_float64 sigma_r, vx_float64 *f_wt_lut, uint8_t out_bitw,
    uint32_t *i_wt_lut_spatial, uint32_t *i_wt_lut_full)
{
    int32_t row, col;
    int32_t lut_w, lut_h;
    uint32_t temp_lut_w;
    uint32_t temp_lutSize = (uint32_t)1U << inp_bitw;
    int32_t lutSize = (int32_t)temp_lutSize;
    int32_t referenceSize = 12;
    uint8_t numspatialDistances;
    uint32_t returnVal = 0U;

    vx_float64 wt_s;
    vx_float64 wt_r;
    vx_float64 wt_sum;
    vx_float64 max = 0.0f;

    /* Translate filter size (5x5, 3x3, 1x1) into spatialDistances */
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start> 
    Rationale: The component level negative test framework and test applications CAN reach this portion.
    No existing test case can reach this portion. A test case will be added to cover this gap in a future release.
    Effect on this unit: If the control reaches here, our code base is expected to set numspatialDistances accordingly.
    However, due to the stated rationale, this is not tested.
    <justification end> */      
    if (filtSize == 3U)
    {
        numspatialDistances = 2;
    }
    else if (filtSize == 1U)
    {
        numspatialDistances = 0;
    }
    else
    /* LDRA_JUSTIFY_END */
    {
        numspatialDistances = 5;
    }

    /*---------------------------------------------------------------------*/
    /* Compute LUT width and height.                                       */
    /*---------------------------------------------------------------------*/
    memset(f_wt_lut, 0, (uint32_t)LUT_ROWS * (uint32_t)lutSize * sizeof(vx_float64));

    temp_lut_w = (uint32_t)1U << inp_bitw;
    lut_w = (int32_t)temp_lut_w;
    lut_h = (int32_t)numspatialDistances;

    /*---------------------------------------------------------------------*/
    /* Actual doubleing pt. LUT creation for Bilateral filter.              */
    /*---------------------------------------------------------------------*/

    /* If the space sigma is 0, then table should remain all zeros */
    if (sigma_s != 0.0f)
    {
        /* Index of the space distance from the center pixel, 0-4 */
        const uint8_t spaceWeightIndex[25] = {  4, 3, 2, 3, 4,
                                                3, 1, 0, 1, 3,
                                                2, 0, 0, 0, 2,
                                                3, 1, 0, 1, 3,
                                                4, 3, 2, 3, 4  };

        /* This distance-squared mapping of each index (above) in a 5x5 to the center pixel */
        const vx_float64 distanceValuesSquared[] =
        {
            1.0, // 0: 1^2 + 0^2
            2.0, // 1: 1^2 + 1^2
            4.0, // 2: 2^2 + 0^2
            5.0, // 3: 2^2 + 1^2
            8.0  // 4: 2^2 + 2^2
        };

        /* If the range sigma is 0, bilateral doesn't make sense, so instead generic mode weights should be generated */
        if (sigma_r == 0.0f)
        {
            /* Generate generic mode weights using gaussian curve from sigma_s */
            for (row = 0; row < 25; row++)
            {
                /* LDRA_JUSTIFY_START
                <metric start> branch <metric end>
                <justification start>
                Rationale: The test framework and test apps cannot reach this portion.
                The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
                Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
                <justification end> */  
                if (spaceWeightIndex[row] < numspatialDistances)
                /* LDRA_JUSTIFY_END */
                {   // gaussian_s = exp(-(x^2 / (2*sigma_s^2))
                    wt_s = distanceValuesSquared[spaceWeightIndex[row]] / (2.0f * sigma_s * sigma_s);
                    f_wt_lut[row] = exp(-wt_s);
                }
            }
            for (row = 13; row < 25; row++)
            {
                /* LDRA_JUSTIFY_START
                <metric start> branch <metric end>
                <justification start> 
                Rationale: The test framework and test apps cannot reach this portion.
                The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
                Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
                <justification end> */
                if (spaceWeightIndex[row] < numspatialDistances)
                /* LDRA_JUSTIFY_END */
                {   // gaussian_s = exp(-(x^2 / (2*sigma_s^2))
                    wt_s = distanceValuesSquared[spaceWeightIndex[row]] / (2.0f * sigma_s * sigma_s);
                    f_wt_lut[row-1] = exp(-wt_s);
                }
            }
            /* Overwrite center pixel weight to be 1 */
            f_wt_lut[12] = max = 1.0;
        }
        else {
            /* LDRA_JUSTIFY_START
            <metric start> branch <metric end>
            <justification start> 
            Rationale: The test framework and test apps cannot reach this portion.
            The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
            Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
            <justification end> */   
            if (mode == 0u) /* Bilateral Filter Weights */
            /* LDRA_JUSTIFY_END */
            {
                for (col = 0; col < lut_w; col++)
                {
                    // Only generate the lut values across the range that are needed
                    uint32_t temp = (uint32_t)1U << ((uint32_t)referenceSize - (uint32_t)inp_bitw);
                    int32_t col_mod = col * (int32_t)temp;
                    // gaussian_r = exp(-(x^2 / (2*sigma_r^2))
                    wt_r = ((vx_float64)col_mod * (vx_float64)col_mod) / (2.0f * sigma_r * sigma_r);

                    for (row = 0; row < lut_h; row++)
                    {
                        // gaussian_s = exp(-(x^2 / (2*sigma_s^2))
                        wt_s = distanceValuesSquared[row] / (2.0f * sigma_s * sigma_s);
                        wt_sum = wt_s + wt_r;
                        f_wt_lut[(row * lut_w) + col] = exp(-wt_sum);
                        //printf("%f\n", f_wt_lut[(row * lut_w) + col]);
                    }
                }
                max = 1.0;
            }
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start> 
            Rationale: The test framework and test apps cannot reach this portion.
            The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
            Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
            <justification end> */   
            else if ((mode == 1u) || (mode == 2u)) /* Passthrough Weights */
            {
                f_wt_lut[0] = 1.0;
                max = 1.0;
            }
            else if (mode == 3u) /* Highest Value Weights */
            {
                for (col = 0; col < lut_w; col++)
                {
                    for (row = 0; row < lut_h; row++)
                    {
                        f_wt_lut[(row * lut_w) + col] = 1.0;
                    }
                }
                max = 1.0;
            }
            else
            {
                returnVal = 1U;
            }
            /* LDRA_JUSTIFY_END */
        }
    }

    /*---------------------------------------------------------------------*/
    /* Fixed point LUT creation here.                                      */
    /*---------------------------------------------------------------------*/
    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start> 
    Rationale:
    The value of returnVal is set to 1U only if an unsupported or invalid mode is passed to the function. 
    In all valid usage scenarios within this codebase, the mode is hardcoded to 0, so returnVal is never set to 1U.
    Effect on this unit:
    If returnVal == 1U, the function exits early and skips LUT initialization and output population. 
    As a result, no LUT data is generated or written, which may cause subsequent processing to operate on uninitialized or invalid LUT data, 
    potentially leading to incorrect filter behavior or undefined results.
    <justification end> */  
    if(returnVal != 1U)
    /* LDRA_JUSTIFY_END */
    {
        /* LDRA_JUSTIFY_START
        <metric start> branch <metric end>
        <justification start> 
        Rationale: The test framework and test apps cannot reach this portion.
        The function is always called with a valid pointer for i_wt_lut_full in the current implementation. 
        Effect on this unit:
        If i_wt_lut_full is NULL, the function skips LUT initialization and output population, resulting in no LUT data being generated or written. 
        This may cause subsequent processing to operate on uninitialized or invalid LUT data, potentially leading to incorrect filter behavior or undefined results.
        <justification end> */ 
        if (i_wt_lut_full != NULL)
        /* LDRA_JUSTIFY_END */
        {
            int32_t i;
            memset(i_wt_lut_full, 0, (uint32_t)LUT_ROWS * (uint32_t)lutSize * sizeof(uint32_t));
            /* LDRA_JUSTIFY_START
            <metric start> statement branch <metric end>
            <justification start>
            Rationale: The test framework and test apps cannot reach this portion.
            The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
            Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
            <justification end> */  
            if (mode == 2u)
            {
                /* do nothing */
            }
            else 
            /* LDRA_JUSTIFY_END */
            {
                for (i = 0; i < (LUT_ROWS * lutSize); i++) {
                    if (0.0f != max)
                    {
                        uint32_t temp_u32 = ((uint32_t)1U << out_bitw) - 1U;
                        int32_t one_lsl_out_bitw_minus_one = (int32_t)temp_u32;
                        vx_float64 temp_f64 = ((f_wt_lut[i] / max) * (vx_float64)one_lsl_out_bitw_minus_one) + 0.5f;
                        i_wt_lut_full[i] = (uint32_t)temp_f64;
                    }
                }
            }
        }

        /* In case spatial lut needs to be generated separatly */
        /* LDRA_JUSTIFY_START
        <metric start> statement branch <metric end>
        <justification start>
        Rationale: The test framework and test apps cannot reach this portion.
        The function is unused because of an un-implemented feature, this code base is not being used/enabled with the current kernel support.
        Effect on this unit: There is no impact, as the feature is unused, cannot be enabled, and control cannot reach this condition.
        <justification end> */ 
        if (i_wt_lut_spatial && i_wt_lut_full){
            memset(i_wt_lut_spatial, 0, (uint32_t)LUT_ROWS * sizeof(i_wt_lut_spatial[0]));
            for (row = 0; row < lut_h; row++)
            {
                i_wt_lut_spatial[row] = i_wt_lut_full[row * lut_w];
            }
        }
        /* LDRA_JUSTIFY_END */
    }
    return returnVal;
}

static void tivxVpacNfBilateralInterleaveTables(uint32_t * const *i_lut, uint8_t numTables,
    uint32_t rangeLutEntries)
{
    uint32_t *const oldLut = *i_lut;
    uint32_t newLutSize = LUT_ROWS * 256 * sizeof(uint32_t);
    uint32_t *newLut = tivxMemAlloc(newLutSize, (vx_enum)TIVX_MEM_EXTERNAL);
    uint32_t i, j;

    /* LDRA_JUSTIFY_START
    <metric start> branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The parameter is expected to be pre-validated from a software layer above imaging. Therefore, this failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to print the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */ 
    if (NULL != newLut)
    /* LDRA_JUSTIFY_END */
    {
        memset(newLut, 0, (uint32_t)LUT_ROWS * 256 * sizeof(uint32_t));

        for (j = 0; j < numTables; j++)
        {
            for (i = 0; i < ((uint32_t)LUT_ROWS * rangeLutEntries); i++)
            {
                newLut[(numTables * i) + j] = oldLut[((j * (uint32_t)LUT_ROWS) * rangeLutEntries) + i];
            }
        }

        memcpy((uint32_t *)oldLut, newLut, (uint32_t)LUT_ROWS*256U*sizeof(uint32_t));

        tivxMemFree(newLut, newLutSize, (vx_enum)TIVX_MEM_EXTERNAL);
    }
    /* LDRA_JUSTIFY_START
    <metric start> statement branch <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The parameter is expected to be pre-validated from a software layer above imaging. Therefore, this failure case is out of scope for the imaging test framework.
    Effect on this unit: If the control reaches here, our code base is expected to print the error.
    However, due to the stated rationale, this is not tested.
    <justification end> */ 
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to Alloc Nf newLut\n");
    }
    /* LDRA_JUSTIFY_END */
}

static uint32_t getSubRangeBits(uint16_t i)
{
    uint32_t out = 0U;
    while ((i >>= 1) != 0U)
    {
        out++;
    }
    return out;
}

/* ========================================================================== */
/*                    Control Command Implementation                          */
/* ========================================================================== */

static vx_status tivxVpacNfBilateralSetHtsLimitCmd(
    tivxVpacNfBilateralObj *nf_bilateral_obj,
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

            fvid2_status = Fvid2_control(nf_bilateral_obj->handle,
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
            else
             /* LDRA_JUSTIFY_END */
            {
                status = (vx_status)VX_SUCCESS;
            }
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

static vx_status tivxVpacNfBilateralSetCoeff(tivxVpacNfBilateralObj *nf_bilateral_obj,
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
            fvid2_status = Fvid2_control(nf_bilateral_obj->handle, IOCTL_VHWA_M2M_NF_SET_FILTER_COEFF,
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

static vx_status tivxVpacNfBilateralGetErrStatusCmd(const tivxVpacNfBilateralObj *nf_bilateral_obj,
    tivx_obj_desc_scalar_t *scalar_obj_desc)
{
    vx_status                           status = (vx_status)VX_SUCCESS;
    uint32_t                            combined_err_stat;

    if (NULL != scalar_obj_desc)
    {
        combined_err_stat = nf_bilateral_obj->err_stat & 0x3FFFFFFF;
        /* LDRA_JUSTIFY_START
        <metric start> branch statement <metric end>
        <justification start> Rationale:
        This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the error.
        Effect on this unit: The unit is NOT expected to result in an error because the branch statement is pre-validated by the application.
        This behaviour is part of the application design. However, due to the stated rationale, this is not tested.
        <justification end> */
        if(0u != (nf_bilateral_obj->wdTimerErrStatus & VHWA_NF_WDTIMER_ERR))
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

static vx_status tivxVpacNfBilateralGetPsaStatusCmd(const tivxVpacNfBilateralObj *nf_bilateral_obj,
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
            fvid2_status = Fvid2_control(nf_bilateral_obj->handle,
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
                    "tivxVpacNfBilateralGetPsa: Fvid2_control Failed\n");
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
            psa_status_ptr->timestamp = nf_bilateral_obj->timestamp;

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

int32_t tivxVpacNfBilateralFrameComplCb(Fvid2_Handle handle, void *appData)
{
    tivxVpacNfBilateralObj *nf_bilateral_obj = (tivxVpacNfBilateralObj *)appData;
    /* LDRA_JUSTIFY_START
    <metric start> branch statement <metric end>
    <justification start>
    Rationale: The component level negative test framework and test applications cannot reach this portion.
    The parameter is expected to be pre-validated from the caller.
    Effect on this unit: If the control reaches here, the code base is NOT expected to accumulate and return an error.
    This behaviour is part of the application design. An error accumulation statement can be added in a future release.
    <justification end> */
    if (NULL != nf_bilateral_obj)
    {
        tivxEventPost(nf_bilateral_obj->waitForProcessCmpl);
    }
    /* LDRA_JUSTIFY_END */

    return FVID2_SOK;
}
/* LDRA_JUSTIFY_START
<metric start> branch statement <metric end>
<justification start> Rationale:
This is covered in a test framework of type negative build since this requires altering the source code macros to trigger the API.
Effect on this unit: If the control reaches here, the code base is expected to accumulate and return the error.
However, due to the stated rationale, this is not tested.
<justification end> */
void tivxVpacNfBilateralErrorCb(Fvid2_Handle handle, uint32_t errEvents, void *appData)
{
    tivxVpacNfBilateralObj *nf_bilateral_obj = (tivxVpacNfBilateralObj *)appData;

    if (NULL != nf_bilateral_obj)
    {
        nf_bilateral_obj->err_stat = nf_bilateral_obj->enableErrorEvents & errEvents;
        
        if(0 != nf_bilateral_obj->err_stat)
        {
            tivxEventPost(nf_bilateral_obj->waitForProcessCmpl);
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
static void tivxVpacNfBilateralWdTimerErrorCb(Fvid2_Handle handle, uint32_t wdTimerErrEvents, void *appData)
{
    tivxVpacNfBilateralObj *nf_bilateral_obj = (tivxVpacNfBilateralObj *)appData;
    if (NULL != nf_bilateral_obj)
    {
        nf_bilateral_obj->wdTimerErrStatus = nf_bilateral_obj->enableErrorEvents & wdTimerErrEvents;

        if(0u != nf_bilateral_obj->wdTimerErrStatus)
        {
            tivxEventPost(nf_bilateral_obj->waitForProcessCmpl);
        }
    }
}
/* LDRA_JUSTIFY_END */

BUILD_ASSERT(((VHWA_NF_RD_ERR == TIVX_VPAC_NF_RD_ERR)? 1 : 0));
BUILD_ASSERT(((VHWA_NF_WR_ERR == TIVX_VPAC_NF_WR_ERR)? 1 : 0));
BUILD_ASSERT(sizeof(Vhwa_M2mNfPsaSign) == sizeof(((tivx_vpac_nf_psa_timestamp_data_t *)0)->psa_values));
