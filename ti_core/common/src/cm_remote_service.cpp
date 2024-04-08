/*
 *
 * Copyright (c) 2021 Texas Instruments Incorporated
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
#include <cm_remote_service.h>

namespace ti_core_common
{

#ifndef PC

int32_t CM_vhwaDmpacSl2Free()
{
    int32_t status;
#if defined (SOC_J722S)
    status = appRemoteServiceRun(APP_IPC_CPU_MCU2_0, APP_VHWA_SERVICE_NAME,
                                 APP_DMPAC_SDE_SL2_FREE,
                                 NULL, 0, 0);
#else
    status = appRemoteServiceRun(APP_IPC_CPU_MCU2_1, APP_VHWA_SERVICE_NAME,
                                 APP_DMPAC_SDE_SL2_FREE,
                                 NULL, 0, 0);
#endif

    if (status == 0)
    {
#if defined (SOC_J722S)
        status = appRemoteServiceRun(APP_IPC_CPU_MCU2_0, APP_VHWA_SERVICE_NAME,
                                     APP_DMPAC_DOF_SL2_FREE,
                                     NULL, 0, 0);

#else
        status = appRemoteServiceRun(APP_IPC_CPU_MCU2_1, APP_VHWA_SERVICE_NAME,
                                     APP_DMPAC_DOF_SL2_FREE,
                                     NULL, 0, 0);
#endif

    }

    return status;
}


int32_t CM_vhwaDmpacSdeSl2Realloc()
{
    int32_t status = 0;

    App_M2mSdeSl2AllocPrms sl2AllocPrms;

    // Reset parameters
    sl2AllocPrms.maxImgWidth  = 2048;
    sl2AllocPrms.inCcsf       = APP_FVID2_CCSF_BITS12_UNPACKED16;
    sl2AllocPrms.searchRange  = APP_SDE_SR_192;
    sl2AllocPrms.disBuffDepth = APP_SDE_DEFAULT_DIS_BUFF_DEPTH;
#if defined (SOC_J722S)
    status = appRemoteServiceRun(APP_IPC_CPU_MCU2_0, APP_VHWA_SERVICE_NAME,
                                 APP_DMPAC_SDE_SL2_REALLOC,
                                 &sl2AllocPrms, sizeof(sl2AllocPrms), 0);
#else
    status = appRemoteServiceRun(APP_IPC_CPU_MCU2_1, APP_VHWA_SERVICE_NAME,
                                 APP_DMPAC_SDE_SL2_REALLOC,
                                 &sl2AllocPrms, sizeof(sl2AllocPrms), 0);
#endif

    return status;
}
#endif

} // namespace ti_core_common
