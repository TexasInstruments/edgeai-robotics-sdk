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
 * licensed and provided to you in appCntxtect code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any appCntxtect code compiled from the source code
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

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include "visloc.h"

static vx_status VISLOC_createInputImageObjects(VISLOC_Context * appCntxt);

vx_status VISLOC_init_VL(VISLOC_Context *appCntxt)
{
    DLInferer            *inferer;
    const VecDlTensor    *dlInfInputs;
    const VecDlTensor    *dlInfOutputs;
    const DlTensor       *ifInfo;
    CM_ScalerNodeCntxt   *scalerObj;
    CM_LdcNodeCntxt      *ldcObj;

    int32_t               i, j;
    int32_t               numInputs;
    int32_t               numOutputs;
    vx_image              scalerInput;
    vx_status             vxStatus;

    CM_PoseCalcNodeCntxt *poseCalcObj;
    CM_PoseVizNodeCntxt  *poseVizObj;

    ldcObj              = &appCntxt->ldcObj;
    scalerObj           = &appCntxt->scalerObj;
    poseCalcObj         = &appCntxt->poseCalcObj;
    poseVizObj          = &appCntxt->poseVizObj;

    inferer             = appCntxt->dlInferer;
    dlInfInputs         = inferer->getInputInfo();
    dlInfOutputs        = inferer->getOutputInfo();
    numInputs           = dlInfInputs->size();
    numOutputs          = dlInfOutputs->size();

    /* The Pose calculation node expects a tensor of size 3 only. */
    // SDK 10.0.0.8: ifInfo->shape = [1, 1, H, W]      => outTensorDims = [1, H, W]
    //               ifInfo->shape = [1, 64, H/4, W/4] => outTensorDims = [64, H/4, W/4]
    for (i = 0; i < numOutputs; i++)
    {
        ifInfo = &dlInfOutputs->at(i);
        appCntxt->outTensorNumDim[i] = ifInfo->dim - 1;

        for (j = 0; j < appCntxt->outTensorNumDim[i]; j++)
        {
            appCntxt->outTensorDims[i][j] = ifInfo->shape[j+1];
        }
    }

    // Create input image objects
    vxStatus = VISLOC_createInputImageObjects(appCntxt);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < numOutputs; i++)
        {
            appCntxt->outTensorSize[i] = 1;
            for (j = 0; j < appCntxt->outTensorNumDim[i]; j++)
            {
                appCntxt->outTensorSize[i] *= appCntxt->outTensorDims[i][j];
            }

            int32_t first = 0;
            int32_t last  = 1;

            for (j = 0; j < appCntxt->pipelineDepth; j++)
            {
                appCntxt->vxOutTensor[i][j] =
                    vxCreateTensor(appCntxt->vxContext,
                                   appCntxt->outTensorNumDim[i],
                                   appCntxt->outTensorDims[i],
                                   (i==0 ? VX_TYPE_UINT8:VX_TYPE_INT8),
                                   0);

                if (appCntxt->vxOutTensor[i][j] == NULL)
                {
                    LOG_ERROR("vxCreateTensor() failed\n");

                    vxStatus = VX_FAILURE;
                    break;
                }
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (numInputs != CM_PREPROC_NUM_OUTPUT_TENSORS)
        {
            LOG_ERROR("Number of DL inputs [%d] does not match the "
                      "number of tensors [%d] output by the pre-processing "
                      "node.\n",
                      numInputs,
                      CM_PREPROC_NUM_OUTPUT_TENSORS);

            vxStatus = VX_FAILURE;
        }
        else
        {
            int32_t status;

            /* Allocate DL input buffers. Currently, the input buffers are held
             * for each pipelined outputs so we need pipeline number of input
             * buffers.
             */
            for (j = 0; j < appCntxt->pipelineDepth; j++)
            {
                /* Allocate the input buffer. */
                vxStatus = CM_createDlBuffers(inferer,
                                              dlInfInputs,
                                              appCntxt->inferInputBuff[j]);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("Failed to allocate input (%d)\n", j);
                    break;
                }
            }

            /* Allocate DL output buffers. Currently, the input buffers are held
             * for each pipelined outputs so we need pipeline number of input
             * buffers.
             */
            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Allocate the output buffer. */
                for (j = 0; j < appCntxt->pipelineDepth; j++)
                {
                    /* Allocate the input buffer. */
                    vxStatus = CM_createDlBuffers(inferer,
                                                  dlInfOutputs,
                                                  appCntxt->inferOutputBuff[j]);

                    if (vxStatus != (vx_status)VX_SUCCESS)
                    {
                        LOG_ERROR("Failed to allocate output (%d)\n", j);
                        break;
                    }
                }
            }
        }
    }

    /* Input image will be the scaler default input. */
    scalerInput = appCntxt->vxInputImage[0];

    /************************/
    /*     LDC node         */
    /************************/
    if ((appCntxt->enableLdcNode) && (vxStatus == (vx_status)VX_SUCCESS))
    {
        CM_LdcCreateParams  params;

        params.width       = appCntxt->inputImageWidth;
        params.height      = appCntxt->inputImageHeight;
        params.ssFactor    = appCntxt->ldcSsFactor;
        params.blockWidth  = appCntxt->ldcBlockWidth;
        params.blockHeight = appCntxt->ldcBlockHeight;
        params.pixelPad    = appCntxt->ldcPixelPad;
        params.lutFilePath = appCntxt->ldcLutFilePath;

        vxStatus = CM_ldcNodeCntxtInit(ldcObj,
                                       appCntxt->vxContext,
                                       &params);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("[%s:%d] CM_ldcNodeCntxtInit() failed\n");
        }
        else
        {
            vxStatus = CM_ldcNodeCntxtSetup(ldcObj,
                                            appCntxt->vxContext,
                                            appCntxt->vxGraph,
                                            appCntxt->vxInputImage[0],
                                            appCntxt->vxRectifiedImage[0]);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("CM_ldcNodeCntxtSetup() failed\n");
                vxStatus = VX_FAILURE;
            }
            else
            {
                /* LDC output will be the scaler input. */
                scalerInput = appCntxt->vxRectifiedImage[0];
            }
        }
    }

    /************************/
    /*    Scaler node       */
    /************************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        CM_ScalerCreateParams   params;

        params.width         = appCntxt->dlImageWidth;
        params.height        = appCntxt->dlImageHeight;
        params.imageType     = VX_DF_IMAGE_NV12;
        params.interpolation = VX_INTERPOLATION_BILINEAR;
        params.pipelineDepth = appCntxt->pipelineDepth;

        vxStatus = CM_scalerNodeCntxtInit(scalerObj,
                                          appCntxt->vxContext,
                                          &params);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_scalerNodeCntxtInit() failed\n");
        }
        else
        {
            vxStatus = CM_scalerNodeCntxtSetup(scalerObj,
                                               appCntxt->vxContext,
                                               appCntxt->vxGraph,
                                               scalerInput);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("CM_scalerNodeCntxtSetup() failed\n");
                vxStatus = VX_FAILURE;
            }
        }
    }

    /**********************/
    /* Pose Calc Node     */
    /**********************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // Set pose calc create params
        appCntxt->poseCalcCreateParams.outWidth      = appCntxt->outImageWidth;
        appCntxt->poseCalcCreateParams.outHeight     = appCntxt->outImageHeight;
        appCntxt->poseCalcCreateParams.dlWidth       = appCntxt->dlImageWidth;
        appCntxt->poseCalcCreateParams.dlHeight      = appCntxt->dlImageHeight;
        appCntxt->poseCalcCreateParams.pipelineDepth = appCntxt->pipelineDepth;

        vxStatus = CM_poseCalcNodeCntxtInit(poseCalcObj,
                                            appCntxt->vxContext,
                                            numOutputs,
                                            appCntxt->outTensorDims,
                                           &appCntxt->poseCalcCreateParams);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_poseCalcNodeCntxtInit() failed\n");
        }
        else
        {
            vxStatus = CM_poseCalcNodeCntxtSetup(poseCalcObj,
                                                 appCntxt->vxContext,
                                                 appCntxt->vxGraph,
                                                 appCntxt->vxOutTensor[0][0],
                                                 appCntxt->vxOutTensor[1][0]);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("CM_poseCalcNodeCntxtSetup() failed\n");
                vxStatus = VX_FAILURE;
            }
        }
    }

    /**********************/
    /* Pose Viz Node      */
    /**********************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {


        // Set pose viz create params
        appCntxt->poseVizCreateParams.outWidth      = appCntxt->outImageWidth;
        appCntxt->poseVizCreateParams.outHeight     = appCntxt->outImageHeight;
        appCntxt->poseVizCreateParams.pipelineDepth = appCntxt->pipelineDepth;

        vxStatus = CM_poseVizNodeCntxtInit(poseVizObj,
                                           appCntxt->vxContext,
                                          &appCntxt->poseVizCreateParams);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_poseVizNodeCntxtInit() failed\n");
        }
        else
        {
            vxStatus = CM_poseVizNodeCntxtSetup(poseVizObj,
                                                appCntxt->vxContext,
                                                appCntxt->vxGraph,
                                                appCntxt->poseCalcObj.poseMatrix[0]);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("CM_poseVizNodeCntxtSetup() failed\n");
                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;
}

vx_status VISLOC_deinit_VL(VISLOC_Context *appCntxt)
{

    DLInferer            *inferer;
    const VecDlTensor    *dlInfOutputs;
    int32_t               numOutputs;
    int32_t               i, j;
    vx_status             vxStatus = VX_SUCCESS;

    inferer             = appCntxt->dlInferer;
    dlInfOutputs        = inferer->getOutputInfo();
    numOutputs          = dlInfOutputs->size();

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        if (appCntxt->vxInputImage[i] != NULL)
        {
            vxReleaseImage(&appCntxt->vxInputImage[i]);
        }
    }

    if (appCntxt->enableLdcNode)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (appCntxt->vxRectifiedImage[i] != NULL)
            {
                vxReleaseImage(&appCntxt->vxRectifiedImage[i]);
            }
        }

        CM_ldcNodeCntxtDeInit(&appCntxt->ldcObj);
    }


    /* Scaler node.  */
    CM_scalerNodeCntxtDeInit(&appCntxt->scalerObj);

    /* Pose Calc node */
    CM_poseCalcNodeCntxtDeInit(&appCntxt->poseCalcObj);

    /* Pose Viz node */
    CM_poseVizNodeCntxtDeInit(&appCntxt->poseVizObj);


    // vxOutTensor is always created
    for (i = 0; i < numOutputs; i++)
    {
        for (j = 0; j < appCntxt->pipelineDepth; j++)
        {
            if (appCntxt->vxOutTensor[i][j] != NULL)
            {
                vxReleaseTensor(&appCntxt->vxOutTensor[i][j]);
            }
        }
    }

    /* De-allocate the input and output buffers. */
    for (j = 0; j < appCntxt->pipelineDepth; j++)
    {
        appCntxt->inferInputBuff[j].clear();
        appCntxt->inferOutputBuff[j].clear();
    }

    return vxStatus;
}

vx_status  VISLOC_setupPipeline(VISLOC_Context * appCntxt)
{
    CM_LdcNodeCntxt                    *ldcObj;
    CM_ScalerNodeCntxt                 *scalerObj;
    CM_PoseCalcNodeCntxt               *poseCalcObj;
    CM_PoseVizNodeCntxt                *poseVizObj;
    VISLOC_graphParams                 *paramDesc;
    vx_graph_parameter_queue_params_t   q[VISLOC_NUM_GRAPH_PARAMS];

    vx_status             vxStatus;
    uint32_t              appValue;
    uint32_t              i;
    uint32_t              cnt;

    ldcObj      = &appCntxt->ldcObj;
    scalerObj   = &appCntxt->scalerObj;
    poseCalcObj = &appCntxt->poseCalcObj;
    poseVizObj  = &appCntxt->poseVizObj;
    cnt         = 0;

    if (appCntxt->enableLdcNode)
    {
        appCntxt->numGraphParams = 7;

        /* LDC node Param 6 (input image)==> graph param 0. */
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          ldcObj->vxNode,
                                          6);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputImage;
        }

        /* LDC node Param 7 (rectified image) ==> graph param 1.  */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* */
            vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                              ldcObj->vxNode,
                                              7);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("CM_addParamByNodeIndex() failed\n");
            }
            else
            {
                q[cnt++].refs_list = (vx_reference*)appCntxt->vxRectifiedImage;
            }
        }
    } else
    {
        appCntxt->numGraphParams = 6;

        /* Scaler node Param 0 (input image) ==> graph param 0. */
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          scalerObj->vxNode,
                                          0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputImage;
        }
    }

    /* Scaler node Param 1 (scaled image)==> graph param 1/2. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          scalerObj->vxNode,
                                          1);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)scalerObj->outImage;
        }
    }

    /* Visual localization (PoseCalc) node Param 4 (feature point)
        ==> graph param 2/3. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          poseCalcObj->node,
                                          4);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutTensor[0];
        }
    }

    /* Visual localization (PoseCalc) node Param 5 (feature descriptor)
        ==> graph param 3/4. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          poseCalcObj->node,
                                          5);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutTensor[1];
        }
    }

    /* Visual localization (PoseCalc) node Param 9 (pose matrix)
        ==> graph param 4/5. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          poseCalcObj->node,
                                          9);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)poseCalcObj->poseMatrix;
        }
    }

    /* Visualization (PoseViz) node Param 3 (output image)
        ==> graph param 5/6. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          poseVizObj->node,
                                          3);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR(" CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)poseVizObj->outputImage;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < cnt; i++)
        {
            q[i].graph_parameter_index = i;
            q[i].refs_list_size        = appCntxt->pipelineDepth;
        }

        // allocate free Q
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            paramDesc                   = &appCntxt->paramDesc[i];
            paramDesc->vxInputImage     = appCntxt->vxInputImage[i];
            paramDesc->vxRectifiedImage = appCntxt->vxRectifiedImage[i];
            paramDesc->vxScalerOut      = scalerObj->outImage[i];

            paramDesc->vxOutTensor[0]   = appCntxt->vxOutTensor[0][i];
            paramDesc->vxOutTensor[1]   = appCntxt->vxOutTensor[1][i];
            paramDesc->inferInputBuff   = &appCntxt->inferInputBuff[i];
            paramDesc->inferOutputBuff  = &appCntxt->inferOutputBuff[i];

            paramDesc->vxPoseMatrix     = poseCalcObj->poseMatrix[i];
            paramDesc->vxOutputImage    = poseVizObj->outputImage[i];

            paramDesc->timestamp        = &appCntxt->timestamp[i];

            VISLOC_enqueInputDesc(appCntxt, paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxSetGraphScheduleConfig() failed\n");
        }
    }

    if (vxStatus == VX_SUCCESS)
    {
        /* explicitly set graph pipeline depth */
        vxStatus = tivxSetGraphPipelineDepth(appCntxt->vxGraph,
                                             appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetGraphPipelineDepth() failed\n");
        }
    }


    // Register graph completion event
    if (vxStatus == VX_SUCCESS)
    {
        appValue = appCntxt->vxEvtAppValBase + VISLOC_GRAPH_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)appCntxt->vxGraph,
                                   VX_EVENT_GRAPH_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("vxRegisterEvent() failed\n");
        }
    }

    // Register scaler node completion event
    if (vxStatus == VX_SUCCESS)
    {
        appValue = appCntxt->vxEvtAppValBase + VISLOC_SCALER_NODE_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)scalerObj->vxNode,
                                   VX_EVENT_NODE_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxRegisterEvent() failed\n");
        }
    }


    return vxStatus;
}


void VISLOC_printStats(VISLOC_Context * appCntxt)
{
    tivx_utils_graph_perf_print(appCntxt->vxGraph);

    appPerfPointPrint(&appCntxt->vlPerf);
    LOG_INFO_RAW("\n");
    appPerfPointPrintFPS(&appCntxt->vlPerf);
    LOG_INFO_RAW("\n");
    CM_printProctime(stdout);
    LOG_INFO_RAW("\n");
}


vx_status VISLOC_exportStats(VISLOC_Context * appCntxt, FILE *fp, bool exportAll)
{
    vx_status vxStatus = tivx_utils_graph_perf_export(fp, appCntxt->vxGraph);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("tivx_utils_graph_perf_export() failed\n");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (exportAll == true)
        {
            app_perf_point_t *perfArr[1] = {&appCntxt->vlPerf};
            appPerfStatsExportAll(fp, perfArr, 1);
        }
    }

    return vxStatus;
}


vx_status VISLOC_waitGraph(VISLOC_Context * appCntxt)
{
    vx_status vxStatus;

    vxStatus = vxWaitGraph(appCntxt->vxGraph);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("vxWaitGraph() failed\n");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Wait for the output queue to get flushed. */
        while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
        {
            std::this_thread::sleep_for (std::chrono::milliseconds(50));
        }
    }

    return vxStatus;
}


vx_status  VISLOC_process(VISLOC_Context     * appCntxt,
                          VISLOC_graphParams * gpDesc,
                          uint64_t             timestamp)
{
    vx_status vxStatus = VX_SUCCESS;
    uint16_t   i;
    uint8_t    cnt = 0;

    vx_reference  obj[VISLOC_NUM_GRAPH_PARAMS];

    // set timestamp of input image
    *gpDesc->timestamp = timestamp;

    obj[cnt++] = (vx_reference)gpDesc->vxInputImage;
    if (appCntxt->enableLdcNode)
    {
        obj[cnt++] = (vx_reference)gpDesc->vxRectifiedImage;
    }
    obj[cnt++] = (vx_reference)gpDesc->vxScalerOut;

    // Enqueue buffers
    // Enqueue vxOutTensor after DLR is completed
    for (i = 0; i < cnt; i++)
    {
        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                   i,
                                                   &obj[i],
                                                   1);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxGraphParameterEnqueueReadyRef(%d) failed\n", i);
            break;
        }
    }

    /* Push the descriptor to the DLR input queue. */
    VISLOC_enquePreprocInputDesc(appCntxt, gpDesc);

    return vxStatus;
}


vx_status VISLOC_preProcess(VISLOC_Context     * appCntxt,
                            vx_image             vxScalerOut,
                            VecDlTensorPtr      *inputTensorVec)
{
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  imgAddr;
    uint8_t                    *srcPtr[2];
    vx_map_id                   mapId[2];
    bool                        mapped[2] = {false, false};
    vx_status                   vxStatus;

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = appCntxt->dlImageWidth;
    rect.end_y   = appCntxt->dlImageHeight;

    /* Map the image planes. */
    for (uint32_t i = 0; i < 2; i++)
    {
        /* Get the pointer to the YUV data plans. */
        vxStatus = vxMapImagePatch(vxScalerOut,
                                   &rect,
                                   i,
                                   &mapId[i],
                                   &imgAddr,
                                   (void **)&srcPtr[i],
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxMapImagePatch() failed.");
        }
        else
        {
            mapped[i] = true;
        }

        /* UV plane has half the height. */
        rect.end_y /= 2;
    }

    /* Take thr output image from the scaler which is in NV12 format and do
     * the following:
     * - Color convert the image from YUV420 to BGR format
     * - Convert to float and write to the DLR input buffer
     */

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        void      **p = reinterpret_cast<void**>(srcPtr);
        int32_t     status;

        status =
        (*appCntxt->preProcObj)(const_cast<const void**>(p),
                                *inputTensorVec);

        if (status < 0)
        {
            LOG_ERROR("Data pre-processing failed.");
            vxStatus = VX_FAILURE;
        }
    }

    /* Unmap the image planes. */
    for (uint32_t i = 0; i < 2; i++)
    {
        if (mapped[i] == true)
        {
            vxUnmapImagePatch(vxScalerOut, mapId[i]);
        }
    }

    return vxStatus;
}


vx_status VISLOC_createScoreOutTensor(VISLOC_Context     * appCntxt,
                                      VecDlTensorPtr     & outputTensorVec,
                                      vx_tensor            vxOutTensor)
{
    uint8_t    * dPtr;

    vx_size     start[VISLOC_MAX_OUT_TENSOR_DIMS];
    vx_size     strides[VISLOC_MAX_OUT_TENSOR_DIMS];
    vx_map_id   mapId;
    vx_status   vxStatus;

    memset(start, 0, sizeof(start));

    vxStatus = tivxMapTensorPatch(vxOutTensor,
                                  appCntxt->outTensorNumDim[0],
                                  start,
                                  appCntxt->outTensorDims[0],
                                  &mapId,
                                  strides,
                                  (void **)&dPtr,
                                  VX_READ_AND_WRITE,
                                  VX_MEMORY_TYPE_HOST);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("tivxMapTensorPatch() failed.");
    }
    else
    {
        auto    *tensor = outputTensorVec[0];

        if (tensor->type == DlInferType_Float32)
        {
            float   *sPtr   = reinterpret_cast<float*>(tensor->data);
            CM_copyTensorData(sPtr, dPtr, tensor->numElem);
        }
        else
        {
            LOG_ERROR("Invalid data type.\n");
        }

        vxStatus = tivxUnmapTensorPatch(vxOutTensor, mapId);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("tivxUnmapTensorPatch() failed.");
        }
    }

    return vxStatus;
}



vx_status VISLOC_createDescOutTensor(VISLOC_Context     * appCntxt,
                                     VecDlTensorPtr     & outputTensorVec,
                                     vx_tensor            vxOutTensor)
{
    int8_t    * dPtr;

    vx_size     start[VISLOC_MAX_OUT_TENSOR_DIMS];
    vx_size     strides[VISLOC_MAX_OUT_TENSOR_DIMS];
    vx_map_id   mapId;
    vx_status   vxStatus;

    memset(start, 0, sizeof(start));


    vxStatus = tivxMapTensorPatch(vxOutTensor,
                                  appCntxt->outTensorNumDim[1],
                                  start,
                                  appCntxt->outTensorDims[1],
                                  &mapId,
                                  strides,
                                  (void **)&dPtr,
                                  VX_READ_AND_WRITE,
                                  VX_MEMORY_TYPE_HOST);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("tivxMapTensorPatch() failed.");
    }
    else
    {
        auto  *tensor = outputTensorVec[1];

        if (tensor->type == DlInferType_Float32)
        {
            float   *sPtr   = reinterpret_cast<float*>(tensor->data);
            CM_copyTensorData(sPtr, dPtr, tensor->numElem);
        }
        else
        {
            LOG_ERROR("Invalid data type.\n");
        }

        vxStatus = tivxUnmapTensorPatch(vxOutTensor, mapId);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("tivxUnmapTensorPatch() failed.");
        }

    }

    return vxStatus;
}


vx_status VISLOC_popFreeInputDesc(VISLOC_Context        *appCntxt,
                                  VISLOC_graphParams   **gpDesc)
{
    vx_status  vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->freeQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status VISLOC_popPreprocInputDesc(VISLOC_Context       *appCntxt,
                                     VISLOC_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->preProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status VISLOC_popDlInferInputDesc(VISLOC_Context       *appCntxt,
                                     VISLOC_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->dlInferQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status VISLOC_popVisLocInputDesc(VISLOC_Context       *appCntxt,
                                    VISLOC_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->visLocQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status VISLOC_getVisLocInputDesc(VISLOC_Context       *appCntxt,
                                    VISLOC_graphParams   **gpDesc)
{
    vx_status            vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->visLocQ.peek();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status VISLOC_getOutputDesc(VISLOC_Context       *appCntxt,
                               VISLOC_graphParams   *gpDesc)
{
    VISLOC_graphParams  *desc;
    vx_status            vxStatus = VX_SUCCESS;

    desc = appCntxt->outputQ.peek();

    if (desc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }
    else
    {
        *gpDesc = *desc;
    }

    return vxStatus;
}

vx_status VISLOC_popOutputDesc(VISLOC_Context       *appCntxt,
                               VISLOC_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->outputQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


void VISLOC_enqueInputDesc(VISLOC_Context      *appCntxt,
                           VISLOC_graphParams  *gpDesc)
{
    appCntxt->freeQ.push(gpDesc);
}


void VISLOC_enquePreprocInputDesc(VISLOC_Context      *appCntxt,
                                  VISLOC_graphParams  *gpDesc)
{
    appCntxt->preProcQ.push(gpDesc);
}

void VISLOC_enqueDlInferInputDesc(VISLOC_Context      *appCntxt,
                                  VISLOC_graphParams  *gpDesc)
{
    appCntxt->dlInferQ.push(gpDesc);
}

void VISLOC_enqueVisLocInputDesc(VISLOC_Context      *appCntxt,
                                 VISLOC_graphParams  *gpDesc)
{
    appCntxt->visLocQ.push(gpDesc);
}


void VISLOC_enqueOutputDesc(VISLOC_Context      *appCntxt,
                            VISLOC_graphParams  *gpDesc)
{
    appCntxt->outputQ.push(gpDesc);
}


vx_status VISLOC_processEvent(VISLOC_Context * appCntxt, vx_event_t * event)
{
    vx_reference            ref;
    uint8_t                 i;
    uint32_t                numRefs;
    uint32_t                index;
    vx_status               vxStatus;

    // For profiling
    float                   diff;

    ref      = NULL;
    vxStatus = (vx_status)VX_SUCCESS;

    if (event->type == VX_EVENT_NODE_COMPLETED)
    {
        uint32_t appValue = appCntxt->vxEvtAppValBase +
                            VISLOC_SCALER_NODE_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            LOG_ERROR("Unknown App Value [%d].\n", event->app_value);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Wakeup the pre-process thread. The pre-process thread will
             * process the descriptor at the head of the queue.
             */
            appCntxt->preProcSem->notify();
        }
    } else
    if (event->type == VX_EVENT_GRAPH_COMPLETED)
    {
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->startPerfCapt == false)
            {
                appCntxt->startPerfCapt = true;
            }
            else
            {
                appPerfPointEnd(&appCntxt->vlPerf);

                appCntxt->profileEnd = GET_TIME();
                diff  = GET_DIFF(appCntxt->profileStart, appCntxt->profileEnd);
                CM_reportProctime("Inter_frame_interval", diff);
            }

            appPerfPointBegin(&appCntxt->vlPerf);
            appCntxt->profileStart = GET_TIME();

            /* Node execution is complete. Deque all the parameters
             * for this node.
             */
            for (i = 0; i < appCntxt->numGraphParams; i++)
            {
                vxStatus = vxGraphParameterDequeueDoneRef(appCntxt->vxGraph,
                                                          i,
                                                          &ref,
                                                          1,
                                                          &numRefs);
                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("vxGraphParameterDequeueDoneRef() failed\n");
                    break;
                }
            }
        }


        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Notify the output semaphoe */
            if (appCntxt->outputCtrlSem)
            {
                appCntxt->outputCtrlSem->notify();
            }
        }
    }

    return vxStatus;
}


vx_status VISLOC_getOutBuff(VISLOC_Context   * appCntxt,
                            vx_image         * inputImage,
                            vx_image         * outputImage,
                            vx_matrix        * outputPose,
                            vx_uint64        * timestamp)
{
    vx_status            vxStatus;
    VISLOC_graphParams   desc;

    vxStatus = VISLOC_getOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        *inputImage  = desc.vxInputImage;
        *outputImage = desc.vxOutputImage;
        *outputPose  = desc.vxPoseMatrix;
        *timestamp   = *desc.timestamp;
    }
    else
    {
        LOG_ERROR("VISLOC_getOutputDesc() failed\n");
    }


    return vxStatus;

}

vx_status VISLOC_releaseOutBuff(VISLOC_Context * appCntxt)
{
    vx_status                   vxStatus;
    VISLOC_graphParams        * desc;

    /* Pop the output descriptor. */
    vxStatus = VISLOC_popOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Enqueue the descriptor to the free descriptor queue. */
        VISLOC_enqueInputDesc(appCntxt, desc);
    } else
    {
        LOG_ERROR("VISLOC_popOutputDesc() failed\n");
    }

    return vxStatus;
}


static vx_status VISLOC_createInputImageObjects(VISLOC_Context * appCntxt)
{
    int32_t i;
    int32_t vxStatus = VX_SUCCESS;

    // Create input image object
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            // create objects
            appCntxt->vxInputImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->inputImageWidth,
                              appCntxt->inputImageHeight,
                              (appCntxt->inputFormat == CM_IMG_FORMAT_UYVY ? VX_DF_IMAGE_UYVY:VX_DF_IMAGE_NV12));

            if (appCntxt->vxInputImage[i] == NULL)
            {
                LOG_ERROR("vxCreateImage() failed\n");

                vxStatus = VX_FAILURE;
                break;
            }

            vxSetReferenceName((vx_reference)appCntxt->vxInputImage[i],
                               "InputImage");
        }
    }

    // Create rectified image object
    if ((vxStatus == (vx_status)VX_SUCCESS) && appCntxt->enableLdcNode)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxRectifiedImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->inputImageWidth,
                              appCntxt->inputImageHeight,
                              VX_DF_IMAGE_NV12);

            if (appCntxt->vxRectifiedImage[i] == NULL)
            {
                LOG_ERROR("vxCreateImage() failed\n");

                vxStatus = VX_FAILURE;
                break;
            }

            vxSetReferenceName((vx_reference)appCntxt->vxRectifiedImage[i],
                               "RectifiedImage_NV12");
        }
    }

    return vxStatus;
}
