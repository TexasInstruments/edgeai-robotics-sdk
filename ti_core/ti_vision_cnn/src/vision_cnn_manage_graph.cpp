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

#include "vision_cnn.h"


vx_status VISION_CNN_init_SS(VISION_CNN_Context *appCntxt)
{
    DLInferer          *inferer;
    const VecDlTensor  *dlInfInputs;
    const VecDlTensor  *dlInfOutputs;
    const DlTensor     *ifInfo;
    CM_LdcNodeCntxt    *ldcObj;
    CM_ScalerNodeCntxt *scalerObj;
    int32_t             numInputs;
    int32_t             numOutputs;
    vx_image            scalerInput;
    vx_enum             tensorDataType;
    vx_status           vxStatus;
    int32_t             i;

    ldcObj       = &appCntxt->ldcObj;
    scalerObj    = &appCntxt->scalerObj;
    inferer      = appCntxt->dlInferer;
    dlInfInputs  = inferer->getInputInfo();
    dlInfOutputs = inferer->getOutputInfo();
    ifInfo       = &dlInfOutputs->at(0);
    numInputs    = dlInfInputs->size();
    numOutputs   = dlInfOutputs->size();
    vxStatus     = VX_SUCCESS;

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // create objects
        appCntxt->vxInputImage[i] = vxCreateImage(appCntxt->vxContext,
                                                  appCntxt->inputImageWidth,
                                                  appCntxt->inputImageHeight,
                                                  appCntxt->inputImageFormat);

        if (appCntxt->vxInputImage[i] == NULL)
        {
            LOG_ERROR("vxCreateImage() failed.\n");
            vxStatus = VX_FAILURE;
            break;
        }

        vxSetReferenceName((vx_reference)appCntxt->vxInputImage[i],
                           "InputImage");
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
                LOG_ERROR("vxCreateImage() failed.\n");
                vxStatus = VX_FAILURE;
                break;
            }

            vxSetReferenceName((vx_reference)appCntxt->vxRectifiedImage[i],
                               "RectifiedImage_NV12");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        switch (ifInfo->type)
        {
            case DlInferType_Int8:
                tensorDataType = VX_TYPE_INT8;
                break;

            case DlInferType_UInt8:
                tensorDataType = VX_TYPE_UINT8;
                break;

            case DlInferType_Int16:
                tensorDataType = VX_TYPE_INT16;
                break;

            case DlInferType_UInt16:
                tensorDataType = VX_TYPE_UINT16;
                break;

            case DlInferType_Int32:
                tensorDataType = VX_TYPE_INT32;
                break;

            case DlInferType_UInt32:
                tensorDataType = VX_TYPE_UINT32;
                break;

            case DlInferType_Int64:
                tensorDataType = VX_TYPE_INT64;
                break;

            case DlInferType_Float32:
                tensorDataType = VX_TYPE_FLOAT32;
                break;

            default:
                LOG_ERROR("Unsupported tensor data type.\n");
                vxStatus = VX_FAILURE;
                break;
        }
    }

    // Generalized to support segmentation and detection
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        const std::string   taskType = appCntxt->postProcObj->getTaskType();
        int32_t             elementSize;
        appCntxt->outTensorNumDim = ifInfo->dim;
        elementSize = getTypeSize(ifInfo->type);

        for (int32_t i = 0; i < appCntxt->outTensorNumDim; i++)
        {
            appCntxt->outTensorDims[i] = ifInfo->shape[i];
        }

        if (taskType == "detection")
        {
            //==> TODO: get this from the model interface info
            //==> one more for storing a field for the number of objects
            /* For ONNX models, the dimensions are negative. Detect this condition
             * and set default max values.
             */
            if (ifInfo->shape[0] < 0)
            {
                appCntxt->outTensorDims[0] = CM_POST_PROCESS_DETECT_MAX_OBJECTS + 1;
            }

            if (ifInfo->shape[1] < 0)
            {
                appCntxt->outTensorDims[1] = CM_POST_PROCESS_DETECT_NUM_FIELDS;
            }
        }
        else if ( taskType == "object_6d_pose_estimation")
        {
            if (ifInfo->shape[0] < 0)
            {
                appCntxt->outTensorDims[0] = CM_POST_PROCESS_DETECT_MAX_OBJECTS + 1;
            }

            if (ifInfo->shape[1] < 0)
            {
                appCntxt->outTensorDims[1] = CM_POST_PROCESS_POSE6D_NUM_FIELDS;
            }
        }
        else if (taskType != "segmentation" && taskType != "keypoint_detection")
        {
            LOG_ERROR("Unsupported taskType");
            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            // pre-compute params for tivxMapTensorPatch()
            appCntxt->outTensorSize = elementSize;
            for (int32_t i = 0; i < appCntxt->outTensorNumDim; i++)
            {
                appCntxt->outTensorSize *= appCntxt->outTensorDims[i];
            }


            // vxCreateTensor
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                appCntxt->vxOutTensor[i] =
                    vxCreateTensor(appCntxt->vxContext,
                                   appCntxt->outTensorNumDim,
                                   appCntxt->outTensorDims,
                                   tensorDataType,
                                   0);

                if (appCntxt->vxOutTensor[i] == NULL)
                {
                    LOG_ERROR("vxCreateTensor() failed.\n");
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
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                /* Allocate the input buffer. */
                status = CM_createDlBuffers(inferer,
                                            dlInfInputs,
                                            appCntxt->inferInputBuff[i]);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("Failed to allocate input (%d)\n",
                                __FUNCTION__, __LINE__, i);
                    break;
                }
            }

            /* Allocate DL output buffers. Currenly, the input buffers are held
             * for each pipelined outputs so we need pipeline number of input
             * buffers.
             */
            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Allocate the output buffer. */
                for (i = 0; i < appCntxt->pipelineDepth; i++)
                {
                    /* Allocate the input buffer. */
                    vxStatus = CM_createDlBuffers(inferer,
                                                  dlInfOutputs,
                                                  appCntxt->inferOutputBuff[i]);

                    if (vxStatus != (vx_status)VX_SUCCESS)
                    {
                        LOG_ERROR("Failed to allocate output (%d)\n",
                                    __FUNCTION__, __LINE__, i);
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

        vxStatus = CM_ldcNodeCntxtInit(&appCntxt->ldcObj,
                                       appCntxt->vxContext,
                                       &params);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_ldcNodeCntxtInit() failed\n");
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

        vxStatus = CM_scalerNodeCntxtInit(&appCntxt->scalerObj,
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

    return vxStatus;
}

vx_status VISION_CNN_deinit_core(VISION_CNN_Context *appCntxt)
{
    vx_status   vxStatus = VX_SUCCESS;
    int32_t     i;

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

    // vxOutTensor is always created
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (appCntxt->vxOutTensor[i] != NULL)
            {
                vxReleaseTensor(&appCntxt->vxOutTensor[i]);
            }
        }
    }

    /* De-allocate the input and output buffers. */
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        appCntxt->inferInputBuff[i].clear();
        appCntxt->inferOutputBuff[i].clear();
    }

    return vxStatus;
}

vx_status  VISION_CNN_setupPipeline(VISION_CNN_Context * appCntxt)
{
    CM_LdcNodeCntxt                    *ldcObj;
    CM_ScalerNodeCntxt                 *scalerObj;
    VISION_CNN_graphParams             *paramDesc;
    vx_graph_parameter_queue_params_t   q[VISION_CNN_NUM_GRAPH_PARAMS];
    vx_status                           vxStatus;
    uint32_t                            appValue;
    uint32_t                            cnt;
    uint32_t                            i;

    ldcObj     = &appCntxt->ldcObj;
    scalerObj  = &appCntxt->scalerObj;
    cnt        = 0;

    if (appCntxt->enableLdcNode)
    {
        /* LDC node Param 6 ==> graph param 0. */
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

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* LDC node Param 7 ==> graph param 1. */
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

        /* Scaler node Param 9 ==> graph param 0 */
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

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Scaler node Param 1 ==> graph param 1/2. */
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

    appCntxt->numGraphParams = cnt;

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
            paramDesc->vxOutTensor      = appCntxt->vxOutTensor[i];
            paramDesc->timestamp        = &appCntxt->timestamp[i];
            paramDesc->inferInputBuff   = &appCntxt->inferInputBuff[i];
            paramDesc->inferOutputBuff  = &appCntxt->inferOutputBuff[i];
            paramDesc->vxScalerOut      = scalerObj->outImage[i];

            if (appCntxt->enableLdcNode)
            {
                paramDesc->vxRectifiedImage = appCntxt->vxRectifiedImage[i];
            }

            VISION_CNN_enqueInputDesc(appCntxt, paramDesc);
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

    if (vxStatus == VX_SUCCESS)
    {
        appValue = appCntxt->vxEvtAppValBase + VISION_CNN_SCALER_NODE_COMPLETE_EVENT;

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

void VISION_CNN_printStats(VISION_CNN_Context * appCntxt)
{
    tivx_utils_graph_perf_print(appCntxt->vxGraph);

    appPerfPointPrint(&appCntxt->visonPerf);
    LOG_INFO_RAW("\n");
    appPerfPointPrintFPS(&appCntxt->visonPerf);
    LOG_INFO_RAW("\n");
    CM_printProctime(stdout);
    LOG_INFO_RAW("\n");
}

vx_status VISION_CNN_exportStats(VISION_CNN_Context * appCntxt, FILE *fp, bool exportAll)
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
            app_perf_point_t *perfArr[1] = {&appCntxt->visonPerf};
            appPerfStatsExportAll(fp, perfArr, 1);
        }
    }

    return vxStatus;
}

vx_status VISION_CNN_waitGraph(VISION_CNN_Context * appCntxt)
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

vx_status  VISION_CNN_process(VISION_CNN_Context     * appCntxt,
                              VISION_CNN_graphParams * gpDesc,
                              uint64_t                 timestamp)
{
    vx_status vxStatus = VX_SUCCESS;
    uint16_t   i;
    uint8_t    cnt = 0;

    vx_reference  obj[VISION_CNN_NUM_GRAPH_PARAMS];

    // set timestamp of input image
    *gpDesc->timestamp = timestamp;

    obj[cnt++] = (vx_reference)gpDesc->vxInputImage;
    if (appCntxt->enableLdcNode)
    {
        obj[cnt++] = (vx_reference)gpDesc->vxRectifiedImage;
    }
    obj[cnt++] = (vx_reference)gpDesc->vxScalerOut;

    // Enqueue buffers
    // Enqueue vxOutTensor after DL is completed
    for (i = 0; i < appCntxt->numGraphParams; i++)
    {
        vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                   i,
                                                   &obj[i],
                                                   1);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxGraphParameterEnqueueReadyRef(%d) "
                       "failed\n", __FUNCTION__, __LINE__, i);
            break;
        }
    }

    /* Push the descriptor to the DL input queue. */
    VISION_CNN_enquePreprocInputDesc(appCntxt, gpDesc);

    return vxStatus;
}

vx_status VISION_CNN_preProcess(VISION_CNN_Context *appCntxt,
                                vx_image            vxScalerOut,
                                VecDlTensorPtr     *inputTensorVec)
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

    /* Take the output image from the scaler which is in NV12 format and do
     * the following:
     * - Color convert the image from YUV420 to BGR format
     * - Convert to float and write to the DL Infer input buffer
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

//==>> Generalized to support segmentation and detection
vx_status VISION_CNN_createOutTensor(VISION_CNN_Context   *appCntxt,
                                     VecDlTensorPtr       *outputTensorVec,
                                     vx_tensor             vxOutTensor)
{
    uint8_t    *dPtr;
    vx_size     start[VISION_CNN_MAX_OUT_TENSOR_DIMS];
    vx_size     strides[VISION_CNN_MAX_OUT_TENSOR_DIMS];
    vx_map_id   mapId;
    vx_status   vxStatus;

    memset(start, 0, sizeof(start));

    //==>> 10.0 debug
    // LOG_INFO("==> outTensorNumDim = %d \n", appCntxt->outTensorNumDim);
    // LOG_INFO("==> outTensorDims = [%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld]\n",
    //     appCntxt->outTensorDims[0],appCntxt->outTensorDims[1],appCntxt->outTensorDims[2],appCntxt->outTensorDims[3],
    //     appCntxt->outTensorDims[4],appCntxt->outTensorDims[5],appCntxt->outTensorDims[6],appCntxt->outTensorDims[7]);

    vxStatus = tivxMapTensorPatch(vxOutTensor,
                                  appCntxt->outTensorNumDim,
                                  start,
                                  appCntxt->outTensorDims,
                                  &mapId,
                                  strides,
                                  (void **)&dPtr,
                                  VX_READ_AND_WRITE,
                                  VX_MEMORY_TYPE_HOST);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("tivxMapTensorPatch() failed.\n");
    }
    else
    {
        (*appCntxt->postProcObj)(reinterpret_cast<void*>(dPtr),
                                 reinterpret_cast<void*>(dPtr),
                                 *outputTensorVec);

        vxStatus = tivxUnmapTensorPatch(vxOutTensor, mapId);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("tivxUnmapTensorPatch() failed.\n");
        }
    }

    return vxStatus;
}

vx_status VISION_CNN_popFreeInputDesc(VISION_CNN_Context        *appCntxt,
                                      VISION_CNN_graphParams   **gpDesc)
{
    vx_status  vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->freeQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status VISION_CNN_popPreprocInputDesc(VISION_CNN_Context       *appCntxt,
                                         VISION_CNN_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->preProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status VISION_CNN_popDlInferInputDesc(VISION_CNN_Context        *appCntxt,
                                         VISION_CNN_graphParams   **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->dlInferQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status VISION_CNN_popPostprocInputDesc(VISION_CNN_Context       *appCntxt,
                                          VISION_CNN_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->postProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status VISION_CNN_getOutputDesc(VISION_CNN_Context       *appCntxt,
                                   VISION_CNN_graphParams   *gpDesc)
{
    VISION_CNN_graphParams  *desc;
    vx_status                vxStatus = VX_SUCCESS;

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

vx_status VISION_CNN_popOutputDesc(VISION_CNN_Context       *appCntxt,
                                   VISION_CNN_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->outputQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

void VISION_CNN_enqueInputDesc(VISION_CNN_Context      *appCntxt,
                               VISION_CNN_graphParams  *gpDesc)
{
    appCntxt->freeQ.push(gpDesc);
}

void VISION_CNN_enquePreprocInputDesc(VISION_CNN_Context      *appCntxt,
                                      VISION_CNN_graphParams  *gpDesc)
{
    appCntxt->preProcQ.push(gpDesc);
}

void VISION_CNN_enqueDlInferInputDesc(VISION_CNN_Context       *appCntxt,
                                      VISION_CNN_graphParams   *gpDesc)
{
    appCntxt->dlInferQ.push(gpDesc);
}

void VISION_CNN_enquePostprocInputDesc(VISION_CNN_Context      *appCntxt,
                                       VISION_CNN_graphParams  *gpDesc)
{
    appCntxt->postProcQ.push(gpDesc);
}

void VISION_CNN_enqueOutputDesc(VISION_CNN_Context      *appCntxt,
                                VISION_CNN_graphParams  *gpDesc)
{
    appCntxt->outputQ.push(gpDesc);
}

vx_status VISION_CNN_processEvent(VISION_CNN_Context * appCntxt, vx_event_t * event)
{
    vx_reference            ref;
    uint8_t                 i;
    uint32_t                numRefs;
    vx_status               vxStatus;

    // For profiling
    float                   diff;

    ref      = NULL;
    vxStatus = (vx_status)VX_SUCCESS;

    if (event->type == VX_EVENT_NODE_COMPLETED)
    {
        uint32_t appValue = appCntxt->vxEvtAppValBase +
                            VISION_CNN_SCALER_NODE_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            LOG_ERROR("Unknown App Value [%d].\n",
                       __FUNCTION__, __LINE__, event->app_value);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->startPerfCapt == false)
            {
                appCntxt->startPerfCapt = true;
            }
            else
            {
                appPerfPointEnd(&appCntxt->visonPerf);

                appCntxt->profileEnd = GET_TIME();
                diff  = GET_DIFF(appCntxt->profileStart, appCntxt->profileEnd);
                CM_reportProctime("Inter_frame_interval", diff);
            }

            appPerfPointBegin(&appCntxt->visonPerf);
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
            /* Wakeup the pre-process thread. The pre-process thread will
             * process the descriptor at the head of the queue.
             */
            appCntxt->preProcSem->notify();
        }
    }

    return vxStatus;
}

vx_status VISION_CNN_getOutBuff(VISION_CNN_Context   * appCntxt,
                                vx_image             * inputImage,
                                vx_reference         * output,
                                vx_uint64            * timestamp)
{
    vx_status                vxStatus;
    VISION_CNN_graphParams   desc;

    vxStatus = VISION_CNN_getOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->enableLdcNode)
            {
                *inputImage = desc.vxRectifiedImage;
            }
            else
            {
                *inputImage = desc.vxInputImage;
            }

            *output = (vx_reference)desc.vxOutTensor;

            *timestamp = *desc.timestamp;
        }
        else
        {
            LOG_ERROR("VISION_CNN_getOutputDesc() failed\n");
        }
    }

    return vxStatus;
}

vx_status VISION_CNN_releaseOutBuff(VISION_CNN_Context * appCntxt)
{
    vx_status                       vxStatus;
    VISION_CNN_graphParams        * desc;

    /* Pop the output descriptor. */
    vxStatus = VISION_CNN_popOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Enqueue the descriptor to the free descriptor queue. */
        VISION_CNN_enqueInputDesc(appCntxt, desc);
    }

    return vxStatus;
}
