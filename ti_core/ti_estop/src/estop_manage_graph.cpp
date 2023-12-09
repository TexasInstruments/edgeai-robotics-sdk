/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
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

#include "estop.h"

vx_status ESTOP_APP_init_LDC(ESTOP_APP_Context *appCntxt)
{
    SDELDC_createParams * createParams;
    int32_t                     i;
    vx_status                   vxStatus = VX_SUCCESS;

    createParams            = &appCntxt->sdeLdcCreateParams;
    createParams->vxContext = appCntxt->vxContext;
    createParams->vxGraph   = appCntxt->vxGraph;

    /*
     * Create input image objects
     */
    createParams->createInputFlag    = 0;
    createParams->inputPipelineDepth = appCntxt->pipelineDepth;

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // input left image
        if (appCntxt->inputFormat == CM_IMG_FORMAT_Y)
        {
            appCntxt->vxInputLeftImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_U8);
        }
        else if (appCntxt->inputFormat == CM_IMG_FORMAT_UYVY)
        {
            appCntxt->vxInputLeftImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_UYVY);
        } else
        {
            LOG_ERROR("Input image format NOT supported\n");
            vxStatus = VX_FAILURE;
            break;
        }

        if (appCntxt->vxInputLeftImage[i] == NULL)
        {
            LOG_ERROR("vxCreateImage() failed\n");
            vxStatus = VX_FAILURE;
            break;
        }

        vxSetReferenceName((vx_reference)appCntxt->vxInputLeftImage[i],
                           "InputLeftImage");

        // input right image
        if (appCntxt->inputFormat == CM_IMG_FORMAT_Y)
        {
            appCntxt->vxInputRightImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_U8);
        }
        else if (appCntxt->inputFormat == CM_IMG_FORMAT_UYVY)
        {
            appCntxt->vxInputRightImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_UYVY);
        } else
        {
            LOG_ERROR("Input image format NOT supported\n");
            vxStatus = VX_FAILURE;
            break;
        }

        if (appCntxt->vxInputRightImage[i] == NULL)
        {
            LOG_ERROR("vxCreateImage() failed\n");
            vxStatus = VX_FAILURE;
            break;
        }

        vxSetReferenceName((vx_reference)appCntxt->vxInputRightImage[i],
                           "InputRightImage");

        // pass to LDC Applib createParams
        createParams->vxInputLeftImage[i]  = appCntxt->vxInputLeftImage[i];
        createParams->vxInputRightImage[i] = appCntxt->vxInputRightImage[i];
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        /*
         * Create output image objects
         */
        createParams->createOutputFlag      = 0;
        createParams->outputPipelineDepth   = 1;

        // output left image
        if (appCntxt->inputFormat == CM_IMG_FORMAT_Y)
        {
            appCntxt->vxLeftRectImage =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_U8);
        }
        else
        {
            appCntxt->vxLeftRectImage =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_NV12);
        }

        if (appCntxt->vxLeftRectImage == NULL)
        {
            LOG_ERROR("vxCreateImage() failed\n");
            vxStatus = VX_FAILURE;
        }

        vxSetReferenceName((vx_reference)appCntxt->vxLeftRectImage,
                           "LeftRectifiedImage");

        // pass to LDC Applib createParams
        createParams->vxOutputLeftImage[0] = appCntxt->vxLeftRectImage;

        // output right image
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            if (appCntxt->inputFormat == CM_IMG_FORMAT_Y)
            {
                appCntxt->vxRightRectImage[i] =
                    vxCreateImage(appCntxt->vxContext,
                                  appCntxt->width,
                                  appCntxt->height,
                                  VX_DF_IMAGE_U8);
            }
            else
            {
                appCntxt->vxRightRectImage[i] =
                    vxCreateImage(appCntxt->vxContext,
                                  appCntxt->width,
                                  appCntxt->height,
                                  VX_DF_IMAGE_NV12);
            }

            if (appCntxt->vxRightRectImage[i] == NULL)
            {
                LOG_ERROR("vxCreateImage() failed\n");
                vxStatus = VX_FAILURE;
                break;
            }

            vxSetReferenceName((vx_reference)appCntxt->vxRightRectImage[i],
                               "RightRectifiedImage");

            // pass to LDC Applib createParams
            createParams->vxOutputRightImage[i] = appCntxt->vxRightRectImage[i];
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->sdeLdcHdl = SDELDC_create(createParams);

        if (appCntxt->sdeLdcHdl == NULL)
        {
            LOG_ERROR("SDELDC_create() failed\n");
            vxStatus = VX_FAILURE;;
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_init_SDE(ESTOP_APP_Context *appCntxt)
{
    int32_t i;
    vx_status vxStatus = VX_SUCCESS;

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDE_createParams   * slSdeCreateParams;

        slSdeCreateParams            = &appCntxt->slSdeCreateParams;
        slSdeCreateParams->vxContext = appCntxt->vxContext;
        slSdeCreateParams->vxGraph   = appCntxt->vxGraph;

        /*
         * Create input image objects for SL SDE
         */
        slSdeCreateParams->createInputFlag     = 0;
        slSdeCreateParams->inputPipelineDepth  = 1;
        slSdeCreateParams->vxLeftRectImage[0]  = appCntxt->vxLeftRectImage;
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            slSdeCreateParams->vxRightRectImage[i] =
                appCntxt->vxRightRectImage[i];
        }

        /*
         * Create output image objects for SL SDE
         */
        slSdeCreateParams->createOutputFlag = 0;

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxSde16BitOutput[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_S16);

            if (appCntxt->vxSde16BitOutput[i] == NULL)
            {
                LOG_ERROR("vxCreateImage() failed\n");
                vxStatus = VX_FAILURE;
                break;
            }
            vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutput[i],
                               "RawDisparityMap");

            slSdeCreateParams->vxSde16BitOutput[i] =
                appCntxt->vxSde16BitOutput[i];
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            appCntxt->slSdeHdl = SL_SDE_create(slSdeCreateParams);

            if (appCntxt->slSdeHdl == NULL)
            {
                LOG_ERROR("SL_SDE_create() failed\n");

                vxStatus = VX_FAILURE;;
            }
        }
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDE_createParams   * mlSdeCreateParams;

        mlSdeCreateParams            = &appCntxt->mlSdeCreateParams;
        mlSdeCreateParams->vxContext = appCntxt->vxContext;
        mlSdeCreateParams->vxGraph   = appCntxt->vxGraph;

        /*
         * Create input image objects for SL SDE
         */
        mlSdeCreateParams->createInputFlag      = 0;
        mlSdeCreateParams->inputPipelineDepth   = 1;
        mlSdeCreateParams->vxLeftRectImageL0[0] = appCntxt->vxLeftRectImage;

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            mlSdeCreateParams->vxRightRectImageL0[i] =
                appCntxt->vxRightRectImage[i];
        }

        /*
         * Create output image objects for ML SDE
         */
        mlSdeCreateParams->createOutputFlag  = 0;
        if (appCntxt->ppMedianFilterEnable)
        {
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                appCntxt->vxMedianFilteredDisparity[i] =
                    vxCreateImage(appCntxt->vxContext,
                                  appCntxt->width,
                                  appCntxt->height,
                                  VX_DF_IMAGE_S16);

                if (appCntxt->vxMedianFilteredDisparity[i] == NULL)
                {
                    LOG_ERROR("vxCreateImage() failed\n");
                    vxStatus = VX_FAILURE;
                    break;
                }

                mlSdeCreateParams->vxMedianFilteredDisparity[i] =
                    appCntxt->vxMedianFilteredDisparity[i];
            }
        }

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxMergeDisparityL0[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width,
                              appCntxt->height,
                              VX_DF_IMAGE_S16);

            if (appCntxt->vxMergeDisparityL0[i] == NULL)
            {
                LOG_ERROR("vxCreateImage() failed\n");
                vxStatus = VX_FAILURE;;
                break;
            }
            mlSdeCreateParams->vxMergeDisparityL0[i] =
                appCntxt->vxMergeDisparityL0[i];
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            appCntxt->mlSdeHdl = ML_SDE_create(mlSdeCreateParams);
            if (appCntxt->mlSdeHdl == NULL)
            {
                LOG_ERROR("ML_SDE_create() failed\n");
                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_init_SS(ESTOP_APP_Context *appCntxt)
{
    DLInferer          *inferer;
    const VecDlTensor  *dlInfInputs;
    const VecDlTensor  *dlInfOutputs;
    const DlTensor     *ifInfo;
    CM_ScalerNodeCntxt *scalerObj;
    int32_t             numInputs;
    int32_t             numOutputs;
    vx_image            scalerInput;
    vx_status           vxStatus;
    int32_t             i;

    scalerObj    = &appCntxt->scalerObj;
    inferer      = appCntxt->dlInferer;
    dlInfInputs  = inferer->getInputInfo();
    dlInfOutputs = inferer->getOutputInfo();
    ifInfo       = &dlInfOutputs->at(0);
    numInputs    = dlInfInputs->size();
    numOutputs   = dlInfOutputs->size();
    vxStatus     = VX_SUCCESS;

    const std::string   taskType = appCntxt->postProcObj->getTaskType();

    if (taskType == "segmentation")
    {
        /* The SDE PC node expects a tensor of size 3 only. */
        // ifInfo->shape = [1, 1, 1, 1, H, W]  => outTensorDims = [1, H, W]
        appCntxt->outTensorNumDim = ifInfo->dim - 3;

        for (int32_t i = 0; i < appCntxt->outTensorNumDim; i++)
        {
            appCntxt->outTensorDims[i] = ifInfo->shape[i+3];
        }
    }
    else
    {
        LOG_ERROR("Unsupported taskType.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // pre-compute params for tivxMapTensorPatch()
        appCntxt->outTensorSize = 1;
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
                               VX_TYPE_UINT8,
                               0);

            if (appCntxt->vxOutTensor[i] == NULL)
            {
                LOG_ERROR("vxCreateTensor() failed.\n");
                vxStatus = VX_FAILURE;
                break;
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
            /* Allocate DL input buffers. Currently, the input buffers are held
             * for each pipelined outputs so we need pipeline number of input
             * buffers.
             */
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                /* Allocate the input buffer. */
                vxStatus = CM_createDlBuffers(inferer,
                                              dlInfInputs,
                                              appCntxt->inferInputBuff[i]);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("Failed to allocate input (%d)\n", i);
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
                        LOG_ERROR("Failed to allocate output (%d)\n", i);
                        break;
                    }
                }
            }
        }
    }

    /* Input image will be the scaler default input. */
    scalerInput = appCntxt->vxRightRectImage[0];

    /************************/
    /*    Scaler node       */
    /************************/
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        CM_ScalerCreateParams   params;

        params.width         = appCntxt->tensor_width;
        params.height        = appCntxt->tensor_height;
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

vx_status ESTOP_APP_deinit_SS(ESTOP_APP_Context *appCntxt)
{
    vx_status   vxStatus = VX_SUCCESS;
    int32_t     i;

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


vx_status ESTOP_APP_init_SS_Detection(ESTOP_APP_Context *appCntxt)
{
    SS_DETECT_createParams  *ssDetectCreateParams;
    vx_tensor                      *vxOutTensor;
    int32_t                         vxStatus = VX_SUCCESS;
    int32_t                         i;

    ssDetectCreateParams                = &appCntxt->ssDetectCreateParams;
    ssDetectCreateParams->vxContext     = appCntxt->vxContext;
    ssDetectCreateParams->vxGraph       = appCntxt->vxGraph;

    /*
     * Create input image objects for SL SDE
     */
    ssDetectCreateParams->createInputFlag = 0;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // right rectified image
        ssDetectCreateParams->vxRightRectImage[i] =
            appCntxt->vxRightRectImage[i];

        // raw disparity image
        if (appCntxt->sdeAlgoType == 0)
        {
            ssDetectCreateParams->vxSde16BitOutput[i] =
                appCntxt->vxSde16BitOutput[i];
        }
        else
        {
            if (appCntxt->ppMedianFilterEnable)
            {
                ssDetectCreateParams->vxSde16BitOutput[i] =
                    appCntxt->vxMedianFilteredDisparity[i];
            }
            else
            {
                ssDetectCreateParams->vxSde16BitOutput[i] =
                    appCntxt->vxMergeDisparityL0[i];
            }
        }
    }

    vxOutTensor = appCntxt->vxOutTensor;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        ssDetectCreateParams->vxSSMapTensor[i] = vxOutTensor[i];
    }

    // output objects are created in applib
    ssDetectCreateParams->createOutputFlag = 0;

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->ssDetectHdl = SS_DETECT_create(ssDetectCreateParams);
        if (appCntxt->ssDetectHdl == NULL)
        {
            LOG_ERROR("SS_DETECT_create() failed\n");
            vxStatus = VX_FAILURE;
        }
    }

    // get the output object from ss_detect applib
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        appCntxt->vx3DBoundBox[i] =
            SS_DETECT_get3DBBObject(appCntxt->ssDetectHdl, i);
    }

    return vxStatus;
}

vx_status  ESTOP_APP_setupPipeline(ESTOP_APP_Context * appCntxt)
{
    uint32_t   appValue;
    vx_status  vxStatus = VX_SUCCESS;

    if (appCntxt->sdeAlgoType == 0)
    {
        vxStatus = ESTOP_APP_setupPipeline_SL(appCntxt);
    } else
    {
        vxStatus = ESTOP_APP_setupPipeline_ML(appCntxt);
    }

    if (vxStatus == VX_SUCCESS)
    {
        appValue = appCntxt->vxEvtAppValBase + ESTOP_APP_GRAPH_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)appCntxt->vxGraph,
                                   VX_EVENT_GRAPH_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("vxRegisterEvent() failed\n");
        }
    }


    if (vxStatus == VX_SUCCESS)
    {
        CM_ScalerNodeCntxt * scalerObj = &appCntxt->scalerObj;

        appValue = appCntxt->vxEvtAppValBase + ESTOP_APP_SCALER_NODE_COMPLETE_EVENT;

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

vx_status ESTOP_APP_setupPipeline_SL(ESTOP_APP_Context * appCntxt)
{
    ESTOP_APP_graphParams             * paramDesc;
    CM_ScalerNodeCntxt                 *scalerObj;
    vx_tensor                          *vxOutTensor;
    vx_graph_parameter_queue_params_t   q[ESTOP_APP_NUM_GRAPH_PARAMS];
    vx_node                             leftLdcNode;
    vx_node                             rightLdcNode;
    vx_node                             sdeNode;
    vx_node                             pcNode;
    vx_node                             ogNode;
    vx_status                           vxStatus;
    uint32_t                            cnt = 0;
    uint32_t                            i;

    appCntxt->numGraphParams = 5;
    scalerObj    = &appCntxt->scalerObj;
    vxOutTensor  = appCntxt->vxOutTensor;
    leftLdcNode  = SDELCD_getLeftLDCNode(appCntxt->sdeLdcHdl);
    rightLdcNode = SDELCD_getRightLDCNode(appCntxt->sdeLdcHdl);
    sdeNode      = SL_SDE_getSDENode(appCntxt->slSdeHdl);
    pcNode       = SS_DETECT_getPCNode(appCntxt->ssDetectHdl);
    ogNode       = SS_DETECT_getOGNode(appCntxt->ssDetectHdl);

    /* LDC left node Param 6 (leftInputImg) ==> graph param 0. */
    vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                      leftLdcNode,
                                      6);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("CM_addParamByNodeIndex() failed\n");
    }
    else
    {
        q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputLeftImage;
    }

    /* LDC right node Param 6 (rightInputImg) ==> graph param 1. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          rightLdcNode,
                                          6);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputRightImage;
        }
    }

    /* LDC right node Param 7 (rightOutputImg)==> graph param 2. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          rightLdcNode,
                                          7);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxRightRectImage;
        }
    }

    /* vxDmpacSdeNode Param 3 (vxSde16BitOutput) ==> graph param 3 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          sdeNode,
                                          3);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxSde16BitOutput;
        }
    }

    /* vxOGNode Param 2 (vx3DBoundBox) ==> graph param 4. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        CM_addParamByNodeIndex(appCntxt->vxGraph,
                               ogNode,
                               2);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vx3DBoundBox;
        }
    }


    /*  scalerObj->vxNode Param 1 (outImage)==> graph param 5 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

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

    /*  pcNode Param 3 (vxSSMapTensor)==> graph param 6 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          pcNode,
                                          3);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)vxOutTensor;
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
            paramDesc                    = &appCntxt->paramDesc[i];
            paramDesc->vxInputLeftImage  = appCntxt->vxInputLeftImage[i];
            paramDesc->vxInputRightImage = appCntxt->vxInputRightImage[i];
            paramDesc->vxRightRectImage  = appCntxt->vxRightRectImage[i];
            paramDesc->vxSde16BitOutput  = appCntxt->vxSde16BitOutput[i];
            paramDesc->vx3DBoundBox      = appCntxt->vx3DBoundBox[i];
            paramDesc->vxScalerOut       = scalerObj->outImage[i];
            paramDesc->vxOutTensor       = vxOutTensor[i];
            paramDesc->inferInputBuff    = &appCntxt->inferInputBuff[i];
            paramDesc->inferOutputBuff   = &appCntxt->inferOutputBuff[i];
            paramDesc->timestamp         = &appCntxt->timestamp[i];

            ESTOP_APP_enqueInputDesc(appCntxt, paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);

        if (vxStatus != VX_SUCCESS)
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
        /*  vxLeftRectImage */
        vxStatus = tivxSetNodeParameterNumBufByIndex(leftLdcNode,
                                                     7,
                                                     appCntxt->pipelineDepth);
        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
        }
    }

    if (vxStatus == VX_SUCCESS)
    {
        // vxPointCloud */
        vxStatus =
            tivxSetNodeParameterNumBufByIndex(pcNode,
                                              4,
                                              appCntxt->pipelineDepth);
        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
        }
    }

    return vxStatus;
}


vx_status ESTOP_APP_setupPipeline_ML(ESTOP_APP_Context * appCntxt)
{
    CM_ScalerNodeCntxt                 *scalerObj;
    vx_tensor                          *vxOutTensor;
    ESTOP_APP_graphParams              *paramDesc;
    vx_graph_parameter_queue_params_t   q[ESTOP_APP_NUM_GRAPH_PARAMS];
    vx_node                             leftLdcNode;
    vx_node                             rightLdcNode;
    vx_node                             sdeNodeL0;
    vx_node                             sdeNodeL1;
    vx_node                             sdeNodeL2;
    vx_node                             medFilterNode;
    vx_node                             mergeNodeL1;
    vx_node                             mergeNodeL2;
    vx_node                             leftMscNodeL1;
    vx_node                             rightMscNodeL1;
    vx_node                             leftMscNodeL2;
    vx_node                             rightMscNodeL2;
    vx_node                             pcNode;
    vx_node                             ogNode;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;
    uint32_t                            i;

    appCntxt->numGraphParams = 5;
    scalerObj      = &appCntxt->scalerObj;
    vxOutTensor    = appCntxt->vxOutTensor;
    leftLdcNode    = SDELCD_getLeftLDCNode(appCntxt->sdeLdcHdl);
    rightLdcNode   = SDELCD_getRightLDCNode(appCntxt->sdeLdcHdl);
    sdeNodeL0      = ML_SDE_getSDENodeL0(appCntxt->mlSdeHdl);
    sdeNodeL1      = ML_SDE_getSDENodeL1(appCntxt->mlSdeHdl);
    sdeNodeL2      = ML_SDE_getSDENodeL2(appCntxt->mlSdeHdl);
    medFilterNode  = ML_SDE_getMedFilterNode(appCntxt->mlSdeHdl);
    mergeNodeL1    = ML_SDE_getMergeNodeL1(appCntxt->mlSdeHdl);
    mergeNodeL2    = ML_SDE_getMergeNodeL2(appCntxt->mlSdeHdl);
    leftMscNodeL1  = ML_SDE_getLeftMSCNodeL1(appCntxt->mlSdeHdl);
    rightMscNodeL1 = ML_SDE_getRightMSCNodeL1(appCntxt->mlSdeHdl);
    leftMscNodeL2  = ML_SDE_getLeftMSCNodeL2(appCntxt->mlSdeHdl);
    rightMscNodeL2 = ML_SDE_getRightMSCNodeL2(appCntxt->mlSdeHdl);
    pcNode         = SS_DETECT_getPCNode(appCntxt->ssDetectHdl);
    ogNode         = SS_DETECT_getOGNode(appCntxt->ssDetectHdl);

    /* LDC left node Param 6 (leftInputImg) ==> graph param 0. */
    vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                      leftLdcNode,
                                      6);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("CM_addParamByNodeIndex() failed\n");
    }
    else
    {
        q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputLeftImage;
    }

    /* LDC rigth node Param 6 (rightInputImg) ==> graph param 1. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          rightLdcNode,
                                          6);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxInputRightImage;
        }
    }

    /* LDC right node Param 7  (rightOutputImg) ==> graph param 2. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          rightLdcNode,
                                          7);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxRightRectImage;
        }
    }

    /* vxDisparityMergeNodeL1 Param 3 (vxMergeDisparityL0) => graph param 3. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          mergeNodeL1,
                                          3);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxMergeDisparityL0;
        }
    }

    // Always ppMedianFilterEnable = 0
    if (vxStatus == (vx_status)VX_SUCCESS && appCntxt->ppMedianFilterEnable)
    {
        appCntxt->numGraphParams += 1;

        /* vxMedianFilterNode Param 2 (vxMedianFilteredDisparity) => graph param 3. */
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          medFilterNode,
                                          2);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxMedianFilteredDisparity;
        }
    }

    /* vxOGNode Param 2 ==> graph param 4. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          ogNode,
                                          2);
        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vx3DBoundBox;
        }
    }


    /*  scalerObj->vxNode Param 1 (outImage)==> graph param 5 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

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

    /*  pcNode Param 3 (vxSSMapTensor)==> graph param 6 */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          pcNode,
                                          3);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)vxOutTensor;
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
            paramDesc                      = &appCntxt->paramDesc[i];
            paramDesc->vxInputLeftImage    = appCntxt->vxInputLeftImage[i];
            paramDesc->vxInputRightImage   = appCntxt->vxInputRightImage[i];
            paramDesc->vxRightRectImage    = appCntxt->vxRightRectImage[i];
            paramDesc->vxMergeDisparityL0  = appCntxt->vxMergeDisparityL0[i];

            if (appCntxt->ppMedianFilterEnable)
            {
                paramDesc->vxMedianFilteredDisparity =
                    appCntxt->vxMedianFilteredDisparity[i];
            }

            paramDesc->vx3DBoundBox    = appCntxt->vx3DBoundBox[i];
            paramDesc->vxScalerOut     = scalerObj->outImage[i];
            paramDesc->vxOutTensor     = vxOutTensor[i];
            paramDesc->inferInputBuff  = &appCntxt->inferInputBuff[i];
            paramDesc->inferOutputBuff = &appCntxt->inferOutputBuff[i];
            paramDesc->timestamp       = &appCntxt->timestamp[i];

            ESTOP_APP_enqueInputDesc(appCntxt, paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);

        if (vxStatus != VX_SUCCESS)
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
        /*  vxLeftRectImage */
        vxStatus =
            tivxSetNodeParameterNumBufByIndex(leftLdcNode,
                                              7,
                                              appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
        }
    }

    if (vxStatus == VX_SUCCESS)
    {
        /* vxSde16BitOutputL0 */
        vxStatus =
            tivxSetNodeParameterNumBufByIndex(sdeNodeL0,
                                              3,
                                              appCntxt->pipelineDepth);
        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
        }
    }

    if (appCntxt->numLayers > 1)
    {
        if (vxStatus == VX_SUCCESS)
        {
            /* vxLeftImageL1 */
            vxStatus =
                tivxSetNodeParameterNumBufByIndex(leftMscNodeL1,
                                                  1,
                                                  appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }

        if (vxStatus == VX_SUCCESS)
        {
            /* vxRightImageL1 */
            vxStatus =
                tivxSetNodeParameterNumBufByIndex(rightMscNodeL1,
                                                  1,
                                                  appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }

        if (vxStatus == VX_SUCCESS)
        {
            /* vxSde16BitOutputL1 */
            vxStatus =
                tivxSetNodeParameterNumBufByIndex(sdeNodeL1,
                                                  3,
                                                  appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }
    }


    if (appCntxt->numLayers > 2)
    {
        if (vxStatus == VX_SUCCESS)
        {
            /* vxLeftImageL2 */
            vxStatus =
                tivxSetNodeParameterNumBufByIndex(leftMscNodeL2,
                                                  1,
                                                  appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }

        if (vxStatus == VX_SUCCESS)
        {
            /* vxRightImageL2 */
            vxStatus =
                tivxSetNodeParameterNumBufByIndex(rightMscNodeL2,
                                                  1,
                                                  appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }

        if (vxStatus == VX_SUCCESS)
        {
            /* vxSde16BitOutputL2 */
            vxStatus =
                tivxSetNodeParameterNumBufByIndex(sdeNodeL2,
                                                  3,
                                                  appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }

        if (vxStatus == VX_SUCCESS)
        {
            /* vxMergeDisparityL1 */
            vxStatus =
                tivxSetNodeParameterNumBufByIndex(mergeNodeL2,
                                                  3,
                                                  appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }
    }

    if (vxStatus == VX_SUCCESS)
    {
        // vxPointCloud */
        vxStatus =
            tivxSetNodeParameterNumBufByIndex(pcNode,
                                              4,
                                              appCntxt->pipelineDepth);
        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
        }
    }

    return vxStatus;
}

void ESTOP_APP_printStats(ESTOP_APP_Context * appCntxt)
{
    tivx_utils_graph_perf_print(appCntxt->vxGraph);
    appPerfPointPrint(&appCntxt->estopPerf);
    LOG_INFO_RAW("\n");
    appPerfPointPrintFPS(&appCntxt->estopPerf);
    LOG_INFO_RAW("\n");
    CM_printProctime(stdout);
    LOG_INFO_RAW("\n");
}

vx_status ESTOP_APP_exportStats(ESTOP_APP_Context * appCntxt, FILE *fp, bool exportAll)
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
            app_perf_point_t *perfArr[1] = {&appCntxt->estopPerf};
            appPerfStatsExportAll(fp, perfArr, 1);
        }
    }

    return vxStatus;
}

vx_status ESTOP_APP_waitGraph(ESTOP_APP_Context * appCntxt)
{
    vx_status vxStatus = VX_SUCCESS;

    vxWaitGraph(appCntxt->vxGraph);

    /* Wait for the output queue to get flushed. */
    while (appCntxt->freeQ.size() != appCntxt->pipelineDepth)
    {
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }

    return vxStatus;
}

vx_status ESTOP_APP_popFreeInputDesc(ESTOP_APP_Context       *appCntxt,
                                     ESTOP_APP_graphParams  **gpDesc)
{
    vx_status vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->freeQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status ESTOP_APP_popPreprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                        ESTOP_APP_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->preProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}

vx_status ESTOP_APP_popDlInferInputDesc(ESTOP_APP_Context      *appCntxt,
                                        ESTOP_APP_graphParams **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->dlrQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status ESTOP_APP_popPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->postProcQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


vx_status ESTOP_APP_getPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc)
{
    ESTOP_APP_graphParams  *desc;
    vx_status               vxStatus = VX_SUCCESS;

    desc = appCntxt->postProcQ.peek();

    if (desc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }
    else
    {
        *gpDesc = desc;
    }


    return vxStatus;
}


vx_status ESTOP_APP_getOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams   *gpDesc)
{
    ESTOP_APP_graphParams  *desc;
    vx_status               vxStatus = VX_SUCCESS;

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


vx_status ESTOP_APP_popOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams  **gpDesc)
{
    vx_status   vxStatus = VX_SUCCESS;

    *gpDesc = appCntxt->outputQ.pop();

    if (*gpDesc == nullptr)
    {
        vxStatus = VX_FAILURE;
    }

    return vxStatus;
}


void ESTOP_APP_enqueInputDesc(ESTOP_APP_Context      *appCntxt,
                              ESTOP_APP_graphParams  *gpDesc)
{
    appCntxt->freeQ.push(gpDesc);
}


void ESTOP_APP_enquePreprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                     ESTOP_APP_graphParams  *gpDesc)
{
    appCntxt->preProcQ.push(gpDesc);
}

void ESTOP_APP_enqueDlInferInputDesc(ESTOP_APP_Context     *appCntxt,
                                     ESTOP_APP_graphParams *gpDesc)
{
    appCntxt->dlrQ.push(gpDesc);
}

void ESTOP_APP_enquePostprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                      ESTOP_APP_graphParams  *gpDesc)
{
    appCntxt->postProcQ.push(gpDesc);
}

void ESTOP_APP_enqueOutputDesc(ESTOP_APP_Context      *appCntxt,
                               ESTOP_APP_graphParams  *gpDesc)
{
    appCntxt->outputQ.push(gpDesc);
}

vx_status  ESTOP_APP_process(ESTOP_APP_Context * appCntxt, ESTOP_APP_graphParams * gpDesc)
{
    vx_status vxStatus = VX_SUCCESS;
    uint16_t   i;
    uint8_t   cnt = 0;

    vx_reference  obj[ESTOP_APP_NUM_GRAPH_PARAMS];

    obj[cnt++] = (vx_reference)gpDesc->vxInputLeftImage;
    obj[cnt++] = (vx_reference)gpDesc->vxInputRightImage;
    obj[cnt++] = (vx_reference)gpDesc->vxRightRectImage;

    if (appCntxt->sdeAlgoType == 0)
    {
        obj[cnt++] = (vx_reference)gpDesc->vxSde16BitOutput;
    }
    else
    {
        /* code */
        obj[cnt++] = (vx_reference)gpDesc->vxMergeDisparityL0;

        // ppMedianFilterEnable is always 0
        if (appCntxt->ppMedianFilterEnable)
        {
            obj[cnt++] = (vx_reference)gpDesc->vxMedianFilteredDisparity;
        }
    }

    obj[cnt++] = (vx_reference)gpDesc->vx3DBoundBox;

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
    ESTOP_APP_enquePreprocInputDesc(appCntxt, gpDesc);

    return vxStatus;
}


vx_status ESTOP_APP_CNN_preProcess(ESTOP_APP_Context   *appCntxt,
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
    rect.end_x   = appCntxt->tensor_width;
    rect.end_y   = appCntxt->tensor_height;

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

vx_status ESTOP_APP_createOutTensor(ESTOP_APP_Context  *appCntxt,
                                    VecDlTensorPtr     *outputTensorVec,
                                    vx_tensor           vxOutTensor)
{
    uint8_t    *dPtr;
    vx_size     start[ESTOP_APP_MAX_OUT_TENSOR_DIMS];
    vx_size     strides[ESTOP_APP_MAX_OUT_TENSOR_DIMS];
    vx_map_id   mapId;
    vx_status   vxStatus;

    memset(start, 0, sizeof(start));

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

vx_status ESTOP_APP_processEvent(ESTOP_APP_Context * appCntxt, vx_event_t * event)
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
                            ESTOP_APP_SCALER_NODE_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            LOG_ERROR("Unknown App Value [%d].\n", event->app_value);

            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Wakeup the DLR thread. The DLR thread will process the
             * descriptor at the head of the queue.
             */
            appCntxt->preProcSem->notify();
        }

    } else
    if (event->type == VX_EVENT_GRAPH_COMPLETED)
    {
        uint32_t appValue;

        appValue = appCntxt->vxEvtAppValBase + ESTOP_APP_GRAPH_COMPLETE_EVENT;

        if (event->app_value != appValue)
        {
            /* Something wrong. We did not register for this event. */
            LOG_ERROR("Unknown App Value [%d].\n", event->app_value);

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
                appPerfPointEnd(&appCntxt->estopPerf);

                appCntxt->profileEnd = GET_TIME();
                diff  = GET_DIFF(appCntxt->profileStart, appCntxt->profileEnd);
                CM_reportProctime("Inter_frame_interval", diff);
            }

            appPerfPointBegin(&appCntxt->estopPerf);
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

        ESTOP_APP_graphParams * desc;
        vxStatus = ESTOP_APP_popPostprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            ESTOP_APP_enqueOutputDesc(appCntxt, desc);

            /* Wakeup the display thread. */
            if (appCntxt->outputCtrlSem)
            {
                appCntxt->outputCtrlSem->notify();
            }
        }
    }

    return vxStatus;
}


vx_status ESTOP_APP_getOutBuff(ESTOP_APP_Context *appCntxt,
                               vx_image *rightRectImage,
                               vx_image *ssOutput,
                               vx_tensor *outTensor,
                               vx_user_data_object *obsBB,
                               vx_image *disparity16,
                               vx_uint64 *timestamp)
{
    vx_status               vxStatus = VX_SUCCESS;
    ESTOP_APP_graphParams   desc;

    vxStatus = ESTOP_APP_getOutputDesc(appCntxt, &desc);

    /* Get the descriptor. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        *rightRectImage = desc.vxRightRectImage;
        *ssOutput       = desc.vxScalerOut;
        *outTensor      = desc.vxOutTensor;

        if (appCntxt->sdeAlgoType == 0)
        {
            *disparity16 = desc.vxSde16BitOutput;
        }
        else if (appCntxt->ppMedianFilterEnable)
        {
            *disparity16 = desc.vxMedianFilteredDisparity;
        }
        else
        {
            *disparity16 = desc.vxMergeDisparityL0;
        }

        *obsBB = desc.vx3DBoundBox;
        *timestamp = *desc.timestamp;
    }

    return vxStatus;
}


vx_status ESTOP_APP_releaseOutBuff(ESTOP_APP_Context * appCntxt)
{
    vx_status                       vxStatus;
    ESTOP_APP_graphParams         * desc;

    /* Pop the output descriptor. */
    vxStatus = ESTOP_APP_popOutputDesc(appCntxt, &desc);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Enqueue the descriptor to the free descriptor queue. */
        ESTOP_APP_enqueInputDesc(appCntxt, desc);
    }

    return vxStatus;
}
