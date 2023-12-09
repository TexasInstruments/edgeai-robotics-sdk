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

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "sde.h"

vx_status SDEAPP_init_LDC(SDEAPP_Context *appCntxt)
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
    createParams->createInputFlag      = 0;
    createParams->inputPipelineDepth   = appCntxt->pipelineDepth;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // input left image
        if (appCntxt->inputFormat == CM_IMG_FORMAT_Y)
        {
            appCntxt->vxInputLeftImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width, appCntxt->height, VX_DF_IMAGE_U8);
        }
        else if (appCntxt->inputFormat == CM_IMG_FORMAT_UYVY)
        {
            appCntxt->vxInputLeftImage[i] =
                vxCreateImage(appCntxt->vxContext,
                              appCntxt->width, appCntxt->height, VX_DF_IMAGE_UYVY);
        }
        else
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
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxInputLeftImage[i],
                               "InputLeftImage");

            // pass to LDC Applib createParams
            createParams->vxInputLeftImage[i] = appCntxt->vxInputLeftImage[i];
        }

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
        }
        else
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
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxInputRightImage[i],
                               "InputRightImage");
            // pass to LDC Applib createParams
            createParams->vxInputRightImage[i] = appCntxt->vxInputRightImage[i];
        }
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
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxLeftRectImage,
                               "LeftRectifiedImage");
            // pass to LDC Applib createParams
            createParams->vxOutputLeftImage[0] = appCntxt->vxLeftRectImage;
        }
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {

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
            } else
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
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxRightRectImage[i],
                                   "RightRectifiedImage");
                // pass to LDC Applib createParams
                createParams->vxOutputRightImage[i] = appCntxt->vxRightRectImage[i];
            }
        }
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        appCntxt->sdeLdcHdl = SDELDC_create(createParams);
        if (appCntxt->sdeLdcHdl == NULL)
        {
            LOG_ERROR("SDELDC_create() failed\n");
            vxStatus = VX_FAILURE;
        }
    }

    return vxStatus;
}

vx_status SDEAPP_init_SDE(SDEAPP_Context *appCntxt)
{
    int32_t   i;
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
            slSdeCreateParams->vxRightRectImage[i] = appCntxt->vxRightRectImage[i];
        }

        /*
         * Create output image objects for SL SDE
         */
        slSdeCreateParams->createOutputFlag    = 0;
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            appCntxt->vxSde16BitOutput[i]  =
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
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxSde16BitOutput[i],
                                   "RawDisparityMap");
                slSdeCreateParams->vxSde16BitOutput[i]  = appCntxt->vxSde16BitOutput[i];
            }
        }

        if (vxStatus == (vx_status) VX_SUCCESS)
        {
            appCntxt->slSdeHdl = SL_SDE_create(slSdeCreateParams);
            if (appCntxt->slSdeHdl == NULL)
            {
                LOG_ERROR("SL_SDE_create() failed\n");
                vxStatus = VX_FAILURE;
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
        mlSdeCreateParams->createInputFlag    = 0;
        mlSdeCreateParams->inputPipelineDepth = 1;
        mlSdeCreateParams->vxLeftRectImageL0[0]  = appCntxt->vxLeftRectImage;

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            mlSdeCreateParams->vxRightRectImageL0[i] = appCntxt->vxRightRectImage[i];
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
                else
                {
                    mlSdeCreateParams->vxMedianFilteredDisparity[i] =
                        appCntxt->vxMedianFilteredDisparity[i];
                }
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
                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                mlSdeCreateParams->vxMergeDisparityL0[i] =
                    appCntxt->vxMergeDisparityL0[i];
            }
        }

        if (vxStatus == (vx_status) VX_SUCCESS)
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



vx_status SDEAPP_init_SDE_Triang(SDEAPP_Context *appCntxt)
{
    SDE_TRIANG_createParams  * sdeTriangCreateParams;

    int32_t                           i;
    vx_status                         vxStatus = VX_SUCCESS;

    PTK_PointCloud                  * cloud;
    PTK_PointCloudConfig              pcConfig;
    int32_t                           pcSize;
    uint8_t                         * cloudMem;

    sdeTriangCreateParams                = &appCntxt->sdeTriangCreateParams;
    sdeTriangCreateParams->vxContext     = appCntxt->vxContext;
    sdeTriangCreateParams->vxGraph       = appCntxt->vxGraph;

    /*
     * Create input image objects for SDE Triangulate
     */
    sdeTriangCreateParams->createInputFlag = 0;
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        // input rectified image
        sdeTriangCreateParams->vxInputRectImage[i] =
            appCntxt->vxRightRectImage[i];

        // raw disparity image
        if (appCntxt->sdeAlgoType == 0)
        {
            sdeTriangCreateParams->vxInputSde16Bit[i] =
                appCntxt->vxSde16BitOutput[i];
        } else
        {
            if (appCntxt->ppMedianFilterEnable)
            {
                sdeTriangCreateParams->vxInputSde16Bit[i] =
                    appCntxt->vxMedianFilteredDisparity[i];
            }
            else
            {
                sdeTriangCreateParams->vxInputSde16Bit[i] =
                    appCntxt->vxMergeDisparityL0[i];
            }
        }
    }

    /*
     * Create output point cloud objects for SDE Triangulate
     */
    // output objects are created in applib
    sdeTriangCreateParams->createOutputFlag = 0;

    pcConfig.maxPoints  = appCntxt->width * appCntxt->height;
    pcSize              = PTK_PointCloud_getSize(&pcConfig);

    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        cloudMem        = new uint8_t[pcSize];
        if (cloudMem == NULL)
        {
            LOG_ERROR("Memory allocation failed.\n");

            vxStatus = VX_FAILURE;
            break;
        } else
        {
            cloud = PTK_PointCloud_init(cloudMem, &pcConfig);

            /* Lidar Point Cloud data. */
            appCntxt->vxOutputTriangPC[i] =
                vxCreateUserDataObject(appCntxt->vxContext,
                                      "PTK_PointCloud",
                                       pcSize,
                                       cloud);

            delete [] cloudMem;

            if (appCntxt->vxOutputTriangPC[i] == NULL)
            {
                LOG_ERROR("vxCreateUserDataObject() failed\n");
                vxStatus = VX_FAILURE;
                break;
            }
            else
            {
                vxSetReferenceName((vx_reference)appCntxt->vxOutputTriangPC[i],
                                   "StereoPointCloud");
                sdeTriangCreateParams->vxOutputTriangPC[i] =
                    appCntxt->vxOutputTriangPC[i];
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->sdeTriangHdl = SDE_TRIANG_create(sdeTriangCreateParams);
        if (appCntxt->sdeTriangHdl == NULL)
        {
            LOG_ERROR("SDE_TRIANG_create() failed\n");
            vxStatus = VX_SUCCESS;
        }
    }


    return vxStatus;
}



vx_status  SDEAPP_setupPipeline(SDEAPP_Context * appCntxt)
{
    vx_status  vxStatus;

    if (appCntxt->sdeAlgoType == 0)
    {
        vxStatus = SDEAPP_setupPipeline_SL(appCntxt);
    } else
    {
        vxStatus = SDEAPP_setupPipeline_ML(appCntxt);
    }

    if (vxStatus == VX_SUCCESS)
    {
        uint32_t   appValue;
        appValue = appCntxt->vxEvtAppValBase + SDEAPP_GRAPH_COMPLETE_EVENT;

        vxStatus = vxRegisterEvent((vx_reference)appCntxt->vxGraph,
                                   VX_EVENT_GRAPH_COMPLETED,
                                   0,
                                   appValue);

        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("vxRegisterEvent() failed\n");
        }
    }

    return vxStatus;
}

vx_status SDEAPP_setupPipeline_SL(SDEAPP_Context * appCntxt)
{
    SDEAPP_graphParams                * paramDesc;
    vx_graph_parameter_queue_params_t   q[SDEAPP_NUM_GRAPH_PARAMS];
    uint32_t                            i;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    vx_node leftLdcNode   = SDELCD_getLeftLDCNode(appCntxt->sdeLdcHdl);
    vx_node rightLdcNode  = SDELCD_getRightLDCNode(appCntxt->sdeLdcHdl);
    vx_node sdeNode       = SL_SDE_getSDENode(appCntxt->slSdeHdl);
    vx_node colorConvNode;
    vx_node triangNode;

    if (appCntxt->enablePC)
    {
        colorConvNode = SDE_TRIANG_getColorConvNode(appCntxt->sdeTriangHdl);
        triangNode    = SDE_TRIANG_getTriangNode(appCntxt->sdeTriangHdl);
    }

    /* Five graph parameters in total */
    appCntxt->numGraphParams = 4;

    /* LDC left node Param 6 ==> graph param 0. */
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

    /* LDC right node Param 6 ==> graph param 1. */
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

    /* LDC right node Param 7 ==> graph param 2. */
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

    /* vxDmpacSdeNode Param 3 ==> graph param 3 */
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

    /* SDE Triangulat node Param 4 ==> graph param 4 */
    /* Only when enablePC = 1                        */
    if (appCntxt->enablePC && vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          triangNode,
                                          4);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutputTriangPC;
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
            paramDesc->timestamp         = &appCntxt->timestamp[i];

            if (appCntxt->enablePC)
            {
                paramDesc->vxOutputTriangPC    = appCntxt->vxOutputTriangPC[i];
            }

            appCntxt->freeQ.push(paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);
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
    else
    {
        LOG_ERROR("vxSetGraphScheduleConfig() failed\n");
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

    if (appCntxt->enablePC && vxStatus == VX_SUCCESS)
    {
        /* RGB rectified right image converted from YUV420 */
        vxStatus = tivxSetNodeParameterNumBufByIndex(colorConvNode,
                                                     1,
                                                     appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
        }
    }

    return vxStatus;
}


vx_status SDEAPP_setupPipeline_ML(SDEAPP_Context * appCntxt)
{
    SDEAPP_graphParams                * paramDesc;
    vx_graph_parameter_queue_params_t   q[SDEAPP_NUM_GRAPH_PARAMS];
    uint32_t                            i;
    uint32_t                            cnt = 0;
    vx_status                           vxStatus;

    vx_node leftLdcNode    = SDELCD_getLeftLDCNode(appCntxt->sdeLdcHdl);
    vx_node rightLdcNode   = SDELCD_getRightLDCNode(appCntxt->sdeLdcHdl);

    vx_node sdeNodeL0      = ML_SDE_getSDENodeL0(appCntxt->mlSdeHdl);
    vx_node sdeNodeL1      = ML_SDE_getSDENodeL1(appCntxt->mlSdeHdl);
    vx_node sdeNodeL2      = ML_SDE_getSDENodeL2(appCntxt->mlSdeHdl);
    vx_node medFilterNode  = ML_SDE_getMedFilterNode(appCntxt->mlSdeHdl);
    vx_node mergeNodeL1    = ML_SDE_getMergeNodeL1(appCntxt->mlSdeHdl);
    vx_node mergeNodeL2    = ML_SDE_getMergeNodeL2(appCntxt->mlSdeHdl);
    vx_node leftMscNodeL1  = ML_SDE_getLeftMSCNodeL1(appCntxt->mlSdeHdl);
    vx_node rightMscNodeL1 = ML_SDE_getRightMSCNodeL1(appCntxt->mlSdeHdl);
    vx_node leftMscNodeL2  = ML_SDE_getLeftMSCNodeL2(appCntxt->mlSdeHdl);
    vx_node rightMscNodeL2 = ML_SDE_getRightMSCNodeL2(appCntxt->mlSdeHdl);

    vx_node colorConvNode;
    vx_node triangNode;

    if (appCntxt->enablePC)
    {
        colorConvNode = SDE_TRIANG_getColorConvNode(appCntxt->sdeTriangHdl);
        triangNode    = SDE_TRIANG_getTriangNode(appCntxt->sdeTriangHdl);
    }

    appCntxt->numGraphParams = 4;


    /* LDC left node Param 6 ==> graph param 0. */
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

    /* LDC rigth node Param 6 ==> graph param 1. */
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

    /* LDC right node Param 7 ==> graph param 2. */
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

    /* vxDisparityMergeNodeL1 Param 3 => graph param 3. */
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
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (appCntxt->ppMedianFilterEnable)
        {
            appCntxt->numGraphParams += 1;

            /* vxMedianFilterNode Param 2 => graph param 3. */
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
    }

    /* SDE Triangulat node Param 4 ==> graph param 4 */
    /* Only when enablePC = 1                        */
    if (appCntxt->enablePC && vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->numGraphParams += 1;

        vxStatus = CM_addParamByNodeIndex(appCntxt->vxGraph,
                                          triangNode,
                                          4);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_addParamByNodeIndex() failed\n");
        }
        else
        {
            q[cnt++].refs_list = (vx_reference*)appCntxt->vxOutputTriangPC;
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
            paramDesc                     = &appCntxt->paramDesc[i];
            paramDesc->vxInputLeftImage   = appCntxt->vxInputLeftImage[i];
            paramDesc->vxInputRightImage  = appCntxt->vxInputRightImage[i];
            paramDesc->vxRightRectImage   = appCntxt->vxRightRectImage[i];
            paramDesc->vxMergeDisparityL0 = appCntxt->vxMergeDisparityL0[i];
            paramDesc->timestamp          = &appCntxt->timestamp[i];

            if (appCntxt->enablePC)
            {
                paramDesc->vxOutputTriangPC = appCntxt->vxOutputTriangPC[i];
            }

            if (appCntxt->ppMedianFilterEnable)
            {
                paramDesc->vxMedianFilteredDisparity = appCntxt->vxMedianFilteredDisparity[i];
            }

            appCntxt->freeQ.push(paramDesc);
        }

        vxStatus = vxSetGraphScheduleConfig(appCntxt->vxGraph,
                                            VX_GRAPH_SCHEDULE_MODE_QUEUE_AUTO,
                                            cnt,
                                            q);
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
    else
    {
        LOG_ERROR("vxSetGraphScheduleConfig() failed\n");
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
        /* vxSde16BitOutputL0 */
        vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL0,
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
            vxStatus = tivxSetNodeParameterNumBufByIndex(leftMscNodeL1,
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
            vxStatus = tivxSetNodeParameterNumBufByIndex(rightMscNodeL1,
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
            vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL1,
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
            vxStatus = tivxSetNodeParameterNumBufByIndex(leftMscNodeL2,
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
            vxStatus = tivxSetNodeParameterNumBufByIndex(rightMscNodeL2,
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
            vxStatus = tivxSetNodeParameterNumBufByIndex(sdeNodeL2,
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
            vxStatus = tivxSetNodeParameterNumBufByIndex(mergeNodeL2,
                                                         3,
                                                         appCntxt->pipelineDepth);
            if (vxStatus != VX_SUCCESS)
            {
                LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
            }
        }
    }


    if (appCntxt->enablePC && vxStatus == VX_SUCCESS)
    {
        /* RGB rectified right image converted from YUV420 */
        vxStatus = tivxSetNodeParameterNumBufByIndex(colorConvNode,
                                                     1,
                                                     appCntxt->pipelineDepth);

        if (vxStatus != VX_SUCCESS)
        {
            LOG_ERROR("tivxSetNodeParameterNumBufByIndex() failed\n");
        }
    }

    return vxStatus;
}

void SDEAPP_printStats(SDEAPP_Context * appCntxt)
{
    tivx_utils_graph_perf_print(appCntxt->vxGraph);
    appPerfPointPrint(&appCntxt->sdePclPerf);
    LOG_INFO_RAW("\n");
    appPerfPointPrintFPS(&appCntxt->sdePclPerf);
    LOG_INFO_RAW("\n");
    CM_printProctime(stdout);
    LOG_INFO_RAW("\n");
}

vx_status SDEAPP_exportStats(SDEAPP_Context * appCntxt, FILE *fp, bool exportAll)
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
            app_perf_point_t *perfArr[1] = {&appCntxt->sdePclPerf};
            appPerfStatsExportAll(fp, perfArr, 1);
        }
    }

    return vxStatus;
}

vx_status SDEAPP_waitGraph(SDEAPP_Context * appCntxt)
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

vx_status SDEAPP_getFreeParamRsrc(SDEAPP_Context       *appCntxt,
                                  SDEAPP_graphParams   **gpDesc)
{
    std::unique_lock<std::mutex>   lock(appCntxt->paramRsrcMutex);
    vx_status                      vxStatus = VX_SUCCESS;

    /* Check if we have free og node descriptors available. */
    if (appCntxt->freeQ.empty())
    {
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        *gpDesc = appCntxt->freeQ.front();
        appCntxt->freeQ.pop();
    }

    return vxStatus;
}

vx_status  SDEAPP_process(SDEAPP_Context * appCntxt, SDEAPP_graphParams * gpDesc)
{
    vx_status     vxStatus;
    uint16_t      i;
    uint8_t       cnt = 0;

    vx_reference  obj[SDEAPP_NUM_GRAPH_PARAMS];

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

    if (appCntxt->enablePC)
    {
        obj[cnt++] = (vx_reference)gpDesc->vxOutputTriangPC;
    }


    // Enqueue buffers
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

    return vxStatus;
}


vx_status SDEAPP_processEvent(SDEAPP_Context * appCntxt, vx_event_t * event)
{
    vx_reference            ref;
    vx_reference            outref;
    uint8_t                 i;
    uint32_t                numRefs;
    uint32_t                index;
    vx_status               vxStatus = VX_SUCCESS;

    // For profiling
    float                   diff;

    if (event->type == VX_EVENT_GRAPH_COMPLETED)
    {
        uint32_t appValue = appCntxt->vxEvtAppValBase + SDEAPP_GRAPH_COMPLETE_EVENT;

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
                appPerfPointEnd(&appCntxt->sdePclPerf);

                appCntxt->profileEnd = GET_TIME();
                diff  = GET_DIFF(appCntxt->profileStart, appCntxt->profileEnd);
                CM_reportProctime("Inter_frame_interval", diff);
            }

            appPerfPointBegin(&appCntxt->sdePclPerf);
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
                if (vxStatus != VX_SUCCESS)
                {
                    LOG_ERROR("vxGraphParameterDequeueDoneRef() failed\n");

                    break;
                }
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {

            /* The last one to deque is vxOutInstMap parameter. Search and
             * identify the resource index.
             */
            index = appCntxt->pipelineDepth;
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                if (appCntxt->enablePC)
                {
                    outref = (vx_reference)appCntxt->vxOutputTriangPC[i];
                } else
                {
                    if (appCntxt->sdeAlgoType  == 0)
                    {
                       outref = (vx_reference)appCntxt->vxSde16BitOutput[i];
                    }
                    else if (appCntxt->ppMedianFilterEnable)
                    {
                        outref = (vx_reference)appCntxt->vxMedianFilteredDisparity[i];
                    } else
                    {
                        outref = (vx_reference)appCntxt->vxMergeDisparityL0[i];
                    }
                }

                if (ref == outref)
                {
                    index = i;
                    break;
                }
            }

            if (index >= appCntxt->pipelineDepth)
            {
                LOG_ERROR("Resource look up failed\n");

                vxStatus = VX_FAILURE;
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Mark the dequeued resource as free. */
            vxStatus = SDEAPP_releaseParamRsrc(appCntxt, index);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("SDEAPP_releaseParamRsrc() failed.\n",
                           __FUNCTION__,
                           __LINE__);
            }
        }
    }

    return vxStatus;
}


vx_status SDEAPP_releaseParamRsrc(SDEAPP_Context  *appCntxt, uint32_t rsrcIndex)
{
    SDEAPP_graphParams           * desc;
    std::unique_lock<std::mutex>   lock(appCntxt->paramRsrcMutex);

    desc = &appCntxt->paramDesc[rsrcIndex];
    appCntxt->outputQ.push(desc);

    return VX_SUCCESS;
}


vx_status SDEAPP_getOutBuff(SDEAPP_Context      *appCntxt,
                            vx_image            *rightRectImage,
                            vx_image            *disparity16,
                            vx_user_data_object *pointcloud,
                            vx_uint64           *timestamp)
{
    vx_status                        vxStatus = VX_SUCCESS;
    vx_map_id                        map_id;

    SDEAPP_graphParams             * desc;
    std::unique_lock<std::mutex>     lock(appCntxt->paramRsrcMutex);

    if (appCntxt->outputQ.empty())
    {
        LOG_ERROR("Output queue empty.\n");

        vxStatus = VX_FAILURE;
    }

    /* Get the descriptor. */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        desc = appCntxt->outputQ.front();

        *rightRectImage = desc->vxRightRectImage;

        if (appCntxt->sdeAlgoType == 0)
        {
            *disparity16 = desc->vxSde16BitOutput;
        }
        else if (appCntxt->ppMedianFilterEnable)
        {
            *disparity16 = desc->vxMedianFilteredDisparity;
        }
        else
        {
            *disparity16 = desc->vxMergeDisparityL0;
        }

        *pointcloud = desc->vxOutputTriangPC;

        *timestamp = *desc->timestamp;
    }

    return vxStatus;
}


vx_status SDEAPP_releaseOutBuff(SDEAPP_Context * appCntxt)
{
    vx_status                       vxStatus;
    SDEAPP_graphParams            * desc;
    std::unique_lock<std::mutex>    lock(appCntxt->paramRsrcMutex);

    vxStatus = VX_SUCCESS;

    if (appCntxt->outputQ.empty())
    {
        LOG_ERROR("No output buffers available.\n");

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        desc = appCntxt->outputQ.front();
        appCntxt->outputQ.pop();

        /* Push the descriptor into the free queue. */
        appCntxt->freeQ.push(desc);
    }

    return vxStatus;
}
