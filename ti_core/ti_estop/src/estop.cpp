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

#include <cm_profile.h>
#include <estop.h>

void ESTOP_APP_setPcParams(ESTOP_APP_Context *appCntxt);
void ESTOP_APP_setOgParams(ESTOP_APP_Context *appCntxt);

static char menu[] = {
    "\n"
    "\n ======================================================"
    "\n Demo : AMR E-STOP                                     "
    "\n ======================================================"
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n e: Export performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};


void ESTOP_APP_setLDCCreateParams(ESTOP_APP_Context *appCntxt)
{
    SDELDC_createParams * createParams = &appCntxt->sdeLdcCreateParams;

    createParams->leftLutFileName  = appCntxt->left_LUT_file_name;
    createParams->rightLutFileName = appCntxt->right_LUT_file_name;

    createParams->width            = appCntxt->width;
    createParams->height           = appCntxt->height;
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
}

void ESTOP_APP_setSLSdeCreateParams(ESTOP_APP_Context *appCntxt)
{
    SL_SDE_createParams * createParams = &appCntxt->slSdeCreateParams;
    createParams->sdeCfg = appCntxt->sde_params;

     if (appCntxt->sde_params.disparity_min == 0)
    {
        createParams->minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        createParams->minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        createParams->maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        createParams->maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        createParams->maxDisparity = 191;
    }

    createParams->width              = appCntxt->width;
    createParams->height             = appCntxt->height;
    createParams->inputFormat        = appCntxt->inputFormat;
    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}

void ESTOP_APP_setMLSdeCreateParams(ESTOP_APP_Context *appCntxt)
{
    ML_SDE_createParams * createParams = &appCntxt->mlSdeCreateParams;
    createParams->sdeCfg = appCntxt->sde_params;

    if (appCntxt->sde_params.disparity_min == 0)
    {
        createParams->minDisparity = 0;
    }
    else if (appCntxt->sde_params.disparity_min == 1)
    {
        createParams->minDisparity = -3;
    }

    if (appCntxt->sde_params.disparity_max == 0)
    {
        createParams->maxDisparity = 63;
    }
    else if (appCntxt->sde_params.disparity_max == 1)
    {
        createParams->maxDisparity = 127;
    }
    else if (appCntxt->sde_params.disparity_max == 2)
    {
        createParams->maxDisparity = 191;
    }

    createParams->inputFormat        = appCntxt->inputFormat;
    createParams->numLayers          = appCntxt->numLayers;
    createParams->enableMedianFilter = appCntxt->ppMedianFilterEnable;
    createParams->width              = appCntxt->width;
    createParams->height             = appCntxt->height;
    createParams->pipelineDepth      = appCntxt->pipelineDepth;
}

void ESTOP_APP_setSSDetectCreateParams(ESTOP_APP_Context *appCntxt)
{
    SS_DETECT_createParams * createParams = &appCntxt->ssDetectCreateParams;

    createParams->width            = appCntxt->width;
    createParams->height           = appCntxt->height;
    createParams->tensorWidth      = appCntxt->tensor_width;
    createParams->tensorHeight     = appCntxt->tensor_height;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->exportGraph      = appCntxt->exportGraph;
    createParams->rtLogEnable      = appCntxt->rtLogEnable;
    createParams->vxEvtAppValBase  = 0;

    ESTOP_APP_setPcParams(appCntxt);
    ESTOP_APP_setOgParams(appCntxt);
}


void ESTOP_APP_setPcParams(ESTOP_APP_Context *appCntxt)
{
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.width        = appCntxt->width;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.height       = appCntxt->height;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.tensorWidth  = appCntxt->tensor_width;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.tensorHeight = appCntxt->tensor_height;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsFactor     = 4;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.confidenceTh = appCntxt->confidence_threshold;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsWidth      =
        appCntxt->ssDetectCreateParams.pcCfg.cfgParams.width / appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsFactor;
    appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsHeight     =
        appCntxt->ssDetectCreateParams.pcCfg.cfgParams.height / appCntxt->ssDetectCreateParams.pcCfg.cfgParams.dsFactor;

    appCntxt->ssDetectCreateParams.pcCfg.camParams.camHeight    = appCntxt->camHeight;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.camRoll      = appCntxt->camRoll;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.camPitch     = appCntxt->camPitch;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.camYaw       = appCntxt->camYaw;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.sinPitch     = sin(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.pcCfg.camParams.cosPitch     = cos(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.pcCfg.camParams.baseline     = appCntxt->baseline;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.dcx          = appCntxt->distCenterX;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.dcy          = appCntxt->distCenterY;
    appCntxt->ssDetectCreateParams.pcCfg.camParams.f            = appCntxt->focalLength;
}


void ESTOP_APP_setOgParams(ESTOP_APP_Context *appCntxt)
{
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.width                      = appCntxt->width;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.height                     = appCntxt->height;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.enableSpatialObjMerge      = appCntxt->enableSpatialObjMerge;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.enableTemporalObjMerge     = appCntxt->enableTemporalObjMerge;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.enableTemporalObjSmoothing = appCntxt->enableTemporalObjSmoothing;
    appCntxt->ssDetectCreateParams.ogCfg.cfgParams.objectDistanceMode         = appCntxt->objectDistanceMode;

    appCntxt->ssDetectCreateParams.ogCfg.camParams.camHeight    = appCntxt->camHeight;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.camRoll      = appCntxt->camRoll;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.camPitch     = appCntxt->camPitch;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.camYaw       = appCntxt->camYaw;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.sinPitch     = sin(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.ogCfg.camParams.cosPitch     = cos(appCntxt->camPitch);
    appCntxt->ssDetectCreateParams.ogCfg.camParams.baseline     = appCntxt->baseline;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.dcx          = appCntxt->distCenterX;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.dcy          = appCntxt->distCenterY;
    appCntxt->ssDetectCreateParams.ogCfg.camParams.f            = appCntxt->focalLength;

    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xGridSize     = appCntxt->xGridSize;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yGridSize     = appCntxt->yGridSize;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xMinRange     = appCntxt->xMinRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xMaxRange     = appCntxt->xMaxRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yMinRange     = appCntxt->yMinRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yMaxRange     = appCntxt->yMaxRange;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.xGridNum      = appCntxt->xGridNum;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.yGridNum      = appCntxt->yGridNum;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.thCnt         = appCntxt->thCnt;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.thObjCnt      = appCntxt->thObjCnt;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.maxNumObject  = appCntxt->maxNumObject;
    appCntxt->ssDetectCreateParams.ogCfg.ogParams.cNeighNum     = appCntxt->cNeighNum;
}

void ESTOP_APP_setAllParams(ESTOP_APP_Context *appCntxt)
{
    appCntxt->vxEvtAppValBase = 0;

    /* LDC params */
    ESTOP_APP_setLDCCreateParams(appCntxt);

    /* SDE params */
    if (appCntxt->sdeAlgoType == 0)
    {
        ESTOP_APP_setSLSdeCreateParams(appCntxt);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ESTOP_APP_setMLSdeCreateParams(appCntxt);
    }

    ESTOP_APP_setSSDetectCreateParams(appCntxt);
}


vx_status ESTOP_APP_init(ESTOP_APP_Context *appCntxt)
{
    const string   &modelPath = appCntxt->dlModelPath;
    int32_t         status;
    vx_status       vxStatus = VX_SUCCESS;

    status = appInit();
    if (status < 0)
    {
        LOG_ERROR("appInit() failed.\n");

        vxStatus = VX_FAILURE;
    }

    // Populate pre-process config
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        status = appCntxt->preProcCfg.getConfig(modelPath);

        //==> DEBUG
        //appCntxt->preProcCfg.dumpInfo();

        if (status < 0)
        {
            LOG_ERROR("getConfig() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }

    // Populate post-process config
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        status = appCntxt->postProcCfg.getConfig(modelPath);

        //==> DEBUG
        //appCntxt->postProcCfg.dumpInfo();

        if (status < 0)
        {
            LOG_ERROR("getConfig() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // Populate infConfig from
        status = appCntxt->dlInferConfig.getConfig(modelPath, true, 1);

        //==> DEBUG
        //appCntxt->dlInferConfig.dumpInfo();

        if (status < 0)
        {
            LOG_ERROR("getConfig() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->dlInferer = DLInferer::makeInferer(appCntxt->dlInferConfig);

        if (appCntxt->dlInferer == nullptr)
        {
            LOG_ERROR("DLInferer::makeInferer() failed.\n");

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->tensor_width  = appCntxt->preProcCfg.resizeWidth;
        appCntxt->tensor_height = appCntxt->preProcCfg.resizeHeight;

        appCntxt->preProcObj =
            CmPreprocessImage::makePreprocessImageObj(appCntxt->preProcCfg);

        if (appCntxt->preProcObj == nullptr)
        {
            LOG_ERROR("CmPreprocessImage::makePreprocessImageObj() failed.\n");

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Set input data width and height based on the infererence engine
         * information. This is only used for semantic segmentation models
         * which have 4 dimensions. The logic is extended to any models that
         * have atleast three dimensions which has the following
         * - Num channels (C)
         * - Height (H)
         * - Width (W)
         */
        DLInferer          *inferer      = appCntxt->dlInferer;
        const VecDlTensor  *dlInfOutputs = inferer->getOutputInfo();
        const DlTensor     *ifInfo       = &dlInfOutputs->at(0);

        /* For segmentation only. */
        if (appCntxt->postProcCfg.taskType == "segmentation")
        {
            appCntxt->postProcCfg.inDataWidth  = ifInfo->shape[ifInfo->dim - 1];
            appCntxt->postProcCfg.inDataHeight = ifInfo->shape[ifInfo->dim - 2];
        }
        else
        {
            LOG_ERROR("Invalid task type..\n");
            vxStatus = VX_FAILURE;
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (appCntxt->tensor_height != appCntxt->postProcCfg.inDataHeight)
            {
                LOG_ERROR("Inference input/output tensor height mismatch.\n");
                vxStatus = VX_FAILURE;
            }

            if (appCntxt->tensor_width != appCntxt->postProcCfg.inDataWidth)
            {
                LOG_ERROR("Inference input/output tensor width mismatch.\n");
                vxStatus = VX_FAILURE;
            }

            //appCntxt->tensor_height = appCntxt->postProcCfg.inDataHeight;
            //appCntxt->tensor_width  = appCntxt->postProcCfg.inDataWidth;

            appCntxt->postProcObj =
            CmPostprocessImage::makePostprocessImageObj(appCntxt->postProcCfg);

            if (appCntxt->postProcObj == nullptr)
            {
                LOG_ERROR("makePostprocessImageObj() failed.\n");
                vxStatus = VX_FAILURE;
            }
        }
    }

    // OpenVX initialization
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->vxContext = vxCreateContext();
        if (appCntxt->vxContext == NULL)
        {
            LOG_ERROR("vxCreateContext() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }

    // create graph
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);
        if (appCntxt->vxGraph == NULL)
        {
            LOG_ERROR("vxCreateGraph() failed\n");
            vxStatus = VX_FAILURE;
        } else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGraph,
                               "AMR E-Stop Graph");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* load image processing kernel */
        tivxImgProcLoadKernels(appCntxt->vxContext);

        /* Load HWA kernels */
        tivxHwaLoadKernels(appCntxt->vxContext);

        /* Load stereo kernels */
        tivxStereoLoadKernels(appCntxt->vxContext);
    }

    /*
     * 0. Setup all parameters before initializing all nodes
     */
    ESTOP_APP_setAllParams(appCntxt);

    /*
     * 1 Setup Stereo LDC nodes
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_LDC(appCntxt);
    }

    /*
     * 2 Setup SDE nodes
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_SDE(appCntxt);
    }

    /*
     * 3 Setup Semantic Segmentation nodes
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_SS(appCntxt);
    }

    /*
     * 4 Setup OG map based detection nodes
     */
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = ESTOP_APP_init_SS_Detection(appCntxt);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->estopPerf , "Emergency Stop GRAPH");

        /*
         * set up the pipeline.
         */
        vxStatus = ESTOP_APP_setupPipeline(appCntxt);

        /* Verify graph */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vxStatus = vxVerifyGraph(appCntxt->vxGraph);
            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("vxVerifyGraph() failed\n");
            }
        }

        /* Set the MSC coefficients. */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            CM_ScalerNodeCntxt  *scalerObj = &appCntxt->scalerObj;

            vxStatus = CM_scalerNodeCntxtSetCoeff(scalerObj);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("CM_scalerNodeCntxtSetCoeff() failed\n");
            }
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // init sacler for ML SDE
        if (appCntxt->sdeAlgoType == 1)
        {
            ML_SDE_initScaler(appCntxt->mlSdeHdl);
        }

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_estop");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }

        appCntxt->exitOutputThread     = false;
        appCntxt->outputCtrlSem        = new Semaphore(0);

        appCntxt->preProcSem           = new Semaphore(0);
        appCntxt->exitPreprocThread    = false;

        appCntxt->dlDataReadySem      = new Semaphore(0);
        appCntxt->exitDlInferThread   = false;

        appCntxt->postProcSem          = new Semaphore(0);
        appCntxt->exitPostprocThread   = false;

        appCntxt->exitInputDataProcess = false;
        appCntxt->state                = ESTOP_APP_STATE_INIT;

        ESTOP_APP_reset(appCntxt);
    }

    return vxStatus;
}

vx_status ESTOP_APP_init_camInfo(ESTOP_APP_Context *appCntxt,
                                 uint32_t width,
                                 uint32_t height,
                                 double   f,
                                 double   dx,
                                 double   dy)
{
    appCntxt->width       = (uint16_t) width;
    appCntxt->height      = (uint16_t) height;
    appCntxt->focalLength = (float)f;
    appCntxt->distCenterX = (float)dx;
    appCntxt->distCenterY = (float)dy;

    appCntxt->hImgOfst    = 0;
    appCntxt->vImgOfst    = 0;

    // See if width and height is larger than maximum SDE resolution
    // For ZED camera, since input is already clipped to the max resolution,
    // it is not needed. But for other input, it would be necessary
    if (appCntxt->width > ESTOP_APP_MAX_IMAGE_WIDTH)
    {
        appCntxt->hImgOfst     =
            (appCntxt->width - ESTOP_APP_MAX_IMAGE_WIDTH) / 2;
        appCntxt->width        = ESTOP_APP_MAX_IMAGE_WIDTH;
        appCntxt->distCenterX -= appCntxt->hImgOfst;
    }

    if (appCntxt->height > ESTOP_APP_MAX_IMAGE_HEIGHT)
    {
        appCntxt->vImgOfst     =
            (appCntxt->height - ESTOP_APP_MAX_IMAGE_HEIGHT) / 2;
        appCntxt->height       = ESTOP_APP_MAX_IMAGE_HEIGHT;
        appCntxt->distCenterY -= appCntxt->vImgOfst;
    }

    return (vx_status)VX_SUCCESS;
}

vx_status ESTOP_APP_run(ESTOP_APP_Context  *appCntxt,
                        const vx_uint8     *inputLeftImage,
                        const vx_uint8     *inputRightImage,
                        vx_uint64           timestamp)
{
    ESTOP_APP_graphParams  *gpDesc;
    vx_status               vxStatus;
    TimePoint               start;
    TimePoint               end;
    float                   diff;

    vxStatus = ESTOP_APP_popFreeInputDesc(appCntxt, &gpDesc);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("ESTOP_APP_popFreeInputDesc() failed. Incoming Frame is dropped!\n");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        start = GET_TIME();
        vxStatus = CM_copyData2Image(inputLeftImage,
                                     gpDesc->vxInputLeftImage,
                                     appCntxt->hImgOfst,
                                     appCntxt->vImgOfst);

        end   = GET_TIME();
        diff  = GET_DIFF(start, end);
        CM_reportProctime("input_image_load", diff);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_copyData2Image() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        start = GET_TIME();
        vxStatus = CM_copyData2Image(inputRightImage,
                                     gpDesc->vxInputRightImage,
                                     appCntxt->hImgOfst,
                                     appCntxt->vImgOfst);

        end   = GET_TIME();
        diff  = GET_DIFF(start, end);
        CM_reportProctime("input_image_load", diff);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("CM_copyData2Image() failed\n");
        }
    }

    // then run AMR applib
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // set time stamp
        *(gpDesc->timestamp) = timestamp;

        vxStatus = ESTOP_APP_process(appCntxt, gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("ESTOP_APP_process() failed\n");
        }
    }

    return vxStatus;
}

void ESTOP_APP_deInit(ESTOP_APP_Context *appCntxt)
{
    int32_t status;
    uint8_t i;

    // release input image object
    vxReleaseImage(&appCntxt->vxLeftRectImage);
    for (i = 0; i < appCntxt->pipelineDepth; i++)
    {
        vxReleaseImage(&appCntxt->vxInputLeftImage[i]);
        vxReleaseImage(&appCntxt->vxInputRightImage[i]);
        vxReleaseImage(&appCntxt->vxRightRectImage[i]);
    }

    // release dispairty object
    if (appCntxt->sdeAlgoType == 0)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseImage(&appCntxt->vxSde16BitOutput[i]);
        }
    }
    else
    {
        if (appCntxt->ppMedianFilterEnable)
        {
            for (i = 0; i < appCntxt->pipelineDepth; i++)
            {
                vxReleaseImage(&appCntxt->vxMedianFilteredDisparity[i]);
            }
        }

        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseImage(&appCntxt->vxMergeDisparityL0[i]);
        }
    }

    // release graph
    vxReleaseGraph(&appCntxt->vxGraph);

    // Unload HWA kernels
    tivxStereoUnLoadKernels(appCntxt->vxContext);
    tivxImgProcUnLoadKernels(appCntxt->vxContext);
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    /* Delete the Pre-proc instance. */
    delete appCntxt->preProcObj;

    /* Delete the Post-proc instance. */
    delete appCntxt->postProcObj;

    /* Delete the DL Inferer instance. */
    delete appCntxt->dlInferer;

    /* De-initialize. */
    status = appDeInit();

    if (status < 0)
    {
        LOG_ERROR("appDeInit() failed\n");
    }
}

static void ESTOP_APP_exitProcThreads(ESTOP_APP_Context *appCntxt)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               ESTOP_APP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        LOG_ERROR("vxSendUserEvent() failed.\n");
    }

    /* Set the exit flag for the pre-process thread. */
    appCntxt->exitPreprocThread = true;

    /* Wake-up the pre-process thread. */
    if (appCntxt->preProcSem)
    {
        appCntxt->preProcSem->notify();
    }

    /* Set the exit flag for the DLR thread. */
    appCntxt->exitDlInferThread = true;

    /* Wake-up the DLR thread. */
    if (appCntxt->dlDataReadySem)
    {
        appCntxt->dlDataReadySem->notify();
    }

    /* Set the exit flag for the post-process thread. */
    appCntxt->exitPostprocThread = true;

    /* Wake-up the post-process thread. */
    if (appCntxt->postProcSem)
    {
        appCntxt->postProcSem->notify();
    }

    /* Let the display thread exit. */
    appCntxt->exitOutputThread = true;

    if (appCntxt->outputCtrlSem)
    {
        appCntxt->outputCtrlSem->notify();
    }

    /* Wait for the thread exit */
    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }

    if (appCntxt->preProcThread.joinable())
    {
        appCntxt->preProcThread.join();
    }

    if (appCntxt->dlInferThread.joinable())
    {
        appCntxt->dlInferThread.join();
    }

    if (appCntxt->postProcThread.joinable())
    {
        appCntxt->postProcThread.join();
    }

    /* Delete the semaphores. */
    if (appCntxt->outputCtrlSem)
    {
        delete appCntxt->outputCtrlSem;
    }

    if (appCntxt->preProcSem)
    {
        delete appCntxt->preProcSem;
    }

    if (appCntxt->dlDataReadySem)
    {
        delete appCntxt->dlDataReadySem;
    }

    if (appCntxt->postProcSem)
    {
        delete appCntxt->postProcSem;
    }

}

static void ESTOP_APP_dumpStats(ESTOP_APP_Context *appCntxt)
{
    if (appCntxt->exportPerfStats == 1)
    {
        const char *name = ESTOP_APP_PERF_OUT_FILE;
        FILE       *fp;

        fp = appPerfStatsExportOpenFile(".", (char *)name);

        if (fp != NULL)
        {
            ESTOP_APP_exportStats(appCntxt, fp, true);

            CM_printProctime(fp);
            appPerfStatsExportCloseFile(fp);
        }
        else
        {
            LOG_ERROR("Could not open [%s] for exporting "
                       "performance data\n", name);
        }
    }
}

void ESTOP_APP_cleanupHdlr(ESTOP_APP_Context *appCntxt)
{
    if (appCntxt->state == ESTOP_APP_STATE_INVALID)
    {
        return;
    }

    appCntxt->state = ESTOP_APP_STATE_INVALID;

    /* Wait for the threads to exit. */
    ESTOP_APP_exitProcThreads(appCntxt);

    LOG_INFO("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    ESTOP_APP_dumpStats(appCntxt);
    ESTOP_APP_printStats(appCntxt);

    LOG_INFO("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", ESTOP_APP_PERF_OUT_FILE);
        tivxLogRtTraceExportToFile(name);
    }

    /* Release the objects. */
    SDELDC_delete(&appCntxt->sdeLdcHdl);

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDE_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDE_delete(&appCntxt->mlSdeHdl);
    }

    SS_DETECT_delete(&appCntxt->ssDetectHdl);

    ESTOP_APP_deinit_SS(appCntxt);

    ESTOP_APP_deInit(appCntxt);

    LOG_INFO("Clean-up complete.\n");
}

void ESTOP_APP_reset(ESTOP_APP_Context * appCntxt)
{
    vx_status vxStatus;

    vx_node ogNode = SS_DETECT_getOGNode(appCntxt->ssDetectHdl);
    vxStatus = tivxNodeSendCommand(ogNode,
                                   0,
                                   TIVX_KERNEL_OCCUPANCY_GRID_DETECTION_RESET,
                                   NULL,
                                   0);
    if (vxStatus != VX_SUCCESS)
    {
        LOG_ERROR("tivxNodeSendCommand() failed.\n");

    }

    appCntxt->startPerfCapt = false;
}

static void ESTOP_APP_evtHdlrThread(ESTOP_APP_Context *appCntxt)
{
    vx_event_t evt;
    vx_status vxStatus = VX_SUCCESS;

    LOG_INFO("Launched.\n");

    /* Clear any pending events. The third argument is do_not_block = true. */
    while (vxStatus == VX_SUCCESS)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_true_e);
    }

    while (true)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_false_e);

        if (vxStatus == VX_SUCCESS)
        {
            if (evt.type == VX_EVENT_USER)
            {
                if (evt.app_value == ESTOP_APP_USER_EVT_EXIT)
                {
                    break;
                }
                else if (evt.app_value == ESTOP_APP_CNN_OUT_AVAIL_EVENT)
                {
                    ESTOP_APP_graphParams * desc;

                    vxStatus = ESTOP_APP_getPostprocInputDesc(appCntxt, &desc);

                    if (vxStatus != (vx_status)VX_SUCCESS)
                    {
                        LOG_ERROR("ESTOP_APP_getPostprocInputDesc(%d) failed\n",
                                  appCntxt->numGraphParams - 1);

                    }

                    // enqueue vxOutTensor
                    if (vxStatus == (vx_status)VX_SUCCESS)
                    {
                        vxStatus =
                            vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                            appCntxt->numGraphParams - 1,
                                                            (vx_reference*)&desc->vxOutTensor,
                                                            1);

                        if (vxStatus != (vx_status)VX_SUCCESS)
                        {
                            LOG_ERROR("vxGraphParameterEnqueueReadyRef(%d) failed\n",
                                      appCntxt->numGraphParams - 1);

                        }
                    }
                }
            }

            if ((evt.type == VX_EVENT_GRAPH_COMPLETED) ||
                (evt.type == VX_EVENT_NODE_COMPLETED))
            {
                ESTOP_APP_processEvent(appCntxt, &evt);
            }
        }

    } // while (true)

    LOG_INFO("Exiting.\n");
}

static int32_t ESTOP_APP_userControlThread(ESTOP_APP_Context *appCntxt)
{
    vx_status   vxStatus = VX_SUCCESS;
    uint32_t    done = 0;

    appPerfStatsResetAll();

    while (!done)
    {
        char ch;

        LOG_INFO_RAW("%s", menu);
        ch = getchar();
        LOG_INFO_RAW("\n");

        switch (ch)
        {
            case 'p':
                appPerfStatsPrintAll();
                ESTOP_APP_printStats(appCntxt);

                CM_resetProctime();
                break;

            case 'e':
                ESTOP_APP_dumpStats(appCntxt);
                break;

            case 'x':
                done = 1;
                break;
        } // switch(ch)

        /* Consume the newline character. */
        if (ch != '\n')
        {
            getchar();
        }
    } // while (!done)

    appCntxt->state = ESTOP_APP_STATE_SHUTDOWN;
    LOG_INFO("Waiting for the graph to finish.\n");

    vxStatus = ESTOP_APP_waitGraph(appCntxt);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("ESTOP_APP_waitGraph() failed\n");
    }

    ESTOP_APP_cleanupHdlr(appCntxt);

    LOG_INFO("\nDEMO FINISHED!\n");

    return vxStatus;
}

static void ESTOP_APP_preProcThread(ESTOP_APP_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        ESTOP_APP_graphParams  *desc;
        vx_status               vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->preProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPreprocThread == true)
        {
            break;
        }

        vxStatus = ESTOP_APP_popPreprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();
            vxStatus =
                ESTOP_APP_CNN_preProcess(appCntxt,
                                        desc->vxScalerOut,
                                        desc->inferInputBuff);

            end = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Preprocessing", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("ESTOP_APP_CNN_preProcess() failed.\n");
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                ESTOP_APP_enqueDlInferInputDesc(appCntxt, desc);

                /* Wakeup the DLR thread. The DLR thread will process the
                 * descriptor at the head of the queue.
                 */
                appCntxt->dlDataReadySem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");
}


static void ESTOP_APP_dlInferThread(ESTOP_APP_Context *appCntxt)
{
    DLInferer  *inferer = appCntxt->dlInferer;
    TimePoint   start;
    TimePoint   end;
    float       diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        ESTOP_APP_graphParams  *desc;
        vx_status               vxStatus;

        /* Wait fot the input buffer. */
        appCntxt->dlDataReadySem->wait();

        if (appCntxt->exitDlInferThread == true)
        {
            break;
        }

        vxStatus = ESTOP_APP_popDlInferInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            int32_t status;

            start = GET_TIME();
            status = inferer->run(*desc->inferInputBuff,
                                  *desc->inferOutputBuff);

            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("DlInfer-node", diff);

            if (status < 0)
            {
                LOG_ERROR("Inference run failed.\n");
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                ESTOP_APP_enquePostprocInputDesc(appCntxt, desc);

                /* Wakeup the post-process thread. The post-process thread will
                 * process the descriptor at the head of the queue.
                 */
                appCntxt->postProcSem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");

}

static void ESTOP_APP_postProcThread(ESTOP_APP_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        ESTOP_APP_graphParams  *desc;
        vx_status               vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->postProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPostprocThread == true)
        {
            break;
        }

        vxStatus = ESTOP_APP_getPostprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            /* Create an output tensor. */
            start = GET_TIME();
            vxStatus =
                ESTOP_APP_createOutTensor(appCntxt,
                                          desc->inferOutputBuff,
                                          desc->vxOutTensor);

            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Output_tensor_creation", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("ESTOP_APP_createOutTensor() failed.\n");
            }

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Let the usr know that output is ready. */
                vxStatus =
                    vxSendUserEvent(appCntxt->vxContext,
                                    ESTOP_APP_CNN_OUT_AVAIL_EVENT,
                                    NULL);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("vxSendUserEvent() failed.\n");
                }
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");
}

void ESTOP_APP_launchProcThreads(ESTOP_APP_Context *appCntxt)
{
    /* Launch the input data thread. */
    if (appCntxt->is_interactive)
    {
        appCntxt->userCtrlThread =
            std::thread(ESTOP_APP_userControlThread, appCntxt);

        appCntxt->userCtrlThread.detach();
    }

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread =
        std::thread(ESTOP_APP_evtHdlrThread, appCntxt);

    /* Launch CNN pre-processing thread */
    appCntxt->preProcThread =
        std::thread(ESTOP_APP_preProcThread, appCntxt);

    /* Launch the dlr thread */
    appCntxt->dlInferThread =
        std::thread(ESTOP_APP_dlInferThread, appCntxt);

    /* launch post-processing thread. */
    appCntxt->postProcThread =
        std::thread(ESTOP_APP_postProcThread, appCntxt);

    /* Launch the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfLaunchCtrlThread("estop");
    }
}

void ESTOP_APP_intSigHandler(ESTOP_APP_Context *appCntxt)
{
    /* Stop the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfStopCtrlThread();
    }

    if (appCntxt->state != ESTOP_APP_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        vx_status   vxStatus = VX_SUCCESS;

        appCntxt->state = ESTOP_APP_STATE_SHUTDOWN;
        LOG_INFO("Waiting for the graph to finish.\n");

        vxStatus = ESTOP_APP_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("ESTOP_APP_waitGraph() failed\n");
        }

        ESTOP_APP_cleanupHdlr(appCntxt);
        LOG_INFO("\nDEMO FINISHED!\n");
    }
}
