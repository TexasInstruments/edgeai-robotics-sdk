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

#include <cm_profile.h>
#include <sde.h>

static char menu[] = {
    "\n"
    "\n ======================================================"
    "\n Demo : SDE & Point Cloud                                "
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


void SDEAPP_setLDCCreateParams(SDEAPP_Context *appCntxt)
{
    SDELDC_createParams * createParams = &appCntxt->sdeLdcCreateParams;

    createParams->leftLutFileName  = appCntxt->left_LUT_file_name;
    createParams->rightLutFileName = appCntxt->right_LUT_file_name;

    createParams->width            = appCntxt->width;
    createParams->height           = appCntxt->height;
    createParams->inputFormat      = appCntxt->inputFormat;
    createParams->pipelineDepth    = appCntxt->pipelineDepth;
}

void SDEAPP_setSLSdeCreateParams(SDEAPP_Context *appCntxt)
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

void SDEAPP_setMLSdeCreateParams(SDEAPP_Context *appCntxt)
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


void SDEAPP_setSdeTriangCreateParams(SDEAPP_Context *appCntxt)
{
    SDE_TRIANG_createParams * createParams = &appCntxt->sdeTriangCreateParams;

    createParams->width                      = appCntxt->width;
    createParams->height                     = appCntxt->height;
    createParams->pipelineDepth              = appCntxt->pipelineDepth;

    createParams->stereoPcCfg.usePCConfig    = appCntxt->usePCConfig;
    createParams->stereoPcCfg.subSampleRatio = appCntxt->pcSubsampleRatio;
    createParams->stereoPcCfg.thConfidence   = appCntxt->dispConfidence;
    createParams->stereoPcCfg.lowLimitX      = appCntxt->lowPtX;
    createParams->stereoPcCfg.highLimitX     = appCntxt->highPtX;
    createParams->stereoPcCfg.lowLimitY      = appCntxt->lowPtY;
    createParams->stereoPcCfg.highLimitY     = appCntxt->highPtY;
    createParams->stereoPcCfg.lowLimitZ      = appCntxt->lowPtZ;
    createParams->stereoPcCfg.highLimitZ     = appCntxt->highPtZ;

    createParams->stereoCamCfg.baseline      = appCntxt->baseline;
    createParams->stereoCamCfg.scale_x       = 1;
    createParams->stereoCamCfg.scale_y       = 1;
    createParams->stereoCamCfg.focal_length  = appCntxt->focalLength;
    createParams->stereoCamCfg.dist_center_x = appCntxt->distCenterX;
    createParams->stereoCamCfg.dist_center_y = appCntxt->distCenterY;
}

void SDEAPP_setAllParams(SDEAPP_Context *appCntxt)
{
    appCntxt->vxEvtAppValBase = 0;

    /* LDC params */
    SDEAPP_setLDCCreateParams(appCntxt);

    /* SDE params */
    if (appCntxt->sdeAlgoType == 0)
    {
        SDEAPP_setSLSdeCreateParams(appCntxt);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        SDEAPP_setMLSdeCreateParams(appCntxt);
    }

    /* SDE Triangulation params */
    if (appCntxt->enablePC)
    {
        SDEAPP_setSdeTriangCreateParams(appCntxt);
    }
}


vx_status SDEAPP_init(SDEAPP_Context *appCntxt)
{
    int32_t   status;
    vx_status vxStatus = VX_SUCCESS;

    status = appInit();
    if (status < 0)
    {
        LOG_ERROR("appInit() failed.\n");
        vxStatus = VX_FAILURE;
    }

#ifndef PC
    // Free DMPAC SL2
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        // Even though CM_vhwaDmpacSl2Free() fails, this demo still works
        // for 720p image. So we do NOT set vxStatus = VX_FAILURE
        status = CM_vhwaDmpacSl2Free();
        if (status < 0)
        {
            LOG_ERROR("CM_vhwaDmpacSl2Free() failed.\n");
        }
    }

    // Reallocate SDE SL2 for 2M inputs
    if (vxStatus == (vx_status) VX_SUCCESS && status == 0)
    {
        // Reallocation of SDE SL2 memory is called only when
        // CM_vhwaDmpacSl2Free() succeeds.
        // If  CM_vhwaDmpacSdeSl2Realloc() fails, we set vxStaus
        // to VX_FAILURE to stop
        status = CM_vhwaDmpacSdeSl2Realloc();
        if (status < 0)
        {
            LOG_ERROR("CM_vhwaDmpacSdeSl2Realloc() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }
#endif

    // OpenVX initialization
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        appCntxt->vxContext = vxCreateContext();
        if (appCntxt->vxContext == NULL)
        {
            LOG_ERROR("vxCreateContext() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }

    // Create graph
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        appCntxt->vxGraph = vxCreateGraph(appCntxt->vxContext);
        if (appCntxt->vxGraph == NULL)
        {
            LOG_ERROR("vxCreateGraph() failed\n");
            vxStatus = VX_FAILURE;
        } else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGraph,
                               "SDE Graph");
        }
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        /* load image processing kernel */
        tivxImgProcLoadKernels(appCntxt->vxContext);

        /* Load HWA kernels */
        tivxHwaLoadKernels(appCntxt->vxContext);

        /* Load stereo kernels */
        tivxStereoLoadKernels(appCntxt->vxContext);
    }

    /*
     * 1 Setup Stereo LDC nodes
     */
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        vxStatus = SDEAPP_init_LDC(appCntxt);
    }

    /*
     * 2 Setup SDE nodes
     */
    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        vxStatus = SDEAPP_init_SDE(appCntxt);
    }

    /*
     * 3 Setup SDE Triangulation nodes
     */
    if (appCntxt->enablePC)
    {
        if (vxStatus == (vx_status) VX_SUCCESS)
        {
            vxStatus = SDEAPP_init_SDE_Triang(appCntxt);
        }
    }

    if (vxStatus == (vx_status) VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->sdePclPerf, "Stereo GRAPH");

        /*
         * set up the pipeline.
         */
        vxStatus = SDEAPP_setupPipeline(appCntxt);

        /* Verify graph */
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vxStatus = vxVerifyGraph(appCntxt->vxGraph);
            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("vxVerifyGraph() failed\n");
            }
        }
    } else
    {
        vxStatus = (vx_status)VX_FAILURE;
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
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_sde_pcl");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }

        appCntxt->exitInputDataProcess = false;
        appCntxt->outputCtrlSem = new Semaphore(0);
        appCntxt->state = SDEAPP_STATE_INIT;

        SDEAPP_reset(appCntxt);
        appPerfStatsResetAll();
    }

    return vxStatus;
}

vx_status SDEAPP_init_camInfo(SDEAPP_Context *appCntxt,
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

    // Check if width and height is larger than maximum SDE resolution:
    // For ZED camera, since input is already clipped to the max resolution,
    // it is not needed. But for other input, it would be necessary
    if (appCntxt->width > SDEAPP_MAX_IMAGE_WIDTH)
    {
        appCntxt->hImgOfst     = (appCntxt->width - SDEAPP_MAX_IMAGE_WIDTH) / 2;
        appCntxt->width        = SDEAPP_MAX_IMAGE_WIDTH;
        appCntxt->distCenterX -= appCntxt->hImgOfst;
    }

    if (appCntxt->height > SDEAPP_MAX_IMAGE_HEIGHT)
    {
        appCntxt->vImgOfst     = (appCntxt->height - SDEAPP_MAX_IMAGE_HEIGHT) / 2;
        appCntxt->height       = SDEAPP_MAX_IMAGE_HEIGHT;
        appCntxt->distCenterY -= appCntxt->vImgOfst;
    }

    return (vx_status)VX_SUCCESS;
}


vx_status SDEAPP_run(SDEAPP_Context *appCntxt,
                     const vx_uint8 * inputLeftImage,
                     const vx_uint8 * inputRightImage,
                     vx_uint64 timestamp)
{
    vx_status vxStatus;

    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    SDEAPP_graphParams* gpDesc;

    vxStatus = SDEAPP_getFreeParamRsrc(appCntxt, &gpDesc);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("SDEAPP_getFreeParamRsrc() failed. Incoming Frame is dropped!\n");
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

    // then run app
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // set time stamp
       *(gpDesc->timestamp) = timestamp;

        vxStatus = SDEAPP_process(appCntxt, gpDesc);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("SDEAPP_process() failed\n");
        }
    }

    return vxStatus;
}

void SDEAPP_deInit(SDEAPP_Context *appCntxt)
{
    int32_t status;
    uint8_t i;


    if (appCntxt->outputCtrlSem)
    {
        delete appCntxt->outputCtrlSem;
    }

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
    } else
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

    // release output point cloud object
    if (appCntxt->enablePC)
    {
        for (i = 0; i < appCntxt->pipelineDepth; i++)
        {
            vxReleaseUserDataObject(&appCntxt->vxOutputTriangPC[i]);
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

    status = appDeInit();

    if (status < 0)
    {
        LOG_ERROR("appDeInit() failed\n");
    }
}

static void SDEAPP_exitProcThreads(SDEAPP_Context *appCntxt)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               SDEAPP_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != VX_SUCCESS)
    {
        LOG_ERROR("vxSendUserEvent() failed.\n");
    }

    if (appCntxt->evtHdlrThread.joinable())
    {
        appCntxt->evtHdlrThread.join();
    }

    /* Let the display thread exit. */
    appCntxt->exitOutputThread = true;

    if (appCntxt->outputCtrlSem)
    {
        appCntxt->outputCtrlSem->notify();
    }

}

static void SDEAPP_dumpStats(SDEAPP_Context *appCntxt)
{
    if (appCntxt->exportPerfStats == 1)
    {
        std::string name = std::string("app_") + appCntxt->logFileName;
        FILE       *fp;

        fp = appPerfStatsExportOpenFile(".", (char *)name.c_str());

        if (fp != NULL)
        {
            SDEAPP_exportStats(appCntxt, fp, true);

            CM_printProctime(fp);
            appPerfStatsExportCloseFile(fp);
        }
        else
        {
            LOG_ERROR("Could not open [%s] for exporting "
                       "performance data\n", name.c_str());
        }
    }
}

void SDEAPP_cleanupHdlr(SDEAPP_Context *appCntxt)
{
    if (appCntxt->state == SDEAPP_STATE_INVALID)
    {
        return;
    }

    appCntxt->state = SDEAPP_STATE_INVALID;

    /* Wait for the threads to exit. */
    SDEAPP_exitProcThreads(appCntxt);

    LOG_INFO("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    SDEAPP_dumpStats(appCntxt);
    SDEAPP_printStats(appCntxt);

    LOG_INFO("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");

    if (appCntxt->rtLogEnable == 1)
    {
        std::string name = std::string("app_") + appCntxt->logFileName + std::string(".bin");
        tivxLogRtTraceExportToFile((char *)name.c_str());
    }

    /* Release the Application context. */
    SDELDC_delete(&appCntxt->sdeLdcHdl);

    if (appCntxt->sdeAlgoType == 0)
    {
        SL_SDE_delete(&appCntxt->slSdeHdl);
    }
    else if (appCntxt->sdeAlgoType == 1)
    {
        ML_SDE_delete(&appCntxt->mlSdeHdl);
    }

    SDE_TRIANG_delete(&appCntxt->sdeTriangHdl);


    SDEAPP_deInit(appCntxt);

    LOG_INFO("Clean-up complete.\n");
}

void SDEAPP_reset(SDEAPP_Context * appCntxt)
{
    /* Reset the performance capture initialization flag. */
    appCntxt->startPerfCapt = false;
}

static void SDEAPP_evtHdlrThread(SDEAPP_Context *appCntxt)
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
                if (evt.app_value == SDEAPP_USER_EVT_EXIT)
                {
                    break;
                }
            }

            if (evt.type == VX_EVENT_GRAPH_COMPLETED)
            {
                SDEAPP_processEvent(appCntxt, &evt);

                /* Wakeup the display thread. */
                if (appCntxt->outputCtrlSem)
                {
                    appCntxt->outputCtrlSem->notify();
                }
            }
        }

    } // while (true)
}

static int32_t SDEAPP_userControlThread(SDEAPP_Context *appCntxt)
{
    vx_status   vxStatus = VX_SUCCESS;

    if (appCntxt->is_interactive)
    {
        uint32_t done = 0;

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
                    SDEAPP_printStats(appCntxt);

                    CM_resetProctime();
                    break;

                case 'e':
                    SDEAPP_dumpStats(appCntxt);
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

        appCntxt->state = SDEAPP_STATE_SHUTDOWN;
        LOG_INFO("Waiting for the graph to finish.\n");

        vxStatus = SDEAPP_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("SDEAPP_waitGraph() failed\n");
        }

        SDEAPP_cleanupHdlr(appCntxt);

        LOG_INFO("\nDEMO FINISHED!\n");
    } // if (appCntxt->is_interactive)

    return vxStatus;
}


void SDEAPP_launchProcThreads(SDEAPP_Context *appCntxt)
{
    /* Launch the input data thread. */
    appCntxt->userCtrlThread = std::thread(SDEAPP_userControlThread, appCntxt);

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread = std::thread(SDEAPP_evtHdlrThread, appCntxt);

    appCntxt->userCtrlThread.detach();

    /* Launch the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfLaunchCtrlThread(appCntxt->logFileName);
    }
}

void SDEAPP_intSigHandler(SDEAPP_Context *appCntxt)
{
    /* Stop the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfStopCtrlThread();
    }

    if (appCntxt->state != SDEAPP_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        vx_status   vxStatus = VX_SUCCESS;

        appCntxt->state = SDEAPP_STATE_SHUTDOWN;
        LOG_INFO("Waiting for the graph to finish.\n");

        vxStatus = SDEAPP_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("SDEAPP_waitGraph() failed\n");
        }

        SDEAPP_cleanupHdlr(appCntxt);
        LOG_INFO("\nDEMO FINISHED!\n");
    }
}
