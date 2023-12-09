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

#include <utils/app_init/include/app_init.h>
#include <cm_profile.h>
#include <vision_cnn.h>

static char menu[] = {
    "\n"
    "\n =================-====="
    "\n Demo : Vision CNN      "
    "\n ======================="
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n e: Export performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};

vx_status VISION_CNN_init(VISION_CNN_Context *appCntxt)
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

        appCntxt->postProcCfg.vizThreshold = appCntxt->detVizThreshold;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // Populate infConfig
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
        appCntxt->dlImageWidth  = appCntxt->preProcCfg.resizeWidth;
        appCntxt->dlImageHeight = appCntxt->preProcCfg.resizeHeight;

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
        /* Set input data width and height based on the inference engine
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
            appCntxt->logFileName = "semseg";
        }
         else if (appCntxt->postProcCfg.taskType == "object_6d_pose_estimation")
        {
            appCntxt->postProcCfg.inDataWidth  = appCntxt->preProcCfg.outDataWidth;
            appCntxt->postProcCfg.inDataHeight = appCntxt->preProcCfg.outDataHeight;
            appCntxt->logFileName = "object6Dpose";
        }
        else
        {
            appCntxt->postProcCfg.inDataWidth  = appCntxt->preProcCfg.outDataWidth;
            appCntxt->postProcCfg.inDataHeight = appCntxt->preProcCfg.outDataHeight;
            appCntxt->logFileName = "objdet";
        }

        appCntxt->outImageHeight = appCntxt->postProcCfg.inDataHeight;
        appCntxt->outImageWidth  = appCntxt->postProcCfg.inDataWidth;

        /* assign the outDataWidth & outDataHeight */
        appCntxt->postProcCfg.outDataWidth  = appCntxt->inputImageWidth;
        appCntxt->postProcCfg.outDataHeight = appCntxt->inputImageHeight;

        appCntxt->postProcObj =
            CmPostprocessImage::makePostprocessImageObj(appCntxt->postProcCfg);

        if (appCntxt->postProcObj == nullptr)
        {
            LOG_ERROR("CmPostprocessImage::makePostprocessImageObj() failed.\n");

            vxStatus = VX_FAILURE;
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
        }
        else
        {
            vxSetReferenceName((vx_reference)appCntxt->vxGraph,
                               "Vision CNN Graph");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* load image processing kernel */
        tivxImgProcLoadKernels(appCntxt->vxContext);

        /* load HWA kernels */
        tivxHwaLoadKernels(appCntxt->vxContext);

        /* Create CNN node */
        vxStatus = VISION_CNN_init_SS(appCntxt);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->visonPerf,
                            "Vision CNN GRAPH");

        vxStatus = VISION_CNN_setupPipeline(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("VISION_CNN_setupPipeline() failed\n");
        }
    }

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

    if (vxStatus == (vx_status)VX_SUCCESS)
    {

        if (appCntxt->exportGraph == 1)
        {
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_vison_cnn");
        }

        if (appCntxt->rtLogEnable == 1)
        {
            tivxLogRtTraceEnable(appCntxt->vxGraph);
        }

        appCntxt->exitOutputThread     = false;
        appCntxt->outputCtrlSem        = new Semaphore(0);

        appCntxt->preProcSem           = new Semaphore(0);
        appCntxt->exitPreprocThread    = false;

        appCntxt->dlDataReadySem       = new Semaphore(0);
        appCntxt->exitDlInferrThread   = false;

        appCntxt->postProcSem          = new Semaphore(0);
        appCntxt->exitPostprocThread   = false;

        appCntxt->exitInputDataProcess = 0;
        appCntxt->state                = VISION_CNN_STATE_INIT;

        VISION_CNN_reset(appCntxt);
        appPerfStatsResetAll();
    }

    return vxStatus;
}

vx_status VISION_CNN_run(VISION_CNN_Context   *appCntxt,
                         const unsigned char  *inputImage,
                         uint64_t              timestamp)
{
    VISION_CNN_graphParams *gpDesc;
    vx_status               vxStatus;
    TimePoint               start;
    TimePoint               end;
    float                   diff;

    vxStatus = VISION_CNN_popFreeInputDesc(appCntxt, &gpDesc);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("VISION_CNN_popFreeInputDesc() failed. Incoming Frame is dropped!\n");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        start = GET_TIME();
        vxStatus = CM_copyData2Image(inputImage,
                                     gpDesc->vxInputImage,
                                     0,
                                     0);

        end   = GET_TIME();
        diff  = GET_DIFF(start, end);
        CM_reportProctime("input_image_load", diff);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            /* Release the descriptor back to the free descriptor queue. */
            VISION_CNN_enqueInputDesc(appCntxt, gpDesc);

            LOG_ERROR("CM_copyData2Image() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = VISION_CNN_process(appCntxt,
                                      gpDesc,
                                      timestamp);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("VISION_CNN_process() failed\n");
        }
    }

    return vxStatus;
}

void VISION_CNN_deInit(VISION_CNN_Context *appCntxt)
{
    int32_t status;

    // release graph
    vxReleaseGraph(&appCntxt->vxGraph);

    //tivxTIDLUnLoadKernels(appCntxt->vxContext);
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

static void VISION_CNN_exitProcThreads(VISION_CNN_Context *appCntxt)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               VISION_CNN_USER_EVT_EXIT,
                               NULL);

    if (vxStatus != (vx_status)VX_SUCCESS)
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

    /* Set the exit flag for the DL Inference thread. */
    appCntxt->exitDlInferrThread = true;

    /* Wake-up the DL Inference thread. */
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

static void VISION_CNN_dumpStats(VISION_CNN_Context *appCntxt)
{
    if (appCntxt->exportPerfStats == 1)
    {
        std::string name = std::string("app_") + appCntxt->logFileName;
        FILE       *fp;

        fp = appPerfStatsExportOpenFile(".", (char *)name.c_str());

        if (fp != NULL)
        {
            VISION_CNN_exportStats(appCntxt, fp, true);

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

void VISION_CNN_cleanupHdlr(VISION_CNN_Context *appCntxt)
{
    if (appCntxt->state == VISION_CNN_STATE_INVALID)
    {
        return;
    }

    appCntxt->state = VISION_CNN_STATE_INVALID;

    /* Wait for the threads to exit. */
    VISION_CNN_exitProcThreads(appCntxt);

    LOG_INFO("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    VISION_CNN_dumpStats(appCntxt);
    VISION_CNN_printStats(appCntxt);

    LOG_INFO("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");

    if (appCntxt->rtLogEnable == 1)
    {
        std::string name = std::string("app_") + appCntxt->logFileName + std::string(".bin");
        tivxLogRtTraceExportToFile((char *)name.c_str());
    }

    /* Deinit vision CNN node */
    VISION_CNN_deinit_core(appCntxt);

    VISION_CNN_deInit(appCntxt);

    LOG_INFO("Clean-up complete.\n");
}

void VISION_CNN_reset(VISION_CNN_Context * appCntxt)
{
    appCntxt->startPerfCapt = false;
}

static void VISION_CNN_evtHdlrThread(VISION_CNN_Context *appCntxt)
{
    vx_event_t  evt;
    vx_status   vxStatus;

    vxStatus = VX_SUCCESS;

    LOG_INFO("Launched.\n");

    /* Clear any pending events. The third argument is do_not_block = true. */
    while (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_true_e);
    }

    while (true)
    {
        vxStatus = vxWaitEvent(appCntxt->vxContext, &evt, vx_false_e);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            if (evt.type == VX_EVENT_USER)
            {
                if (evt.app_value == VISION_CNN_USER_EVT_EXIT)
                {
                    break;
                }
                else if (evt.app_value == VISION_CNN_OUT_AVAIL_EVT)
                {
                    /* Wakeup the display thread. */
                    if (appCntxt->outputCtrlSem)
                    {
                        appCntxt->outputCtrlSem->notify();
                    }
                }
            }

            if (evt.type == VX_EVENT_NODE_COMPLETED)
            {
                vxStatus =
                    VISION_CNN_processEvent(appCntxt, &evt);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("VISION_CNN_processEvent() failed\n");
                }

            } // if (evt.type == VX_EVENT_GRAPH_COMPLETED)

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");
}

static int32_t VISION_CNN_userControlThread(VISION_CNN_Context *appCntxt)
{
    vx_status   vxStatus = VX_SUCCESS;
    uint32_t    done = 0;

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
                VISION_CNN_printStats(appCntxt);

                CM_resetProctime();
                break;

            case 'e':
                VISION_CNN_dumpStats(appCntxt);
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

    appCntxt->state = VISION_CNN_STATE_SHUTDOWN;
    LOG_INFO("Waiting for the graph to finish.\n");

    vxStatus = VISION_CNN_waitGraph(appCntxt);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("VISION_CNN_waitGraph() failed\n");
    }

    VISION_CNN_cleanupHdlr(appCntxt);

    LOG_INFO("\nDEMO FINISHED!\n");

    return vxStatus;
}

static void VISION_CNN_preProcThread(VISION_CNN_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        VISION_CNN_graphParams  *desc;
        vx_status                vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->preProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPreprocThread == true)
        {
            break;
        }

        vxStatus = VISION_CNN_popPreprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();
            vxStatus =
                VISION_CNN_preProcess(appCntxt,
                                      desc->vxScalerOut,
                                      desc->inferInputBuff);

            end = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Preprocessing", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("VISION_CNN_preProcess() failed.\n");
            }
            else
            {
                /* Push the descriptor to the DL Inference queue. */
                VISION_CNN_enqueDlInferInputDesc(appCntxt, desc);

                /* Wakeup the DL Inference thread. The DL Inference thread will
                 * process the descriptor at the head of the queue.
                 */
                appCntxt->dlDataReadySem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");
}

static void VISION_CNN_dlInferThread(VISION_CNN_Context  *appCntxt)
{
    DLInferer  *inferer = appCntxt->dlInferer;
    TimePoint   start;
    TimePoint   end;
    float       diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        VISION_CNN_graphParams  *desc;
        vx_status                vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->dlDataReadySem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitDlInferrThread == true)
        {
            break;
        }

        vxStatus = VISION_CNN_popDlInferInputDesc(appCntxt, &desc);

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
                /* Push the descriptor to the Post-process queue. */
                VISION_CNN_enquePostprocInputDesc(appCntxt, desc);

                /* Wakeup the post-process thread. The post-process thread will
                 * process the descriptor at the head of the queue.
                 */
                appCntxt->postProcSem->notify();
            }

        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");
}

static void VISION_CNN_postProcThread(VISION_CNN_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        VISION_CNN_graphParams  *desc;
        vx_status                vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->postProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPostprocThread == true)
        {
            break;
        }

        vxStatus = VISION_CNN_popPostprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            // create vxOutTensor always
            start = GET_TIME();
            vxStatus =
                VISION_CNN_createOutTensor(appCntxt,
                                           desc->inferOutputBuff,
                                           desc->vxOutTensor);

            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Output_tensor_creation", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("VISION_CNN_createOutTensor() failed.\n");
            }

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Push the descriptor to the output queue. */
                VISION_CNN_enqueOutputDesc(appCntxt, desc);
            }

            if (vxStatus == (vx_status)VX_SUCCESS)
            {
                /* Let the usr know that output is ready. */
                vxStatus =
                    vxSendUserEvent(appCntxt->vxContext,
                                    VISION_CNN_OUT_AVAIL_EVT,
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

void VISION_CNN_launchProcThreads(VISION_CNN_Context *appCntxt)
{
    /* Launch the graph processing thread. */
    if (appCntxt->is_interactive)
    {
        appCntxt->userCtrlThread =
            std::thread(VISION_CNN_userControlThread, appCntxt);

        appCntxt->userCtrlThread.detach();
    }

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread =
        std::thread(VISION_CNN_evtHdlrThread, appCntxt);

    /* Launch CNN pre-processing thread */
    appCntxt->preProcThread =
        std::thread(VISION_CNN_preProcThread, appCntxt);

    /* Launch DL Inference processing thread */
    appCntxt->dlInferThread =
        std::thread(VISION_CNN_dlInferThread, appCntxt);

    /* launch post-processing thread. */
    appCntxt->postProcThread =
            std::thread(VISION_CNN_postProcThread, appCntxt);

    /* Launch the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfLaunchCtrlThread(appCntxt->logFileName);
    }
}

void VISION_CNN_intSigHandler(VISION_CNN_Context *appCntxt)
{
    /* Stop the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfStopCtrlThread();
    }

    if (appCntxt->state != VISION_CNN_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        vx_status   vxStatus = VX_SUCCESS;

        appCntxt->state = VISION_CNN_STATE_SHUTDOWN;
        LOG_INFO("Waiting for the graph to finish.\n");

        vxStatus = VISION_CNN_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("VISION_CNN_waitGraph() failed\n");
        }

        VISION_CNN_cleanupHdlr(appCntxt);
        LOG_INFO("\nDEMO FINISHED!\n");
    }
}
