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

#include <utils/app_init/include/app_init.h>
#include <cm_profile.h>
#include <visloc.h>

static char menu[] = {
    "\n"
    "\n =========================================="
    "\n Demo : Visual Localization                "
    "\n =========================================="
    "\n"
    "\n p: Print performance statistics"
    "\n"
    "\n e: Export performance statistics"
    "\n"
    "\n x: Exit"
    "\n"
    "\n Enter Choice: "
};

vx_status VISLOC_init(VISLOC_Context *appCntxt)
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

        if (status < 0)
        {
            LOG_ERROR("getConfig() failed.\n");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // Populate infConfig
        status = appCntxt->dlInferConfig.getConfig(modelPath, true, 1);

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
                               "Visual Localization Graph");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* load image processing kernel */
        tivxImgProcLoadKernels(appCntxt->vxContext);

        /* load HWA kernels */
        tivxHwaLoadKernels(appCntxt->vxContext);

        /* Create Visual localization nodes */
        vxStatus = VISLOC_init_VL(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("VISLOC_init_VL() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        appPerfPointSetName(&appCntxt->vlPerf,
                            "Visual Localization GRAPH");

        vxStatus = VISLOC_setupPipeline(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("VISLOC_setupPipeline() failed\n");
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
            tivxExportGraphToDot(appCntxt->vxGraph, ".", "vx_app_visloc");
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
        appCntxt->exitDlInferThread    = false;

        appCntxt->visLocSem            = new Semaphore(0);
        appCntxt->exitVisLocThread     = false;

        appCntxt->exitInputDataProcess = 0;
        appCntxt->state                = VISLOC_STATE_INIT;

        VISLOC_reset(appCntxt);
        appPerfStatsResetAll();
    }

    return vxStatus;
}

vx_status VISLOC_run(VISLOC_Context       *appCntxt,
                     const unsigned char  *inputImage,
                     uint64_t              timestamp)
{
    VISLOC_graphParams* gpDesc;
    vx_status           vxStatus;
    TimePoint           start;
    TimePoint           end;
    float               diff;

    vxStatus = VISLOC_popFreeInputDesc(appCntxt, &gpDesc);
    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_WARN("VISLOC_popFreeInputDesc() failed. Incoming Frame is dropped!\n");
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
            VISLOC_enqueInputDesc(appCntxt, gpDesc);

            LOG_ERROR("CM_copyData2Image() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = VISLOC_process(appCntxt,
                                  gpDesc,
                                  timestamp);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("VISLOC_process() failed\n");
        }
    }

    return vxStatus;
}

void VISLOC_deInit(VISLOC_Context *appCntxt)
{
    int32_t status;

    // release graph
    vxReleaseGraph(&appCntxt->vxGraph);

    tivxImgProcUnLoadKernels(appCntxt->vxContext);
    tivxHwaUnLoadKernels(appCntxt->vxContext);

    /* Release the context. */
    vxReleaseContext(&appCntxt->vxContext);

    /* Delete the Pre-proc instance. */
    delete appCntxt->preProcObj;

    /* Delete the DL Inferer instance. */
    delete appCntxt->dlInferer;

    /* De-initialize. */
    status = appDeInit();

    if (status < 0)
    {
        LOG_ERROR("appDeInit() failed.\n");
    }
}

static void VISLOC_exitProcThreads(VISLOC_Context *appCntxt)
{
    vx_status vxStatus;

    appCntxt->exitInputDataProcess = true;

    /* Let the event handler thread exit. */
    vxStatus = vxSendUserEvent(appCntxt->vxContext,
                               VISLOC_USER_EVT_EXIT,
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

    /* Set the exit flag for the DLR thread. */
    appCntxt->exitDlInferThread = true;

    /* Wake-up the DLR thread. */
    if (appCntxt->dlDataReadySem)
    {
        appCntxt->dlDataReadySem->notify();
    }

    /* Set the exit flag for the post-process thread. */
    appCntxt->exitVisLocThread = true;

    /* Wake-up the post-process thread. */
    if (appCntxt->visLocSem)
    {
        appCntxt->visLocSem->notify();
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

    if (appCntxt->visLocThread.joinable())
    {
        appCntxt->visLocThread.join();
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

    if (appCntxt->visLocSem)
    {
        delete appCntxt->visLocSem;
    }
}

static void VISLOC_dumpStats(VISLOC_Context *appCntxt)
{
    if (appCntxt->exportPerfStats == 1)
    {
        const char *name = VISLOC_PERF_OUT_FILE;
        FILE       *fp;

        fp = appPerfStatsExportOpenFile(".", (char *)name);

        if (fp != NULL)
        {
            VISLOC_exportStats(appCntxt, fp, true);

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

void VISLOC_cleanupHdlr(VISLOC_Context *appCntxt)
{
    if (appCntxt->state == VISLOC_STATE_INVALID)
    {
        return;
    }

    appCntxt->state = VISLOC_STATE_INVALID;

    /* Wait for the threads to exit. */
    VISLOC_exitProcThreads(appCntxt);

    LOG_INFO("========= BEGIN:PERFORMANCE STATS SUMMARY =========\n");
    VISLOC_dumpStats(appCntxt);
    VISLOC_printStats(appCntxt);

    LOG_INFO("========= END:PERFORMANCE STATS SUMMARY ===========\n\n");

    if (appCntxt->rtLogEnable == 1)
    {
        char name[256];

        snprintf(name, 255, "%s.bin", VISLOC_PERF_OUT_FILE);
        tivxLogRtTraceExportToFile(name);
    }

    /* Deinit visual localization nodes */
    VISLOC_deinit_VL(appCntxt);

    VISLOC_deInit(appCntxt);

    LOG_INFO("Clean-up complete.\n");
}


void VISLOC_reset(VISLOC_Context * appCntxt)
{
    appCntxt->startPerfCapt = false;
}


static void VISLOC_evtHdlrThread(VISLOC_Context *appCntxt)
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
                if (evt.app_value == VISLOC_USER_EVT_EXIT)
                {
                    break;
                }
            }

            if (evt.type == VX_EVENT_GRAPH_COMPLETED ||
                evt.type == VX_EVENT_NODE_COMPLETED)
            {
                vxStatus =
                    VISLOC_processEvent(appCntxt, &evt);

                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("VISLOC_processEvent() failed\n");
                }

            }
        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");
}

static int32_t VISLOC_userControlThread(VISLOC_Context *appCntxt)
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
                VISLOC_printStats(appCntxt);

                CM_resetProctime();
                break;

            case 'e':
                VISLOC_dumpStats(appCntxt);
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

    appCntxt->state = VISLOC_STATE_SHUTDOWN;
    LOG_INFO("Waiting for the graph to finish.\n");

    vxStatus = VISLOC_waitGraph(appCntxt);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("VISLOC_waitGraph() failed\n");
    }

    VISLOC_cleanupHdlr(appCntxt);

    LOG_INFO("\nDEMO FINISHED!\n");

    return vxStatus;
}

static void VISLOC_preProcThread(VISLOC_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        VISLOC_graphParams  *desc;
        vx_status            vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->preProcSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitPreprocThread == true)
        {
            break;
        }

        vxStatus = VISLOC_popPreprocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();
            vxStatus =
                VISLOC_preProcess(appCntxt,
                                  desc->vxScalerOut,
                                  desc->inferInputBuff);

            end = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Preprocessing", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("VISLOC_preProcess() failed \n");
            }
            else
            {
                /* Push the descriptor to the DLR queue. */
                VISLOC_enqueDlInferInputDesc(appCntxt, desc);

                /* Wakeup the DLR thread. The DLR thread will process the
                 * descriptor at the head of the queue.
                 */
                appCntxt->dlDataReadySem->notify();
            }
        } // if (vxStatus == (vx_status)VX_SUCCESS)
        else
        {
            LOG_ERROR("VISLOC_popPreprocInputDesc() failed. \n");
        }

    } // while (true)

    LOG_INFO("Exiting.\n");
}

static void VISLOC_dlInferThread(VISLOC_Context  *appCntxt)
{
    DLInferer  *inferer = appCntxt->dlInferer;
    TimePoint   start;
    TimePoint   end;
    float       diff;

    LOG_INFO("Launched.\n");

    while (true)
    {
        VISLOC_graphParams  *desc;
        vx_status            vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->dlDataReadySem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitDlInferThread == true)
        {
            break;
        }

        vxStatus = VISLOC_popDlInferInputDesc(appCntxt, &desc);

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
                VISLOC_enqueVisLocInputDesc(appCntxt, desc);

                /* Wakeup the post-process thread. The post-process thread will
                 * process the descriptor at the head of the queue.
                 */
                appCntxt->visLocSem->notify();
            }
        }
        else
        {
            LOG_ERROR("VISLOC_popDLRInputDesc() failed.\n");
        }

    } // while (true)

    LOG_INFO("Exiting.\n");
}

static void VISLOC_visLocThread(VISLOC_Context  *appCntxt)
{
    // For profiling
    chrono::time_point<chrono::system_clock> start, end;
    float diff;

    vx_reference  obj[VISLOC_NUM_GRAPH_PARAMS];
    uint8_t       cnt, startCnt;
    int32_t       i;

    LOG_INFO("Launched.\n");

    while (true)
    {
        VISLOC_graphParams    *desc;
        vx_status            vxStatus;

        /* Wait for the input buffer availability. */
        appCntxt->visLocSem->wait();

        /*  Check if we need to exit the thread. */
        if (appCntxt->exitVisLocThread == true)
        {
            break;
        }

        /*
         * Pop VisLocInputDesc
         * If we get VisLocInputDesc() here and pop upon graph completion event,
         * the same buffer can be overwritten when visLocSem is called again before
         * grpah completion.
         */
        vxStatus = VISLOC_popVisLocInputDesc(appCntxt, &desc);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();

            vxStatus =
                VISLOC_createScoreOutTensor(appCntxt,
                                            *desc->inferOutputBuff,
                                            desc->vxOutTensor[0]);

            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Output_tensor_creation", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("VISLOC_createScoreOutTensor() failed.\n");
            }
        } else
        {
                LOG_ERROR("VISLOC_popVisLocInputDesc() failed.\n");
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            start = GET_TIME();

            vxStatus =
                VISLOC_createDescOutTensor(appCntxt,
                                           *desc->inferOutputBuff,
                                           desc->vxOutTensor[1]);

            end   = GET_TIME();
            diff  = GET_DIFF(start, end);
            CM_reportProctime("Output_tensor_creation", diff);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("VISLOC_createDescOutTensor() failed.\n");
            }
        }


        // Enqueue for visual localization (pose calculation and pose visualization)
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            cnt      = appCntxt->enableLdcNode? 3: 2;
            startCnt = cnt;

            obj[cnt++] = (vx_reference)desc->vxOutTensor[0];
            obj[cnt++] = (vx_reference)desc->vxOutTensor[1];
            obj[cnt++] = (vx_reference)desc->vxPoseMatrix;
            obj[cnt++] = (vx_reference)desc->vxOutputImage;

            for (i = startCnt; i < cnt; i++)
            {
                vxStatus = vxGraphParameterEnqueueReadyRef(appCntxt->vxGraph,
                                                           i,
                                                           &obj[i],
                                                           1);
                if (vxStatus != (vx_status)VX_SUCCESS)
                {
                    LOG_ERROR("vxGraphParameterEnqueueReadyRef(%d) failed.\n", i);
                    break;
                }
            }

            /*
             * It is fine to enqueue OuputDesc here in advance,
             * since outputCtrlSem is notified when the grpah completed
             */
            VISLOC_enqueOutputDesc(appCntxt, desc);
        } // if (vxStatus == (vx_status)VX_SUCCESS)

    } // while (true)

    LOG_INFO("Exiting.\n");
}


void VISLOC_launchProcThreads(VISLOC_Context *appCntxt)
{
    /* Launch the graph processing thread. */
    if (appCntxt->is_interactive)
    {
        appCntxt->userCtrlThread =
            std::thread(VISLOC_userControlThread, appCntxt);

        appCntxt->userCtrlThread.detach();
    }

    /* Launch the event handler thread. */
    appCntxt->evtHdlrThread =
        std::thread(VISLOC_evtHdlrThread, appCntxt);

    /* Launch CNN pre-processing thread */
    appCntxt->preProcThread =
        std::thread(VISLOC_preProcThread, appCntxt);

    /* Launch DLR processing thread */
    appCntxt->dlInferThread =
        std::thread(VISLOC_dlInferThread, appCntxt);

    /* launch visual localization thread. */
    appCntxt->visLocThread =
            std::thread(VISLOC_visLocThread, appCntxt);

    /* Launch the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfLaunchCtrlThread("visloc");
    }
}

void VISLOC_intSigHandler(VISLOC_Context *appCntxt)
{
    /* Stop the performance stats reset thread. */
    if (appCntxt->exportPerfStats == 1)
    {
        CM_perfStopCtrlThread();
    }

    if (appCntxt->state != VISLOC_STATE_INVALID)
    {
        /* Wait for the threads to exit. */
        vx_status   vxStatus = VX_SUCCESS;

        appCntxt->state = VISLOC_STATE_SHUTDOWN;
        LOG_INFO("Waiting for the graph to finish.\n");

        vxStatus = VISLOC_waitGraph(appCntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("VISLOC_waitGraph() failed\n");
        }

        VISLOC_cleanupHdlr(appCntxt);
        LOG_INFO("\nDEMO FINISHED!\n");
    }
}

vx_status VISLOC_extractPoseData(double          *outPose,
                                 double          *outQuaternion,
                                 const vx_matrix  pose)
{

    vx_status  vxStatus = (vx_status)VX_SUCCESS;
    vx_float32 posemat[12];

    vxStatus = vxCopyMatrix(pose, (void *)posemat, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        Eigen::Matrix3d mat;
        mat(0,0) = posemat[0];
        mat(0,1) = posemat[1];
        mat(0,2) = posemat[2];
        mat(1,0) = posemat[4];
        mat(1,1) = posemat[5];
        mat(1,2) = posemat[6];
        mat(2,0) = posemat[8];
        mat(2,1) = posemat[9];
        mat(2,2) = posemat[10];

#if 0
        Eigen::Vector3d ea = mat.eulerAngles(0, 1, 2);

        // This gives the same result as q(mat)
        Eigen::Quaterniond q = Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY())
                             * Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX());

        //Eigen::Quaterniond q(mat);

        // Eigen library get value in the order of roll, pitch, yaw
        // Axis seems slight different from ROS coordinate (Roll <-> Pitch)
        // And localization coordinate switch Y and Z
        outQuaternion[0] = q.x();
        outQuaternion[1] = q.y();
        outQuaternion[2] = q.z();
        outQuaternion[3] = q.w();
#else
        // Simply use unit quaternion
        outQuaternion[0] = 0;
        outQuaternion[1] = 0;
        outQuaternion[2] = 0;
        outQuaternion[3] = 1;
#endif

        outPose[0] = posemat[3];
        outPose[1] = posemat[11];   // switch y and z
        outPose[2] = posemat[7];
    }

    return vxStatus;
}
