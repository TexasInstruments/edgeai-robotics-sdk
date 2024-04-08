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
#ifndef _APP_VISION_CNN_H_
#define _APP_VISION_CNN_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <thread>
#include <queue>
#include <vector>

#include <itidl_ti.h>
#include <TI/j7_tidl.h>
#include <TI/tivx_img_proc.h>

#include <cm_scaler_node_cntxt.h>
#include <cm_preproc_node_cntxt.h>
#include <cm_ldc_node_cntxt.h>
#include <cm_profile.h>
#include <cm_pre_process_image.h>
#include <cm_post_process_image.h>
#include <ti_queue.h>

/**
 * \defgroup group_ticore_vision_cnn CNN based vision processing
 *
 * \brief It performs versatile deep-learning (DL) inference that is optimized on
 *        DL cores and TDA4 HWAs. It supports compute-intensive DL inference operations
 *        including 2D object detection and semantic segmentation.
 *
 *        The figure below shows the high-level block diagram for semantic segmentation
 *        as an example, which consists of multiple processing blocks that are deployed
 *        on HWAs and DSP processors for pre-processing and post-processing in an
 *        optimized manner.
 *
 *        \image html semseg_demo_block_diagram.svg "CNN based semantic segmentation" width = 1000
 *
 * \ingroup  group_ticore_apps
 *
 */

using namespace ti_core_common;
using namespace ti::utils;
using namespace ti::dl_inferer;

#ifdef __cplusplus
extern "C" {
#endif

#include <utils/perf_stats/include/app_perf_stats.h>
#include <tivx_utils_graph_perf.h>

#ifdef __cplusplus
}
#endif

/**
 * \brief Default input image width
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_DEFAULT_IMAGE_WIDTH  (1280)

/**
 * \brief Default input image height
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_DEFAULT_IMAGE_HEIGHT (720)

/**
 * \brief LDC down sampling factor
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_LDC_DS_FACTOR            (2)

/**
 * \brief LDC block width
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_LDC_BLOCK_WIDTH          (64)

/**
 * \brief LDC block height
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_LDC_BLOCK_HEIGHT         (16)

/**
 * \brief Pixel padding flag in LDC
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_LDC_PIXEL_PAD            (1)

/**
 * \brief Maximum file name length
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_MAX_LINE_LEN         (1024U)

/**
 * \brief Appliation state id - Invalid
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_STATE_INVALID        (0U)

/**
 * \brief Appliation state id - Initialized
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_STATE_INIT           (1U)

/**
 * \brief Appliation state id - Shutdown
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_STATE_SHUTDOWN       (2U)

/**
 * \brief Scaler node completion event id
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_SCALER_NODE_COMPLETE_EVENT (0U)

/**
 * \brief Event id that notifiy CNN ouptut tensor is available
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_OUT_AVAIL_EVT              (VISION_CNN_SCALER_NODE_COMPLETE_EVENT + 1)

/**
 * \brief Event id that notify application exits by user
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_USER_EVT_EXIT              (VISION_CNN_OUT_AVAIL_EVT + 1)

/**
 * \brief Number of graph parameters
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_NUM_GRAPH_PARAMS      (3U)

/**
 * \brief Maximum output tensor dimension
 * \ingroup group_ticore_vision_cnn
 */
#define VISION_CNN_MAX_OUT_TENSOR_DIMS   (8U)

/**
 * \brief Maximum number of objects to detect for detection
 * \ingroup group_ticore_vision_cnn
 */
#define CM_POST_PROCESS_DETECT_MAX_OBJECTS    (100U)

/**
 * \brief Output object dimension for detection
 * \ingroup group_ticore_vision_cnn
 */
#define CM_POST_PROCESS_DETECT_OUTTENSOR_DIM  (2U)

/**
 * \brief Number of fields for detected object
 * \ingroup group_ticore_vision_cnn
 */
#define CM_POST_PROCESS_DETECT_NUM_FIELDS     (6U)

/**
 * \brief Number of fields for detected object in 6D Pose Model
 * \ingroup group_ticore_vision_cnn
 */
#define CM_POST_PROCESS_POSE6D_NUM_FIELDS     (15U)

/**
 * \brief Number of fields for detected object in Human Pose Model
 * \ingroup group_ticore_vision_cnn
 */
#define CM_POST_PROCESS_HUMANPOSE_NUM_FIELDS     (57U)

using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

/**
 * \brief Graph parameters
 * \ingroup group_ticore_vision_cnn
 */
struct VISION_CNN_graphParams
{
    /** Graph parameter 0 */
    vx_image                vxInputImage;

    /** Graph parameter 1 */
    vx_image                vxRectifiedImage;

    /** Graph parameter 2 */
    vx_image                vxScalerOut;

    /** Output Tensor after post-processing. */
    vx_tensor               vxOutTensor;

    /** Input to DL Inference engine. */
    VecDlTensorPtr         *inferInputBuff;

    /** Output from the DL Inference engine. */
    VecDlTensorPtr         *inferOutputBuff;

    /** timestamp - Not a Graph param */
    vx_uint64              *timestamp;
};

using VISION_CNN_Queue =
     MultiThreadQ<VISION_CNN_graphParams>;

/**
 * \brief Application context parameters
 * \ingroup group_ticore_vision_cnn
 */
struct VISION_CNN_Context
{
    /** Application state */
    uint32_t                state;

    /** openVX context handle. */
    vx_context              vxContext;

    /** Graph handle. */
    vx_graph                vxGraph{};

    /** LDC node context object. */
    CM_LdcNodeCntxt         ldcObj{};

    /** Scaler node context object. */
    CM_ScalerNodeCntxt      scalerObj{};

    /** Pre-process configuration. */
    PreprocessImageConfig   preProcCfg;

    /** Post-processing configuration.*/
    PostprocessImageConfig  postProcCfg;

    /** DL Inference configuration. */
    InfererConfig           dlInferConfig{};

    /** DL Inference context. */
    DLInferer              *dlInferer{nullptr};

    /** Pre-process context. */
    CmPreprocessImage      *preProcObj{nullptr};

    /** Post-process context. */
    CmPostprocessImage     *postProcObj{nullptr};

    /** Input buffers to the inference. */
    VecDlTensorPtr          inferInputBuff[GRAPH_MAX_PIPELINE_DEPTH];

    /** Output buffers to the inference. */
    VecDlTensorPtr          inferOutputBuff[GRAPH_MAX_PIPELINE_DEPTH];

    /** Input image object  */
    vx_image                vxInputImage[GRAPH_MAX_PIPELINE_DEPTH];

    /** Rectified image object  */
    vx_image                vxRectifiedImage[GRAPH_MAX_PIPELINE_DEPTH];

    /** Output Tensor after post-processing. */
    vx_tensor               vxOutTensor[GRAPH_MAX_PIPELINE_DEPTH];

    /** Input timestamp */
    vx_uint64               timestamp[GRAPH_MAX_PIPELINE_DEPTH];

    /** Input image object to be displayed */
    vx_image                vxInputDisplayImage;

    /** output tensor: number of dimemsion */
    int32_t                 outTensorNumDim;

    /** output tensor: tensor dims */
    vx_size                 outTensorDims[VISION_CNN_MAX_OUT_TENSOR_DIMS];

    /** output tensor size in bytes */
    uint32_t                outTensorSize;

    /** LDC rectification table path */
    char                    ldcLutFilePath[VISION_CNN_MAX_LINE_LEN];

    /** DL model artifacts folder path */
    char                    dlModelPath[VISION_CNN_MAX_LINE_LEN];

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint8_t                 is_interactive;

    /** output tensor write flag */
    uint8_t                 enOutTensorWrite;

    /** Total frames processed */
    uint32_t                frameCnt;

    /** Input image width in pixels. */
    int32_t                 inputImageWidth;

    /** Input image height in pixels. */
    int32_t                 inputImageHeight;

    /** Input image format: VX_DF_IMAGE_UYVY or VX_DF_IMAGE_NV12 */
    vx_df_image             inputImageFormat;

    /** LDC sub-sampling factor. Ignored if enableLdcNode is false. */
    uint32_t                ldcSsFactor{VISION_LDC_DS_FACTOR};

    /** LDC block width. Ignored if enableLdcNode is false. */
    uint32_t                ldcBlockWidth{VISION_LDC_BLOCK_WIDTH};

    /** LDC block height. Ignored if enableLdcNode is false. */
    uint32_t                ldcBlockHeight{VISION_LDC_BLOCK_HEIGHT};

    /** Pixed padding. Ignored if enableLdcNode is false. */
    uint32_t                ldcPixelPad{VISION_LDC_PIXEL_PAD};

    /** DL image width in pixels. */
    int32_t                 dlImageWidth;

    /** DL image height in pixels. */
    int32_t                 dlImageHeight;

    /** Output image width in pixels. */
    int32_t                 outImageWidth;

    /** Output image height in pixels. */
    int32_t                 outImageHeight;

    /** Number of classed that will be color coded in a post processing node */
    int16_t                 numClasses;

    /** Flag to indicate if LDC node will need to be created. If true, then
     *  the LDC node will be the head node, otherwise the scaler node will
     *  be the head node.
     */
    bool                    enableLdcNode{true};

    /** Input image format */
    uint8_t                 inputFormat;

    /** OpenVX pipeline depth */
    uint8_t                 pipelineDepth;

    /** output time stamp */
    vx_uint64               outTimestamp;

    /** Number of graph params */
    uint8_t                 numGraphParams;

    /** graph parameter tracking */
    VISION_CNN_graphParams  paramDesc[GRAPH_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    VISION_CNN_Queue        freeQ;

    /** Pre-process task queue. */
    VISION_CNN_Queue        preProcQ;

    /** DL task queue. */
    VISION_CNN_Queue        dlInferQ;

    /** Post-process task queue. */
    VISION_CNN_Queue        postProcQ;

    /** Queue for output processing. */
    VISION_CNN_Queue        outputQ;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                    exitInputDataProcess;

    /** Flag to indicate that the output thread should exit. */
    bool                    exitOutputThread;

    /** Flag to control DL process exit. */
    bool                    exitPreprocThread;

    /** Flag to control DL process exit. */
    bool                    exitDlInferrThread;

    /** Flag to control DL process exit. */
    bool                    exitPostprocThread;

    /** User interactive thread. */
    std::thread             userCtrlThread;

    /** Event handler thread. */
    std::thread             evtHdlrThread;

    /** pre-processing thread. */
    std::thread             preProcThread;

    /** DL Inference thread. */
    std::thread             dlInferThread;

    /** post-processing thread. */
    std::thread             postProcThread;

    /** Semaphore for controlling the pre-processing thread. */
    Semaphore              *preProcSem;

    /** Semaphore to synchronize the DL input data and DL threads. */
    Semaphore              *dlDataReadySem;

    /** Semaphore for controlling the post-processing thread. */
    Semaphore              *postProcSem;

    /** Semaphore to synchronize the processing and output threads. */
    Semaphore              *outputCtrlSem;

    /** For graph profiling: graph start time */
    TimePoint               profileStart;

    /** For OpenVX graph profiling: graph end time */
    TimePoint               profileEnd;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                 exportGraph;

    /** Flag to indicate if the performance data should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                 exportPerfStats;

    /** Vilualization thteshold for object detection. */
    float                   detVizThreshold;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                 rtLogEnable;

    /** Base value to be used for any programmed VX events. */
    uint32_t                vxEvtAppValBase{0};

    /** Performance monitoring. */
    app_perf_point_t        visonPerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                    startPerfCapt;

    /** File name to save performance stats. */
    const char             *logFileName{nullptr};
};

/**
 * \brief Initialize app, e.g. create graph, load kernels,
 *        initialize SemSeg nodes, setup pipelines, etc
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_init(VISION_CNN_Context *appCntxt);


/**
 * \brief Reset app's parameters
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void      VISION_CNN_reset(VISION_CNN_Context * appCntxt);

/**
 * \brief Function to initialize Semantic Segmentation nodes
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_init_SS(VISION_CNN_Context *appCntxt);

/**
 * \brief Function to de-initialize vision CNN OpenVX nodes
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_deinit_core(VISION_CNN_Context *appCntxt);

/**
 * \brief Function to set up a graph pipeline
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_setupPipeline(VISION_CNN_Context * appCntxt);


/**
 * \brief Run a graph for the given input
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] inputImage input image
 *
 * \param [in] timestamp input images' time stamp
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_run(VISION_CNN_Context   *appCntxt,
                         const unsigned char  *inputImage,
                         uint64_t              timestamp);


/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void      VISION_CNN_printStats(VISION_CNN_Context * appCntxt);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] fp file to export
 *
 * \param [in] exportAll flag to export all statistics
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_exportStats(VISION_CNN_Context * appCntxt, FILE *fp, bool exportAll);


/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_waitGraph(VISION_CNN_Context * appCntxt);

/**
 * \brief Function to process the events programmed to be handled by the
 *        . Currently VX_EVENT_NODE_COMPLETED event is handled.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] event pointer to event
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_processEvent(VISION_CNN_Context * appCntxt, vx_event_t * event);



/**
 * \brief Function to process input images. This function is non-blocking
 *        and returns success if the pipeline is not full.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \param [in] timestamp timeStamp of input image to be processed
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_process(VISION_CNN_Context     * appCntxt,
                             VISION_CNN_graphParams * gpDesc,
                             uint64_t                 timestamp);

/**
 * \brief Function to returns references to the {input, output} images at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        VISION_CNN_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to VISION_CNN_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in]  appCntxt APP context
 *
 * \param [out] inputImage Input image passed to the graph.
 *
 * \param [out] output Reference to the output object from the graph
 *              corresponding to the 'inputImage'. The reference could
 *              be either a tensor or an image. See following.
 *              - output refere to a tensor
 *
 * \param [out] timestamp Timestamp of inputImage
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_getOutBuff(VISION_CNN_Context   * appCntxt,
                                vx_image             * inputImage,
                                vx_reference         * output,
                                vx_uint64            * timestamp);

/**
 * \brief Function to release buffer in the output queue by moving
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_releaseOutBuff(VISION_CNN_Context * appCntxt);

/**
 * \brief Function to convert the scaler output to RGB
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] vxScalerOut Scaler (MSC) output image
 *
 * \param [out] inputTensorVec A vector of input tensors.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_preProcess(VISION_CNN_Context     * appCntxt,
                                vx_image                 vxScalerOut,
                                VecDlTensorPtr         * inputTensorVec);

/**
 * \brief Function to convert semantic segmentation output to tensor
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] outputTensorVec A vector of output tensors.
 *
 * \param [out] vxOutTensor tensor object created from outputTensorVec
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_createOutTensor(VISION_CNN_Context   * appCntxt,
                                     VecDlTensorPtr       * outputTensorVec,
                                     vx_tensor              vxOutTensor);

/**
 * \brief Function to get and pop free resource from the free input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_popFreeInputDesc(VISION_CNN_Context       *appCntxt,
                                      VISION_CNN_graphParams  **gpDesc);

/**
 * \brief Function to get and pop free resource from the pre-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_popPreprocInputDesc(VISION_CNN_Context       *appCntxt,
                                         VISION_CNN_graphParams  **gpDesc);

/**
 * \brief Function to get and pop free resource from the DL Infer input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_popDlInferInputDesc(VISION_CNN_Context        *appCntxt,
                                         VISION_CNN_graphParams   **gpDesc);


/**
 * \brief Function to get and pop free resource from the post-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_popPostprocInputDesc(VISION_CNN_Context       *appCntxt,
                                          VISION_CNN_graphParams  **gpDesc);

/**
 * \brief Function to get the buffer from the output queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_getOutputDesc(VISION_CNN_Context       *appCntxt,
                                   VISION_CNN_graphParams   *gpDesc);

/**
 * \brief Function to release the buffer from the output queue.
 *        The release output buffer is moved to the input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_vision_cnn
 */
vx_status VISION_CNN_popOutputDesc(VISION_CNN_Context       *appCntxt,
                                   VISION_CNN_graphParams  **gpDesc);

/**
 * \brief Function to add the buffer to the free input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void      VISION_CNN_enqueInputDesc(VISION_CNN_Context      *appCntxt,
                                    VISION_CNN_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the pre-processing input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void      VISION_CNN_enquePreprocInputDesc(VISION_CNN_Context      *appCntxt,
                                           VISION_CNN_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the DL Infer input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void      VISION_CNN_enqueDlInferInputDesc(VISION_CNN_Context       *appCntxt,
                                           VISION_CNN_graphParams   *gpDesc);

/**
 * \brief Function to add the buffer to the post-proc input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void      VISION_CNN_enquePostprocInputDesc(VISION_CNN_Context      *appCntxt,
                                            VISION_CNN_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the output queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void      VISION_CNN_enqueOutputDesc(VISION_CNN_Context      *appCntxt,
                                     VISION_CNN_graphParams  *gpDesc);

/**
 * \brief Launch input data thread, event handler thread and DL Infer thread
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void VISION_CNN_launchProcThreads(VISION_CNN_Context *appCntxt);

/**
 * \brief Handle intercept signal (Ctrl+C) to exit
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void VISION_CNN_intSigHandler(VISION_CNN_Context *appCntxt);

/**
 * \brief Clean up all the resources before exiting
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_vision_cnn
 */
void VISION_CNN_cleanupHdlr(VISION_CNN_Context *appCntxt);

#endif /* _APP_VISION_CNN_H_ */
