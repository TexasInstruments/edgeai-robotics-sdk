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
#ifndef _APP_VISLOC_H_
#define _APP_VISLOC_H_

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
#include <cm_pose_calc_node_cntxt.h>
#include <cm_pose_viz_node_cntxt.h>
#include <cm_profile.h>
#include <cm_pre_process_image.h>
#include <ti_queue.h>

#include <Eigen/Dense>

/**
 * \defgroup group_ticore_visloc CNN based visual localization
 *
 * \brief It performs the ego vehicle localization that estimates an ego vehicle's 6-DOF
 *        (Degree of Freedom) pose from calibrated camera images using a 3D sparse map
 *        created offline. The 3D sparse map consists of a set of key points with (X, Y, Z)
 *        positions and 64-dimensional descriptors. To localize the ego vehicle's pose,
 *        key points are detected with descriptors from the input camera image and
 *        these key points are matched against the key points in the map. The ego vehicle's
 *        pose is estimated using the Perspective-n-Point (PnP) approach. The overall data
 *        flow is descriedb in the figure below: <br>
 *
 *        \image html visloc_demo_block_diagram.svg "CNN based visual localization" width=1000
 *
 *        Key-point descriptor plays critical role in the visual localization. A deep
 *        neural network is used to learn hand computed feature descriptor like KAZE in
 *        a supervised manner. We refer such descriptor as DKAZE. The DAKZE network was used
 *        to create key features and their descriptors for the sparse 3D map and is also
 *        used to detect key feature points with descriptors for every input image in
 *        the localization process.  For more details about the DKAZE network and the
 *        localization process, refer to
 *        <a href="https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-jacinto7/latest/exports/docs/vision_apps/docs/user_guide/group_apps_dl_demos_app_tidl_vl.html">
 *        Vision Apps User Guide</a>. <br>
 *
 * \ingroup  group_ticore_apps
 *
 */

using namespace ti_core_common;
using namespace ti::dl_inferer;

#include <utils/app_init/include/app_init.h>
#include <utils/perf_stats/include/app_perf_stats.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <tivx_utils_graph_perf.h>

#ifdef __cplusplus
}
#endif

/**
 * \brief Default input image width
 * \ingroup group_ticore_visloc
 */
#define VISLOC_DEFAULT_IMAGE_WIDTH        (1280)

/**
 * \brief Default input image height
 * \ingroup group_ticore_visloc
 */
#define VISLOC_DEFAULT_IMAGE_HEIGHT       (720)

/**
 * \brief LDC down sampling factor
 * \ingroup group_ticore_visloc
 */
#define VISLOC_LDC_DS_FACTOR              (2)

/**
 * \brief LDC block width
 * \ingroup group_ticore_visloc
 */
#define VISLOC_LDC_BLOCK_WIDTH            (64)

/**
 * \brief LDC block height
 * \ingroup group_ticore_visloc
 */
#define VISLOC_LDC_BLOCK_HEIGHT           (16)

/**
 * \brief Pixel padding flag in LDC
 * \ingroup group_ticore_visloc
 */
#define VISLOC_LDC_PIXEL_PAD              (1)

/**
 * \brief Output file name to save performance stats
 * \ingroup group_ticore_visloc
 */
#define VISLOC_PERF_OUT_FILE              "app_visloc"

/**
 * \brief Maximum file name length
 * \ingroup group_ticore_visloc
 */
#define VISLOC_MAX_LINE_LEN               (1024U)

/**
 * \brief Number of output tensors from CNN model
 * \ingroup group_ticore_visloc
 */
#define VISLOC_NUM_MODEL_OUTPUT           (2U)

/**
 * \brief Maximum output tensor dimension
 * \ingroup group_ticore_visloc
 */
#define VISLOC_MAX_OUT_TENSOR_DIMS        (4U)

/**
 * \brief Number of graph parameters
 * \ingroup group_ticore_visloc
 */
#define VISLOC_NUM_GRAPH_PARAMS           (7U)

/**
 * \brief Appliation state id - Invalid
 * \ingroup group_ticore_visloc
 */
#define VISLOC_STATE_INVALID              (0U)

/**
 * \brief Appliation state id - Initialized
 * \ingroup group_ticore_visloc
 */
#define VISLOC_STATE_INIT                 (1U)

/**
 * \brief Appliation state id - Shutdown
 * \ingroup group_ticore_visloc
 */
#define VISLOC_STATE_SHUTDOWN             (2U)

/**
 * \brief Graph completion event id
 * \ingroup group_ticore_visloc
 */
#define VISLOC_GRAPH_COMPLETE_EVENT       (0U)

/**
 * \brief Scaler node completion event id
 * \ingroup group_ticore_visloc
 */
#define VISLOC_SCALER_NODE_COMPLETE_EVENT (VISLOC_GRAPH_COMPLETE_EVENT + 1)

/**
 * \brief Event id that notify application exits by user
 * \ingroup group_ticore_visloc
 */
#define VISLOC_USER_EVT_EXIT              (VISLOC_SCALER_NODE_COMPLETE_EVENT + 1)


using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

/**
 * \brief Graph parameters
 * \ingroup group_ticore_visloc
 */
struct VISLOC_graphParams
{
    /** Graph parameter 0 */
    vx_image                vxInputImage;

    /** Graph parameter 1 */
    vx_image                vxRectifiedImage;

    /** Graph parameter 2 */
    vx_image                vxScalerOut;

    /** Graph parameter 3, 4:
     * Output Tensor from DLR output buffer
     */
    vx_tensor               vxOutTensor[2];

    /** Graph parameter 5 */
    vx_matrix               vxPoseMatrix;

    /** Graph parameter 6 */
    vx_image                vxOutputImage;

    /** Input to DL Inference engine. */
    VecDlTensorPtr         *inferInputBuff;

    /** Output from the DL Inference engine. */
    VecDlTensorPtr         *inferOutputBuff;

    /** timestamp - Not a Graph param */
    vx_uint64              *timestamp;
};

using VISLOC_Queue =
     MultiThreadQ<VISLOC_graphParams>;

/**
 * \brief Application context parameters
 * \ingroup group_ticore_visloc
 */
struct VISLOC_Context
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

    /** DL Inference configuration. */
    InfererConfig           dlInferConfig{};

    /** DL Inference context. */
    DLInferer              *dlInferer{nullptr};

    /** Pre-process context. */
    CmPreprocessImage      *preProcObj{nullptr};

    /** Input buffers to the inference. */
    VecDlTensorPtr          inferInputBuff[GRAPH_MAX_PIPELINE_DEPTH];

    /** Output buffers to the inference. */
    VecDlTensorPtr          inferOutputBuff[GRAPH_MAX_PIPELINE_DEPTH];;

    /** output tensor: number of dimemsion */
    int32_t                 outTensorNumDim[VISLOC_NUM_MODEL_OUTPUT];

    /** output tensor: tensor dims */
    vx_size                 outTensorDims[VISLOC_NUM_MODEL_OUTPUT][VISLOC_MAX_OUT_TENSOR_DIMS];

    /** output tensor size in bytes */
    uint32_t                outTensorSize[VISLOC_NUM_MODEL_OUTPUT];

    /** Pose Calculation node context object. */
    CM_PoseCalcNodeCntxt    poseCalcObj{};

    /** Pose Calculation node context object. */
    CM_PoseVizNodeCntxt     poseVizObj{};

    /** Pose calculate create params */
    CM_PoseCalcCreateParams poseCalcCreateParams;

    /** Pose visualize create params */
    CM_PoseVizCreateParams  poseVizCreateParams;

    /** Input image object  */
    vx_image                vxInputImage[GRAPH_MAX_PIPELINE_DEPTH];

    /** Rectified image object  */
    vx_image                vxRectifiedImage[GRAPH_MAX_PIPELINE_DEPTH];

    /** Input timestamp */
    vx_uint64               timestamp[GRAPH_MAX_PIPELINE_DEPTH];

    /** Output Tensors that are crated from dlr output buffers. */
    vx_tensor               vxOutTensor[VISLOC_NUM_MODEL_OUTPUT][GRAPH_MAX_PIPELINE_DEPTH];

    /** Input image corresponding to the output image and pose. */
    vx_image                vxInputDisplayImage;

    /** Output top-dowm image with pose overlaid */
    vx_image                vxOutputImage;

    /** Output pose matrix from visual localization */
    vx_matrix               vxOutputPose;

    /** LDC LUT file path */
    char                    ldcLutFilePath[CM_MAX_FILE_LEN];

    /** DLR modle path */
    char                    dlModelPath[CM_MAX_FILE_LEN];

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint8_t                 is_interactive;


    /** Input image width in pixels. */
    int32_t                 inputImageWidth;

    /** Input image height in pixels. */
    int32_t                 inputImageHeight;

    /** LDC sub-sampling factor. Ignored if enableLdcNode is false. */
    uint32_t                ldcSsFactor;

    /** LDC block width. Ignored if enableLdcNode is false. */
    uint32_t                ldcBlockWidth;

    /** LDC block height. Ignored if enableLdcNode is false. */
    uint32_t                ldcBlockHeight;

    /** Pixed padding. Ignored if enableLdcNode is false. */
    uint32_t                ldcPixelPad;

    /** DL image width in pixels. */
    int32_t                 dlImageWidth;

    /** DL image height in pixels. */
    int32_t                 dlImageHeight;

    /** Output image width in pixels. */
    int32_t                 outImageWidth;

    /** Output image height in pixels. */
    int32_t                 outImageHeight;

    /** Flag to indicate if LDC node will need to be created. If true, then
     *  the LDC node will be the head node, otherwise the scaler node will
     *  be the head node.
     */
    uint8_t                 enableLdcNode;

    /** Input image format */
    uint8_t                 inputFormat;

    /** OpenVX pipeline depth */
    uint8_t                 pipelineDepth;

    /** output time stamp */
    vx_uint64               outTimestamp;

    /** Number of graph params */
    uint8_t                 numGraphParams;

    /** graph parameter tracking */
    VISLOC_graphParams      paramDesc[GRAPH_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    VISLOC_Queue            freeQ;

    /** Pre-process task queue. */
    VISLOC_Queue            preProcQ;

    /** DLR task queue. */
    VISLOC_Queue            dlInferQ;

    /** Visual localization task queue. */
    VISLOC_Queue            visLocQ;

    /** Queue for output processing. */
    VISLOC_Queue            outputQ;

    /** Flag to indicate that the graph processing thread should exit. */
    bool                    exitInputDataProcess;

    /** Flag to indicate that the output thread should exit. */
    bool                    exitOutputThread;

    /** Flag to control DLR process exit. */
    bool                    exitPreprocThread;

    /** Flag to control DLR process exit. */
    bool                    exitDlInferThread;

    /** Flag to control DLR process exit. */
    bool                    exitVisLocThread;

    /** User interactive thread. */
    std::thread             userCtrlThread;

    /** Event handler thread. */
    std::thread             evtHdlrThread;

    /** pre-processing thread. */
    std::thread             preProcThread;

    /** DLR processing thread. */
    std::thread             dlInferThread;

    /** visual localization thread. */
    std::thread             visLocThread;

    /** Semaphore for controlling the pre-processing thread. */
    Semaphore              *preProcSem;

    /** Semaphore to synchronize the DLR iput data and DLR threads. */
    Semaphore              *dlDataReadySem;

    /** Semaphore for controlling the visual localization thread. */
    Semaphore              *visLocSem;

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

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                 rtLogEnable;

    /** Base value to be used for any programmed VX events. */
    uint32_t                vxEvtAppValBase{0};

    /** Performance monitoring. */
    app_perf_point_t        vlPerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                    startPerfCapt;
};

/**
 * \brief Initialize app, e.g. create graph, load kernels,
 *        initialize SemSeg nodes, setup pipelines, etc
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_init(VISLOC_Context *appCntxt);


/**
 * \brief Reset app's parameters
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void      VISLOC_reset(VISLOC_Context * appCntxt);

/**
 * \brief Function to initialize Visual Localization nodes
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_init_VL(VISLOC_Context *appCntxt);

/**
 * \brief Function to de-initialize Visual Localization nodes
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_deinit_VL(VISLOC_Context *appCntxt);

/**
 * \brief Function to set up a graph pipeline
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_setupPipeline(VISLOC_Context * appCntxt);


/**
 * \brief Run a graph for the given input
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] inputImage input image
 *
 * \param [in] timestamp timeStamp of input image to be processed
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_run(VISLOC_Context       *appCntxt,
                     const unsigned char  *inputImage,
                     uint64_t              timestamp);


/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void      VISLOC_printStats(VISLOC_Context * appCntxt);

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
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_exportStats(VISLOC_Context * appCntxt, FILE *fp, bool exportAll);


/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_waitGraph(VISLOC_Context * appCntxt);

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
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_processEvent(VISLOC_Context * appCntxt, vx_event_t * event);



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
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_process(VISLOC_Context     * appCntxt,
                         VISLOC_graphParams * gpDesc,
                         uint64_t             timestamp);

/**
 * \brief Function to returns references to the {input, output} images at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        VISLOC_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to VISLOC_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in]  appCntxt APP context
 *
 * \param [out] inputImage Input image passed to the graph.
 *
 * \param [out] output Reference to the output object from the graph
 *              corresponding to the 'inputImage'.
 *
 * \param [out] outputPose Reference to the output pose matrix from the graph.
 *
 * \param [out] timestamp Timestamp of inputImage
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_getOutBuff(VISLOC_Context   * appCntxt,
                            vx_image         * inputImage,
                            vx_image         * output,
                            vx_matrix        * outputPose,
                            vx_uint64        * timestamp);

/**
 * \brief Function to release buffer in the output queue by moving
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_releaseOutBuff(VISLOC_Context * appCntxt);

/**
 * \brief Function to convert the scaler output to RGB
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] vxScalerOut Scaler (MSC) output image
 *
 * \param [out] inputTensorVec A vector of input tensors
 *
 * \return VX_SUCCESS on success
 *
 */
vx_status VISLOC_preProcess(VISLOC_Context     * appCntxt,
                            vx_image             vxScalerOut,
                            VecDlTensorPtr     * inputTensorVec);

/**
 * \brief Function to convert Score output to tensor
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] outputTensorVec A vector of output tensors
 *
 * \param [out] vxOutTensor tensor object created from dlrOutputBuff
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_createScoreOutTensor(VISLOC_Context     * appCntxt,
                                      VecDlTensorPtr     & outputTensorVec,
                                      vx_tensor            vxOutTensor);


/**
 * \brief Function to convert Descriptor output to tensor
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] outputTensorVec A vector of output tensors
 *
 * \param [out] vxOutTensor tensor object created from dlrOutputBuff
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_createDescOutTensor(VISLOC_Context     * appCntxt,
                                     VecDlTensorPtr     & outputTensorVec,
                                     vx_tensor            vxOutTensor);


/**
 * \brief Function to get and pop free resource from the free input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_popFreeInputDesc(VISLOC_Context       *appCntxt,
                                  VISLOC_graphParams  **gpDesc);


/**
 * \brief Function to get and pop free resource from the pre-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_popPreprocInputDesc(VISLOC_Context       *appCntxt,
                                     VISLOC_graphParams  **gpDesc);


/**
 * \brief Function to get and pop free resource from the DLR input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_popDlInferInputDesc(VISLOC_Context       *appCntxt,
                                     VISLOC_graphParams  **gpDesc);


/**
 * \brief Function to get and pop free resource from the visual localization input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_popVisLocInputDesc(VISLOC_Context       *appCntxt,
                                    VISLOC_graphParams  **gpDesc);

/**
 * \brief Function to get free resource from the visual localization input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_getVisLocInputDesc(VISLOC_Context       *appCntxt,
                                    VISLOC_graphParams   **gpDesc);


/**
 * \brief Function to get the buffer from the output queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_getOutputDesc(VISLOC_Context       *appCntxt,
                               VISLOC_graphParams   *gpDesc);


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
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_popOutputDesc(VISLOC_Context       *appCntxt,
                               VISLOC_graphParams  **gpDesc);

/**
 * \brief Function to add the buffer to the free input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \ingroup group_ticore_visloc
 */
void      VISLOC_enqueInputDesc(VISLOC_Context      *appCntxt,
                                VISLOC_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the pre-processing input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void      VISLOC_enquePreprocInputDesc(VISLOC_Context      *appCntxt,
                                       VISLOC_graphParams  *gpDesc);


/**
 * \brief Function to add the buffer to the DLR input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void      VISLOC_enqueDlInferInputDesc(VISLOC_Context      *appCntxt,
                                       VISLOC_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the visual localization input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void      VISLOC_enqueVisLocInputDesc(VISLOC_Context      *appCntxt,
                                      VISLOC_graphParams  *gpDesc);


/**
 * \brief Function to add the buffer to the output queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void      VISLOC_enqueOutputDesc(VISLOC_Context      *appCntxt,
                                 VISLOC_graphParams  *gpDesc);



/**
 * \brief Launch input data thread, event handler thread and DLR thread
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void VISLOC_launchProcThreads(VISLOC_Context *appCntxt);

/**
 * \brief Handle intercept signal (Ctrl+C) to exit
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void VISLOC_intSigHandler(VISLOC_Context *appCntxt);

/**
 * \brief Clean up all the resources before exiting
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_visloc
 */
void VISLOC_cleanupHdlr(VISLOC_Context *appCntxt);


/**
 * \brief Function to extract pose data from an openVX matrix object.
 *
 * \param [out] outPose Output position data
 *
 * \param [out] outQuaternion Output quaternion data
 *
 * \param [in]  pose OpenVX Matrix object
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_visloc
 */
vx_status VISLOC_extractPoseData(double          *outPose,
                                 double          *outQuaternion,
                                 const vx_matrix  pose);

#endif /* _APP_VISLOC_H_ */
