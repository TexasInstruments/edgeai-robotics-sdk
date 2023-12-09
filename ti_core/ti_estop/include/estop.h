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
#ifndef _APP_ESTOP_H_
#define _APP_ESTOP_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <TI/tivx.h>
#include <TI/tivx_debug.h>
#include <TI/tivx_stereo.h>

#include <sde_ldc.h>
#include <sde_singlelayer.h>
#include <sde_multilayer.h>
#include <ss_sde_detection.h>

#include <cm_scaler_node_cntxt.h>
#include <cm_preproc_node_cntxt.h>
#include <cm_profile.h>
#include <cm_pre_process_image.h>
#include <cm_post_process_image.h>
#include <ti_queue.h>

/**
 * \defgroup group_ticore_estop 3D obstacle detection (e-Stop) processing
 *
 * \brief It performs the 3D obstacle detection (e-Stop) using the disparity map from
 *        stereo depth engine and the semantic segmentation map from the deep-learning
 *        network. As shown in the figure below, it consists of the following three
 *        main processes:
 *
 *        \image html estop_demo_block_diagram.svg "3D obstacle detection (e-Stop)" width = 1000
 *
 *        1. Stereo Vision Processing <br>
 *           This is the same process described in @ref group_ticore_sde without
 *           the point-cloud generation process. The output disparity map is fed to
 *           the 3D obstacle detection process as an input.
 *
 *        2. Semantic Segmentation Processing <br>
 *           This is the same semantic segmentation process described in
 *           @ref group_ticore_vision_cnn. The output tensor of the CNN network is
 *           fed to the 3D obstacle detection process as another input.
 *
 *        3. 3D Obstacle Detection Processing <br>
 *           This process outputs the 3D bounding box coordinates of the detected obstacles.
 *           First, it creates 3D point cloud using the disparity map and the camera parameters.
 *           Note that it maps only pixels that belongs to particular classes, e.g., car,
 *           pedestrian, bicycle, rider, etc. into the 3D space. Then it projects the 3D point
 *           cloud on a 2D occupancy grid map. Finally it detects individual obstacles by
 *           grouping closely-located occupied cells with an identical class using
 *           a "connected component analysis" algorithm.
 *
 *
 * \ingroup  group_ticore_apps
 *
 */

using namespace std;
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
 * \brief Output file name to save performance stats
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_PERF_OUT_FILE     "app_estop"

/**
 * \brief Maximum file name length
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_MAX_LINE_LEN         (1024U)

/**
 * \brief Maximum input image width
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_MAX_IMAGE_WIDTH      (2048)

/**
 * \brief Maximum input image height
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_MAX_IMAGE_HEIGHT     (1024)

/**
 * \brief Number of graph parameters
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_NUM_GRAPH_PARAMS      (7U)

/**
 * \brief Graph completion event id
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_GRAPH_COMPLETE_EVENT        (0U)

/**
 * \brief Scaler node completion event id
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_SCALER_NODE_COMPLETE_EVENT  (ESTOP_APP_GRAPH_COMPLETE_EVENT + 1)

/**
 * \brief Event id that notifiy CNN ouptut tensor is available
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_CNN_OUT_AVAIL_EVENT         (ESTOP_APP_SCALER_NODE_COMPLETE_EVENT + 1)

/**
 * \brief Event id that notify application exits by user
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_USER_EVT_EXIT               (ESTOP_APP_CNN_OUT_AVAIL_EVENT + 1)

/**
 * \brief Appliation state id - Invalid
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_STATE_INVALID    (0U)

/**
 * \brief Appliation state id - Initialized
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_STATE_INIT       (1U)

/**
 * \brief Appliation state id - Shutdown
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_STATE_SHUTDOWN   (2U)

/**
 * \brief OG map grid value in field of view
 * \ingroup group_ticore_estop
 */
#define FOV_GRID_VALUE             (-1)

/**
 * \brief OG map grid value out of field of view
 * \ingroup group_ticore_estop
 */
#define NON_FOV_GRID_VALUE          (0)

/**
 * \brief OG map grid value where emergency stop is needed
 * \ingroup group_ticore_estop
 */
#define ESTOP_AREA_GRID_VALUE      (98)

/**
 * \brief Maximum tensor dimension
 * \ingroup group_ticore_estop
 */
#define ESTOP_APP_MAX_OUT_TENSOR_DIMS   (4U)

using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

/**
 * \brief Graph parameters
 * \ingroup group_ticore_estop
 */
struct ESTOP_APP_graphParams
{
    /** Graph parameter 0: left input image */
    vx_image                vxInputLeftImage;

    /** Graph parameter 1: right input image */
    vx_image                vxInputRightImage;

    /** Graph parameter 2: rectified right (base) image */
    vx_image                vxRightRectImage;

    /** Graph parameter 3: 16-bit raw SDE output */
    vx_image                vxSde16BitOutput;

    /** Graph parameter 3: output disparity in multi-layer SDE */
    vx_image                vxMergeDisparityL0;

    /** Graph parameter 3: median filtered output disparity in multi-layer SDE */
    vx_image                vxMedianFilteredDisparity;

    /** Graph parameter 4 */
    vx_user_data_object     vx3DBoundBox;

    /** Graph parameter 5 */
    vx_image                vxScalerOut;

    /** Graph Parameter 6 */
    vx_tensor               vxOutTensor;

    /** Input to DL Inference engine. */
    VecDlTensorPtr         *inferInputBuff;

    /** Output from the DL Inference engine. */
    VecDlTensorPtr         *inferOutputBuff;

    /** Timestamp - Not a Graph param */
    vx_uint64             * timestamp;
};

using ESTOP_APP_Queue =
     MultiThreadQ<ESTOP_APP_graphParams>;

/**
 * \brief Application context parameters
 * \ingroup group_ticore_estop
 */
struct ESTOP_APP_Context
{
    /** Application state */
    uint32_t                                state;

    /** OpenVX references */
    vx_context                              vxContext;

    /** OpenVX graph */
    vx_graph                                vxGraph;

    /** Scaler node context object. */
    CM_ScalerNodeCntxt                      scalerObj{};

    /** Pre-process configuration. */
    PreprocessImageConfig                   preProcCfg;

    /** Post-processing configuration.*/
    PostprocessImageConfig                  postProcCfg;

    /** DL Inference configuration. */
    InfererConfig                           dlInferConfig{};

    /** DL Inference context. */
    DLInferer                              *dlInferer{nullptr};

    /** Pre-process context. */
    CmPreprocessImage                      *preProcObj{nullptr};

    /** Post-process context. */
    CmPostprocessImage                     *postProcObj{nullptr};

    /** Input buffers to the inference. */
    VecDlTensorPtr                          inferInputBuff[GRAPH_MAX_PIPELINE_DEPTH];

    /** Output buffers to the inference. */
    VecDlTensorPtr                          inferOutputBuff[GRAPH_MAX_PIPELINE_DEPTH];

    /** output tensor: number of dimemsion */
    int32_t                                 outTensorNumDim;

    /** output tensor: tensor dims */
    vx_size                                 outTensorDims[ESTOP_APP_MAX_OUT_TENSOR_DIMS];

    /** output tensor size in bytes */
    uint32_t                                outTensorSize;

    /** Input left image object */
    vx_image                                vxInputLeftImage[GRAPH_MAX_PIPELINE_DEPTH];

    /** Input right image object */
    vx_image                                vxInputRightImage[GRAPH_MAX_PIPELINE_DEPTH];

    /** Input rectified left image object */
    vx_image                                vxLeftRectImage;

    /** Input rectified right image object */
    vx_image                                vxRightRectImage[GRAPH_MAX_PIPELINE_DEPTH];

    /* Output Tensor after post-processing. */
    vx_tensor                               vxOutTensor[GRAPH_MAX_PIPELINE_DEPTH];

    /** Raw disparity map object */
    vx_image                                vxSde16BitOutput[GRAPH_MAX_PIPELINE_DEPTH];

    /** Merged disparity map object at base layer */
    vx_image                                vxMergeDisparityL0[GRAPH_MAX_PIPELINE_DEPTH];

    /** Median filtered disparity map object at base layer */
    vx_image                                vxMedianFilteredDisparity[GRAPH_MAX_PIPELINE_DEPTH];

    /** Ouput point cloud object */
    vx_user_data_object                     vx3DBoundBox[GRAPH_MAX_PIPELINE_DEPTH];

    /** Input timestamp */
    vx_uint64                               timestamp[GRAPH_MAX_PIPELINE_DEPTH];

    /** Object that right image is passed to after processing graph */
    vx_image                                vxDispRightImage;

    /** Object that output semantic segmentation image is passed to after processing graph */
    vx_image                                vxDispSSImage;

    /** Object that output 3D bounding box is passed to after processing graph */
    vx_user_data_object                     vxDisp3DBoundBox;

    /** Object that output disparity map is passed to after processing graph */
    vx_image                                vxDisparity16;

    /** output time stamp */
    vx_uint64                               outTimestamp;

    /** 3D bounding box memory size */
    int32_t                                 bbSize;

    /** Input image width */
    int16_t                                 width;

    /** Input image height */
    int16_t                                 height;

    /** Horizontal pixel offset from which the image is processed */
    int16_t                                 hImgOfst;

    /** Vertical pixel offset from which the image is processed */
    int16_t                                 vImgOfst;

    /** Tensor width */
    int16_t                                 tensor_width;

    /** Tensor height */
    int16_t                                 tensor_height;

    /** Number of classed that will be color coded in a post processing node */
    int16_t                                 numClasses;

    /** Input image format */
    uint8_t                                 inputFormat;

    /** OpenVX pipeline depth */
    uint8_t                                 pipelineDepth;

    /** SDE algo type (0: Single-layer SDE, 1: Multi-layer SDE) */
    uint8_t                                 sdeAlgoType;

    /** the number of layers (2, 3) for multi-layer SDE */
    uint8_t                                 numLayers;

    /** whether enable median filter in multi-layer SDE */
    uint8_t                                 ppMedianFilterEnable;

    /** disparity confidence threshold */
    uint8_t                                 confidence_threshold;

    /** SDE configuration parameters */
    tivx_dmpac_sde_params_t                 sde_params;

    /** Camera parameters: horizontal distortion center */
    float                                   distCenterX;

    /** Camera parameters: vetical distortion center */
    float                                   distCenterY;

    /** Camera parameters: focal length */
    float                                   focalLength;

    /** Camera parameters: roll angle */
    float                                   camRoll;

    /** Camera parameters: pitch angle */
    float                                   camPitch;

    /** Camera parameters: yaw angle */
    float                                   camYaw;

    /** Camera parameters: mounting height */
    float                                   camHeight;

    /** Stereo parameters: baseline between left and right cameras */
    float                                   baseline;

    /** OG map parameters: horizontal grid size */
    int32_t                                 xGridSize;

    /** OG map parameters: vertical grid size */
    int32_t                                 yGridSize;

    /** OG map parameters: Min X range in mm */
    int32_t                                 xMinRange;

    /** OG map parameters: Min X range in mm */
    int32_t                                 xMaxRange;

    /** OG map parameters: Min Y range in mm */
    int32_t                                 yMinRange;

    /** OG map parameters: Min Y range in mm */
    int32_t                                 yMaxRange;

    /** OG map parameters: Number of grid in X dimension */
    int32_t                                 xGridNum;

    /** OG map parameters: Number of grid in Y dimension */
    int32_t                                 yGridNum;

    /** Pixel count threshold of grid for occupied/non-occupied decision */
    int16_t                                 thCnt;

    /** Pixel count threshold of object for occupied/non-occupied decision */
    int16_t                                 thObjCnt;

    /** Maximum number of objects to be detected */
    int16_t                                 maxNumObject;

    /** Number of neighboring grids to check for connected component analysis */
    int16_t                                 cNeighNum;

    /** Enable flag of spatial object merge */
    uint8_t                                 enableSpatialObjMerge;

    /** Enable flag of temporal object merge */
    uint8_t                                 enableTemporalObjMerge;

    /** Enable flag of temporal object smoothing */
    uint8_t                                 enableTemporalObjSmoothing;

    /** Method to compute distance between objects
     *  0: distance between centers
     *  1: distacne between corners
     */
    uint8_t                                 objectDistanceMode;

    /** graph parameter tracking */
    ESTOP_APP_graphParams                   paramDesc[GRAPH_MAX_PIPELINE_DEPTH];

    /** A queue for holding free descriptors. */
    ESTOP_APP_Queue                         freeQ;

    /** Pre-process task queue. */
    ESTOP_APP_Queue                         preProcQ;

    /** DLR task queue. */
    ESTOP_APP_Queue                         dlrQ;

    /** Post-process task queue. */
    ESTOP_APP_Queue                         postProcQ;

    /** Queue for output processing. */
    ESTOP_APP_Queue                         outputQ;

    /** TIDL network file name */
    char                                    dlModelPath[ESTOP_APP_MAX_LINE_LEN];

    /** Left rectification LUT file name */
    char                                    left_LUT_file_name[ESTOP_APP_MAX_LINE_LEN];

    /** Right rectification LUT file name */
    char                                    right_LUT_file_name[ESTOP_APP_MAX_LINE_LEN];

    /** Application interactive status, 0=non-interactive, 1=interactive. */
    uint8_t                                 is_interactive;

    /** LDC applib create params */
    SDELDC_createParams               sdeLdcCreateParams;

    /** LDC applib handler */
    SDELDC_Handle                     sdeLdcHdl;

    /** Single-layer SDE applib create params */
    SL_SDE_createParams               slSdeCreateParams;

    /** Single-layer SDE applib handler */
    SL_SDE_Handle                     slSdeHdl;

    /** Multi-layer SDE applib create params */
    ML_SDE_createParams               mlSdeCreateParams;

    /** Multi-layer SDE applib handler */
    ML_SDE_Handle                     mlSdeHdl;

    /** OG Map based detector applib create params */
    SS_DETECT_createParams           ssDetectCreateParams;

    /** OG Map based detector applib handler */
    SS_DETECT_Handle                 ssDetectHdl;

    /** Number of graph params */
    uint8_t                                 numGraphParams;

    /** Number of output tensor */
    uint8_t                                 numOutTensors;

    /** Render periodicity in milli-sec. */
    uint64_t                                renderPeriod;


    /** Flag to indicate that the input data processing thread should exit. */
    bool                                    exitInputDataProcess;

    /** Flag to indicate that the output thread should exit. */
    bool                                    exitOutputThread;

    /** Flag to control DL process exit. */
    bool                                    exitPreprocThread;

    /** Flag to control DL process exit. */
    bool                                    exitDlInferThread;

    /** Flag to control DL process exit. */
    bool                                    exitPostprocThread;

    /** Input data thread. */
    std::thread                             userCtrlThread;

    /** Event handler thread. */
    std::thread                             evtHdlrThread;

    /** pre-processing thread. */
    std::thread                             preProcThread;

    /** DL Inference thread */
    std::thread                             dlInferThread;

    /** post-processing thread. */
    std::thread                             postProcThread;

    /** Semaphore for controlling the pre-processing thread. */
    Semaphore                              *preProcSem;

    /** Semaphore to synchronize the DL input data and DLR threads. */
    Semaphore                              *dlDataReadySem;

    /** Semaphore for controlling the post-processing thread. */
    Semaphore                              *postProcSem;

    /** Semaphore to synchronize the output and display threads. */
    Semaphore                              *outputCtrlSem;

    /** For graph profiling: graph start time */
    TimePoint                               profileStart;

    /** For OpenVX graph profiling: graph end time */
    TimePoint                               profileEnd;

    /** Flag to indicate if the graph should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                                 exportGraph;

    /** Flag to indicate if the performance data should be exported
     * 0 - disable
     * 1 - enable
     */
    uint8_t                                 exportPerfStats;

    /** Real-time logging enable.
     * 0 - disable
     * 1 - enable
     */
    uint8_t                                 rtLogEnable;

    /** Base value to be used for any programmed VX events. */
    uint32_t                                vxEvtAppValBase{0};

    /** Performance monitoring. */
    app_perf_point_t                        estopPerf;

    /** Flag to track if the performance counter has been initialized. */
    bool                                    startPerfCapt;

};


/**
 * \brief Set the create parameters for LDC, SDE, SemSeg and detection
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void      ESTOP_APP_setAllParams(ESTOP_APP_Context *appCntxt);

/**
 * \brief Initialize app, e.g. create graph, load kernels,
 *        initialize LDC node, SDE node, SemSeg node, and detection node,
 *        setup pipeline, etc.
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_init(ESTOP_APP_Context *appCntxt);

/**
 * \brief Reset app's parameters
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
void      ESTOP_APP_reset(ESTOP_APP_Context * appCntxt);

/**
 * \brief Launch input data thread and event handler thread.
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void      ESTOP_APP_launchProcThreads(ESTOP_APP_Context *appCntxt);

/**
 * \brief Handle intercept signal (Ctrl+C) to exit
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void      ESTOP_APP_intSigHandler(ESTOP_APP_Context *appCntxt);

/**
 * \brief Clean up all the resources before exiting
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void      ESTOP_APP_cleanupHdlr(ESTOP_APP_Context *appCntxt);


/**
 * \brief Initialize camera info
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] width image width

 * \param [in] height image height
 *
 * \param [in] f camera focal length

 * \param [in] dx camera horizontal distortion center
 *
 * \param [in] dy camera vertical distortion center
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_init_camInfo(ESTOP_APP_Context *appCntxt,
                                 uint32_t width,
                                 uint32_t height,
                                 double   f,
                                 double   dx,
                                 double   dy);

/**
 * \brief Run a graph for the given input
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] inputLeftImage input left image
 *
 * \param [in] inputRightImage input right image
 *
 * \param [in] timestamp input images' time stamp
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_run(ESTOP_APP_Context *appCntxt,
                        const vx_uint8 * inputLeftImage,
                        const vx_uint8 * inputRightImage,
                        vx_uint64 timestamp);

/**
 * \brief Function to initialize LDC nodes and graphs
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status   ESTOP_APP_init_LDC(ESTOP_APP_Context *appCntxt);

/**
 * \brief Function to initialize SDE nodes and graphs
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status   ESTOP_APP_init_SDE(ESTOP_APP_Context *appCntxt);

/**
 * \brief Function to initialize Semantic Segmentation nodes
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_init_SS(ESTOP_APP_Context *appCntxt);


/**
 * \brief Function to de-initialize Semantic Segmentation nodes
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_deinit_SS(ESTOP_APP_Context *appCntxt);


/**
 * \brief Function to initialize Detection nodes and graphs
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_init_SS_Detection(ESTOP_APP_Context *appCntxt);

/**
 * \brief Main function to set up a whole graph pipeline, which calls
 *        either ESTOP_APP_setupPipeline_SL or ESTOP_APP_setupPipeline_ML
 *        depending on SDE type
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_setupPipeline(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to set up a whole graph pipeline when a single-layer SDE is used
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_setupPipeline_SL(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to set up a whole graph pipeline when a multi-layer SDE is used
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_setupPipeline_ML(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to print the performance statistics to stdout.
 *
 * \param [in] appCntxt APP context
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void      ESTOP_APP_printStats(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to export the performance statistics to a file.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] fp file to export
 *
 * \param [in] exportAll flag to export all statistics
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_exportStats(ESTOP_APP_Context * appCntxt, FILE *fp, bool exportAll);

/**
 * \brief Function to wait for the pending graph execution completions.
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_waitGraph(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to process input images. This function is non-blocking
 *        and returns success if the pipeline is not full.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_process(ESTOP_APP_Context * appCntxt, ESTOP_APP_graphParams * gpDesc);

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
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_CNN_preProcess(ESTOP_APP_Context   *appCntxt,
                                   vx_image             vxScalerOut,
                                   VecDlTensorPtr      *inputTensorVec);

/**
 * \brief Function to convert semantic sementation output to tensor
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] outputTensorVec A vector of output tensors. The vector has only
 *             one entry and it carries sematic segmentation class information.
 *
 * \param [out] vxOutTensor tensor object created from outputTensorVec
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_createOutTensor(ESTOP_APP_Context  *appCntxt,
                                    VecDlTensorPtr     *outputTensorVec,
                                    vx_tensor           vxOutTensor);

/**
 * \brief Function to process the events programmed to be handled by the
 *        . Currently only VX_EVENT_GRAPH_COMPLETED event is handled.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] event pointer to event
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_processEvent(ESTOP_APP_Context * appCntxt, vx_event_t * event);

/**
 * \brief Function to returns references to the input and output at the
 *        head of the output queue. The data is not popped out of the queue.
 *        If the caller calls this functions repeatedly without calling
 *        ESTOP_APP_releaseOutBuff() in between, then the same output
 *        is returned.
 *
 *        Once the graph execution is complete, the output buffer is stored in
 *        a queue for the use by the calling application. The caller can use
 *        this API to get a reference to the input/output image pair for post
 *        processing. One the caller is done with the input/output pair, a call
 *        must be made to ESTOP_APP_releaseOutBuff()for releasing the
 *        buffered input/output pair. Failing to do will result in subsequent
 *        graph execution failures due to non-availability of the output
 *        buffers.
 *
 * \param [in] appCntxt APP context
 *
 * \param [out] rightRectImage right rectified image passed from a graph
 *
 * \param [out] ssOutput output color-coded semantic segmentation map passed from a graph
 *
 * \param [out] outTensor output semantic segmentation tensor passed from a graph
 *
 * \param [out] obsBB output obstacle bounding boxes passed from a graph
*
 * \param [out] disparity16 output raw disparity map passed from a graph
 *
 * \param [out] timestamp time stamp passed from a graph, which should be identical
 *                        to input time stamp
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_getOutBuff(ESTOP_APP_Context *appCntxt,
                               vx_image *rightRectImage,
                               vx_image *ssOutput,
                               vx_tensor *outTensor,
                               vx_user_data_object *obsBB,
                               vx_image *disparity16,
                               vx_uint64 *timestamp);

/**
 * \brief Function to release buffer in the output queue by moving
 *        the buffer to the free queue
 *
 * \param [in] appCntxt APP context
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_releaseOutBuff(ESTOP_APP_Context * appCntxt);

/**
 * \brief Function to get and pop free resource from the free input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_popFreeInputDesc(ESTOP_APP_Context       *appCntxt,
                                     ESTOP_APP_graphParams  **gpDesc);

/**
 * \brief Function to get and pop free resource from the pre-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_popPreprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                        ESTOP_APP_graphParams  **gpDesc);

/**
 * \brief Function to get and pop free resource from the DLR input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_popDlInferInputDesc(ESTOP_APP_Context      *appCntxt,
                                        ESTOP_APP_graphParams **gpDesc);

/**
 * \brief Function to get and pop free resource from the post-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_popPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc);
                                         /**
 * \brief Function to get free resource from the post-processing input queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_getPostprocInputDesc(ESTOP_APP_Context       *appCntxt,
                                         ESTOP_APP_graphParams  **gpDesc);

/**
 * \brief Function to get the buffer from the output queue
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_getOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams   *gpDesc);

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
 * \ingroup group_ticore_estop
 */
vx_status ESTOP_APP_popOutputDesc(ESTOP_APP_Context       *appCntxt,
                                  ESTOP_APP_graphParams  **gpDesc);

/**
 * \brief Function to add the buffer to the free input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void ESTOP_APP_enqueInputDesc(ESTOP_APP_Context      *appCntxt,
                              ESTOP_APP_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the pre-processing input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void ESTOP_APP_enquePreprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                     ESTOP_APP_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the DLR input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void ESTOP_APP_enqueDlInferInputDesc(ESTOP_APP_Context     *appCntxt,
                                     ESTOP_APP_graphParams *gpDesc);

/**
 * \brief Function to add the buffer to the post-proce input queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void ESTOP_APP_enquePostprocInputDesc(ESTOP_APP_Context      *appCntxt,
                                      ESTOP_APP_graphParams  *gpDesc);

/**
 * \brief Function to add the buffer to the output queue.
 *
 * \param [in] appCntxt APP context
 *
 * \param [in] gpDesc pointer to graph parameters
 *
 * \return
 *
 * \ingroup group_ticore_estop
 */
void ESTOP_APP_enqueOutputDesc(ESTOP_APP_Context      *appCntxt,
                               ESTOP_APP_graphParams  *gpDesc);


#endif /* _APP_ESTOP_H_ */
