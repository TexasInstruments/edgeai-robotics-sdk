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

#include <cv_bridge/cv_bridge.h>

#include <sde_node.h>
#include <sde.h>

#include <cm_common.h>

using namespace common_msgs;

#define PC_POINT_SIZE                (16)

SDEAppNode::SDEAppNode(const rclcpp::NodeOptions   &options,
                       const std::string           &name):
    Node(name, options)
{
    std::string cfgFile;
    std::string leftLUTFile;
    std::string rightLUTFile;
    vx_status   vxStatus = VX_SUCCESS;

    // Create initialization control semaphore
    initCtrlSem = new Semaphore(0);

    // Create App context
    m_cntxt = new SDEAPP_Context();
    if (m_cntxt == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "new SDEAPP_Context() failed.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == VX_SUCCESS)
    {
        // camera parameter should be initialized
        m_cntxt->state = SDEAPP_STATE_INIT;

        // launch the subscriber thread
        auto subThread = std::thread([this]{subscriberCamInfoThread();});
        subThread.detach();

        auto procThread = std::thread([this]{inputProcessThread();});
        procThread.detach();
    }

    // register a callback to run at rclcpp::shutdown()
    rclcpp::on_shutdown(std::bind(&SDEAppNode::onShutdown, this));
}

void SDEAppNode::process()
{
    std::string leftTopicName;
    std::string rightTopicName;
    bool        status;

    // Query the topicname to subscribe to
    status = get_parameter("left_input_topic", leftTopicName);
    if (status == true)
    {
        status = get_parameter("right_input_topic", rightTopicName);
    }

    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'input_topic_name' not found.");
        exit(-1);
    }

    m_leftImageSub = new ImgSub(this, leftTopicName);
    m_rightImageSub = new ImgSub(this, rightTopicName);

    m_sync = new TimeSync(*m_leftImageSub, *m_rightImageSub, 10);

    if (m_sync == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "new TimeSync() failed.");
        exit(-1);
    }

    m_conObj = m_sync->registerCallback(&SDEAppNode::imgCb, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to the topic: %s", leftTopicName.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to the topic: %s", rightTopicName.c_str());
}

void SDEAppNode::inputProcessThread()
{
    // wait until camera params are set
    if (initCtrlSem)
    {
        initCtrlSem->wait();
    }

    /* Done with getting the configuration information so de-register. */
    m_camInfoConObj.disconnect();

    // Initialize the App context
    auto vxStatus = this->init();

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Application Initialization failed.");
        exit(-1);
    }

    m_imgWidth  = m_cntxt->width;
    m_imgHeight = m_cntxt->height;

    // launch the publisher threads
    auto pubThread = std::thread([this]{publisherThread();});
    pubThread.detach();

    // process input frames
    process();
}

void SDEAppNode::subscriberCamInfoThread()
{
    std::string camInfoTopicName;
    bool        status;

    status = get_parameter("camera_info_topic", camInfoTopicName);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'camera_info_topic' not found.");
        exit(-1);
    }

    m_sub = new CamInfoSub(this, camInfoTopicName);
    if (m_sub == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "new CamInfoSub() failed.");
        exit(-1);
    }

    m_camInfoConObj = m_sub->registerCallback(&SDEAppNode::camInfoCb, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to the topic: %s", camInfoTopicName.c_str());
}

void SDEAppNode::publisherThread()
{
    std::string rawDispTopic;
    std::string pcTopic;
    bool        status;
    uint32_t    offset;
    int8_t      i;

    // Query the topic name to publish the output image.
    status = get_parameter("disparity_topic", rawDispTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'disparity_topic' not found.");
        exit(-1);
    }

    status = get_parameter("point_cloud_topic", pcTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'point_cloud_topic' not found.");
        exit(-1);
    }

    /* Disparity message */
    m_disparitySize = m_imgWidth * m_imgHeight;

    m_disparityPubData.data.assign(m_disparitySize, 0);
    m_disparityPubData.width           = m_imgWidth;
    m_disparityPubData.height          = m_imgHeight;
    m_disparityPubData.min_disp        = 0;
    if (m_cntxt->sde_params.disparity_max == 0)
    {
        m_disparityPubData.max_disp = 63;
    } else if (m_cntxt->sde_params.disparity_max == 1)
    {
        m_disparityPubData.max_disp = 127;
    } else
    {
        m_disparityPubData.max_disp = 191;
    }
    m_disparityPubData.step            = m_imgWidth * 2;
    m_disparityPubData.header.frame_id = "map";

    // Create the publisher for the disparity output
    m_disparityPub = this->create_publisher<Disparity>(rawDispTopic, 1);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", rawDispTopic.c_str());

    /* Point cloud message */
    // Pre-allocate vectors used for publishing out point cloud data
    // XYZRGB is total 4*3 + 1*4 = 16 bytes
    if (m_cntxt->enablePC)
    {
        m_pcSize = m_imgWidth * m_imgHeight * PC_POINT_SIZE;
        m_pcPubData.data.assign(m_pcSize, 0);

        m_pcPubData.header.frame_id = "PointCloud";
        m_pcPubData.height          = 1;
        m_pcPubData.width           = 0;  // should be updated for every PC frame
        m_pcPubData.fields.resize(4);

        m_pcPubData.fields[0].name = "x";
        m_pcPubData.fields[1].name = "y";
        m_pcPubData.fields[2].name = "z";
        m_pcPubData.fields[3].name = "rgb";

        offset = 0;
        for(i = 0; i < 3; i++)
        {
            m_pcPubData.fields[i].offset   = offset;
            m_pcPubData.fields[i].datatype = 7; // float32
            m_pcPubData.fields[i].count    = 1;
            offset                        += 4;
        }

        for(i = 3; i < 4; i++)
        {
            m_pcPubData.fields[i].offset   = offset;
            m_pcPubData.fields[i].datatype = 6; // unit32
            m_pcPubData.fields[i].count    = 1;

            offset                        += 4;
        }

        m_pcPubData.is_bigendian    = false;
        m_pcPubData.point_step      = offset;  // should be PC_POINT_SIZE
        m_pcPubData.row_step        = 0;       // should be updated for every PC frame
        m_pcPubData.is_dense        = 1;

        // Create the publisher for the PC data
        m_pcPub = this->create_publisher<PointCloud2>(pcTopic, 1);

        RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", pcTopic.c_str());
    }

    // Event handler
    graphCompleteEvtHdlr();
}

void SDEAppNode::camInfoCb(const CameraInfo::ConstSharedPtr& cam_info)
{
    SDEAPP_init_camInfo(m_cntxt,
                        cam_info->width, cam_info->height,
                        cam_info->p[0], cam_info->p[2], cam_info->p[6]);

    if (initCtrlSem)
    {
        initCtrlSem->notify();
    }
}

void SDEAppNode::imgCb(const Image::ConstSharedPtr& left_image_ptr,
                       const Image::ConstSharedPtr& right_image_ptr)
{
    if (m_cntxt->state == SDEAPP_STATE_INIT)
    {
        uint64_t nanoSec = right_image_ptr->header.stamp.sec * 1e9 +
                           right_image_ptr->header.stamp.nanosec;

        SDEAPP_run(m_cntxt,
                   left_image_ptr->data.data(),
                   right_image_ptr->data.data(),
                   nanoSec);
    }
    else if (m_cntxt->state == SDEAPP_STATE_INVALID)
    {
        m_conObj.disconnect();
    }
}

void SDEAppNode::graphCompleteEvtHdlr()
{
    vx_status                    vxStatus = VX_SUCCESS;

    RCLCPP_INFO(this->get_logger(), "%s Launched.", __FUNCTION__);

    while (m_cntxt->exitOutputThread == false)
    {
        /* Wait for the data ready semaphore. */
        if (m_cntxt->outputCtrlSem)
        {
            m_cntxt->outputCtrlSem->wait();
        }

        if (m_cntxt->exitOutputThread == true)
        {
            break;
        }

        SDEAPP_getOutBuff(m_cntxt,
                              &m_cntxt->vxDispRightImage,
                              &m_cntxt->vxDisparity16,
                              &m_cntxt->vxDispTriangPC,
                              &m_cntxt->outTimestamp);

        /* Raw Disparity */
        // Extract raw disparity data
        vxStatus = CM_extractImageData((uint8_t *)m_disparityPubData.data.data(),
                                       m_cntxt->vxDisparity16,
                                       m_disparityPubData.width,
                                       m_disparityPubData.height,
                                       1,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "CM_extractImageData() failed.");
        }

        /* Point cloude data */
        if (m_cntxt->enablePC)
        {
            // Get the number of points
            vxStatus = CM_extractPointCloudData((uint8_t *)m_pcPubData.data.data(),
                                                m_cntxt->vxDispTriangPC,
                                                m_pcPubData.point_step,
                                                &m_numPcPoints,
                                                0);
            // resize m_pcPubData.data
            m_pcPubData.data.resize(m_numPcPoints*m_pcPubData.point_step);

            // Extract point cloud data
            vxStatus = CM_extractPointCloudData((uint8_t *)m_pcPubData.data.data(),
                                                m_cntxt->vxDispTriangPC,
                                                m_pcPubData.point_step,
                                                &m_numPcPoints,
                                                1);
        }

        // Release the output buffers
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            SDEAPP_releaseOutBuff(m_cntxt);
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            rclcpp::Time time = this->get_clock()->now();

            // Publish raw Disparity map
            m_disparityPubData.header.stamp = time;
            m_disparityPub->publish(m_disparityPubData);

            if (m_cntxt->enablePC)
            {
                // Publish point cloud data
                m_pcPubData.width           = m_numPcPoints;
                m_pcPubData.row_step        = m_numPcPoints * m_pcPubData.point_step;
                m_pcPubData.header.stamp    = time;
                m_pcPub->publish(m_pcPubData);
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s Exiting.", __FUNCTION__);
}

SDEAppNode::~SDEAppNode()
{
    if (m_sync)
    {
        delete m_sync;
    }

    if (m_cntxt)
    {
        delete m_cntxt;
    }

    if (m_sub)
    {
        delete m_sub;
    }

    if (m_leftImageSub)
    {
        delete m_leftImageSub;
    }

    if (m_rightImageSub)
    {
        delete m_rightImageSub;
    }

    if (initCtrlSem)
    {
        delete initCtrlSem;
    }
}

void SDEAppNode::readParams()
{
    std::string                     str;
    bool                            status;
    int32_t                         tmp;
    float                           ftmp;

    m_cntxt->sde_params.confidence_score_map[0] = 0;
    m_cntxt->sde_params.confidence_score_map[1] = 4;
    m_cntxt->sde_params.confidence_score_map[2] = 9;
    m_cntxt->sde_params.confidence_score_map[3] = 18;
    m_cntxt->sde_params.confidence_score_map[4] = 28;
    m_cntxt->sde_params.confidence_score_map[5] = 43;
    m_cntxt->sde_params.confidence_score_map[6] = 109;
    m_cntxt->sde_params.confidence_score_map[7] = 127;


    /* Get left LUT file path information. */
    status = get_parameter("left_lut_file_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'left_lut_file_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->left_LUT_file_name, SDEAPP_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get right LUT file path information. */
    status = get_parameter("right_lut_file_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'right_lut_file_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->right_LUT_file_name, SDEAPP_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get input image format information */
    get_parameter_or("input_format", tmp, (int32_t)CM_IMG_FORMAT_UYVY);
    m_cntxt->inputFormat = (uint8_t)tmp;

    /* Get stereo algorithm type information */
    get_parameter_or("sde_algo_type", tmp, 0);
    m_cntxt->sdeAlgoType = (uint8_t)tmp;

    /* Get the number of layers information when sde_algo_typ = 1*/
    get_parameter_or("num_layers", tmp, 2);
    m_cntxt->numLayers = (uint8_t)tmp;

    /* Get the median filtering flag information when sde_algo_typ = 1*/
    get_parameter_or("pp_median_filter_enable", tmp, 0);
    m_cntxt->ppMedianFilterEnable = (uint8_t)tmp;

    /* SDE Parameters */
    /* Get the SDE minimum disparity information */
    /* Always set to 0 */
    m_cntxt->sde_params.disparity_min = 0;

    /* Get the SDE maximum disparity information */
    get_parameter_or("disparity_max", tmp, 1);
    m_cntxt->sde_params.disparity_max = (uint16_t)tmp;

    /* Get the SDE left-right consistency check threshold information */
    get_parameter_or("threshold_left_right", tmp, 3);
    m_cntxt->sde_params.threshold_left_right = (uint16_t)tmp;

    /* Get the SDE texture based filtering enable flag information */
    get_parameter_or("texture_filter_enable", tmp, 0);
    m_cntxt->sde_params.texture_filter_enable = (uint16_t)tmp;

    /* Get the SDE texture filtering threshold information */
    get_parameter_or("threshold_texture", tmp, 0);
    m_cntxt->sde_params.threshold_texture = (uint16_t)tmp;

    /* Get the SDE aggregation penalty p1 information */
    get_parameter_or("aggregation_penalty_p1", tmp, 32);
    m_cntxt->sde_params.aggregation_penalty_p1 = (uint16_t)tmp;

    /* Get the SDE aggregation penalty p2 information */
    get_parameter_or("aggregation_penalty_p2", tmp, 197);
    m_cntxt->sde_params.aggregation_penalty_p2 = (uint16_t)tmp;

    /* Get the SDE median filter enable flag information */
    get_parameter_or("median_filter_enable", tmp, 1);
    m_cntxt->sde_params.median_filter_enable = (uint16_t)tmp;

    /* Get the SDE reduced range search enable flag information */
    get_parameter_or("reduced_range_search_enable", tmp, 0);
    m_cntxt->sde_params.reduced_range_search_enable = (uint16_t)tmp;

    /* Get the stereo camera baseline information */
    get_parameter_or("stereo_baseline", ftmp, 0.12f);
    m_cntxt->baseline = (float)ftmp;

    /* Get PC creation flag information */
    get_parameter_or("enable_pc", tmp, 0);
    m_cntxt->enablePC = (uint8_t)tmp;

    /* Set logFileName based on enablePC */
    m_cntxt->logFileName = (m_cntxt->enablePC == 0) ? "sde" : "sdepcl";

    /*Sub-sampling ratio of point cloud */
    get_parameter_or("pc_subsample_ratio", tmp, 1);
    m_cntxt->pcSubsampleRatio = (uint8_t)tmp;

    /* Get use of PC configuration flag information*/
    get_parameter_or("use_pc_config", tmp, 1);
    m_cntxt->usePCConfig = (uint8_t)tmp;

    /* Get the disparity confidence threshold information */
    get_parameter_or("sde_confidence_threshold", tmp, 0);
    m_cntxt->dispConfidence = (uint8_t)tmp;

    /* Low X pos limit for which 3D point is rendered  */
    get_parameter_or("point_low_x", ftmp, -20.0f);
    m_cntxt->lowPtX = (float)ftmp;

    /* High X pos limit for which 3D point is rendered  */
    get_parameter_or("point_high_x", ftmp, 20.0f);
    m_cntxt->highPtX = (float)ftmp;

    /* Low Y pos (depth) limit for which 3D point is rendered  */
    get_parameter_or("point_low_y", ftmp, 0.0f);
    m_cntxt->lowPtY = (float)ftmp;

    /* High Y pos (depth) limit for which 3D point is rendered  */
    get_parameter_or("point_high_y", ftmp, 20.0f);
    m_cntxt->highPtY = (float)ftmp;

    /* Low Z pos limit for which 3D point is rendered  */
    get_parameter_or("point_low_Z", ftmp, -2.0f);
    m_cntxt->lowPtZ = (float)ftmp;

    /* High Z pos limit for which 3D point is rendered  */
    get_parameter_or("point_high_Z", ftmp, 5.0f);
    m_cntxt->highPtZ = (float)ftmp;

    /* OpenVX Graph Operation Parameters */
    /* Get interactive mode flag information. */
    get_parameter_or("is_interactive", tmp, 0);
    m_cntxt->is_interactive = (uint8_t)tmp;

    /* Get graph export flag information. */
    get_parameter_or("exportGraph", tmp, 0);
    m_cntxt->exportGraph = (uint8_t)tmp;

    /* Get perf export flag information. */
    get_parameter_or("exportPerfStats", tmp, 0);
    m_cntxt->exportPerfStats = (uint8_t)tmp;

    /* Get real-time logging enable information. */
    get_parameter_or("rtLogEnable", tmp, 0);
    m_cntxt->rtLogEnable = (uint8_t)tmp;

    /* Get pipeline depth information. */
    get_parameter_or("pipeline_depth", tmp, GRAPH_MAX_PIPELINE_DEPTH);
    m_cntxt->pipelineDepth = (uint8_t)tmp;

    /* Get the disparity merge core information. */
    get_parameter("disp_merge_deploy_core", str);
    m_cntxt->mlSdeCreateParams.dispMergeNodeCore =
        CM_getCoreName((const char *)str.c_str());

    /* Get the hole filling core information. */
    get_parameter("hole_filling_deploy_core", str);
    m_cntxt->mlSdeCreateParams.holeFillingNodeCore =
        CM_getCoreName((const char *)str.c_str());

    /* Get the point cloud generation core information. */
    get_parameter("color_conv_deploy_core", str);
    m_cntxt->sdeTriangCreateParams.ccNodeCore =
        CM_getCoreName((const char *)str.c_str());

    /* Get the occupancy grid generation core information. */
    get_parameter("triang_deploy_core", str);
    m_cntxt->sdeTriangCreateParams.triangNodeCore =
        CM_getCoreName((const char *)str.c_str());

    if (m_cntxt->width < 128)
    {
        m_cntxt->width = 128;
    }
    if (m_cntxt->height < 128)
    {
        m_cntxt->height = 128;
    }

    if (m_cntxt->pipelineDepth > GRAPH_MAX_PIPELINE_DEPTH)
    {
        RCLCPP_INFO(this->get_logger(), "Pipeline depth is larger than maximum pipeline depth allowed. Clipped..");
        m_cntxt->pipelineDepth = GRAPH_MAX_PIPELINE_DEPTH;
    }

    if (m_cntxt->inputFormat != CM_IMG_FORMAT_UYVY)
    {
        RCLCPP_INFO(this->get_logger(), "Input format should be YUV_UYVY...");
        exit(-1);
    }

    if (m_cntxt->enablePC)
    {
        if ((m_cntxt->pcSubsampleRatio != 1) ||
            (m_cntxt->pcSubsampleRatio != 2) ||
            (m_cntxt->pcSubsampleRatio != 4))
        {
            RCLCPP_INFO(this->get_logger(), "Improper point cloud sub-sample ratio. Set to 1");
            m_cntxt->pcSubsampleRatio = 1;
        }
    }

    // when multi-layer SDE is used
    if (m_cntxt->sdeAlgoType == 1)
    {
        int8_t  factor = (m_cntxt->numLayers - 1) * 2;
        int32_t w, i;

        if ((m_cntxt->height % factor != 0) ||
            (m_cntxt->width % factor != 0))
        {
            RCLCPP_INFO(this->get_logger(), "Improper stereo image resolution...");
            exit(-1);
        }

        for (i = 1; i < m_cntxt->numLayers; i++)
        {
            w = m_cntxt->width  / (2*i);

            if (w % 16 != 0)
            {
                RCLCPP_INFO(this->get_logger(), "Improper image width is not multiple of 16...\n");
                exit(-1);
            }
        }
    }

    m_cntxt->renderPeriod = 0;

    return;

}

vx_status SDEAppNode::init()
{
    PTK_CRT             ptkConfig;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    //tivx_set_debug_zone(VX_ZONE_INFO);

    PTK_init(&ptkConfig);


    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* parse command line */
        readParams();

        SDEAPP_setAllParams(m_cntxt);

        /* Initialize the Application context */
        vxStatus = SDEAPP_init(m_cntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "SDEAPP_init() failed.");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Launch processing threads. */
        SDEAPP_launchProcThreads(m_cntxt);
    }

    return vxStatus;
}

void SDEAppNode::onShutdown()
{
    if (m_cntxt)
    {
        SDEAPP_intSigHandler(m_cntxt);
    }
}


