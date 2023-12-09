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

#include <cv_bridge/cv_bridge.h>

#include <estop_node.h>
#include <estop.h>

EStopNode::EStopNode(const rclcpp::NodeOptions &options,
                     const std::string         &name):
    Node(name, options)
{
    std::string cfgFile;
    std::string leftLUTFile;
    std::string rightLUTFile;
    vx_status   vxStatus = VX_SUCCESS;

    // Create initialization control semaphore
    initCtrlSem = new Semaphore(0);

    // Create App context
    m_cntxt = new ESTOP_APP_Context();

    if (m_cntxt == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "new ESTOP_APP_Context() failed.");
        vxStatus = VX_FAILURE;
    }

    // camera parameter should be initialized
    m_cntxt->state = ESTOP_APP_STATE_INIT;

    // launch the subscriber thread
    auto subThread = std::thread([this]{subscriberCamInfoThread();});
    subThread.detach();

    // launch the input process thread
    auto localThread = std::thread([this]{inputProcessThread();});
    localThread.detach();

    // register a callback to run at rclcpp::shutdown()
    rclcpp::on_shutdown(std::bind(&EStopNode::onShutdown, this));
}

void EStopNode::setupInputImgSubscribers()
{
    std::string leftTopicName;
    std::string rightTopicName;
    bool        status;

    // Query the topicname to subscribe to
    status = get_parameter("left_input_topic_name", leftTopicName);
    if (status == true)
    {
        status = get_parameter("right_input_topic_name", rightTopicName);
    }

    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'input_topic_name' not found.");
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

    m_conObj = m_sync->registerCallback(&EStopNode::imgCb, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to the topic: %s", leftTopicName.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to the topic: %s", rightTopicName.c_str());
}

void EStopNode::inputProcessThread()
{
    // wait until camera params are set
    if (initCtrlSem)
    {
        initCtrlSem->wait();
    }

    // Initialize the App context
    auto vxStatus = this->init();

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Application Initialization failed.");
        exit(-1);
    }

    m_imgTrans = new ImgTrans(this->create_sub_node("image_transport"));


    // Cache the input image and tensor sizes
    m_ssMapImgWidth  = m_cntxt->tensor_width;
    m_ssMapImgHeight = m_cntxt->tensor_height;

    m_imgWidth  = m_cntxt->width;
    m_imgHeight = m_cntxt->height;

    // launch the publisher threads
    auto pubThread = std::thread([this]{publisherThread();});
    pubThread.detach();
}

void EStopNode::subscriberCamInfoThread()
{
    std::string camInfoTopicName;
    bool        status;

    status = get_parameter("camera_info_topic", camInfoTopicName);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'camera_info_topic' not found.");
        exit(-1);
    }

    m_sub = new CamInfoSub(this, camInfoTopicName);
    if (m_sub == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "new CamInfoSub() failed.");
        exit(-1);
    }

    m_camInfoConObj = m_sub->registerCallback(&EStopNode::camInfoCb, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to the topic: %s", camInfoTopicName.c_str());
}

void EStopNode::publisherThread()
{
    std::string ssTensorTopic;
    std::string rectImgTopic;
    std::string bbTopic;
    std::string rawDispTopic;
    std::string ogMapTopic;
    std::string eStopTopic;
    vx_status   vxStatus;
    bool        status;
    bool        latch = true;

    // Query the topic name to publish the output tensor.
    status = get_parameter("semseg_cnn_tensor_topic", ssTensorTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'semseg_cnn_tensor_topic' not found.");
        exit(-1);
    }

    status = get_parameter("rectified_image_topic", rectImgTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'rectified_image_topic' not found.");
        exit(-1);
    }

    status = get_parameter("bounding_box_topic", bbTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'bounding_box_topic' not found.");
        exit(-1);
    }

    status = get_parameter("raw_disparity_topic_name", rawDispTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'raw_disparity_topic_name' not found.");
        exit(-1);
    }

    status = get_parameter("ogmap_topic_name", ogMapTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'ogmap_topic_name' not found.");
        exit(-1);
    }

    status = get_parameter("estop_topic_name", eStopTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'estop_topic_name' not found.");
        exit(-1);
    }

    /* SS Map image tensor */
    // Pre-allocate vectors used for publishing out SS Map tensor data
    m_ssTensorSize = m_cntxt->outTensorSize;
    m_ssTensorPubData.data.assign(m_ssTensorSize, 0);
    m_ssTensorPubData.width          = m_ssMapImgWidth;
    m_ssTensorPubData.height         = m_ssMapImgHeight;
    m_ssTensorPubData.step           = m_ssMapImgWidth;
    m_ssTensorPubData.encoding       = "mono8";
    m_ssTensorPubData.header.frame_id = "map";

    const std::string taskType = m_cntxt->postProcObj->getTaskType();

    if (taskType == "segmentation")
    {
        // Create the publisher for the SS map output image
        m_ssTensorPub = m_imgTrans->advertise(ssTensorTopic, latch);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported taskType");
        exit(-1);
    }

    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", ssTensorTopic.c_str());

    /* Rectified Right Image */
    m_rectImageSize = m_imgWidth * m_imgHeight;
    m_rectImagePubData.data.assign(m_rectImageSize, 0);
    m_rectImagePubData.width           = m_imgWidth;
    m_rectImagePubData.height          = m_imgHeight;
    m_rectImagePubData.step            = m_imgWidth;
    m_rectImagePubData.encoding        = "mono8";
    m_rectImagePubData.header.frame_id = "map";

    // Create the publisher for the rectified image
    m_rectImgPub = m_imgTrans->advertise(rectImgTopic, latch);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", rectImgTopic.c_str());

    /* Bounding box */
    m_bbSize = m_cntxt->maxNumObject;
    m_bbPubData.reserve(m_bbSize*sizeof(ObjectPos3D));
    m_bbData = new ObjectPos3D[m_bbSize];

    // Create the publisher for the rectified image
    m_bbPub = this->create_publisher<Detection3D>(bbTopic, 1);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", bbTopic.c_str());

    /* Raw disparity */
    // Pre-allocate vectors used for publishing out raw disparity
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

    /* OG Map data */
    // Pre-allocate vectors used for publishing out OG data
    m_ogMapSize = m_cntxt->xGridNum * m_cntxt->yGridNum;
    m_ogMapPubData.data.assign(m_ogMapSize, 0);
    m_ogMapPubData.info.resolution           = m_cntxt->xGridSize*1.0/1000;
    m_ogMapPubData.info.width                = m_cntxt->xGridNum;
    m_ogMapPubData.info.height               = m_cntxt->yGridNum;
    m_ogMapPubData.info.origin.position.x    = m_cntxt->xMinRange/1000;
    m_ogMapPubData.info.origin.position.y    = m_cntxt->yMinRange/1000;
    m_ogMapPubData.info.origin.position.z    = 0;
    m_ogMapPubData.info.origin.orientation.x = 0.0;
    m_ogMapPubData.info.origin.orientation.y = 0.0;
    m_ogMapPubData.info.origin.orientation.z = 0.0;
    m_ogMapPubData.info.origin.orientation.w = 1.0;
    m_ogMapPubData.header.frame_id           = "map";

    // Create the publisher for the OG data
    m_ogMapPub = this->create_publisher<OccupancyGrid>(ogMapTopic, 1);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", ogMapTopic.c_str());

    /* E-Stop grid init */
    m_freeRun    = 0;
    m_obsRun     = 0;
    m_eStop.data = false;

    // define eSTOP area
    m_eStopGridIdx = new int32_t[m_ogMapSize];

    vxStatus = setEStopArea(m_eStopGridIdx,
                            m_minEStopDistance,
                            m_maxEStopDistance,
                            m_minEStopWidth,
                            m_maxEStopWidth,
                            m_cntxt->xGridNum,
                            m_cntxt->yGridNum,
                            m_cntxt->xMinRange,
                            m_cntxt->yMinRange,
                            m_cntxt->xGridSize);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "setEStopArea() failed.");
        exit(-1);
    }

    // Create the publisher for eStop flag (0; Go, 1: Stop)
    m_eStopPub = this->create_publisher<std_msgs::msg::Bool>(eStopTopic, latch);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", eStopTopic.c_str());

    // setup input subscribers
    setupInputImgSubscribers();

    // Event handler
    processCompleteEvtHdlr();

    // Shutdown the publishers
    m_ssTensorPub.shutdown();
    m_rectImgPub.shutdown();
}

void EStopNode::camInfoCb(const CameraInfo::ConstSharedPtr& cam_info)
{
    ESTOP_APP_init_camInfo(m_cntxt,
                           cam_info->width, cam_info->height,
                           cam_info->p[0], cam_info->p[2], cam_info->p[6]);

    if (initCtrlSem)
    {
        initCtrlSem->notify();
    }
}

void EStopNode::imgCb(const Image::ConstSharedPtr& left_image_ptr,
                      const Image::ConstSharedPtr& right_image_ptr)
{
    if (m_cntxt->state == ESTOP_APP_STATE_INIT)
    {
        uint64_t nanoSec = right_image_ptr->header.stamp.sec * 1e9 +
                           right_image_ptr->header.stamp.nanosec;

        ESTOP_APP_run(m_cntxt,
                      left_image_ptr->data.data(),
                      right_image_ptr->data.data(),
                      nanoSec);
    }
    else if (m_cntxt->state == ESTOP_APP_STATE_INVALID)
    {
        m_conObj.disconnect();
    }
}

void EStopNode::processCompleteEvtHdlr()
{
    Detection3D             bbMsg;
    vx_status               vxStatus = VX_SUCCESS;
    uint8_t                 numObjects;

    RCLCPP_INFO(this->get_logger(), "%s Launched.", __FUNCTION__);

    while (m_cntxt->exitOutputThread == false)
    {
        vx_tensor   vxOutputTensor;

        /* Wait for the data ready semaphore. */
        if (m_cntxt->outputCtrlSem)
        {
            m_cntxt->outputCtrlSem->wait();
        }

        if (m_cntxt->exitOutputThread == true)
        {
            break;
        }

        ESTOP_APP_getOutBuff(m_cntxt,
                            &m_cntxt->vxDispRightImage,
                            &m_cntxt->vxDispSSImage,
                            &vxOutputTensor,
                            &m_cntxt->vxDisp3DBoundBox,
                            &m_cntxt->vxDisparity16,
                            &m_cntxt->outTimestamp);

        /* SS MAP */
        // Extract raw tensor data
        vxStatus = CM_extractTensorData(m_ssTensorPubData.data.data(),
                                        vxOutputTensor,
                                        m_cntxt->outTensorSize);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "CM_extractTensorData() failed.");
        }

        /* Rectified (Right) Image */
        // Extract rectified right image data (Luma only)
        vxStatus = CM_extractImageData(m_rectImagePubData.data.data(),
                                       m_cntxt->vxDispRightImage,
                                       m_rectImagePubData.width,
                                       m_rectImagePubData.height,
                                       1,
                                       0);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "CM_extractImageData() failed.");
        }

        /* 3D Bounding box data */
        vxStatus = extract3DBBData(m_bbData,
                                   m_cntxt->vxDisp3DBoundBox,
                                   &numObjects);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "extract3DBBData() failed.");
        }

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

        /* OG Map Data */
        vxStatus = extractOGMapData(m_ogMapPubData.data.data(),
                                    m_bbData,
                                    m_cntxt->vxDisp3DBoundBox,
                                    m_ogMapPubData.info.width,
                                    m_ogMapPubData.info.height,
                                    m_cntxt->xMinRange,
                                    m_cntxt->yMinRange,
                                    m_cntxt->xGridSize);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "extractOGMapData() failed.");
        }

        /* Make eStop decision */
        vxStatus = makeEStopDecision(m_ogMapPubData.data.data(),
                                     m_eStopGridIdx,
                                     m_ogMapPubData.info.width,
                                     m_ogMapPubData.info.height);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "makeEStopDecision() failed.");
        }

        // Release the output buffers
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            ESTOP_APP_releaseOutBuff(m_cntxt);
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            rclcpp::Time time = this->get_clock()->now();

            // Publish SS MAP tensor
            m_ssTensorPubData.header.stamp = time;
            m_ssTensorPub.publish(m_ssTensorPubData);

            // Publish BB data
            m_bbPubData.assign(m_bbData, &m_bbData[numObjects]);
            bbMsg.obj_pos3d       = m_bbPubData;
            bbMsg.num_objects     = numObjects;
            bbMsg.header.stamp    = time;
            bbMsg.header.frame_id = "map";

            m_bbPub->publish(bbMsg);

            // Publish Rectified (Right) image
            m_rectImagePubData.header.stamp = time;
            m_rectImgPub.publish(m_rectImagePubData);

            // Publish raw Disparity map
            m_disparityPubData.header.stamp = time;
            m_disparityPub->publish(m_disparityPubData);

            // Publish OG map data
            m_ogMapPubData.header.stamp    = time;
            m_ogMapPub->publish(m_ogMapPubData);

            // Publish estop flag
            m_eStopPub->publish(m_eStop);
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s Exiting.", __FUNCTION__);
}

EStopNode::~EStopNode()
{
    if (m_bbData)
    {
        delete [] m_bbData;
    }

    if (m_eStopGridIdx)
    {
        delete [] m_eStopGridIdx;
    }

    if (m_imgTrans)
    {
        delete m_imgTrans;
    }

    if (m_sync)
    {
        delete m_sync;
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

    if (m_cntxt)
    {
        delete m_cntxt;
    }
}

void EStopNode::readParams()
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
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'left_lut_file_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->left_LUT_file_name, ESTOP_APP_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get right LUT file path information. */
    status = get_parameter("right_lut_file_path", str);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'right_lut_file_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->right_LUT_file_name, ESTOP_APP_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get TIDL network path information. */
    status = get_parameter("dl_model_path", str);

    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'dl_model_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->dlModelPath, ESTOP_APP_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get input image format information */
    get_parameter_or("input_format", tmp, (int32_t)CM_IMG_FORMAT_UYVY);
    m_cntxt->inputFormat = (uint8_t)tmp;

    /* Get number of semseg classes information. */
    get_parameter_or("num_classes", tmp, 20);
    m_cntxt->numClasses = (uint16_t)tmp;

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

    /* Get the disparity confidence threshold information */
    get_parameter_or("sde_confidence_threshold", tmp, 0);
    m_cntxt->confidence_threshold = (uint8_t)tmp;

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

    /* Get the camera roll information */
    get_parameter_or("camera_roll", ftmp, 0.0f);
    m_cntxt->camRoll = (float)ftmp;

    /* Get the camera pitch information */
    get_parameter_or("camera_pitch", ftmp, 0.0f);
    m_cntxt->camPitch = (float)ftmp;

    /* Get the camera yaw information */
    get_parameter_or("camera_yaw", ftmp, 0.0f);
    m_cntxt->camYaw = (float)ftmp;

    /* Get the camera height information */
    get_parameter_or("camera_height", ftmp, 650.0f);
    m_cntxt->camHeight = (float)ftmp;

    /* Get the stereo camera baseline information */
    get_parameter_or("stereo_baseline", ftmp, 120.0f);
    m_cntxt->baseline = (float)ftmp;

    /* OG Map Parameters */
    /* Get the horizontal grid size information */
    get_parameter_or("grid_x_size", tmp, 200);
    m_cntxt->xGridSize = (int32_t)tmp;

    /* Get the vertical grid size information */
    get_parameter_or("grid_y_size", tmp, 200);
    m_cntxt->yGridSize = (int32_t)tmp;

    /* Get the minimum X range information covered by OG map */
    get_parameter_or("min_x_range", tmp, -20000);
    m_cntxt->xMinRange = (int32_t)tmp;

    /* Get the maximum X range information covered by OG map */
    get_parameter_or("max_x_range", tmp, 20000);
    m_cntxt->xMaxRange = (int32_t)tmp;

    /* Get the minimum Y range information covered by OG map */
    get_parameter_or("min_y_range", tmp, -20000);
    m_cntxt->yMinRange = (int32_t)tmp;

    /* Get the maximum Y range information covered by OG map */
    get_parameter_or("max_y_range", tmp, 20000);
    m_cntxt->yMaxRange = (int32_t)tmp;

    /* Get the pixel count threshold information for grid occupied/non-occupied decision*/
    get_parameter_or("min_pixel_count_grid", tmp, 5);
    m_cntxt->thCnt = (int16_t)tmp;

    /* Get the pixel count threshold information for object occupied/non-occupied decision*/
    get_parameter_or("min_pixel_count_object", tmp, 15);
    m_cntxt->thObjCnt = (int16_t)tmp;

    /* Get the maximum number of detected objects information */
    get_parameter_or("max_object_to_detect", tmp, 50);
    m_cntxt->maxNumObject = (int16_t)tmp;

    /* Get the CCA neighboring grid number information */
    get_parameter_or("num_neighbor_grid", tmp, 24);
    m_cntxt->cNeighNum = (int16_t)tmp;

    /* Get the spatial object merge enable flag information */
    get_parameter_or("enable_spatial_obj_merge", tmp, 1);
    m_cntxt->enableSpatialObjMerge = (uint8_t)tmp;

    /* Get the temporal object merge enable flag information */
    get_parameter_or("enable_temporal_obj_merge", tmp, 1);
    m_cntxt->enableTemporalObjMerge = (uint8_t)tmp;

    /* Get the temporal object smoothing enable flag information */
    get_parameter_or("enable_temporal_obj_smoothing", tmp, 0);
    m_cntxt->enableTemporalObjSmoothing = (uint8_t)tmp;

    /* Get the distance metric mode information */
    get_parameter_or("object_distance_mode", tmp, 0);
    m_cntxt->objectDistanceMode = (uint8_t)tmp;

    /* E-Stop Configs */
    /* Min distance of estop range in mm */
    get_parameter_or("min_estop_distance", tmp, 0);
    m_minEStopDistance = (int32_t)tmp;

    /* Max distance of estop range in mm */
    get_parameter_or("max_estop_distance", tmp, 2000);
    m_maxEStopDistance = (int32_t)tmp;

    /* Width of estop range at min_estop_distance in mm */
    get_parameter_or("min_estop_width", tmp, 1500);
    m_minEStopWidth = (int32_t)tmp;

    /* Width of estop range at max_estop_distance */
    get_parameter_or("max_estop_width", tmp, 1500);
    m_maxEStopWidth = (int32_t)tmp;

    /* Minimum number of consecutive frames without any obstacle
       in EStop range to set m_eStop = 0 */
    get_parameter_or("min_free_frame_run", tmp, 3);
    m_minFreeRun = (int8_t)tmp;

    /* Minimum number of consecutive frames with any obstacle
       in EStop range to set m_eStop = 1 */
    get_parameter_or("min_obs_frame_run", tmp, 1);
    m_minObsRun = (int8_t)tmp;

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

    /* set the ti_logger level */
    get_parameter_or("log_level", tmp, static_cast<int32_t>(ERROR));
    logSetLevel((LogLevel) tmp);

    /* Get pipeline depth information. */
    get_parameter_or("pipeline_depth", tmp, GRAPH_MAX_PIPELINE_DEPTH);
    m_cntxt->pipelineDepth = (uint8_t)tmp;

    /* Get the disparity merge core information. */
    get_parameter("disp_merge_deploy_core", str);
    m_cntxt->mlSdeCreateParams.dispMergeNodeCore = CM_getCoreName((const char *)str.c_str());

    /* Get the hole filling core information. */
    get_parameter("hole_filling_deploy_core", str);
    m_cntxt->mlSdeCreateParams.holeFillingNodeCore = CM_getCoreName((const char *)str.c_str());

    /* Get the point cloud generation core information. */
    get_parameter("pc_deploy_core", str);
    m_cntxt->ssDetectCreateParams.pcNodeCore = CM_getCoreName((const char *)str.c_str());

    /* Get the occupancy grid generation core information. */
    get_parameter("og_deploy_core", str);
    m_cntxt->ssDetectCreateParams.ogNodeCore = CM_getCoreName((const char *)str.c_str());

    /* Check paramerters' values */
    m_cntxt->xGridNum = ceil((m_cntxt->xMaxRange - m_cntxt->xMinRange) / m_cntxt->xGridSize);
    m_cntxt->yGridNum = ceil((m_cntxt->yMaxRange - m_cntxt->yMinRange) / m_cntxt->yGridSize);

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
        RCLCPP_ERROR(this->get_logger(), "Pipeline depth is larger than maximum pipeline depth allowed. Clipped..");
        m_cntxt->pipelineDepth = GRAPH_MAX_PIPELINE_DEPTH;
    }

    if (m_cntxt->inputFormat != CM_IMG_FORMAT_UYVY)
    {
        RCLCPP_ERROR(this->get_logger(), "Input format should be YUV_UYVY...");
        exit(-1);
    }

    // when multi-layer SDE is used
    if (m_cntxt->sdeAlgoType == 1)
    {
        int8_t  factor = (m_cntxt->numLayers - 1) * 2;
        int32_t w, i;

        if (m_cntxt->height % factor != 0 || m_cntxt->width % factor != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Improper stereo image resolution...");
            exit(-1);
        }

        for (i = 1; i < m_cntxt->numLayers; i++)
        {
            w = m_cntxt->width  / (2*i);

            if (w % 16 != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Improper image width is not multiple of 16...\n");
                exit(-1);
            }
        }
    }

    m_cntxt->renderPeriod = 0;

    return;

}

vx_status EStopNode::init()
{
    PTK_CRT             ptkConfig;
    vx_status           vxStatus;

    vxStatus = VX_SUCCESS;

    /* Initialize PTK library. */
    ptkConfig.exit   = exit;
    ptkConfig.printf = printf;
    ptkConfig.time   = NULL;

    PTK_init(&ptkConfig);

    /* parse command line */
    readParams();

    /* Initialize the Application context */
    vxStatus = ESTOP_APP_init(m_cntxt);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "ESTOP_APP_init() failed.");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Launch processing threads. */
        ESTOP_APP_launchProcThreads(m_cntxt);
    }

    return vxStatus;
}

void EStopNode::onShutdown()
{
    if (m_cntxt)
    {
        ESTOP_APP_intSigHandler(m_cntxt);
    }
}

vx_status EStopNode::extract3DBBData(ObjectPos3D          * bbData,
                                     vx_user_data_object    vx3DBB,
                                     uint8_t              * numObjects)
{
    int32_t n;
    double   floatX, floatY;
    float frontDepth;

    PTK_Alg_StereoOG_obs3DBox      * obs3DBB;
    PTK_Alg_StereoOG_BoxProp       * boxProp;

    obs3DBB = (tivx_ss_sde_obs_3d_bound_box_t *) CM_getUserObjDataPayload(vx3DBB);
    boxProp = (PTK_Alg_StereoOG_BoxProp *) PTK_Alg_StereOG_get3DBB(obs3DBB);

    // init
    *numObjects = obs3DBB->numObject;

    for (n = 0; n < obs3DBB->numObject; n++)
    {
        bbData[n].class_id = boxProp[n].classId;
        bbData[n].pf1x     = boxProp[n].pf1x;
        bbData[n].pf1y     = boxProp[n].pf1y;
        bbData[n].pf2x     = boxProp[n].pf2x;
        bbData[n].pf2y     = boxProp[n].pf2y;
        bbData[n].pf3x     = boxProp[n].pf3x;
        bbData[n].pf3y     = boxProp[n].pf3y;
        bbData[n].pf4x     = boxProp[n].pf4x;
        bbData[n].pf4y     = boxProp[n].pf4y;

        bbData[n].pr1x     = boxProp[n].pr1x;
        bbData[n].pr1y     = boxProp[n].pr1y;
        bbData[n].pr2x     = boxProp[n].pr2x;
        bbData[n].pr2y     = boxProp[n].pr2y;
        bbData[n].pr3x     = boxProp[n].pr3x;
        bbData[n].pr3y     = boxProp[n].pr3y;
        bbData[n].pr4x     = boxProp[n].pr4x;
        bbData[n].pr4y     = boxProp[n].pr4y;

        // compute radial distance from camera
        floatX     = ((boxProp[n].topLeftGridX + boxProp[n].bottomRightGridX) / 2);
        floatY     = (boxProp[n].bottomRightGridY);
        frontDepth = (float)sqrt(floatX * floatX + floatY * floatY);

        bbData[n].front_depth         = frontDepth;
        bbData[n].rear_depth          = boxProp[n].rearDepth;
        bbData[n].top_left_grid_x     = boxProp[n].topLeftGridX;
        bbData[n].top_left_grid_y     = boxProp[n].topLeftGridY;
        bbData[n].bottom_right_grid_x = boxProp[n].bottomRightGridX;
        bbData[n].bottom_right_grid_y = boxProp[n].bottomRightGridY;
    }

    return (vx_status)VX_SUCCESS;
}

vx_status EStopNode::extractOGMapData(int8_t               *ogData,
                                      ObjectPos3D          *bbData,
                                      vx_user_data_object   vx3DBB,
                                      int32_t               width,
                                      int32_t               height,
                                      int32_t               xMinRange,
                                      int32_t               yMinRange,
                                      int32_t               gridSize)
{
    PTK_Alg_StereoOG_obs3DBox  *obs3DBB;
    PTK_Alg_StereoOG_BoxProp   *boxProp;
    float                       angle;
    float                       tanAngle;
    int32_t                     minX;
    int32_t                     maxX;
    int32_t                     minY;
    int32_t                     maxY;
    int8_t                      cost;
    int32_t                     offset;
    int32_t                     numGrids;
    int32_t                     n;
    int32_t                     i;
    int32_t                     j;

    numGrids = width*height;

    obs3DBB =
        (tivx_ss_sde_obs_3d_bound_box_t *) CM_getUserObjDataPayload(vx3DBB);

    boxProp =
        (PTK_Alg_StereoOG_BoxProp *) PTK_Alg_StereOG_get3DBB(obs3DBB);

    // init FOV (assuming 120 degree)
    angle    = 60 * M_PI / 180.0;
    tanAngle = tan(angle);

    // init to NON_FOV_GRID_VALUE.
    memset(ogData, NON_FOV_GRID_VALUE, width * height);

    // init grids in FOV to FOV_GRID_VALUE
    for (j = height/2; j < height; j++)
    {
       offset = floor((j - height/2)*tanAngle);
       if (offset > width/2)
       {
           offset = width/2;
       }

       for (i = width/2 - offset; i < width/2 + offset; i++)
       {
           ogData[j * width + i] = FOV_GRID_VALUE;
       }
    }

    /* e-stop area */
    for (i = 0; i < numGrids; i++)
    {
        if (m_eStopGridIdx[i] == -1)
        {
            break;
        }

        ogData[m_eStopGridIdx[i]] = ESTOP_AREA_GRID_VALUE; // light red for e-stop area
    }

    // mark detected objects
    for (n = 0; n < obs3DBB->numObject; n++)
    {
        minX = (boxProp[n].topLeftGridX - xMinRange) / gridSize;
        maxX = (boxProp[n].bottomRightGridX - xMinRange) / gridSize;

        minY = (boxProp[n].bottomRightGridY - yMinRange) / gridSize;
        maxY = (boxProp[n].topLeftGridY - yMinRange) / gridSize;

        if (bbData[n].front_depth < 5000)
        {
            cost = 80;  // red (inside safety bubble)
        } else
        {
            cost = 10;  // blue (outside safety buddle)
        }

        for (j = minY; j < maxY; j++)
        {
            for (i = minX; i < maxX; i++)
            {
                ogData[j * width + i] = cost;
            }
        }
    }

    return (vx_status)VX_SUCCESS;
}

vx_status EStopNode::setEStopArea(int32_t  *eStopGridIdx,
                                  int32_t   minEStopDist,
                                  int32_t   maxEStopDist,
                                  int32_t   minEStopWidth,
                                  int32_t   maxEStopWidth,
                                  int32_t   width,
                                  int32_t   height,
                                  int32_t   xMinRange,
                                  int32_t   yMinRange,
                                  int32_t   gridSize)
{
    int32_t   minEStopXl;
    int32_t   maxEStopXl;
    int32_t   minEStopXr;
    int32_t   maxEStopXr;
    int32_t   minEStopY;
    int32_t   maxEStopY;
    int32_t   hGridSize;
    int32_t   idx;
    int32_t   i;
    int32_t   j;
    int32_t   xl;
    int32_t   xr;
    float     leftSlope;
    float     rightSlope;
    vx_status vxStatus;

    /*
    |<--- max_estop_range --->|
    --------------------------- -> max_estop_distance
    -                         -
     -                      -
      -                    -
       --------------------     -> min_estop_distance = 0
                @               -> camera on the robot
       |<-min_estop_range->|

    eStop range is defined by 4 params.
    In this function, we create a table of grid indices, which is in estop range.
    */

    (void)height;

    hGridSize = gridSize / 2;
    vxStatus  = (vx_status) VX_SUCCESS;

    minEStopXl = (-minEStopWidth/2 - xMinRange) / gridSize;
    minEStopXr = ( minEStopWidth/2 - xMinRange + hGridSize) / gridSize - 1;
    maxEStopXl = (-maxEStopWidth/2 - xMinRange) / gridSize;
    maxEStopXr = ( maxEStopWidth/2 - xMinRange + hGridSize) / gridSize - 1;
    minEStopY  = (minEStopDist  - yMinRange) / gridSize;
    maxEStopY  = (maxEStopDist  - yMinRange + hGridSize) / gridSize - 1;

    // X increment per Y increment for left line
    leftSlope  = 1.0 * (maxEStopXl - minEStopXl) /(maxEStopY - minEStopY);
    // X increment per Y increment for right line
    rightSlope = 1.0 * (maxEStopXr - minEStopXr) / (maxEStopY - minEStopY);

    idx = 0;
    for (j = minEStopY; j <= maxEStopY; j++)
    {
        xl = minEStopXl + (int32_t)((j - minEStopY)*leftSlope);
        xr = minEStopXr + (int32_t)((j - minEStopY)*rightSlope);

        for (i = xl; i <= xr; i++)
        {
            eStopGridIdx[idx++] = j * width + i;
        }
    }

    eStopGridIdx[idx] = -1;

    return vxStatus;
}

vx_status EStopNode::makeEStopDecision(int8_t  *ogData,
                                       int32_t *eStopGridIdx,
                                       int32_t  width,
                                       int32_t  height)
{
    int32_t   numGrids;
    bool      eStop;
    int32_t   i;
    vx_status vxStatus;

    numGrids = width*height;
    eStop    = false;
    vxStatus = (vx_status) VX_SUCCESS;

    for (i = 0; i < numGrids; i++)
    {
        if (eStopGridIdx[i] == -1)
        {
            break;
        }

        if (ogData[eStopGridIdx[i]] != ESTOP_AREA_GRID_VALUE &&
            ogData[eStopGridIdx[i]] != FOV_GRID_VALUE &&
            ogData[eStopGridIdx[i]] != NON_FOV_GRID_VALUE)
        {
            eStop = true;
            break;
        }
    }

    if (eStop == false)
    {
        m_freeRun++;

        if (m_freeRun >= m_minFreeRun)
        {
            m_obsRun     = 0;
            m_eStop.data = false;
        }
    }
    else
    {
        m_obsRun++;

        if (m_obsRun >= m_minObsRun)
        {
            m_freeRun    = 0;
            m_eStop.data = true;
        }
    }

    return vxStatus;
}
