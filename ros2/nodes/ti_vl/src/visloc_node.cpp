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
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <visloc_node.h>
#include <visloc.h>

VisLocNode::VisLocNode(const rclcpp::NodeOptions &options,
                       const std::string         &name):
    Node(name, options)
{
    bool        status;
    vx_status   vxStatus;

    // Initialize the CNN context
    vxStatus = this->init();

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Application Initialization failed.");
        exit(-1);
    }

    m_imgTrans = new ImgTrans(this->create_sub_node("image_transport"));

    // Cache the input and output image sizes
    m_inputImgWidth  = m_cntxt->inputImageWidth;
    m_inputImgHeight = m_cntxt->inputImageHeight;
    m_outImgWidth    = m_cntxt->outImageWidth;
    m_outImgHeight   = m_cntxt->outImageHeight;

    // launch the publisher threads
    auto pubThread = std::thread([this]{publisherThread();});
    pubThread.detach();
}

void VisLocNode::setupInputImgSubscribers()
{
    std::string topicName;
    bool        status;

    // Query the topicname to subscribe to
    status = get_parameter("input_topic", topicName);

    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'input_topic' not found.");
        exit(-1);
    }

    m_sub = new ImgSub(this, topicName);

    if (m_sub == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "new ImgSub() failed.");
        exit(-1);
    }

    m_conObj = m_sub->registerCallback(&VisLocNode::imgCb, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to the topic: %s", topicName.c_str());
}

void VisLocNode::publisherThread()
{
    std::string outImgTopic;
    std::string outPoseTopic;
    std::string mapTopic;

    bool        status;
    bool        latch = true;

    uint32_t    i;
    uint32_t    offset;

    /**************************/
    /* Output image publisher */
    /**************************/
    // Query the topic name to publish the output image.
    status = get_parameter("out_image_topic", outImgTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'out_image_topic' not found.");
        exit(-1);
    }

    // Pre-allocate vectors used for publishing out image data
    m_outImageSize = 3 * m_outImgWidth * m_outImgHeight;
    m_outPubData.data.assign(m_outImageSize, 0);
    m_outPubData.width           = m_outImgWidth;
    m_outPubData.height          = m_outImgHeight;
    m_outPubData.step            = m_outImgWidth * 3;
    m_outPubData.encoding        = "rgb8";
    m_outPubData.header.frame_id = "map";

    // Create the publisher for the output image
    m_outImgPub = m_imgTrans->advertise(outImgTopic, latch);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", outImgTopic.c_str());


    /*************************/
    /* Output pose publisher */
    /*************************/
    status = get_parameter("out_pose_topic", outPoseTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'out_pose_topic' not found.");
        exit(-1);
    }

    m_outPose       = new double[3];
    m_outQuaternion = new double[4];

    // Create the publisher for the output pose
    m_posePub = this->create_publisher<PoseStamped>(outPoseTopic, 1);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", outPoseTopic.c_str());

    /*************************/
    /*   Offline map topic   */
    /*************************/
    status = get_parameter("map_topic", mapTopic);
    if (status == false)
    {
        RCLCPP_ERROR(this->get_logger(), "Config parameter 'map_topic' not found.");
        exit(-1);
    }

    m_mapSize = m_cntxt->poseCalcCreateParams.numMapFeat * 12; // float32 * (x, y, z)
    m_mapPubData.data.assign(m_mapSize, 0);

    // Set map topic's header
    m_mapPubData.header.frame_id = "map";
    m_mapPubData.height          = 1;
    m_mapPubData.width           = m_cntxt->poseCalcCreateParams.numMapFeat;
    m_mapPubData.fields.resize(3);

    // In the map data, z and y are switched
    m_mapPubData.fields[0].name = "x";
    m_mapPubData.fields[1].name = "z";
    m_mapPubData.fields[2].name = "y";

    offset = 0;
    for(i = 0; i < 3; i++)
    {
        m_mapPubData.fields[i].offset   = offset;
        m_mapPubData.fields[i].datatype = 7; // float32
        m_mapPubData.fields[i].count    = 1;
        offset                   += 4;
    }

    m_mapPubData.is_bigendian    = false;
    m_mapPubData.point_step      = offset;
    m_mapPubData.row_step        = m_mapPubData.width * m_mapPubData.point_step;
    m_mapPubData.is_dense        = 1;
    m_mapPubData.header.frame_id = "map";

    // Create the publisher for the offline map
    m_mapPub = this->create_publisher<PointCloud2>(mapTopic, 1);
    RCLCPP_INFO(this->get_logger(), "Created Publisher for topic: %s", mapTopic.c_str());

    // setup input subscribers
    setupInputImgSubscribers();

    processCompleteEvtHdlr();

    m_outImgPub.shutdown();
}

void VisLocNode::imgCb(const Image::ConstSharedPtr& imgPtr)
{
    if (m_cntxt->state == VISLOC_STATE_INIT)
    {
        uint64_t nanoSec = imgPtr->header.stamp.sec * 1e9 +
                           imgPtr->header.stamp.nanosec;

        VISLOC_run(m_cntxt, imgPtr->data.data(), nanoSec);
    }
    else
    {
        // In some cases, m_conObjs.disconnect() is not properly executed
        // and causes error. So disable it.
        //m_conObj.disconnect();
    }

}

void VisLocNode::processCompleteEvtHdlr()
{
    PoseStamped  outPose;

    vx_status    vxStatus = VX_SUCCESS;
    int32_t      i;
    FILE        *fp;

    RCLCPP_INFO(this->get_logger(), "%s Launched.", __FUNCTION__);

    // Set out pose topic's header 
    // frame_id is same as m_mapPubData frame id
    outPose.header.frame_id = "map";

    // Read map file 
    fp = fopen(m_cntxt->poseCalcCreateParams.inputMapFeatPtPath,"rb");

    if (fp == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "input Map file cannot be opened \n");
    }
    else
    {
        int32_t data_read = fread((void *)m_mapPubData.data.data(), 1, m_mapPubData.width*m_mapPubData.point_step, fp);
        float32_t * temp  = (float32_t *)m_mapPubData.data.data();

        // In map data, the sign of Z is flipped
        for (i = 0; i < m_mapPubData.width; i++)
        {
            temp[i*3 + 1] = -temp[i*3 + 1];
        }
    }

    fclose(fp);

    while (m_cntxt->exitOutputThread == false)
    {
        vx_status       vxStatus;

        /* Wait for the data ready semaphore. */
        if (m_cntxt->outputCtrlSem)
        {
            m_cntxt->outputCtrlSem->wait();
        }

        if (m_cntxt->exitOutputThread == true)
        {
            break;
        }

        VISLOC_getOutBuff(m_cntxt,
                         &m_cntxt->vxInputDisplayImage,
                         &m_cntxt->vxOutputImage,
                         &m_cntxt->vxOutputPose,
                         &m_cntxt->outTimestamp);

        // Extract output image data
        vxStatus = CM_extractImageData(m_outPubData.data.data(),
                                       m_cntxt->vxOutputImage,
                                       m_outPubData.width,
                                       m_outPubData.height,
                                       2,
                                       true);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "CM_extractImageData() failed.");
        }

        // Extract pose data
        vxStatus = VISLOC_extractPoseData(m_outPose,
                                          m_outQuaternion,
                                          m_cntxt->vxOutputPose);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "VISLOC_extractPoseData() failed.");
        }

        // Release the output buffers
        VISLOC_releaseOutBuff(m_cntxt);

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            rclcpp::Time time = this->get_clock()->now();

            // Publish output image
            m_outPubData.header.stamp = time;
            m_outImgPub.publish(m_outPubData);

            // Publish pose data
            outPose.header.stamp    = time;
            outPose.header.frame_id = "map";
            
            outPose.pose.position.x    = m_outPose[0];
            outPose.pose.position.y    = m_outPose[1];  
            outPose.pose.position.z    = m_outPose[2];
            outPose.pose.orientation.x = m_outQuaternion[0];
            outPose.pose.orientation.y = m_outQuaternion[1]; 
            outPose.pose.orientation.z = m_outQuaternion[2];
            outPose.pose.orientation.w = m_outQuaternion[3];

            m_posePub->publish(outPose);

            // Publish map data 
            m_mapPubData.header.stamp = time;
            m_mapPub->publish(m_mapPubData);
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s Exiting.", __FUNCTION__);
}

VisLocNode::~VisLocNode()
{
    if (m_outPose)
    {
        delete [] m_outPose;
    }

    if (m_outQuaternion)
    {
        delete [] m_outQuaternion;
    }

    if (m_imgTrans)
    {
        delete m_imgTrans;
    }

    if (m_sub)
    {
        delete m_sub;
    }

    if (m_cntxt)
    {
        delete m_cntxt;
    }
}

void VisLocNode::readParams()
{
    std::string                     str;
    bool                            status;
    int32_t                         tmp;

    // set default parameters
    m_cntxt->ldcSsFactor        = VISLOC_LDC_DS_FACTOR;
    m_cntxt->ldcBlockWidth      = VISLOC_LDC_BLOCK_WIDTH;
    m_cntxt->ldcBlockHeight     = VISLOC_LDC_BLOCK_HEIGHT;
    m_cntxt->ldcPixelPad        = VISLOC_LDC_PIXEL_PAD;
    m_cntxt->vxEvtAppValBase    = 0;

    /* Get input format, which enable flag of LDC depends on */
    get_parameter_or("input_format", tmp, (int32_t)CM_IMG_FORMAT_UYVY);
    m_cntxt->inputFormat   = (uint8_t)tmp;
    if (m_cntxt->inputFormat != CM_IMG_FORMAT_NV12 && m_cntxt->inputFormat != CM_IMG_FORMAT_UYVY)
    {
        RCLCPP_INFO(this->get_logger(), "Input image format is not supported.");
        exit(-1);
    }
    m_cntxt->enableLdcNode = (m_cntxt->inputFormat == CM_IMG_FORMAT_UYVY);

    /* Get LUT file path information. */
    status = get_parameter("lut_file_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'lut_file_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->ldcLutFilePath, CM_MAX_FILE_LEN-1, "%s", str.c_str());

    /* Get TIDL network path information. */
    status = get_parameter("dl_model_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'dl_model_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->dlModelPath, CM_MAX_FILE_LEN-1, "%s", str.c_str());

    /* Get top-down view image path. */
    status = get_parameter("top_view_img_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'top_view_img_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->poseVizCreateParams.topViewImgPath, CM_MAX_FILE_LEN-1, "%s", str.c_str());

    /* Get input voxel info path. */
    status = get_parameter("input_voxel_info_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'input_voxel_info_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->poseCalcCreateParams.inputVoxelInfoPath, CM_MAX_FILE_LEN-1, "%s", str.c_str());

    /* Get input map feature points path. */
    status = get_parameter("input_map_feat_pt_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'input_map_feat_pt_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->poseCalcCreateParams.inputMapFeatPtPath, CM_MAX_FILE_LEN-1, "%s", str.c_str());

    /* Get input map feature descriptor path. */
    status = get_parameter("input_map_feat_desc_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'input_map_feat_desc_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->poseCalcCreateParams.inputMapFeatDescPath, CM_MAX_FILE_LEN-1, "%s", str.c_str());

    /* Get input map upsampling weigth path. */
    status = get_parameter("input_upsample_wt_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'input_upsample_wt_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->poseCalcCreateParams.inputUpsampleWtPath, CM_MAX_FILE_LEN-1, "%s", str.c_str());

    /* Get input map upsampling bias path. */
    status = get_parameter("input_upsample_bias_path", str);
    if (status == false)
    {
        RCLCPP_INFO(this->get_logger(), "Config parameter 'input_upsample_bias_path' not found.");
        exit(-1);
    }

    snprintf(m_cntxt->poseCalcCreateParams.inputUpsampleBiasPath, CM_MAX_LINE_LEN-1, "%s", str.c_str());

    /* Get input image width information. */
    get_parameter_or("width", m_cntxt->inputImageWidth, VISLOC_DEFAULT_IMAGE_WIDTH);

    /* Get input image height information. */
    get_parameter_or("height", m_cntxt->inputImageHeight, VISLOC_DEFAULT_IMAGE_HEIGHT);

    /* Get input TIDL output image width information. */
    get_parameter_or("out_width", m_cntxt->outImageWidth, VISLOC_DEFAULT_IMAGE_WIDTH);

    /* Get input TIDL output image height information. */
    get_parameter_or("out_height", m_cntxt->outImageHeight, VISLOC_DEFAULT_IMAGE_HEIGHT);

    /* Get number of voxels */
    get_parameter_or("num_voxels", m_cntxt->poseCalcCreateParams.numVoxels, 111556);

    /* Get number of features in a map */
    get_parameter_or("num_map_feat", m_cntxt->poseCalcCreateParams.numMapFeat, 13718);

    /* Get max number of frame features */
    get_parameter_or("max_map_feat", m_cntxt->poseCalcCreateParams.maxMapFeat, 5000);

    /* Get max number of map features */
    get_parameter_or("max_frame_feat", m_cntxt->poseCalcCreateParams.maxFrameFeat, 1000); 

    /* Get score threshold for picking good points */
    get_parameter_or("score_th", m_cntxt->poseCalcCreateParams.scoreTh, 128);  


    /* Get initial pose estimate used in campling the map data */
    std::vector<double>   pose = {0.0, 0.0, 0.0};
    rclcpp::Parameter poseParam;
    get_parameter_or("init_pose_est",
                           poseParam,
                           rclcpp::Parameter("init_pose_est", pose));

    pose = poseParam.as_double_array();

    m_cntxt->poseCalcCreateParams.initPoseEst[0] = pose[0];
    m_cntxt->poseCalcCreateParams.initPoseEst[1] = pose[1];
    m_cntxt->poseCalcCreateParams.initPoseEst[2] = pose[2];

    /* Calculate pose for every frame */
    m_cntxt->poseCalcCreateParams.skipFlag = 0;

    /* Get upsampling filter scale in power of 2 */
    get_parameter_or("filter_scale_pw2", m_cntxt->poseCalcCreateParams.filterScalePw2, 0);

    /* Get output descriptor (at original resolution) scale in power of 2 */
    get_parameter_or("hi_res_desc_scale_pw2", m_cntxt->poseCalcCreateParams.hiResDescScalePw2, 0);

    /* Get interactive mode flag information. */
    get_parameter_or("is_interactive", tmp, 0);
    m_cntxt->is_interactive = (uint8_t)tmp;

    /* Get pipeline depth information. */
    get_parameter_or("pipeline_depth", tmp, 1);
    m_cntxt->pipelineDepth = (uint8_t)tmp;

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
    logSetLevel(static_cast<LogLevel>(tmp));

    return;
}

vx_status VisLocNode::init()
{
    PTK_CRT   ptkConfig;
    vx_status vxStatus = VX_SUCCESS;

    /* Initialize PTK library. */
    ptkConfig.exit     = exit;
    ptkConfig.printf   = printf;
    ptkConfig.time     = NULL;

    PTK_init(&ptkConfig);

    m_cntxt = new VISLOC_Context();

    if (m_cntxt == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "VISLOC_Context() failed.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Read the node parameters. */
        readParams();

        /* Initialize the Application context */
        vxStatus = VISLOC_init(m_cntxt);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "VISLOC_init() failed.");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Launch processing threads. */
        VISLOC_launchProcThreads(m_cntxt);
    }

    return vxStatus;
}

void VisLocNode::sigHandler(int32_t  sig)
{
    (void)sig;

    if (m_cntxt)
    {
        VISLOC_intSigHandler(m_cntxt);
    }
}
