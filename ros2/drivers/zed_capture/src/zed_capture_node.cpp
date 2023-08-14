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

#include "zed_capture_node.h"

static std::shared_ptr<ZedCameraNode>  zed_ros_node = nullptr;

static void sigHandler(int32_t sig)
{
    (void) sig;
    std::exit(EXIT_SUCCESS);
}

/**
 * @brief ZED camera ROS node
 *
 * @param [in] resolution  Resolution
 * @param [in] frame_rate  Frame rate
 */
ZedCameraNode::ZedCameraNode(const std::string&         name,
                             const rclcpp::NodeOptions& options):
    Node(name, options)
{
    // parse ROS parameters
    get_parameter_or("camera_mode", camera_mode_, std::string("HD"));
    get_parameter_or("frame_rate", frame_rate_, 15.0);
    get_parameter_or("frame_id_left", frame_id_left_, std::string("left_camera"));
    get_parameter_or("frame_id_right", frame_id_right_, std::string("right_camera"));
    get_parameter_or("device_name", device_name_, std::string("/dev/video1"));
    get_parameter_or("encoding", encoding_, std::string("yuv422"));
    get_parameter_or("image_topic_left",  image_topic_left_,  std::string("camera/left/image_raw"));
    get_parameter_or("image_topic_right", image_topic_right_, std::string("camera/right/image_raw"));
    get_parameter_or("camera_info_topic_left",  camera_info_topic_left_,  std::string("camera/left/camera_info"));
    get_parameter_or("camera_info_topic_right", camera_info_topic_right_, std::string("camera/right/camera_info"));
    get_parameter_or("camera_info_left_yaml", camera_info_left_yaml_, std::string(""));
    get_parameter_or("camera_info_right_yaml", camera_info_right_yaml_, std::string(""));

    // validate frame rate and update if necessary
    validateFrameRate(camera_mode_, frame_rate_);

    RCLCPP_INFO(this->get_logger(), "Initialize the ZED camera");
    StereoCamera zed(device_name_, camera_mode_, frame_rate_, encoding_);

    // setup publisher
    auto imgTrans    = new ImgTrans(static_cast<rclcpp::Node::SharedPtr>(this));
    auto pub_image_left  = imgTrans->advertise(image_topic_left_, 1);
    auto pub_image_right = imgTrans->advertise(image_topic_right_, 1);

    auto pub_caminfo_left  = this->create_publisher<CameraInfo>(camera_info_topic_left_, 1);
    auto pub_caminfo_right = this->create_publisher<CameraInfo>(camera_info_topic_right_, 1);

    // populate camera_info
    RCLCPP_INFO(this->get_logger(), "Loading camera info from yaml files");

    CameraInfoManager caminfo_left_manager(this, "camera/left", camera_info_left_yaml_);
    CameraInfoManager caminfo_right_manager(this, "camera/right", camera_info_right_yaml_);

    auto caminfo_left             = caminfo_left_manager.getCameraInfo();
    auto caminfo_right            = caminfo_right_manager.getCameraInfo();
    caminfo_left.header.frame_id  = frame_id_left_;
    caminfo_right.header.frame_id = frame_id_right_;

    // capture and publish images
    cv::Mat   img_left;
    cv::Mat   img_right;
    rclcpp::Rate framerate(frame_rate_);

    while (true)
    {
        if (!zed.getImages(img_left, img_right))
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Cannot find the ZED camera");
        }
        else
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Successfully found the ZED camera");
        }

        rclcpp::Time now = this->get_clock()->now();

        publishImage(pub_image_left, img_left, "left_frame", now);
        publishImage(pub_image_right, img_right, "right_frame", now);
        publishCamInfo(pub_caminfo_left, caminfo_left, now);
        publishCamInfo(pub_caminfo_right, caminfo_right, now);

        framerate.sleep();
    }
}

ZedCameraNode::~ZedCameraNode() { }

/**
 * @brief Publish the camera_info
 *
 * @param [in] pub_cam_info  camera_info publisher
 * @param [in] cam_info_msg  camera_info message
 * @param [in] tstamp        timestamp
 */
void ZedCameraNode::publishCamInfo(const rclcpp::Publisher<CameraInfo>::SharedPtr pub_cam_info,
                                   CameraInfo &cam_info_msg,
                                   rclcpp::Time tstamp)
{
    cam_info_msg.header.stamp = tstamp;
    pub_cam_info->publish(cam_info_msg);
}

/**
 * @brief Publish the image
 *
 * @param [in]  pub_img       Image publisher
 * @param [in]  img           Image message
 * @param [in]  img_frame_id  Image frame identifier
 * @param [in]  tstamp        Timestamp
 * @param [in]  encoding      image_transport encoding method
 */
void ZedCameraNode::publishImage(const ImgPub &pub_img,
                                 const cv::Mat &img,
                                 const std::string &img_frame_id,
                                 rclcpp::Time tstamp)
{
    cv_bridge::CvImage cv_img;
    cv_img.image           = img;
    cv_img.encoding        = encoding_;
    cv_img.header.frame_id = img_frame_id;
    cv_img.header.stamp    = tstamp;
    pub_img.publish(cv_img.toImageMsg());
}

/**
 * @brief Validate the frame rate and update it if necessary
 *
 * @param [in] camera_mode Camera mode string
 * @param [in] frame_rate  Camera frame rate
 */
void ZedCameraNode::validateFrameRate(std::string camera_mode, double& frame_rate)
{
    double max_frame_rate;

    if(camera_mode.compare("2K")==0)
        max_frame_rate= 15;
    else if(camera_mode.compare("FHD")==0)
        max_frame_rate= 30;
    else if(camera_mode.compare("FHD2")==0)
        max_frame_rate= 30;
    else if(camera_mode.compare("HD")==0)
        max_frame_rate= 60;
    else if(camera_mode.compare("HD2")==0)
        max_frame_rate= 30;
    else if(camera_mode.compare("VGA")==0)
        max_frame_rate= 100;
    else
        RCLCPP_FATAL(this->get_logger(), "Unknow camera_mode passed");

    if (frame_rate > max_frame_rate)
    {
        RCLCPP_WARN(this->get_logger(), "frame_rate = %f exceeds the max rate with %s, set to %f",
                 frame_rate, camera_mode.c_str(), max_frame_rate);
        frame_rate = max_frame_rate;
    }
}

int main(int argc, char** argv)
{
    try
    {
        rclcpp::InitOptions initOptions{};
        rclcpp::NodeOptions nodeOptions{};

        /* Prevent the RCLCPP signal handler binding. */
        initOptions.shutdown_on_signal = false;

        rclcpp::init(argc, argv, initOptions);

        nodeOptions.allow_undeclared_parameters(true);
        nodeOptions.automatically_declare_parameters_from_overrides(true);

        signal(SIGINT, sigHandler);

        zed_ros_node = std::make_shared<ZedCameraNode>("zed_camera", nodeOptions);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

