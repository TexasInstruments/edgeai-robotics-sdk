/*
 *  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <common_msgs/msg/bounding_box2_dp.hpp>
#include <common_msgs/msg/detection2_dp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <bits/stdc++.h>
#include "viz_objdet.h"
#include <cmath>

using namespace sensor_msgs::msg;
using namespace common_msgs::msg;
using namespace message_filters;
using namespace message_filters::sync_policies;
using namespace std;
using namespace cv;

using std::placeholders::_1;
using std::placeholders::_2;

#define NUM_FRAC_BITS 4

static void sigHandler(int32_t sig)
{
    (void) sig;

    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}

// calculate range in X-Z plane in camera frame
static float point2Range(const geometry_msgs::msg::Point32& point)
{
    float range = sqrt(point.x*point.x + point.z*point.z);
    return range;
}

static cv::Mat overlayBoundingBox(cv::Mat &img,
                                  const BoundingBox2Dp &bbox,
                                  const cv::Scalar box_color,
                                  const cv::Scalar text_color,
                                  const cv::Scalar text_bg_color)
{
    int32_t label_id = bbox.label_id;
    int32_t label_id_mod = label_id % 10;
    int32_t box_width = bbox.xmax - bbox.xmin;
    float range = point2Range(bbox.position);

    std::string label_str = classnames_coco[label_id];
    std::string position_str = std::to_string(round(range*100)/100).substr(0,4) + std::string(" m");

    // Draw bounding box for the detected object
    Point topleft     = Point(bbox.xmin, bbox.ymin);
    Point bottomright = Point(bbox.xmax, bbox.ymax);
    float boxTickness  = 1.5;
    float fontSize     = 0.5;
    float fontTickness = 1.0;
    if (box_width>60) {
        boxTickness  = 2.0;
        fontSize     = 0.7;
        fontTickness = 2.0;
    }
    rectangle(img, topleft, bottomright, box_color, boxTickness);

    // Draw a background box
    // Point center        = Point((bbox.xmin + bbox.xmax)/2, (bbox.ymin + bbox.ymax)/2);
    // Point t_topleft     = center + Point(-60, -25);
    // Point t_bottomright = center + Point( 60, 25);
    // rectangle(img, t_topleft, t_bottomright, text_bg_color, -1);

    // Draw text with detected class
    putText(img, label_str, topleft + Point(5, 20), FONT_HERSHEY_SIMPLEX, fontSize, text_color, fontTickness);
    // if (range < 10)
    // {
        putText(img, position_str, topleft + Point(5, 45), FONT_HERSHEY_SIMPLEX, fontSize-0.1, text_color, fontTickness);
    // }

    return img;
}

/**
 * @brief  VizObjdetRange ROS node class
 */

class VizObjdetRange: public rclcpp::Node
{
public:
    /**
     * @brief { Overlays the bounding boxes on the image }
     *
     */

    VizObjdetRange(const std::string &name,
        const rclcpp::NodeOptions &options): Node(name, options)
    {
        // std::string rawDisparityTopic;
        std::string rectImgTopic;
        std::string fusedObjRangeTopic;
        std::string outImgTopic;
        std::vector<long int> vec;

        // input topics
        // private_nh->param("disparity_topic", rawDisparityTopic, std::string(""));
        get_parameter_or("fused_obj_range_topic", fusedObjRangeTopic, std::string(""));
        get_parameter_or("rectified_image_topic", rectImgTopic, std::string(""));
        get_parameter_or("output_image_topic", outImgTopic, std::string(""));

        get_parameter_or("box_color_rgb", vec, {0,0,0});
        m_box_color = Scalar(vec[0], vec[1], vec[2]) ;
        get_parameter_or("patch_color_rgb", vec, {0,0,0});
        m_patch_color = Scalar(vec[0], vec[1], vec[2]) ;
        get_parameter_or("text_color_rgb", vec, {0,0,0});
        m_text_color = Scalar(vec[0], vec[1], vec[2]) ;
        get_parameter_or("text_bg_color_rgb", vec, {0,0,0});
        m_text_bg_color = Scalar(vec[0], vec[1], vec[2]);

        m_pub = this->create_publisher<Image>(outImgTopic, 10);

        message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);
        message_filters::Subscriber<Detection2Dp> detectSub(this, fusedObjRangeTopic);

        typedef ApproximateTime<Detection2Dp, Image> approxPolicy;
        Synchronizer<approxPolicy> approxTimeSync(approxPolicy(10), detectSub, rectImgSub);
        approxTimeSync.registerCallback(std::bind(&VizObjdetRange::callback_VizObjdetRange, this, _1, _2));

        rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));

    }

    ~VizObjdetRange() { }

    void callback_VizObjdetRange(const Detection2Dp::ConstSharedPtr& detPtr,
                                 const Image::ConstSharedPtr& imagePtr)
    {
        cv_bridge::CvImagePtr cv_outImgPtr;
        cv_outImgPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);
        int32_t class_id;
        int32_t num_objects = detPtr->num_objects;
        BoundingBox2Dp bbox;
        cv::Scalar box_color;

        for (int32_t i = 0; i < num_objects; i++)
        {
            bbox = detPtr->bounding_boxes[i];
            box_color = (bbox.label_id==92) ? m_patch_color : m_box_color;

            overlayBoundingBox(cv_outImgPtr->image,
                               bbox,
                               box_color,
                               m_text_color,
                               m_text_bg_color);
        }

        auto imgPtr = cv_outImgPtr->toImageMsg();
        auto hdr = &imgPtr->header;

        hdr->frame_id = "map";

        m_pub->publish(*imgPtr);
    }

private:
    rclcpp::Publisher<Image>::SharedPtr m_pub;
    cv::Scalar m_box_color;
    cv::Scalar m_patch_color;
    cv::Scalar m_text_color;
    cv::Scalar m_text_bg_color;
};

/**
 * Main
 */
int main(int argc, char **argv)
{
    try
    {
        rclcpp::InitOptions initOptions{};
        rclcpp::NodeOptions nodeOptions{};

        rclcpp::init(argc, argv, initOptions);

        signal(SIGINT, sigHandler);

        nodeOptions.allow_undeclared_parameters(true);
        nodeOptions.automatically_declare_parameters_from_overrides(true);
        nodeOptions.use_intra_process_comms(false);

        auto vizObjdetRange = std::make_shared<VizObjdetRange>("viz_objdet_range", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

