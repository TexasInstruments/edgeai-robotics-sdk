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
#include <common_msgs/msg/bounding_box2_dpva.hpp>
#include <common_msgs/msg/detection2_dpva.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <bits/stdc++.h>
#include "viz_objdet.h"
#include <cmath>
#include <string>
#include <cstdio>

using namespace sensor_msgs::msg;
using namespace common_msgs::msg;
using namespace message_filters;
using namespace message_filters::sync_policies;
using namespace std;
using namespace cv;

using std::placeholders::_1;
using std::placeholders::_2;

static void sigHandler(int32_t sig)
{
    (void) sig;

    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}


std::string num2str(double value)
{
    char buffer[16];
    std::sprintf(buffer, "%4.2f", value);
    std::string strValue = buffer;

    return strValue;
}

/**
 * @brief  VizObjdetRadar ROS node class
 */

class VizObjdetRadar: public rclcpp::Node
{
public:
    /**
     * @brief { Overlays the bounding boxes on the image }
     *
     */

    VizObjdetRadar(const std::string &name,
        const rclcpp::NodeOptions &options): Node(name, options)
    {
        std::string rectImgTopic;
        std::string fusedObjRadarTopic;
        std::string outImgTopic;
        std::vector<long int> vec;

        // input topics
        get_parameter_or("fused_obj_radar_topic", fusedObjRadarTopic, std::string(""));
        get_parameter_or("rectified_image_topic", rectImgTopic, std::string(""));
        get_parameter_or("output_image_topic", outImgTopic, std::string(""));

        get_parameter_or("box_color_rgb", vec, {0,0,0});
        m_box_color = Scalar(vec[0], vec[1], vec[2]) ;
        get_parameter_or("text_color_rgb", vec, {0,0,0});
        m_text_color = Scalar(vec[0], vec[1], vec[2]) ;
        get_parameter_or("text_bg_color_rgb", vec, {0,0,0});
        m_text_bg_color = Scalar(vec[0], vec[1], vec[2]);

        m_pub = this->create_publisher<Image>(outImgTopic, 10);

        message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);
        message_filters::Subscriber<Detection2Dpva> detectSub(this, fusedObjRadarTopic);

        typedef ApproximateTime<Detection2Dpva, Image> approxPolicy;
        Synchronizer<approxPolicy> approxTimeSync(approxPolicy(10), detectSub, rectImgSub);
        approxTimeSync.registerCallback(std::bind(&VizObjdetRadar::callback_VizObjdetRadar, this, _1, _2));

        rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));

    }

    ~VizObjdetRadar() { }

    void callback_VizObjdetRadar(const Detection2Dpva::ConstSharedPtr& detPtr,
                                 const Image::ConstSharedPtr& imagePtr)
    {
        cv_bridge::CvImagePtr cv_outImgPtr;
        cv_outImgPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);
        int32_t class_id;
        int32_t num_objects = detPtr->num_objects;
        BoundingBox2Dpva bbox;
        cv::Scalar box_color;

        for (int32_t i = 0; i < num_objects; i++)
        {
            bbox = detPtr->bounding_boxes[i];
            overlayBoundingBox(
                cv_outImgPtr->image,
                bbox,
                m_box_color,
                m_text_color,
                m_text_bg_color,
                this->get_logger());
        }

        auto imgPtr = cv_outImgPtr->toImageMsg();
        auto hdr = &imgPtr->header;

        hdr->frame_id = "map"; //TODO: update

        m_pub->publish(*imgPtr);
    }

    static cv::Mat overlayBoundingBox(cv::Mat &img,
                                      const BoundingBox2Dpva &bbox,
                                      const cv::Scalar box_color,
                                      const cv::Scalar text_color,
                                      const cv::Scalar text_bg_color,
                                      rclcpp::Logger logger)
    {
        int32_t label_id = bbox.label_id;
        int32_t label_id_mod = label_id % 10;
        int32_t box_width = bbox.xmax - bbox.xmin;
        std::string label_str;
        std::string pos_str;
        std::string vel_str;
        std::string acc_str;

        // RCLCPP_INFO(logger, "overlayBoundingBox(): pva_valid = %d, bb = [%d, %d, %d, %d]",
        //     bbox.pva_valid, bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax);

        if (label_id>0)
        {
            label_str = classnames_coco[label_id];
        }

        if (bbox.pva_valid)
        {
            pos_str = "pos = [" + num2str(bbox.position.x) + ", " + num2str(bbox.position.y) + ", " + num2str(bbox.position.z) + "]";
            vel_str = "vel = [" + num2str(bbox.velocity.x) + ", " + num2str(bbox.velocity.y) + ", " + num2str(bbox.velocity.z) + "]";
            acc_str = "acc = [" + num2str(bbox.acceleration.x) + ", " + num2str(bbox.acceleration.y) + ", " + num2str(bbox.acceleration.z) + "]";
        }
        else
        {
            pos_str = "";
            vel_str = "";
            acc_str = "";
        }

        // Draw bounding box for the detected object
        cv::Point topleft     = Point(bbox.xmin, bbox.ymin);
        cv::Point bottomright = Point(bbox.xmax, bbox.ymax);
        float boxTickness  = 1.5;
        float fontSize     = 0.5;
        float fontTickness = 1.0;
        if (box_width>60) {
            boxTickness  = 2.0;
            fontSize     = 0.7;
            fontTickness = 2.0;
        }
        cv::rectangle(img, topleft, bottomright, box_color, boxTickness);

        // Draw the radar_track projected on the image
        if (bbox.pva_valid)
        {
            cv::Point center(bbox.radar_track_x, bbox.radar_track_y);
            cv::Scalar color(0, 255, 0); // green color
            int radius = 5;
            cv::circle(img, center, radius, color, cv::FILLED);
        }

        // Draw text with detected class
        // cv::Scalar light_gray(192, 192, 192);
        cv::putText(img, label_str, topleft + Point(5, 20), FONT_HERSHEY_SIMPLEX, fontSize, text_color, fontTickness, cv::LINE_AA);
        cv::putText(img, pos_str, topleft + Point(5, 45), FONT_HERSHEY_SIMPLEX, fontSize-0.2, text_color, fontTickness, cv::LINE_AA);
        cv::putText(img, vel_str, topleft + Point(5, 70), FONT_HERSHEY_SIMPLEX, fontSize-0.2, text_color, fontTickness, cv::LINE_AA);
        cv::putText(img, acc_str, topleft + Point(5, 95), FONT_HERSHEY_SIMPLEX, fontSize-0.2, text_color, fontTickness, cv::LINE_AA);

        return img;
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

        auto vizObjdetRadar = std::make_shared<VizObjdetRadar>("viz_objdet_radar", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

