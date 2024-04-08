/*
 *
 * Copyright (c) 2022 Texas Instruments Incorporated
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
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include <common_msgs/msg/human_pose.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "viz_humanpose.h"

using namespace sensor_msgs::msg;
using namespace common_msgs::msg;
using namespace message_filters;
using namespace message_filters::sync_policies;
using namespace cv;
using namespace std;

using std::placeholders::_1;
using std::placeholders::_2;

static void sigHandler(int32_t sig)
{
    (void) sig;

    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}

static cv::Mat overlayBoundingBox(cv::Mat &img,
                                  int32_t  *box,
                                  vector<int> color_map,
                                  int32_t det_label,
                                  int32_t img_width,
                                  int32_t img_height)
{
    Point p1(box[0], box[1]);
    Point p2(box[2], box[3]);

    Scalar color = Scalar(color_map[0],
                          color_map[1],
                          color_map[2]);

    float scale = abs((box[2] - box[0]) * (box[3] - box[1]))\
                         / float((img_width * img_height));

    rectangle(img, p1, p2, Scalar(color_map[0], color_map[1], color_map[2]), 2);
    string id = "Id : " + to_string(det_label);
    putText(img, id, Point(box[0] + 5, box[1] + 15),
                    FONT_HERSHEY_DUPLEX, 2.5 * scale, Scalar(color_map[0], color_map[1],
                    color_map[2]), 2);

    return img;
}

std::vector<float> extractKeypoints(vector<float>  data,
                                    int32_t        index,
                                    int32_t        num_objects)
{
    int32_t num_kpts = data.size()/num_objects;
    std::vector<float> keypoints;

    for (int32_t i = 0; i < num_kpts; i++)
    {
        keypoints.push_back(data[index * num_kpts + i]);
    }
    return keypoints;
}

static cv::Mat overlayKeypoints(cv::Mat               &img,
                                vector<float>         keypoints,
                                vector<vector<int>>   pose_kpt_color,
                                float                 scale_x,
                                float                 scale_y)
{
    for(int kid = 0; kid < 17; kid++)
    {
        int r = pose_kpt_color[kid][0];
        int g = pose_kpt_color[kid][1];
        int b = pose_kpt_color[kid][2];

        int x_coord = keypoints[steps * kid]     * scale_x;
        int y_coord = keypoints[steps * kid + 1] * scale_y;
        float conf  = keypoints[steps * kid + 2];

        if(conf > 0.5)
        {
            circle(img, Point(x_coord, y_coord), radius, Scalar(r, g, b), -1);
        }
    }
    return img;
}

static cv::Mat overlayLimbs(cv::Mat               &img,
                            vector<float>         keypoints,
                            vector<vector<int>>   pose_limb_color,
                            float                 scale_x,
                            float                 scale_y)
{
    for(uint64_t sk_id = 0; sk_id < skeleton.size(); sk_id++)
    {
        int r = pose_limb_color[sk_id][0];
        int g = pose_limb_color[sk_id][1];
        int b = pose_limb_color[sk_id][2];

        int p11 = keypoints[(skeleton[sk_id][0] - 1) * steps]     * scale_x;
        int p12 = keypoints[(skeleton[sk_id][0] - 1) * steps + 1] * scale_y;
        Point pos1 = Point(p11, p12);

        int p21 = keypoints[(skeleton[sk_id][1] - 1) * steps]     * scale_x;
        int p22 = keypoints[(skeleton[sk_id][1] - 1) * steps + 1] * scale_y;
        Point pos2 = Point(p21, p22);

        float conf1 = keypoints[(skeleton[sk_id][0] - 1) * steps + 2];
        float conf2 = keypoints[(skeleton[sk_id][1] - 1) * steps + 2];

        if(conf1 > 0.5 && conf2 > 0.5)
        {
            line(img, pos1, pos2, Scalar(r, g, b), 2, LINE_AA);
        }
    }
    return img;
}


namespace ti_ros2
{
    /**
     * @brief  VizHumanPose ROS warpper class
     */

    class VizHumanPose: public rclcpp::Node
    {
        public:
            /**
             * @brief { Overlays the bounding boxes on the image }
             *
             */

            VizHumanPose(const std::string &name,
                      const rclcpp::NodeOptions &options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string tensorTopic;
                std::string HumanPoseimgTopic;
                bool useApproxTimeSync;

                // input topics
                get_parameter_or("rectified_image_topic",   rectImgTopic, std::string(""));
                get_parameter_or("vision_cnn_tensor_topic", tensorTopic,  std::string(""));

                // output topics
                get_parameter_or("vision_cnn_image_topic", HumanPoseimgTopic, std::string(""));
                get_parameter_or("approx_time_sync", useApproxTimeSync, false);

                m_humanposeImgPub = this->create_publisher<Image>(HumanPoseimgTopic, 10);

                message_filters::Subscriber<HumanPose> tensorSub(this, tensorTopic);
                message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);

                TimeSynchronizer<HumanPose, Image> exactTimeSync(tensorSub, rectImgSub, 10);

                typedef ApproximateTime<HumanPose, Image> approxPloicy;
                Synchronizer<approxPloicy> approxTimeSync(approxPloicy(10), tensorSub, rectImgSub);

                if (!useApproxTimeSync)
                {
                    exactTimeSync.registerCallback(std::bind(&VizHumanPose::callback_vizHumanPoseDet, this, _1, _2));
                }
                else
                {
                    approxTimeSync.registerCallback(std::bind(&VizHumanPose::callback_vizHumanPoseDet, this, _1, _2));

                }

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~VizHumanPose()
            {
            }

            void callback_vizHumanPoseDet(const HumanPose::ConstSharedPtr& poseMsg,
                                          const Image::ConstSharedPtr& imagePtr)
            {
                cv_bridge::CvImagePtr cv_outImgPtr;
                cv_outImgPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);

                float scale_x = static_cast<float>(imagePtr->width)/static_cast<float>(poseMsg->img_width);
                float scale_y = static_cast<float>(imagePtr->height)/static_cast<float>(poseMsg->img_height);

                int32_t num_objects = poseMsg->num_objects;
                int32_t label_id, score;
                int32_t box[4];


                for (int32_t i = 0; i < num_objects; i++)
                {

                    box[0] = poseMsg->bounding_boxes[i].xmin * scale_x;
                    box[1] = poseMsg->bounding_boxes[i].ymin * scale_y;
                    box[2] = poseMsg->bounding_boxes[i].xmax * scale_x;
                    box[3] = poseMsg->bounding_boxes[i].ymax * scale_y;
                    score  = poseMsg->bounding_boxes[i].confidence;
                    label_id = poseMsg->bounding_boxes[i].label_id;

                    if (label_id < 10)
                    {
                        vector<int> color_map = CLASS_COLOR_MAP[0];
                        overlayBoundingBox(cv_outImgPtr->image, box, color_map, label_id, imagePtr->width, imagePtr->height);
                    }

                    std::vector<float> object_kps = extractKeypoints(poseMsg->keypoint, i, num_objects);

                    overlayKeypoints(cv_outImgPtr->image, object_kps, pose_kpt_color, scale_x, scale_y);

                    overlayLimbs(cv_outImgPtr->image, object_kps, pose_limb_color, scale_x, scale_y);

                }
                auto imgPtr = cv_outImgPtr->toImageMsg();
                auto hdr = &imgPtr->header;

                hdr->frame_id = "map";

                m_humanposeImgPub->publish(*imgPtr);
            }
        private:
            rclcpp::Publisher<Image>::SharedPtr m_humanposeImgPub;
    };
}
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

        auto objDetViz = std::make_shared<ti_ros2::VizHumanPose>("viz_objdet", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}
