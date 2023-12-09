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
#include <common_msgs/msg/detection2_d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "viz_objdet.h"

using namespace sensor_msgs::msg;
using namespace common_msgs::msg;
using namespace message_filters;
using namespace message_filters::sync_policies;
using namespace cv;

using std::placeholders::_1;
using std::placeholders::_2;

static void sigHandler(int32_t sig)
{
    (void) sig;

    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}

static cv::Mat overlayBoundingBox(cv::Mat &img,
                                  int32_t *box,
                                  int32_t label_id)
{
    // Mat img = Mat(outDataHeight, outDataWidth, CV_8UC3, frame);
    int32_t label_id_mod = label_id % 20;
    Scalar box_color     = Scalar(color_map[label_id_mod][0],
                                  color_map[label_id_mod][1],
                                  color_map[label_id_mod][2]);
    Scalar text_color    = Scalar(244,  35, 232);
    Scalar text_bg_color = Scalar(120, 120, 120);
    std::string label = classnames_coco[label_id];

    // Draw bounding box for the detected object
    Point topleft     = Point(box[0], box[1]);
    Point bottomright = Point(box[2], box[3]);
    rectangle(img, topleft, bottomright, box_color, 3);

    // Draw text with detected class with a background box
    Point center        = Point((box[0] + box[2])/2, (box[1] + box[3])/2);
    Point t_topleft     = center + Point(-5, -10);
    Point t_bottomright = center + Point( 80, 10);
    rectangle(img, t_topleft, t_bottomright, text_bg_color, -1);
    putText(img, label, t_topleft + Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.6, text_color);

    return img;
}

namespace ti_ros2
{
    /**
     * @brief  VizObjDet ROS wrapper class
     */

    class VizObjDet: public rclcpp::Node
    {
        public:
            /**
             * @brief { Overlays the bounding boxes on the image }
             *
             */

            VizObjDet(const std::string &name,
                      const rclcpp::NodeOptions &options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string tensorTopic;
                std::string odMapImgTopic;
                bool        useApproxTimeSync;

                // input topics
                get_parameter_or("rectified_image_topic",   rectImgTopic, std::string(""));
                get_parameter_or("vision_cnn_tensor_topic", tensorTopic,  std::string(""));

                // output topics
                get_parameter_or("vision_cnn_image_topic", odMapImgTopic, std::string(""));

                // time sync policy
                get_parameter_or("approx_time_sync", useApproxTimeSync, false);

                m_odMapImgPub = this->create_publisher<Image>(odMapImgTopic, 10);

                message_filters::Subscriber<Detection2D> tensorSub(this, tensorTopic);
                message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);


                TimeSynchronizer<Detection2D, Image> exactTimeSync(tensorSub, rectImgSub, 10);

                typedef ApproximateTime<Detection2D, Image> approxPloicy;
                Synchronizer<approxPloicy> approxTimeSync(approxPloicy(10), tensorSub, rectImgSub);

                if (!useApproxTimeSync)
                {
                    exactTimeSync.registerCallback(std::bind(&VizObjDet::callback_vizObjDet, this, _1, _2));
                }
                else
                {
                    approxTimeSync.registerCallback(std::bind(&VizObjDet::callback_vizObjDet, this, _1, _2));
                }

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~VizObjDet()
            {
            }

            void callback_vizObjDet(const Detection2D::ConstSharedPtr& detMsg,
                                    const Image::ConstSharedPtr& imagePtr)
            {

                cv_bridge::CvImagePtr cv_outImgPtr;
                cv_outImgPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);

                int32_t box[4];
                int32_t class_id;
                int32_t num_objects = detMsg->num_objects;
                int32_t in_width = detMsg->img_width;
                int32_t in_height = detMsg->img_height;
                int32_t out_width = imagePtr->width;
                int32_t out_height = imagePtr->height;
                float   scale_x = 1.0f;
                float   scale_y = 1.0f;

                if (in_width > 0)
                {
                    scale_x = static_cast<float>(out_width) / in_width;
                }

                if (in_height > 0)
                {
                    scale_y = static_cast<float>(out_height) / in_height;
                }

                for (int32_t i = 0; i < num_objects; i++)
                {
                    box[0] = detMsg->bounding_boxes[i].xmin * scale_x;
                    box[1] = detMsg->bounding_boxes[i].ymin * scale_y;
                    box[2] = detMsg->bounding_boxes[i].xmax * scale_x;
                    box[3] = detMsg->bounding_boxes[i].ymax * scale_y;
                    class_id = detMsg->bounding_boxes[i].label_id;

                    overlayBoundingBox(cv_outImgPtr->image, box, class_id);
                }

		        auto imgPtr = cv_outImgPtr->toImageMsg();
		        auto hdr = &imgPtr->header;

		        hdr->frame_id = "map";

                m_odMapImgPub->publish(*imgPtr);
            }

        private:
            rclcpp::Publisher<Image>::SharedPtr m_odMapImgPub;
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

        auto objDetViz = std::make_shared<ti_ros2::VizObjDet>("viz_objdet", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

