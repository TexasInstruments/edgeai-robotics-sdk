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
#include <sensor_msgs/msg/image.h>
#include <cv_bridge/cv_bridge.h>
#include <common_msgs/msg/object_pos3_d.hpp>
#include <common_msgs/msg/detection3_d.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.h>

using namespace sensor_msgs::msg;
using namespace message_filters;
using namespace common_msgs::msg;
using namespace geometry_msgs::msg;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// OpenCV uses BGR format
static const uint8_t color_map[3] = {128, 64, 128};

static void sigHandler(int32_t sig)
{
    (void) sig;
    rclcpp::shutdown();
}

namespace ti_ros2
{
    /**
     * @brief  SDE ROS warpper class
     */

    class VizEStop: public rclcpp::Node
    {
        public:
            VizEStop(const std::string         &name,
                     const rclcpp::NodeOptions &options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string bbTopic;
                std::string ssTensorTopic;
                std::string bbImgTopic;

                // input and tensor sizes
                get_parameter_or("width",         m_width,        1280);
                get_parameter_or("height",        m_height,       720);
                get_parameter_or("tensor_width",  m_tensorWidth,  768);
                get_parameter_or("tensor_height", m_tensorHeight, 432);
                get_parameter_or("show_ground",   m_showGround,   0);

                // input topics
                get_parameter_or("rectified_image_topic",   rectImgTopic,  std::string(""));
                get_parameter_or("bounding_box_topic",      bbTopic,       std::string(""));
                get_parameter_or("semseg_cnn_tensor_topic", ssTensorTopic, std::string(""));

                // output topics
                get_parameter_or("bounding_box_image_topic", bbImgTopic, std::string(""));

                m_bbImgPub = this->create_publisher<Image>(bbImgTopic, 10);

                m_safetyPub = this->create_publisher<PolygonStamped>("/estop/safety_bubble", 1);

                message_filters::Subscriber<Detection3D> bbSub(this,  bbTopic);
                message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);
                message_filters::Subscriber<Image> ssTensorSub(this, ssTensorTopic);

                TimeSynchronizer<Detection3D, Image, Image> sync(bbSub, rectImgSub, ssTensorSub, 10);
                sync.registerCallback(std::bind(&VizEStop::callback_vizEStop, this, _1, _2, _3));

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~VizEStop()
            {
            }

            void callback_vizEStop(const Detection3D::ConstSharedPtr& objMsg,
                                   const Image::ConstSharedPtr& imagePtr,
                                   const Image::ConstSharedPtr& tensorPtr)
            {
                // create and publish bounding box images
                publishBBImage(objMsg, imagePtr, tensorPtr);

                // create polygons for zones
                int numPoints    = 12; // number of points for polygon
                float sbDistance = 5.0;
                PolygonStamped safetyBubble;

                safetyBubble.header.frame_id = "map";

                for (int i = 0; i < numPoints; ++i)
                {
                      double angle = i * (360.0/(double) numPoints)*M_PI/180.0;

                      Point32 stop_point;
                      stop_point.z = 0.0;
                      stop_point.x = cos(angle) * sbDistance;
                      stop_point.y = sin(angle) * sbDistance;
                      safetyBubble.polygon.points.push_back(stop_point);
                }

                m_safetyPub->publish(safetyBubble);
            }

            void publishBBImage(const Detection3D::ConstSharedPtr& objMsg,
                                   const Image::ConstSharedPtr& imagePtr,
                                   const Image::ConstSharedPtr& tensorPtr)
            {
                int32_t i, j, y, x;
                int32_t numObjs;
                char    strDistance[50];

                uint8_t classId;

                // blending factor
                float   bf        = 0.5;
                int32_t stride    = m_width * 3;

                float   horScale  = m_tensorWidth *1.0/m_width;
                float   verScale  = m_tensorHeight*1.0/m_height;

                int     fontface  = cv::FONT_HERSHEY_SIMPLEX;
                int     thickness = 1;
                int     baseline  = 0;
                double  scale     = 0.6;

                cv::Scalar lineColor;
                cv::Point  pos;
                cv::Size   text;

                Detection3D   detectionMsg = *objMsg;
                ObjectPos3D   objPos;
                ObjectPos3D * objPosArr;

                cv_bridge::CvImagePtr cv_bbPtr;
                cv_bbPtr = cv_bridge::toCvCopy(imagePtr,  sensor_msgs::image_encodings::RGB8);

                numObjs = detectionMsg.num_objects;
                objPosArr = (ObjectPos3D *) detectionMsg.obj_pos3d.data();

                /* draw bounding boxes on detected objects */
                for (i = 0; i < numObjs; i++)
                {
                    objPos = objPosArr[i];

#if 1
                    // Color based on distance
                    if (objPos.front_depth <= 5000)
                    {
                        lineColor = CV_RGB(0, 0, 255);
                    } else
                    {
                        lineColor = CV_RGB(255, 0, 0);
                    }
#else
                    // Color based on class
                    if (objPos.classId == 11)
                    {
                        lineColor = CV_RGB(255, 0, 0);
                    }
                    else if (objPos.classId == 12)
                    {
                        lineColor = CV_RGB(0, 255, 0);
                    }
                    else if (objPos.classId == 13)
                    {
                        lineColor = CV_RGB(0, 0, 255);
                    }
                    else if (objPos.classId == 14)
                    {
                        lineColor = CV_RGB(0, 255, 255);
                    }
                    else if (objPos.classId == 15)
                    {
                        lineColor = CV_RGB(255, 255, 0);
                    }
#endif

                    // Front box
                    // P3 P4
                    // P1 P2
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf1x, objPos.pf1y), cv::Point(objPos.pf2x, objPos.pf2y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf2x, objPos.pf2y), cv::Point(objPos.pf3x, objPos.pf3y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf3x, objPos.pf3y), cv::Point(objPos.pf4x, objPos.pf4y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf4x, objPos.pf4y), cv::Point(objPos.pf1x, objPos.pf1y), lineColor, 2);

                    // Front box
                    // P3 P4
                    // P1 P2
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pr1x, objPos.pr1y), cv::Point(objPos.pr2x, objPos.pr2y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pr2x, objPos.pr2y), cv::Point(objPos.pr3x, objPos.pr3y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pr3x, objPos.pr3y), cv::Point(objPos.pr4x, objPos.pr4y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pr4x, objPos.pr4y), cv::Point(objPos.pr1x, objPos.pr1y), lineColor, 2);

                    // pf1 - pr1, pf2 - pr2, pf3 - pr3, pf4 - pr4
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf1x, objPos.pf1y), cv::Point(objPos.pr1x, objPos.pr1y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf2x, objPos.pf2y), cv::Point(objPos.pr2x, objPos.pr2y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf3x, objPos.pf3y), cv::Point(objPos.pr3x, objPos.pr3y), lineColor, 2);
                    cv::line(cv_bbPtr->image, cv::Point(objPos.pf4x, objPos.pf4y), cv::Point(objPos.pr4x, objPos.pr4y), lineColor, 2);

                    sprintf(strDistance, "%.1fm", float(objPos.front_depth / 1000));

                    text = cv::getTextSize(strDistance, fontface, scale, thickness, &baseline);
                    pos  = cv::Point((objPos.pf1x+objPos.pf2x)/2 - 25, (objPos.pf1y+objPos.pf4y)/2);
                    cv::rectangle(cv_bbPtr->image, pos + cv::Point(0, baseline), pos + cv::Point(text.width, -text.height), CV_RGB(0,0,0), cv::FILLED);
                    cv::putText(cv_bbPtr->image, strDistance, pos, fontface, scale, CV_RGB(255,255,255), thickness);
                    /*
                    cv::putText(cv_bbPtr->image, strDistance, cv::Point((objPos.pf1x+objPos.pf2x)/2 - 25, (objPos.pf1y+objPos.pf4y)/2),
                                fontface, scale, CV_RGB(255,255,255), thickness);
                    */
                }

                if (m_showGround)
                {
                    for (j = 0; j < m_height; j++)
                    {
                        // y pos in tensor
                        y = (j * verScale + 0.5);
                        for (i = 0; i < m_width; i++)
                        {
                            // x pos in tensor
                            x = (i * horScale + 0.5);
                            classId =  tensorPtr->data[y*m_tensorWidth + x];

                            // road and sidewalk
                            if (classId == 0 || classId == 1)
                            {
                                cv_bbPtr->image.data[j*stride + i*3]     =
                                    (1-bf)*cv_bbPtr->image.data[j*stride + i*3]     + bf*color_map[0];
                                cv_bbPtr->image.data[j*stride + i*3 + 1] =
                                    (1-bf)*cv_bbPtr->image.data[j*stride + i*3 + 1] + bf*color_map[1];
                                cv_bbPtr->image.data[j*stride + i*3 + 2] =
                                    (1-bf)*cv_bbPtr->image.data[j*stride + i*3 + 2] + bf*color_map[2];
                            }
                        }
                    }
                }

                auto imgPtr = cv_bbPtr->toImageMsg();
                auto hdr = &imgPtr->header;

                hdr->frame_id = "map";

                m_bbImgPub->publish(*imgPtr);
            }


        private:
            rclcpp::Publisher<Image>::SharedPtr             m_bbImgPub;
            rclcpp::Publisher<PolygonStamped>::SharedPtr    m_safetyPub;
            int32_t                                         m_width;
            int32_t                                         m_height;
            int32_t                                         m_tensorWidth;
            int32_t                                         m_tensorHeight;
            int32_t                                         m_showGround;
    };
}

static ti_ros2::VizEStop   *estopViz = nullptr;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    try
    {
        rclcpp::InitOptions initOptions{};
        rclcpp::NodeOptions nodeOptions{};

        /* Prevent the RCLCPP signal handler binding. */
        initOptions.shutdown_on_signal = false;

        rclcpp::init(argc, argv, initOptions);

        signal(SIGINT, sigHandler);

        nodeOptions.allow_undeclared_parameters(true);
        nodeOptions.automatically_declare_parameters_from_overrides(true);
        nodeOptions.use_intra_process_comms(false);

        estopViz = new ti_ros2::VizEStop("app_viz_estop", nodeOptions);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

}

