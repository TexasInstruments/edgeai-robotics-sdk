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
#include <common_msgs/msg/pose6_d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "viz_pose6d.h"

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
                                  vector<int> color_map,
                                  vector<vector<float>> cuboid_corners,
                                  float scale_x,
                                  float scale_y,
                                  int32_t  label_id,
                                  int32_t  *box)
{
    int thickness = 2;
    vector<cv::Point> cuboid_points;
    Scalar color = Scalar(color_map[0],
                          color_map[1],
                          color_map[2]);

    Scalar text_color    = Scalar(244,  35, 232);
    Scalar text_bg_color = Scalar(120, 120, 120);
    string label = classnames_ycbv[label_id];

    for (unsigned int j = 0; j < cuboid_corners.size(); j++)
    {
        int x = (int)(cuboid_corners[j][0] * scale_x);
        int y = (int)(cuboid_corners[j][1] * scale_y);
        cuboid_points.push_back(Point(x,y));
        circle(img, cuboid_points[j], thickness+3, color, -1);
    }

    //Back
    cv::line(img, cuboid_points[4], cuboid_points[5], color, thickness);
    cv::line(img, cuboid_points[5], cuboid_points[6], color, thickness);
    cv::line(img, cuboid_points[6], cuboid_points[7], color, thickness);
    cv::line(img, cuboid_points[7], cuboid_points[4], color, thickness);

    //Sides
    cv::line(img, cuboid_points[0], cuboid_points[4], color, thickness);
    cv::line(img, cuboid_points[1], cuboid_points[5], color, thickness);
    cv::line(img, cuboid_points[2], cuboid_points[6], color, thickness);
    cv::line(img, cuboid_points[3], cuboid_points[7], color, thickness);

    //Front
    cv::line(img, cuboid_points[0], cuboid_points[1], color, thickness);
    cv::line(img, cuboid_points[1], cuboid_points[2], color, thickness);
    cv::line(img, cuboid_points[2], cuboid_points[3], color, thickness);
    cv::line(img, cuboid_points[3], cuboid_points[0], color, thickness);

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
     * @brief  VizPose6D ROS warpper class
     */

    class VizPose6D: public rclcpp::Node
    {
        public:
            /**
             * @brief { Overlays the bounding boxes on the image }
             *
             */

            VizPose6D(const std::string &name,
                      const rclcpp::NodeOptions &options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string tensorTopic;
                std::string pose6DimgTopic;
                bool useApproxTimeSync;

                // input topics
                get_parameter_or("rectified_image_topic",   rectImgTopic, std::string(""));
                get_parameter_or("vision_cnn_tensor_topic", tensorTopic,  std::string(""));

                // output topics
                get_parameter_or("vision_cnn_image_topic", pose6DimgTopic, std::string(""));
                get_parameter_or("approx_time_sync", useApproxTimeSync, false);

                m_6DposeImgPub = this->create_publisher<Image>(pose6DimgTopic, 10);

                message_filters::Subscriber<Pose6D> tensorSub(this, tensorTopic);
                message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);

                TimeSynchronizer<Pose6D, Image> exactTimeSync(tensorSub, rectImgSub, 10);

                typedef ApproximateTime<Pose6D, Image> approxPloicy;
                Synchronizer<approxPloicy> approxTimeSync(approxPloicy(10), tensorSub, rectImgSub);

                if (!useApproxTimeSync)
                {
                    exactTimeSync.registerCallback(std::bind(&VizPose6D::callback_vizpose6DDet, this, _1, _2));
                }
                else
                {
                    approxTimeSync.registerCallback(std::bind(&VizPose6D::callback_vizpose6DDet, this, _1, _2));

                }

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~VizPose6D()
            {
            }

            void matrix_multiply(vector<vector<float>> &mat1,
                                 vector<vector<float>> &mat2,
                                 vector<vector<float>> &result)
            {
                int R1 = mat1.size();
                int C1 = mat1[0].size();
                int R2 = mat2.size();
                int C2 = mat2[0].size();
                if (C1 != R2)
                {
                    return;
                }

                for (int i = 0; i < R1; i++) {
                    vector<float> column{};
                    for (int j = 0; j < C2; j++) {
                        float value = 0;
                        for (int k = 0; k < R2; k++) {
                            value += mat1[i][k] * mat2[k][j];
                        }
                        column.push_back(value);
                    }
                    result.push_back(column);
                }
            }

            void cross_product_and_transpose(vector<float>         &mat1,
                                             vector<float>         &mat2,
                                             vector<vector<float>> &result)
            {
                float val1 = (mat1[1] * mat2[2]) - (mat1[2] * mat2[1]);
                float val2 = -((mat1[0] * mat2[2]) - (mat1[2] * mat2[0]));
                float val3 = (mat1[0] * mat2[1]) - (mat1[1] * mat2[0]);

                result.push_back({mat1[0],mat1[1],mat1[2]});
                result.push_back({mat2[0],mat2[1],mat2[2]});
                result.push_back({val1,val2,val3});
            }

            void get_cuboid_corners_2d(vector<vector<float>> &vertices,
                                       vector<vector<float>> &rotation_matrix,
                                       vector<float>         &translation_vector,
                                       vector<vector<float>> *m_cameraMatrix,
                                       vector<vector<float>> &cuboid_corners)
            {
                vector<vector<float>> result_matrix;
                matrix_multiply(vertices,rotation_matrix,result_matrix);

                for (unsigned i = 0; i<result_matrix.size(); i++)
                {
                    for (unsigned j = 0; j < result_matrix[i].size(); j++)
                    {
                        result_matrix[i][j] +=  translation_vector[j];
                    }

                    for (unsigned j = 0; j < result_matrix[i].size(); j++)
                    {
                        result_matrix[i][j] /=  result_matrix[i][2];
                    }
                }

                //Transpose Camera Matrix
                vector<vector<float>> camera_matrix_transpose;
                for (unsigned int i = 0; i < m_cameraMatrix->capacity(); i++) {
                    vector<float> column{};
                    for (unsigned int j = 0; j < m_cameraMatrix->at(i).size(); j++) {
                        column.push_back(m_cameraMatrix->at(j)[i]);
                    }
                    camera_matrix_transpose.push_back(column);
                }

                matrix_multiply(result_matrix,camera_matrix_transpose,cuboid_corners);
            }


            void callback_vizpose6DDet(const Pose6D::ConstSharedPtr& poseMsg,
                                    const Image::ConstSharedPtr& imagePtr)
            {
                cv_bridge::CvImagePtr cv_outImgPtr;
                cv_outImgPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);

                float scale_x = static_cast<float>(imagePtr->width)/static_cast<float>(poseMsg->img_width);
                float scale_y = static_cast<float>(imagePtr->height)/static_cast<float>(poseMsg->img_height);

                int32_t num_objects = poseMsg->num_objects;
                int32_t label_id, score;
                int32_t box[4];

                vector<vector<float>> *vertex;
                m_cameraMatrix = &YCBV_CAMERA_MATRIX;
                vertex = &YCBV_VERTICES;

                int vertex_size = (int)vertex->capacity();
                for (int i = 0 ; i < vertex_size ; i++)
                {
                    vector<vector<float>> temp_vector_1 = {};
                    for (unsigned j = 0; j < vertices_order.size() ; j++)
                    {
                        vector<float> temp_vector_2 = {};
                        for (unsigned k = 0; k < vertices_order[j].size() ; k++)
                        {
                            temp_vector_2.push_back(vertices_order[j][k]*(vertex->at(i)[k]));
                        }
                        temp_vector_1.push_back(temp_vector_2);
                    }
                    m_vertices.push_back(temp_vector_1);
                }

                vector<vector<float>> rotation_matrix;
                vector<float> rot1;
                vector<float> rot2;
                vector<float> translation_vector;
                vector<int> color_map;
                vector<vector<float>> vertices;
                vector<vector<float>> cuboid_corners;

                for (int32_t i = 0; i < num_objects; i++)
                {

                    box[0] = poseMsg->bounding_boxes[i].xmin * scale_x;
                    box[1] = poseMsg->bounding_boxes[i].ymin * scale_y;
                    box[2] = poseMsg->bounding_boxes[i].xmax * scale_x;
                    box[3] = poseMsg->bounding_boxes[i].ymax * scale_y;
                    score  = poseMsg->bounding_boxes[i].confidence;
                    label_id = poseMsg->bounding_boxes[i].label_id;

                    if (label_id < 0 || label_id > 20)
                    {
                        continue;
                    }

                    rot1.push_back(poseMsg->transform_matrix[i].rot1_x);
                    rot1.push_back(poseMsg->transform_matrix[i].rot1_y);
                    rot1.push_back(poseMsg->transform_matrix[i].rot1_z);

                    rot2.push_back(poseMsg->transform_matrix[i].rot2_x);
                    rot2.push_back(poseMsg->transform_matrix[i].rot2_y);
                    rot2.push_back(poseMsg->transform_matrix[i].rot2_z);

                    translation_vector.push_back(poseMsg->transform_matrix[i].trans_x);
                    translation_vector.push_back(poseMsg->transform_matrix[i].trans_y);
                    translation_vector.push_back(poseMsg->transform_matrix[i].trans_z);

                    color_map = COLOR_MAP[label_id];
                    // color_map = {0,113,188};

                    cross_product_and_transpose(rot1,rot2,rotation_matrix);
                    vertices = m_vertices[label_id];
                    get_cuboid_corners_2d(vertices,rotation_matrix,translation_vector,m_cameraMatrix,cuboid_corners);

                    overlayBoundingBox(cv_outImgPtr->image, color_map, cuboid_corners, scale_x, scale_y, label_id, box);

                    rot1.clear();
                    rot2.clear();
                    rotation_matrix.clear();
                    translation_vector.clear();
                    color_map.clear();
                    cuboid_corners.clear();
                    vertices.clear();
                }
		        auto imgPtr = cv_outImgPtr->toImageMsg();
		        auto hdr = &imgPtr->header;

		        hdr->frame_id = "map";

                m_6DposeImgPub->publish(*imgPtr);
            }

        private:
            rclcpp::Publisher<Image>::SharedPtr m_6DposeImgPub;
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

        auto objDetViz = std::make_shared<ti_ros2::VizPose6D>("viz_objdet", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}
