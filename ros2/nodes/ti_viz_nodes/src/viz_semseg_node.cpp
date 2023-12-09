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

using namespace sensor_msgs::msg;
using namespace message_filters;

using std::placeholders::_1;
using std::placeholders::_2;

// OpenCV uses BGR format
static const uint8_t color_map[20][3] =
{{128,  64, 128},{244,  35, 232},{ 70,  70,  70},{102, 102, 156},{190, 153, 153},
 {153, 153, 153},{250, 170,  30},{220, 220,   0},{107, 142,  35},{152, 251, 152},
 { 70, 130, 180},{220,  20,  60},{255,   0,   0},{  0,   0, 142},{  0,   0,  70},
 {  0,  60, 100},{  0,  80, 100},{  0,   0, 230},{119,  11,  32},{128, 128, 128}};

static void sigHandler(int32_t sig)
{
    (void) sig;

    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}

namespace ti_ros2
{
    /**
     * @brief  VizSemSeg ROS warpper class
     */

    class VizSemSeg: public rclcpp::Node
    {
        public:
            VizSemSeg(const std::string         &name,
                      const rclcpp::NodeOptions &options):
                Node(name, options)
            {
                std::string rectImgTopic;
                std::string ssTensorTopic;
                std::string ssMapImgTopic;

                // input topics
                get_parameter_or("rectified_image_topic",   rectImgTopic,  std::string(""));
                get_parameter_or("vision_cnn_tensor_topic", ssTensorTopic, std::string(""));

                // output topics
                get_parameter_or("vision_cnn_image_topic", ssMapImgTopic, std::string(""));

                m_ssMapImgPub = this->create_publisher<Image>(ssMapImgTopic, 10);

                message_filters::Subscriber<Image> ssTensorSub(this, ssTensorTopic);
                message_filters::Subscriber<Image> rectImgSub(this, rectImgTopic);

                TimeSynchronizer<Image, Image> sync(ssTensorSub, rectImgSub, 10);
                sync.registerCallback(std::bind(&VizSemSeg::callback_vizSemSeg, this, _1, _2));

                rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
            }

            ~VizSemSeg()
            {
            }

            void callback_vizSemSeg(const Image::ConstSharedPtr& tensorPtr,
                                    const Image::ConstSharedPtr& imagePtr)
            {
                // position in original image
                int32_t i, j;
                // position in tensor
                int32_t y, x;
                // Semantic class
                uint8_t classId;

                // input image size
                int32_t width  = imagePtr->width;
                int32_t height = imagePtr->height;

                // tensor size
                int32_t tensorWidth  = tensorPtr->width;
                int32_t tensorHeight = tensorPtr->height;

                // blending factor
                float bf = 0.5;

                int32_t stride = width * 3;

                float   horScale = tensorWidth *1.0/width;
                float   verScale = tensorHeight*1.0/height;

                cv_bridge::CvImagePtr cv_ssPtr;
                cv_ssPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);

                for (j = 0; j < height; j++)
                {
                    // y pos in tensor
                    y = (j * verScale + 0.5);
                    for (i = 0; i < width; i++)
                    {
                        // x pos in tensor
                        x = (i * horScale + 0.5);
                        classId =  tensorPtr->data[y*tensorWidth + x];

                        cv_ssPtr->image.data[j*stride + i*3]     =
                            (1-bf)*cv_ssPtr->image.data[j*stride + i*3]     + bf*color_map[classId][0];
                        cv_ssPtr->image.data[j*stride + i*3 + 1] =
                            (1-bf)*cv_ssPtr->image.data[j*stride + i*3 + 1] + bf*color_map[classId][1];
                        cv_ssPtr->image.data[j*stride + i*3 + 2] =
                            (1-bf)*cv_ssPtr->image.data[j*stride + i*3 + 2] + bf*color_map[classId][2];
                    }
                }

		        auto imgPtr = cv_ssPtr->toImageMsg();
		        auto hdr = &imgPtr->header;

		        hdr->frame_id = "map";

                m_ssMapImgPub->publish(*imgPtr);
            }

        private:
            rclcpp::Publisher<Image>::SharedPtr m_ssMapImgPub;
    };
}

// static std::shared_ptr<ti_ros2::VizSemSeg>  semSegViz = nullptr;
// static ti_ros2::VizSemSeg  *semSegViz = nullptr;

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

        /* Allow any parameter name to be set on the node without first being
         * declared. Otherwise, setting an undeclared parameter will raise an
         * exception.
         *
         * This option being true does not affect parameter_overrides, as the
         * first set action will implicitly declare the parameter and therefore
         * consider any parameter overrides.
         */
        nodeOptions.allow_undeclared_parameters(true);

        /* Automatically iterate through the node's parameter overrides and
         * implicitly declare any that have not already been declared.
         * Otherwise, parameters passed to the node's parameter_overrides,
         * and/or the global arguments (e.g. parameter overrides from a YAML
         * file), which are not explicitly declared will not appear on the node
         * at all, even if allow_undeclared_parameters is true. Already
         * declared parameters will not be re-declared, and parameters declared
         * in this way will use the default constructed ParameterDescriptor.
         */
        nodeOptions.automatically_declare_parameters_from_overrides(true);

        /* Messages on topics which are published and subscribed to within this
         * context will go through a special intra-process communication code
         * code path which can avoid serialization and deserialization,
         * unnecessary copies, and achieve lower latencies in some cases.
         *
         * Defaults to false for now, as there are still some cases where it is
         * not desirable.
         */
        nodeOptions.use_intra_process_comms(false);

        auto semSegViz = std::make_shared<ti_ros2::VizSemSeg>("viz_semseg", nodeOptions);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}

