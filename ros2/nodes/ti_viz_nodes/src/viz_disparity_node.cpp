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
#include <common_msgs/msg/disparity.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace sensor_msgs::msg;
using namespace message_filters;
using namespace common_msgs::msg;

#define NUM_FRAC_BITS        4
#define SDE_DISPARITY_OFFSET 3

const unsigned char SDE_falseColorLUT_RGB[3][260] = {
    {128,64,0,24,33,28,24,21,18,14,10,6,3,1,2,2,2,3,2,3,2,2,2,2,3,3,3,2,2,2,3,3,2,3,1,3,3,2,2,3,2,3,3,2,2,3,2,2,3,3,3,3,2,2,4,2,3,3,2,3,3,2,2,3,3,3,2,2,3,2,2,3,1,3,2,3,2,3,3,3,2,2,2,2,3,2,3,2,3,3,3,3,2,2,2,3,2,3,2,4,2,1,3,2,2,2,3,3,3,2,2,2,1,8,13,20,26,31,38,44,50,56,63,67,74,81,86,93,99,104,110,117,123,129,136,140,147,155,159,166,172,177,183,191,196,202,209,214,219,225,231,238,244,249,255,254,255,255,255,255,255,255,255,255,254,255,255,254,255,255,255,255,255,255,255,255,254,255,255,255,255,255,255,255,255,255,255,255,255,255,254,255,255,255,255,255,255,255,255,255,255,255,254,255,255,254,255,255,255,255,255,255,255,254,254,255,254,255,255,255,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,255,255,255,254,255,255,255,254,255,254,255,255,255,255,255,254,255,255,255,255,255,255,254,255},
    {128,64,0,4,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,6,12,19,25,32,37,43,51,57,63,70,76,82,89,95,101,109,115,120,127,133,140,146,152,158,165,172,178,184,190,196,204,209,216,222,229,236,241,247,254,254,254,255,254,254,255,254,254,255,254,255,254,254,254,254,253,254,253,254,254,253,255,253,253,254,254,254,254,254,254,254,253,254,253,254,254,253,254,254,254,254,253,254,253,254,254,253,254,254,254,253,254,254,254,254,253,254,253,254,255,254,254,254,254,254,254,255,254,255,254,254,255,254,254,254,255,254,255,255,255,255,255,252,249,247,244,241,239,237,234,231,230,227,225,222,219,217,215,211,209,207,205,201,200,198,195,192,189,187,184,181,179,177,174,171,169,168,164,162,160,157,154,152,150,147,144,142,139,138,135,132,130,126,124,122,120,116,114,112,109,107,105,100,97,94,90,87,83,81,76,73,70,67,63,59,57,52,49,45,43,39,35,31,29,25,21,18,15,11,7,4,1,0,0,1,0,1,0,1,1,0,1,1,1,1,1,255},
    {128,64,0,74,96,101,104,108,113,116,120,125,129,135,142,148,153,160,166,174,179,185,192,198,205,211,217,224,230,235,242,248,255,255,255,255,255,254,255,255,255,255,255,254,253,255,255,255,254,255,255,255,255,255,255,255,254,254,255,255,255,255,254,254,255,255,255,255,255,255,255,255,255,249,242,236,231,224,217,210,205,199,192,186,179,173,169,162,155,149,144,138,130,123,117,112,105,99,91,87,80,73,67,60,54,48,41,35,28,23,17,9,2,5,4,4,3,3,4,3,3,2,3,4,4,4,4,4,3,3,2,3,3,2,5,4,4,4,3,4,3,3,2,3,3,4,4,4,4,3,3,4,3,3,2,2,3,4,5,2,3,4,5,2,3,4,3,3,4,4,3,3,4,3,3,3,4,3,4,3,4,3,3,4,2,3,3,4,3,4,3,2,3,4,3,2,3,4,4,3,3,4,2,3,4,3,2,3,4,2,2,3,4,2,3,2,2,3,3,2,2,3,2,2,3,3,2,2,3,3,2,2,3,3,2,2,3,2,2,2,3,2,2,2,3,9,16,23,27,34,40,48,53,59,66,73,77,85,89,255}
};

namespace ti_ros2
{
    using ImgSub   = message_filters::Subscriber<Disparity>;
    using SubCon   = message_filters::Connection;

    /**
     * @brief  SDE ROS warpper class
     */

    class VizDisparity: public rclcpp::Node
    {
        public:
            /**
             * @brief { function_description }
             *
             * @param[in]  resolution  The resolution
             * @param[in]  frame_rate  The frame rate
             */

            VizDisparity(const std::string         &name,
                         const rclcpp::NodeOptions &options):
                Node(name, options)
            {
                std::string rawDisparityTopic;
                std::string ccDisparityTopic;
                std::string ccConfidenceTopic;

                // input size
                get_parameter_or("width", width, 1280);
                get_parameter_or("height", height, 720);

                // input topic
                get_parameter_or("disparity_topic",     rawDisparityTopic,  std::string(""));

                // output topics
                get_parameter_or("cc_disparity_topic",  ccDisparityTopic,   std::string(""));
                get_parameter_or("cc_confidence_topic", ccConfidenceTopic,  std::string(""));

                // allocation for CC disparity
                ccDisparity  = (uint8_t *)malloc(width * height * 3);
                ccConfidence = (uint8_t *)malloc(width * height * 3);

                disp_sub   = new ImgSub(this, rawDisparityTopic);
                conObj     = disp_sub->registerCallback(&VizDisparity::callback_vizDiarpity, this);
                ccDisp_pub = this->create_publisher<Image>(ccDisparityTopic, 10);
                ccConf_pub = this->create_publisher<Image>(ccConfidenceTopic, 10);

            }

            ~VizDisparity()
            {
                // free ccDisparity and ccConfidence
                if (ccDisparity != NULL)
                {
                    free(ccDisparity);
                }

                if (ccConfidence != NULL)
                {
                    free(ccConfidence);
                }
            }

            void callback_vizDiarpity(const Disparity::ConstSharedPtr& msg)
            {
                Disparity dispMsg = *msg;

                // create color-coded disparity and confidence
                visualizeSdeDisparity((uint16_t *)dispMsg.data.data(),
                                      ccDisparity,
                                      ccConfidence,
                                      dispMsg.width,
                                      dispMsg.height,
                                      dispMsg.min_disp,
                                      dispMsg.max_disp);

                // Publish ccDisparity image
                std::vector<uint8_t> vec_d(ccDisparity, ccDisparity + width*height*3);
                ccDisp_msg.width        = dispMsg.width;
                ccDisp_msg.height       = dispMsg.height;
                ccDisp_msg.encoding     = "rgb8";
                ccDisp_msg.step         = dispMsg.width*3;
                ccDisp_msg.data         = vec_d;
                ccDisp_msg.header.stamp = dispMsg.header.stamp;
		        ccDisp_msg.header.frame_id = "map";

                ccDisp_pub->publish(ccDisp_msg);

                // Publish ccConfidence image
                std::vector<uint8_t> vec_c(ccConfidence, ccConfidence + width*height*3);
                ccConf_msg.width        = dispMsg.width;
                ccConf_msg.height       = dispMsg.height;
                ccConf_msg.encoding     = "rgb8";
                ccConf_msg.step         = dispMsg.width*3;
                ccConf_msg.data         = vec_c;
                ccConf_msg.header.stamp = dispMsg.header.stamp;
		        ccConf_msg.header.frame_id = "map";

                ccConf_pub->publish(ccConf_msg);
            }

            void visualizeSdeDisparity(uint16_t * rawDisparity,
                                       uint8_t  * ccDisparity,
                                       uint8_t  * ccConfidence,
                                       int32_t    dispWidth,
                                       int32_t    dispHeight,
                                       int32_t    minDisp,
                                       int32_t    maxDisp)
            {
                int32_t i, j, value;
                int32_t outDisparity;

                float scaleFactor = (float)((1 << NUM_FRAC_BITS) * (maxDisp - minDisp)) / 255;

                uint8_t valid;
                uint8_t * R, *G, *B;

                int16_t pixDisparity;

                // color coded confidence RGB = [0, confG, 0]
                uint8_t * confG;

                // init ccDisparity pointer
                R = (uint8_t *)ccDisparity;
                G = (uint8_t *)ccDisparity + 1;
                B = (uint8_t *)ccDisparity + 2;

                // init ccConfidence
                memset(ccConfidence, 0, dispWidth*dispHeight*3);
                confG = (uint8_t *)ccConfidence + 1;

                // create the falseColor map
                scaleFactor = ((float)1.0) / scaleFactor;

                for (j = 0; j < dispHeight; j++)
                {
                    for (i = 0; i < dispWidth; i++)
                    {
                        // In operation mode, minimum disparity should be non-negative,
                        // so sign bit can be ignored
                        pixDisparity = rawDisparity[i];
                        //sign         = (pixDisparity >> 15) == 0 ? 1: -1;
                        outDisparity = (pixDisparity >> 3) & 0xFFF;
                        //outDisparity *= sign;

                        // check if disparity confidence is larger than 0
                        valid = ((pixDisparity & 0x3) > 0);

                        // Shift disparity so that minimum disparity and unknown disparity are both zero
                        value = (int)(outDisparity * scaleFactor * valid) + SDE_DISPARITY_OFFSET;

                        *R = (unsigned char)(SDE_falseColorLUT_RGB[0][value]);
                        *G = (unsigned char)(SDE_falseColorLUT_RGB[1][value]);
                        *B = (unsigned char)(SDE_falseColorLUT_RGB[2][value]);

                        R += 3;
                        G += 3;
                        B += 3;

                        // confidence
                        *confG  = 36 * (pixDisparity & 0x07);
                        confG +=3;

                    } // for (i = 0; i < dispWidth; i++)

                    rawDisparity += dispWidth;

                } // for (j = 0; j < dispHeight; j++)

            } // visualizeSdeDisparity()

        private:
            ImgSub     *disp_sub;
            SubCon      conObj;
            rclcpp::Publisher<Image>::SharedPtr ccDisp_pub;
            rclcpp::Publisher<Image>::SharedPtr ccConf_pub;
            Image       ccDisp_msg;
            Image       ccConf_msg;
            int32_t     width;
            int32_t     height;
            uint8_t    *ccDisparity;
            uint8_t    *ccConfidence;
    };
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    try
    {
        rclcpp::InitOptions initOptions{};
        rclcpp::NodeOptions nodeOptions{};

        rclcpp::init(argc, argv, initOptions);

        // signal(SIGINT, sigHandler);

        nodeOptions.allow_undeclared_parameters(true);
        nodeOptions.automatically_declare_parameters_from_overrides(true);
        nodeOptions.use_intra_process_comms(false);

        auto dispViz = std::make_shared<ti_ros2::VizDisparity>("viz_disparity", nodeOptions);

        rclcpp::spin(dispViz);

        rclcpp::shutdown();

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}


