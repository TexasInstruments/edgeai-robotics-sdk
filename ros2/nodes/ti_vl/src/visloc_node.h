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
#ifndef _APP_VISLOC_NODE_H_
#define _APP_VISLOC_NODE_H_

#include <stdio.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <message_filters/subscriber.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <visloc.h>

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace message_filters;

class VisLocNode: public rclcpp::Node
{
    using ImgSub   = message_filters::Subscriber<Image>;
    using ImgPub   = image_transport::Publisher;
    using ImgTrans = image_transport::ImageTransport;
    using SubCon   = message_filters::Connection;

    public:
        VisLocNode(const rclcpp::NodeOptions &options,
                   const std::string         &name="visloc");
        ~VisLocNode();
        void sigHandler(int32_t  sig);

    private:
        vx_status init();
        void readParams();
        void imgCb(const Image::ConstSharedPtr& imgPtr);
        void setupInputImgSubscribers();
        void publisherThread();
        void processCompleteEvtHdlr();

    private:
        /** Visual localization app context */
        VISLOC_Context                             *m_cntxt{};

        /** Input image topic subscriber */
        ImgSub                                     *m_sub{};

        /** Image transport to publish output image */
        ImgTrans                                   *m_imgTrans{};

        /** Output position from a graph */
        double                                     *m_outPose{};

        /** Output quaternion from a graph */
        double                                     *m_outQuaternion{};

        SubCon                                      m_conObj;

        /** Publisher for output image */
        ImgPub                                      m_outImgPub;

        /** Publisher for output pose */
        rclcpp::Publisher<PoseStamped>::SharedPtr   m_posePub;

        /** Publisher for offline map data */
        rclcpp::Publisher<PointCloud2>::SharedPtr   m_mapPub;

        /** Input topic image width */
        uint32_t                                    m_inputImgWidth;

        /** Input topic image height */
        uint32_t                                    m_inputImgHeight;

        /** Output image width */
        uint32_t                                    m_outImgWidth;

        /** Output image width */
        uint32_t                                    m_outImgHeight;

        /** Output image data to be published */    
        Image                                       m_outPubData;

        /** Offline map data to be published */
        PointCloud2                                 m_mapPubData;

        /** Output image data size */
        uint32_t                                    m_outImageSize;

        /** offline map size */
        uint32_t                                    m_mapSize;
};

#endif /* _APP_VISLOC_NODE_H_ */

