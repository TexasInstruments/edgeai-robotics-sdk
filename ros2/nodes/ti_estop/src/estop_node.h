/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
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
#ifndef _APP_ESTOP_NODE_H_
#define _APP_ESTOP_NODE_H_

#include <stdio.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>

#include <common_msgs/msg/disparity.hpp>
#include <common_msgs/msg/object_pos3_d.hpp>
#include <common_msgs/msg/detection3_d.hpp>

#include <estop.h>

using namespace sensor_msgs::msg;
using namespace message_filters;
using namespace common_msgs::msg;
using namespace nav_msgs::msg;

class EStopNode: public rclcpp::Node
{
    using ImgSub     = message_filters::Subscriber<Image>;
    using CamInfoSub = message_filters::Subscriber<CameraInfo>;
    using TimeSync   = TimeSynchronizer<Image, Image>;
    using ImgPub     = image_transport::Publisher;
    using ImgTrans   = image_transport::ImageTransport;
    using SubCon     = message_filters::Connection;

    public:
        EStopNode(const rclcpp::NodeOptions &options,
                  const std::string         &name="estop");
        ~EStopNode();

        /** Intercept signal handler */
        void onShutdown();

    private:
        /** Initialize App cntxt */
        vx_status init();

        /** Setup input image subscribers */
        void setupInputImgSubscribers();

        /** Callback to process camera info data */
        void camInfoCb(const CameraInfo::ConstSharedPtr& cam_info);

        /** Callback to process input data */
        void imgCb(const Image::ConstSharedPtr& left_image_ptr,
                   const Image::ConstSharedPtr& right_image_ptr);

        /** Run camera info subscriber thread */
        void subscriberCamInfoThread();

        /** Run publisher thread */
        void publisherThread();

        /** Run iput process thread */
        void inputProcessThread();

        /** OpenVX graph completion event handler */
        void processCompleteEvtHdlr();

        /** Extact bounding box data from 3D bounding box object */
        vx_status extract3DBBData(ObjectPos3D         * bbData,
                                  vx_user_data_object   vx3DBB,
                                  uint8_t             * numObjects);

        /** Extact occupancy grid data from 3D bounding box object */
        vx_status extractOGMapData(int8_t                  * ogData,
                                   ObjectPos3D             * bbData,
                                   vx_user_data_object       vx3DBB,
                                   int32_t                   width,
                                   int32_t                   height,
                                   int32_t                   xMinRange,
                                   int32_t                   yMinRange,
                                   int32_t                   gridSize);

        /** Set emergency stop region */
        vx_status setEStopArea(int32_t    * eStopGridIdx,
                               int32_t      minEStopDist,
                               int32_t      maxEStopDist,
                               int32_t      minEStopWidth,
                               int32_t      maxEStopWidth,
                               int32_t      width,
                               int32_t      height,
                               int32_t      xMinRange,
                               int32_t      yMinRange,
                               int32_t      gridSize);

        /** Make decision whether ego is forced to stop */
        vx_status makeEStopDecision(int8_t     * ogData,
                                    int32_t    * eStopGridIdx,
                                    int32_t      width,
                                    int32_t      height);

        /** Read yaml algorithm paramters file */
        void readParams();

    private:
        ESTOP_APP_Context      *m_cntxt{};
        TimeSync               *m_sync{};
        CamInfoSub             *m_sub{};
        ImgTrans               *m_imgTrans{};
        ImgSub                 *m_leftImageSub{};
        ImgSub                 *m_rightImageSub{};

        int32_t                *m_eStopGridIdx{};
        SubCon                  m_conObj;
        SubCon                  m_camInfoConObj;

        /** Semantic segmenation output image publisher */
        uint32_t                m_ssMapImgWidth;
        uint32_t                m_ssMapImgHeight;

        /** Semantic segmentation output tensor publisher */
        Image                   m_ssTensorPubData;
        uint32_t                m_ssTensorSize;
        ImgPub                  m_ssTensorPub;

        /** Rectified image publisher */
        uint32_t                m_imgWidth;
        uint32_t                m_imgHeight;
        Image                   m_rectImagePubData;
        uint32_t                m_rectImageSize;
        ImgPub                  m_rectImgPub;

        /** 3D bounding box publisher */
        ObjectPos3D            *m_bbData{};
        std::vector<ObjectPos3D> m_bbPubData;
        uint32_t                m_bbSize;
        rclcpp::Publisher<Detection3D>::SharedPtr m_bbPub;

        /** Disparity map publisher */
        Disparity               m_disparityPubData;
        uint32_t                m_disparitySize;
        rclcpp::Publisher<Disparity>::SharedPtr m_disparityPub;

        /** OG map publisher */
        OccupancyGrid           m_ogMapPubData;
        uint32_t                m_ogMapSize;
        rclcpp::Publisher<OccupancyGrid>::SharedPtr m_ogMapPub;

        /** e-Stop flag publisher */
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_eStopPub;

        /** Initialization control semaphore */
        Semaphore              *initCtrlSem;

        /** Min distance of estop range
         *  Should be 0
         */
        int32_t                 m_minEStopDistance;

        /** Max distance of estop range in mm */
        int32_t                 m_maxEStopDistance;

        /** Width of estop range at min_estop_distance in mm */
        int32_t                 m_minEStopWidth;

        /** Width of estop range at max_estop_distance in mm */
        int32_t                 m_maxEStopWidth;

        /** When m_eStop = 1, the minimum number of consecutive frames without any obstacle
         *  in EStop range should be larger than or equal to m_minFreRun to set m_eStop = 0
         */
        int8_t                  m_minFreeRun;

        /** When m_eStop = 0, the minimum number of consecutive frames with any obstacle
         *  in EStop range should be larger than or equal to m_minObsRun to set m_eStop = 1
         */
        int8_t                  m_minObsRun;

        /** the number of consecutive frames without any obstacle in EStop range*/
        int32_t                 m_freeRun;

        /** the number of consecutive frames with any obstacle in EStop range*/
        int32_t                 m_obsRun;

        /** EStop flag */
        std_msgs::msg::Bool     m_eStop;
};

#endif /* _APP_ESTOP_NODE_H_ */

