/*
 *  Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <common_msgs/msg/bounding_box2_d.hpp>
#include <common_msgs/msg/detection2_d.hpp>
#include <common_msgs/msg/detection2_dpva.hpp>
#include <ti_mmwave_rospkg_msgs/msg/radar_track_array.hpp>

using namespace message_filters;
using namespace sensor_msgs::msg;
using namespace common_msgs::msg;
using namespace ti_mmwave_rospkg_msgs::msg;

/**
 * @brief  ObjdetRadarFusion ROS node class
 */

class ObjdetRadarFusion : public rclcpp::Node
{
public:
    ObjdetRadarFusion(const std::string &name,
        const rclcpp::NodeOptions &options);

    ~ObjdetRadarFusion();

private:
    void callback_CameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

    void callback_RadarTrackArray(
        const ti_mmwave_rospkg_msgs::msg::RadarTrackArray::SharedPtr radar_track_array_msg);

    void callback_Detection2D(
        const common_msgs::msg::Detection2D::SharedPtr detection_2d_msg);

    double diffStamps(const std_msgs::msg::Header& header1,
                      const std_msgs::msg::Header& header2);

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cameraInfo_sub;
    rclcpp::Subscription<common_msgs::msg::Detection2D>::SharedPtr m_detection2D_sub;
    rclcpp::Subscription<ti_mmwave_rospkg_msgs::msg::RadarTrackArray>::SharedPtr m_radarTrackArray_sub;
    rclcpp::Publisher<common_msgs::msg::Detection2Dpva>::SharedPtr m_Detection2Dpva_pub;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    sensor_msgs::msg::CameraInfo m_cameraInfo;
    sensor_msgs::msg::CameraInfo m_cameraInfo_rect;
    ti_mmwave_rospkg_msgs::msg::RadarTrackArray m_radarTrackArray_last;
    std::string m_radarTrackerTopic;
    std::string m_tensorTopic;
    std::string m_outputTopic;
    std::string m_cameraInfoTopic;
    std::string m_radarFrameId;
    std::string m_cameraFrameId;
    std::string m_cameraInfoFile;
    bool m_radarTrackXyOnly;
    double m_maxSyncOffset;
};
