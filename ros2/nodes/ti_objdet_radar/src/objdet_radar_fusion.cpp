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

#include <signal.h>
#include "objdet_radar.h"

static void sigHandler(int32_t sig)
{
    (void) sig;
    rclcpp::shutdown();
    std::exit(EXIT_SUCCESS);
}


/**
 * @brief { Fusion for the radar tracker output and the vision object detection }
 *
 */
ObjdetRadarFusion::ObjdetRadarFusion(
    const std::string &name,
    const rclcpp::NodeOptions &options) : Node(name, options)
{

    // parse params from launch file
    get_parameter_or("radar_tracker_topic", m_radarTrackerTopic, std::string("ti_mmwave/radar_track_array"));
    get_parameter_or("vision_cnn_tensor_topic", m_tensorTopic, std::string("vision_cnn/tensor"));
    get_parameter_or("camera_info_topic", m_cameraInfoTopic, std::string("camera/camera_info"));
    get_parameter_or("outout_topic", m_outputTopic, std::string("camera/fused_objdet_radar"));
    get_parameter_or("radar_frame_id", m_radarFrameId, std::string("ti_mmwave_0"));
    get_parameter_or("camera_frame_id", m_cameraFrameId, std::string("camera"));
    get_parameter_or("camera_info_file", m_cameraInfoFile, std::string("file:///opt/robotics_sdk/tools/camera_info/IMX219_HD_camera_info.yaml"));
    get_parameter_or("radar_track_xy_only", m_radarTrackXyOnly, true);
    get_parameter_or("max_sync_offset", m_maxSyncOffset, 0.03333);
    RCLCPP_INFO(this->get_logger(), "===> m_maxSyncOffset = %f", m_maxSyncOffset);

    // subscriber objects
    m_cameraInfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        m_cameraInfoTopic, 10, std::bind(&ObjdetRadarFusion::callback_CameraInfo,
        this, std::placeholders::_1));

    m_detection2D_sub = this->create_subscription<common_msgs::msg::Detection2D>(
        m_tensorTopic, 10, std::bind(&ObjdetRadarFusion::callback_Detection2D,
        this, std::placeholders::_1));

    m_radarTrackArray_sub = this->create_subscription<ti_mmwave_rospkg_msgs::msg::RadarTrackArray>(
        m_radarTrackerTopic, 10, std::bind(&ObjdetRadarFusion::callback_RadarTrackArray,
        this, std::placeholders::_1));

    // publisher object
    m_Detection2Dpva_pub = this->create_publisher<common_msgs::msg::Detection2Dpva>(m_outputTopic, 1);

    // create a TransformListener
    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    // spin
    rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
}

ObjdetRadarFusion::~ObjdetRadarFusion()
{

}

void ObjdetRadarFusion::callback_CameraInfo(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
    m_cameraInfo_rect = *camera_info_msg;

    // set the distortion params to all zeros for rectified images
    std::fill(m_cameraInfo_rect.d.begin(), m_cameraInfo_rect.d.end(), 0.0);

    // RCLCPP_INFO(this->get_logger(), "callback_CameraInfo: camera_info: K = [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
    //     m_cameraInfo_rect.k[0], m_cameraInfo_rect.k[1], m_cameraInfo_rect.k[2],
    //     m_cameraInfo_rect.k[3], m_cameraInfo_rect.k[4], m_cameraInfo_rect.k[5],
    //     m_cameraInfo_rect.k[6], m_cameraInfo_rect.k[7], m_cameraInfo_rect.k[8]);

    if (m_cameraInfo_rect.k[0] == 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "callback_CameraInfo(): camera_info is not parsed corrected");
    }

    // Unsubscribe from the topic after receiving the first message
    if ((m_cameraInfo_rect.k[0] != 0.0) && (m_cameraInfo_sub != nullptr))
    {
        m_cameraInfo_sub.reset();
        RCLCPP_INFO(this->get_logger(), "===> m_cameraInfo_sub reset");
    }
}

void ObjdetRadarFusion::callback_RadarTrackArray(
    const ti_mmwave_rospkg_msgs::msg::RadarTrackArray::SharedPtr radar_track_array_msg)
{
    m_radarTrackArray_last = *radar_track_array_msg;
}

void ObjdetRadarFusion::callback_Detection2D(
    const common_msgs::msg::Detection2D::SharedPtr visionDetPtr)
{
    RadarTrackArray radarTrackArray = m_radarTrackArray_last;
    Detection2D visionDet = *visionDetPtr;
    Detection2Dpva outDetArray;

    /* synchronize */
    double diff_stamp = diffStamps(visionDetPtr->header, m_radarTrackArray_last.header);
    // RCLCPP_INFO(this->get_logger(), "===> diffStamps(): diff_stamp = %9.6f", diff_stamp);

    if (abs(diff_stamp) < m_maxSyncOffset)
    {
        // RCLCPP_INFO(this->get_logger(), "Synched");

        /* Project radar_track_objs onto the camera_frame */
        std::string target_frame = m_cameraFrameId.c_str();
        int32_t num_radar_tracks = radarTrackArray.num_tracks;
        std::vector<geometry_msgs::msg::PointStamped>   radar_track_pos_in_camera_frame(num_radar_tracks);
        std::vector<geometry_msgs::msg::Vector3Stamped> radar_track_vel_in_camera_frame(num_radar_tracks);
        std::vector<geometry_msgs::msg::Vector3Stamped> radar_track_acc_in_camera_frame(num_radar_tracks);
        std::vector<cv::Point2d> radar_track_pos_in_image(num_radar_tracks);

        double fx = m_cameraInfo_rect.k[0];
        double cx = m_cameraInfo_rect.k[2];
        double fy = m_cameraInfo_rect.k[4];
        double cy = m_cameraInfo_rect.k[5];

        for (int32_t idx=0; idx<num_radar_tracks; idx++)
        {
            /* Transform position to the camera frame */
            geometry_msgs::msg::PointStamped point_in;
            // if m_radarTrackXyOnly, project the tracked object onto the XY plane, i.e.,
            // ignore z coordinate considering low elevation-resolution of IWR6843ISK
            point_in.point.x = radarTrackArray.track[idx].posx;
            point_in.point.y = radarTrackArray.track[idx].posy;
            point_in.point.z = (m_radarTrackXyOnly)? 0.0 : radarTrackArray.track[idx].posz;
            point_in.header.frame_id = m_radarFrameId.c_str();
            radar_track_pos_in_camera_frame[idx] = m_tfBuffer->transform(point_in, target_frame);
            // RCLCPP_INFO(this->get_logger(), "point_out: frame_id = %s, x = %f, y = %f, z = %f",
            //             point_out.header.frame_id.c_str(), point_out.point.x, point_out.point.y, point_out.point.z);

            /* Project position onto image_rect */
            double pos_x = radar_track_pos_in_camera_frame[idx].point.x;
            double pos_y = radar_track_pos_in_camera_frame[idx].point.y;
            double pos_z = radar_track_pos_in_camera_frame[idx].point.z;
            radar_track_pos_in_image[idx].x = fx * pos_x / pos_z + cx;
            radar_track_pos_in_image[idx].y = fy * pos_y / pos_z + cy;
            // RCLCPP_INFO(this->get_logger(), "radar_track_pos_in_image[%d] = [%f, %f]",
            //     idx, radar_track_pos_in_image[idx].x, radar_track_pos_in_image[idx].y);

            /* Transform velocity to the camera frame */
            geometry_msgs::msg::Vector3Stamped vector_in;
            vector_in.vector.x = radarTrackArray.track[idx].velx;
            vector_in.vector.y = radarTrackArray.track[idx].vely;
            vector_in.vector.z = radarTrackArray.track[idx].velz;
            vector_in.header.frame_id = m_radarFrameId.c_str();
            radar_track_vel_in_camera_frame[idx] = m_tfBuffer->transform(vector_in, target_frame);

            /* Transform acceleration to the camera frame */
            vector_in.vector.x = radarTrackArray.track[idx].accx;
            vector_in.vector.y = radarTrackArray.track[idx].accy;
            vector_in.vector.z = radarTrackArray.track[idx].accz;
            vector_in.header.frame_id = m_radarFrameId.c_str();
            radar_track_acc_in_camera_frame[idx] = m_tfBuffer->transform(vector_in, target_frame);
        }

        /* Associate the radar track objects and publish */
        // pass though the header
        outDetArray.header = visionDetPtr->header;
        outDetArray.num_objects = visionDetPtr->num_objects;
        outDetArray.bounding_boxes.assign(outDetArray.num_objects, BoundingBox2Dpva{});

        for (int32_t v_idx=0; v_idx<visionDetPtr->num_objects; v_idx++)
        {
            // pass though the bounding box info from vision object detection
            outDetArray.bounding_boxes[v_idx].xmin = visionDetPtr->bounding_boxes[v_idx].xmin;
            outDetArray.bounding_boxes[v_idx].ymin = visionDetPtr->bounding_boxes[v_idx].ymin;
            outDetArray.bounding_boxes[v_idx].xmax = visionDetPtr->bounding_boxes[v_idx].xmax;
            outDetArray.bounding_boxes[v_idx].ymax = visionDetPtr->bounding_boxes[v_idx].ymax;
            outDetArray.bounding_boxes[v_idx].label_id = visionDetPtr->bounding_boxes[v_idx].label_id;
            outDetArray.bounding_boxes[v_idx].confidence = visionDetPtr->bounding_boxes[v_idx].confidence;

            BoundingBox2D bbox = visionDet.bounding_boxes[v_idx];

            for (int32_t r_idx = 0; r_idx<num_radar_tracks; r_idx++)
            {
                // check if any of radar_tracked obj in the bbox
                bool inside_bbox = (bbox.xmin <= radar_track_pos_in_image[r_idx].x) &&
                                (radar_track_pos_in_image[r_idx].x <= bbox.xmax) &&
                                (bbox.ymin <= radar_track_pos_in_image[r_idx].y) &&
                                (radar_track_pos_in_image[r_idx].y <= bbox.ymax);
                if (inside_bbox)
                {
                    outDetArray.bounding_boxes[v_idx].pva_valid      = true;
                    outDetArray.bounding_boxes[v_idx].radar_track_x  = static_cast<int32_t>(radar_track_pos_in_image[r_idx].x);
                    outDetArray.bounding_boxes[v_idx].radar_track_y  = static_cast<int32_t>(radar_track_pos_in_image[r_idx].y);

                    // PVA all in the camera frame
                    outDetArray.bounding_boxes[v_idx].position.x     = radar_track_pos_in_camera_frame[r_idx].point.x;
                    outDetArray.bounding_boxes[v_idx].position.y     = radar_track_pos_in_camera_frame[r_idx].point.y;
                    outDetArray.bounding_boxes[v_idx].position.z     = radar_track_pos_in_camera_frame[r_idx].point.z;
                    outDetArray.bounding_boxes[v_idx].velocity.x     = radar_track_vel_in_camera_frame[r_idx].vector.x;
                    outDetArray.bounding_boxes[v_idx].velocity.y     = radar_track_vel_in_camera_frame[r_idx].vector.y;
                    outDetArray.bounding_boxes[v_idx].velocity.z     = radar_track_vel_in_camera_frame[r_idx].vector.z;
                    outDetArray.bounding_boxes[v_idx].acceleration.x = radar_track_acc_in_camera_frame[r_idx].vector.x;
                    outDetArray.bounding_boxes[v_idx].acceleration.y = radar_track_acc_in_camera_frame[r_idx].vector.y;
                    outDetArray.bounding_boxes[v_idx].acceleration.z = radar_track_acc_in_camera_frame[r_idx].vector.z;

                    break;
                }
                else
                {
                    outDetArray.bounding_boxes[v_idx].pva_valid = false;
                }
            }
        }

    }

    else
    {
        // pass though the message
        outDetArray.header = visionDetPtr->header;
        outDetArray.num_objects = visionDetPtr->num_objects;
        outDetArray.bounding_boxes.assign(outDetArray.num_objects, BoundingBox2Dpva{});

        for (int32_t v_idx=0; v_idx<visionDetPtr->num_objects; v_idx++)
        {
            // pass though the bounding box info from vision object detection
            outDetArray.bounding_boxes[v_idx].xmin = visionDetPtr->bounding_boxes[v_idx].xmin;
            outDetArray.bounding_boxes[v_idx].ymin = visionDetPtr->bounding_boxes[v_idx].ymin;
            outDetArray.bounding_boxes[v_idx].xmax = visionDetPtr->bounding_boxes[v_idx].xmax;
            outDetArray.bounding_boxes[v_idx].ymax = visionDetPtr->bounding_boxes[v_idx].ymax;
            outDetArray.bounding_boxes[v_idx].label_id = visionDetPtr->bounding_boxes[v_idx].label_id;
            outDetArray.bounding_boxes[v_idx].confidence = visionDetPtr->bounding_boxes[v_idx].confidence;
        }
    }

    // publish
    m_Detection2Dpva_pub->publish(outDetArray);

}

double ObjdetRadarFusion::diffStamps(const std_msgs::msg::Header& header1,
                                     const std_msgs::msg::Header& header2)
{
    double time1 = header1.stamp.sec + header1.stamp.nanosec*1e-9;
    double time2 = header2.stamp.sec + header2.stamp.nanosec*1e-9;
    double offset = time1 - time2;

    return offset;
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

        nodeOptions.allow_undeclared_parameters(true);
        nodeOptions.automatically_declare_parameters_from_overrides(true);

        signal(SIGINT, sigHandler);

        auto objdetRadarFusion = std::make_shared<ObjdetRadarFusion>("objdet_radar_fusion", nodeOptions);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}
