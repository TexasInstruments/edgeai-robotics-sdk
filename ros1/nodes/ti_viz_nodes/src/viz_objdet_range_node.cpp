#include <stdio.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <common_msgs/BoundingBox2Dp.h>
#include <common_msgs/Detection2Dp.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <bits/stdc++.h>
#include "viz_objdet.h"
#include <cmath>

using namespace sensor_msgs;
using namespace common_msgs;
using namespace message_filters;
using namespace std;
using namespace cv;

#define NUM_FRAC_BITS 4

// calculate range in X-Z plane in camera frame
static float point2Range(const geometry_msgs::Point32& point)
{
    float range = sqrt(point.x*point.x + point.z*point.z);
    return range;
}

static cv::Mat overlayBoundingBox(cv::Mat &img,
                                  const BoundingBox2Dp &bbox,
                                  const cv::Scalar box_color,
                                  const cv::Scalar text_color,
                                  const cv::Scalar text_bg_color)
{
    int32_t label_id = bbox.label_id;
    int32_t label_id_mod = label_id % 10;
    int32_t box_width = bbox.xmax - bbox.xmin;
    float range = point2Range(bbox.position);

    std::string label_str = classnames_coco[label_id];
    std::string position_str = std::to_string(round(range*100)/100).substr(0,4) + std::string(" m");

    // Draw bounding box for the detected object
    Point topleft     = Point(bbox.xmin, bbox.ymin);
    Point bottomright = Point(bbox.xmax, bbox.ymax);
    float boxTickness  = 1.5;
    float fontSize     = 0.5;
    float fontTickness = 1.0;
    if (box_width>60) {
        boxTickness  = 2.0;
        fontSize     = 0.7;
        fontTickness = 2.0;
    }
    rectangle(img, topleft, bottomright, box_color, boxTickness);

    // Draw a background box
    // Point center        = Point((bbox.xmin + bbox.xmax)/2, (bbox.ymin + bbox.ymax)/2);
    // Point t_topleft     = center + Point(-60, -25);
    // Point t_bottomright = center + Point( 60, 25);
    // rectangle(img, t_topleft, t_bottomright, text_bg_color, -1);

    // Draw text with detected class
    putText(img, label_str, topleft + Point(5, 20), FONT_HERSHEY_SIMPLEX, fontSize, text_color, fontTickness);
    // if (range < 10)
    // {
        putText(img, position_str, topleft + Point(5, 45), FONT_HERSHEY_SIMPLEX, fontSize-0.1, text_color, fontTickness);
    // }

    return img;
}

/**
 * @brief  VizObjdetRange ROS node class
 */

class VizObjdetRange
{
public:
    /**
     * @brief { Overlays the bounding boxes on the image }
     *
     */

    VizObjdetRange(ros::NodeHandle *nh, ros::NodeHandle *private_nh)
    {
        // std::string rawDisparityTopic;
        std::string rectImgTopic;
        std::string fusedObjRangeTopic;
        std::string outImgTopic;
        std::vector<int> vec;

        // input topics
        // private_nh->param("disparity_topic", rawDisparityTopic, std::string(""));
        private_nh->param("fused_obj_range_topic", fusedObjRangeTopic, std::string(""));
        private_nh->param("rectified_image_topic", rectImgTopic, std::string(""));
        private_nh->param("output_image_topic", outImgTopic, std::string(""));
        private_nh->param("box_color_rgb", vec, {0,0,0});
        m_box_color = Scalar(vec[0], vec[1], vec[2]) ;
        private_nh->param("patch_color_rgb", vec, {0,0,0});
        m_patch_color = Scalar(vec[0], vec[1], vec[2]) ;
        private_nh->param("text_color_rgb", vec, {0,0,0});
        m_text_color = Scalar(vec[0], vec[1], vec[2]) ;
        private_nh->param("text_bg_color_rgb", vec, {0,0,0});
        m_text_bg_color = Scalar(vec[0], vec[1], vec[2]);

        m_pub = nh->advertise<Image>(outImgTopic, 1);

        message_filters::Subscriber<Image> rectImgSub(*nh, rectImgTopic, 1);
        message_filters::Subscriber<Detection2Dp> detectSub(*nh, fusedObjRangeTopic, 1);

        typedef sync_policies::ApproximateTime<Detection2Dp, Image> SyncPolicy;
        Synchronizer<SyncPolicy> sync(SyncPolicy(10), detectSub, rectImgSub);
        sync.registerCallback(boost::bind(&VizObjdetRange::callback_VizObjdetRange, this, _1, _2));

        ros::spin();
    }

    ~VizObjdetRange() { }

    void callback_VizObjdetRange(const Detection2Dp::ConstPtr& detPtr,
                                 const ImageConstPtr& imagePtr)
    {
        cv_bridge::CvImagePtr cv_outImgPtr;
        cv_outImgPtr = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::RGB8);
        int32_t class_id;
        int32_t num_objects = detPtr->num_objects;
        BoundingBox2Dp bbox;
        cv::Scalar box_color;

        for (int32_t i = 0; i < num_objects; i++)
        {
            bbox = detPtr->bounding_boxes[i];
            box_color = (bbox.label_id==92) ? m_patch_color : m_box_color;

            overlayBoundingBox(cv_outImgPtr->image,
                               bbox,
                               box_color,
                               m_text_color,
                               m_text_bg_color);
        }

        m_pub.publish(cv_outImgPtr->toImageMsg());
    }

private:
    ros::Publisher m_pub;
    cv::Scalar m_box_color;
    cv::Scalar m_patch_color;
    cv::Scalar m_text_color;
    cv::Scalar m_text_bg_color;
};

/**
 * Main
 */
int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "app_viz_objdet_range");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        VizObjdetRange vizObjdetRange(&nh, &private_nh);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }
}

