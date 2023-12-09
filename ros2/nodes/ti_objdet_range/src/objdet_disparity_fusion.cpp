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
#include "objdet_disparity.h"

static void sigHandler(int32_t sig)
{
    (void) sig;
    rclcpp::shutdown();
    std::exit(EXIT_SUCCESS);
}

/**
 * @brief Function for calculating the median
 */
static float findMedian(std::vector<int> a, int n)
{
    if (n == 0)
    {
        return 0.0;
    }

    if (n % 2 == 0)
    {
        // applying nth_element on n/2th index
        nth_element(a.begin(), a.begin() + n / 2, a.end());

        // applying nth_element on (n-1)/2 th index
        nth_element(a.begin(), a.begin() + (n - 1) / 2, a.end());

        // find the average of value at index N/2 and (N-1)/2
        return (float)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
    }
    else
    {
        // applying nth_element on n/2
        nth_element(a.begin(), a.begin() + n / 2, a.end());

        // value at index (N/2)th is the median
        return (float)a[n / 2];
    }
}

/**
 * @brief Function for calculating the max
 */
static float findMax(std::vector<int> a, int n)
{
    if (n == 0)
    {
        return 0.0;
    }

    int maxval = 0;
    for (int32_t i = 0; i < n; i++)
    {
        if (a[i] > maxval) maxval = a[i];
    }

    return (float)maxval;
}

/**
 * @brief Function for calculating the mean
 */
static float findMean(std::vector<int> a, int n)
{
    if (n == 0)
    {
        return 0.0;
    }

    long sum = 0;
    for (int32_t i = 0; i < n; i++)
    {
        sum += a[i];
    }

    return (float)sum / n;
}

/**
 * @brief Wrapper function for filters
 */
static float runFilter(const rclcpp::Logger &logger, std::vector<int> patch, const int filterType)
{
    if (filterType==0)
    {
        return findMax(patch, (int) patch.size());
    }
    else if (filterType==1)
    {
        return findMedian(patch, (int) patch.size());
    }
    else if (filterType==2)
    {
        return findMean(patch, (int) patch.size());
    }
    else
    {
        // RCLCPP_ERROR(this->get_logger(), "filterType = %d not valid", filterType);
        return -1;
    }
}


/**
 * @brief { Calculate and add spatial information to each bounding box }
 *
 */
ObjdetDisparityFusion::ObjdetDisparityFusion(
    const std::string &name,
    const rclcpp::NodeOptions &options)
    : Node(name, options)
{

    // parse params from launch file
    get_parameter_or("disparity_topic", m_rawDisparityTopic, std::string("camera/disparity/raw"));
    get_parameter_or("vision_cnn_tensor_topic", m_tensorTopic, std::string("vision_cnn/tensor"));
    get_parameter_or("camera_info_topic", m_cameraInfoTopic, std::string("camera/right/camera_info"));
    get_parameter_or("outout_topic", m_outputTopic, std::string("camera/fused_objdet_range"));
    get_parameter_or("band_height", m_bandHeight, 7);
    get_parameter_or("camera_baseline", m_baseline, 0.12f);
    get_parameter_or("confidenceTh", m_confidenceTh, 1);
    get_parameter_or("disparity_filter", m_disparityFilter, 0);

    //==> PCL
    // get_parameter_or("horiz_scan_pcl", m_horizScanPcl, true);
    get_parameter_or("pcl_topic", m_pclTopic, std::string("camera/horiz_scan_pcl"));
    get_parameter_or("pcl_frame", m_pclFrame, std::string("camera_horiz_scan"));
    get_parameter_or("disparity_width", m_disparityWidth, 1280);
    get_parameter_or("pixels_to_exclude", m_pixelsToExclude, 128);
    get_parameter_or("patch_size", m_patchSize, 16);
    get_parameter_or("patch_stride", m_patchStride, 8);
    get_parameter_or("pcl_disparity_filter", m_pclDisparityFilter, 0);
    get_parameter_or("patch_row_to_use", m_patchRowToUse, 0);
    get_parameter_or("valid_pixel_ratio_th", m_validPixRatioTh, 0.3f);

    m_patchCols = (m_disparityWidth - m_pixelsToExclude - m_patchSize) / m_patchStride;
    RCLCPP_INFO(this->get_logger(), "m_patchCols = %d", m_patchCols);

    // subscriber object
    m_cameraInfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        m_cameraInfoTopic, 10, std::bind(&ObjdetDisparityFusion::callback_CameraInfo,
        this, std::placeholders::_1));

    // publisher object
    m_Detection2Dp_pub = this->create_publisher<common_msgs::msg::Detection2Dp>(m_outputTopic, 1);

    //==> PCL
    m_fltPatch = new FilteredPatch[m_patchCols];
    m_fltDispModified = new float[m_patchCols];

    m_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pclTopic, 1);

    // subscribers & synchronizer
    message_filters::Subscriber<Disparity> disparitySub(this, m_rawDisparityTopic);
    message_filters::Subscriber<Detection2D> tensorSub(this, m_tensorTopic);

    typedef sync_policies::ApproximateTime<Disparity, Detection2D> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), disparitySub, tensorSub);
    sync.registerCallback(std::bind(&ObjdetDisparityFusion::callback_ObjdetDisparityFusion,
        this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::spin(static_cast<rclcpp::Node::SharedPtr>(this));
}

ObjdetDisparityFusion::~ObjdetDisparityFusion()
{
    delete[] m_fltPatch;
    delete[] m_fltDispModified;
}

void ObjdetDisparityFusion::callback_CameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    m_cameraInfo = *msg;
    // Unsubscribe from the topic after receiving the first message
    if (m_cameraInfo_sub != nullptr)
    {
        m_cameraInfo_sub.reset();
    }
}

void ObjdetDisparityFusion::callback_ObjdetDisparityFusion(
    const Disparity::ConstSharedPtr& rawDispPtr,
    const Detection2D::ConstSharedPtr& detPtr)
{
    int32_t num_objects = detPtr->num_objects;
    BoundingBox2D bbox;
    Detection2Dp outDetArray;
    geometry_msgs::msg::Point32 position;

    // extract camera params from camera_info
    m_focalLength = m_cameraInfo.p[0];
    m_distCenterX = m_cameraInfo.p[2];
    m_distCenterY = m_cameraInfo.p[6];

    // pass though
    outDetArray.header = detPtr->header;
    outDetArray.num_objects = detPtr->num_objects + 1;
    outDetArray.bounding_boxes.assign(outDetArray.num_objects, BoundingBox2Dp{});

    for (int32_t idx = 0; idx < num_objects; idx++)
    {
        bbox = detPtr->bounding_boxes[idx];

        // coordinate values of bounding box are sometimes out of range
        // when object is near the edge of the image.
        if (bbox.xmin < 0) { bbox.xmin = 0; }
        if (bbox.ymin < 0) { bbox.ymin = 0; }
        if (bbox.xmax >= rawDispPtr->width)  { bbox.xmax = rawDispPtr->width-1; }
        if (bbox.ymax >= rawDispPtr->height) { bbox.ymax = rawDispPtr->height-1; }

        position = calcBboxPosition(bbox,
                                    (uint16_t *) rawDispPtr->data.data(),
                                    rawDispPtr->width,
                                    rawDispPtr->height);
        // RCLCPP_INFO(this->get_logger(),"class_id = %3d, confidence = %.2f, position: (%5.2f, %5.2f, %5.2f) ",
        //     bbox.label_id, bbox.confidence, position.x, position.y, position.z);

        // form the output message
        outDetArray.bounding_boxes[idx].xmin       = bbox.xmin;
        outDetArray.bounding_boxes[idx].ymin       = bbox.ymin;
        outDetArray.bounding_boxes[idx].xmax       = bbox.xmax;
        outDetArray.bounding_boxes[idx].ymax       = bbox.ymax;
        outDetArray.bounding_boxes[idx].label_id   = bbox.label_id;
        outDetArray.bounding_boxes[idx].confidence = bbox.confidence;
        outDetArray.bounding_boxes[idx].position   = position;
    }

    /*
    * Horizontally scan the disparity map and generate pcl::PointXYZ message
    */
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ pclPoint;
    geometry_msgs::msg::Point32 pos;
    int32_t rowIdx = m_patchRowToUse;
    int32_t idxStart = rowIdx*(m_disparityWidth*m_patchSize);
    float cx = ((float)m_patchSize - 1.0)/2.0;
    float cy = ((float)m_patchSize - 1.0)/2.0 + rowIdx*m_patchSize;
    int32_t pixelsPerPatch = m_patchSize*m_patchSize;
    float disp;
    int32_t numValidPix;
    float validPixelRatio;

    for(int32_t c = 0; c < m_patchCols; c++)
    {
        disp = extractProcessPatch((uint16_t *) rawDispPtr->data.data(),
                                   idxStart,
                                   m_disparityWidth,
                                   numValidPix);
        idxStart += m_patchStride;

        // calculate position in camera_right frame
        pos = disparity2Position(disp, cx, cy);

        validPixelRatio = (float)numValidPix / pixelsPerPatch;
        m_fltPatch[c].validPixelRatio = validPixelRatio;
        m_fltPatch[c].fltDisp = disp;
        m_fltPatch[c].cx      = cx;
        m_fltPatch[c].cy      = cy;
        // m_fltPatch[c].posx    = pos.x;
        // m_fltPatch[c].posy    = pos.y;
        // m_fltPatch[c].posz    = pos.z;
        // m_fltPatch[c].range   = point2Range(pos_);
        // RCLCPP_INFO(this->get_logger()," m_fltPatch[%2d].fltDisp = %6.0f, %.3f", c, m_fltPatch[c].fltDisp, m_fltPatch[c].validPixelRatio);

        // update center of patch
        cx += (float) m_patchStride;

        // check reliability and populate point cloud
        if ((validPixelRatio > m_validPixRatioTh))
        {
            m_fltDispModified[c] = m_fltPatch[c].fltDisp;

            //==> PCL
            pclPoint.x = pos.z;
            pclPoint.y = -(pos.x);
            pclPoint.z = -(pos.y);
            cloud.points.push_back(pclPoint);
        }
        else
        {
            m_fltDispModified[c] = -99.;
        }
    }

    // select a patch that has largest disparity
    int32_t optIdx = std::distance(m_fltDispModified,
        std::max_element(m_fltDispModified, m_fltDispModified + m_patchCols));
    // RCLCPP_INFO(this->get_logger(),"===> optIdx = %d, value = %6.0f, %.3f",
    //     optIdx, m_fltPatch[optIdx].fltDisp, m_fltPatch[optIdx].validPixelRatio);

    // include the selected patch as the last bounding box in the msg
    outDetArray.bounding_boxes[num_objects].xmin = m_fltPatch[optIdx].cx - (m_patchSize/2 - 1);
    outDetArray.bounding_boxes[num_objects].ymin = m_fltPatch[optIdx].cy - (m_patchSize/2 - 1);
    outDetArray.bounding_boxes[num_objects].xmax = m_fltPatch[optIdx].cx + m_patchSize/2;
    outDetArray.bounding_boxes[num_objects].ymax = m_fltPatch[optIdx].cy + m_patchSize/2;
    outDetArray.bounding_boxes[num_objects].label_id = CLASS_ID_PATCH;
    outDetArray.bounding_boxes[num_objects].confidence = 1.0;
    pos = disparity2Position(m_fltPatch[optIdx].fltDisp, m_fltPatch[optIdx].cx, m_fltPatch[optIdx].cy);
    outDetArray.bounding_boxes[num_objects].position.x = pos.x;
    outDetArray.bounding_boxes[num_objects].position.y = pos.y;
    outDetArray.bounding_boxes[num_objects].position.z = pos.z;

    // publish
    m_Detection2Dp_pub->publish(outDetArray);

    //==> PCL
    cloud.width = (int) cloud.points.size();
    cloud.height = 1; // height =1 implies this is not an "ordered" point
    cloud.header.frame_id = m_pclFrame;
    // RCLCPP_INFO(this->get_logger(),"cloud.width = %d", cloud.width);

    // convert the cloud to ROS message
    sensor_msgs::msg::PointCloud2 pcl_output;
    pcl::toROSMsg(cloud, pcl_output);
    m_pcl_pub->publish(pcl_output);
}

geometry_msgs::msg::Point32 ObjdetDisparityFusion::calcBboxPosition(
    const BoundingBox2D &bbox,
    const uint16_t *rawDisparity,
    const int32_t dispWidth,
    const int32_t dispHeight)
{
    uint16_t rawDisparityPix;
    uint16_t pixRawDisparity;
    uint8_t confidence;
    float fltDisparity;
    geometry_msgs::msg::Point32 position;

    // sanity check
    if (m_bandHeight > (bbox.ymax - bbox.ymin))
    {
        RCLCPP_WARN(this->get_logger(), "m_bandHeight is larger than height of the bounding box");
    }

    // center of of the bounding box
    float ctrX = ((float) bbox.xmin + bbox.xmax) / 2.0;
    float ctrY = ((float) bbox.ymin + bbox.ymax) / 2.0;

    // extract a horizontal band around (ctrX, ctrY)
    std::vector<int> horizBand;
    horizBand.reserve(m_bandHeight*(bbox.xmax-bbox.xmin));

    int32_t jStart = ctrY - m_bandHeight/2;
    int32_t jEnd   = jStart + m_bandHeight;

    for(int32_t j = jStart; j < jEnd; j++)
    {
        int32_t jstep = j*dispWidth;
        for(int32_t i = bbox.xmin; i < bbox.xmax; i++)
        {
            rawDisparityPix = rawDisparity[jstep + i];
            confidence = (rawDisparityPix & 0x07);
            if (confidence >= m_confidenceTh)
            {
                horizBand.push_back((int) ((rawDisparityPix & 0x7FFF) >> 3));
            }
        }
    }

    // filter the horizBand
    fltDisparity = runFilter(this->get_logger(), horizBand, m_disparityFilter);

    // calculate position in camera_right frame
    position = disparity2Position(fltDisparity, ctrX, ctrY);

    return position;
}

float ObjdetDisparityFusion::extractProcessPatch(
    const uint16_t *rawDisparity,
    const int32_t idxStart,
    const int32_t rowStep,
    int32_t& outValidPix)
{
    uint8_t confidence;
    uint16_t rawDisparityPix;
    int32_t jStep = idxStart;
    float fltPatch;

    std::vector<int> patch;
    patch.reserve(m_patchSize*m_patchSize);

    for(int32_t j = 0; j < m_patchSize; j++)
    {
        for(int32_t i = 0; i < m_patchSize; i++)
        {
            rawDisparityPix = rawDisparity[jStep + i];
            confidence = (rawDisparityPix & 0x07);
            if (confidence >= m_confidenceTh)
            {
                patch.push_back((int) ((rawDisparityPix & 0x7FFF) >> 3));
            }
        }
        jStep += rowStep;
    }

    // filter the patch
    fltPatch = runFilter(this->get_logger(), patch, m_pclDisparityFilter);

    outValidPix = (int32_t) patch.size();
    return fltPatch;
}

geometry_msgs::msg::Point32 ObjdetDisparityFusion::disparity2Position(
    const float& disp,
    const float& img_x,
    const float& img_y)
{
    geometry_msgs::msg::Point32 point;
    float baselineDisparityRatio = m_baseline/(disp * m_scale);

    point.z = m_focalLength * baselineDisparityRatio;
    point.x = (img_x - m_distCenterX) * baselineDisparityRatio;
    point.y = (img_y - m_distCenterY) * baselineDisparityRatio;

    return point;
}

// calculate range in X-Z plane in camera frame
float ObjdetDisparityFusion::point2Range(const geometry_msgs::msg::Point32& point)
{
    float range = sqrt(point.x*point.x + point.z*point.z);
    return range;
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

        auto objdetDisparityFusion = std::make_shared<ObjdetDisparityFusion>("objdet_disparity_fusion", nodeOptions);

        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}
