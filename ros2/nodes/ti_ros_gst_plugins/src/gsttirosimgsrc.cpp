/*
 * Copyright (c) [2023] Texas Instruments Incorporated
 * 
 * All rights reserved not granted herein.
 * 
 * Limited License.  
 * 
 * Texas Instruments Incorporated grants a world-wide, royalty-free, 
 * non-exclusive license under copyrights and patents it now or hereafter 
 * owns or controls to make, have made, use, import, offer to sell and sell 
 * ("Utilize") this software subject to the terms herein.  With respect to 
 * the foregoing patent license, such license is granted  solely to the extent
 * that any such patent is necessary to Utilize the software alone. 
 * The patent license shall not apply to any combinations which include 
 * this software, other than combinations with devices manufactured by or
 * for TI (“TI Devices”).  No hardware patent is licensed hereunder.
 * 
 * Redistributions must preserve existing copyright notices and reproduce 
 * this license (including the above copyright notice and the disclaimer 
 * and (if applicable) source code license limitations below) in the 
 * documentation and/or other materials provided with the distribution
 * 
 * Redistribution and use in binary form, without modification, are permitted 
 * provided that the following conditions are met:
 * 
 * *	No reverse engineering, decompilation, or disassembly of this software 
 *      is permitted with respect to any software provided in binary form.
 * 
 * *	Any redistribution and use are licensed by TI for use only with TI Devices.
 * 
 * *	Nothing shall obligate TI to provide you with source code for the
 *      software licensed and provided to you in object code.
 * 
 * If software source code is provided to you, modification and redistribution
 * of the source code are permitted provided that the following conditions are met:
 * 
 * *	Any redistribution and use of the source code, including any resulting 
 *      derivative works, are licensed by TI for use only with TI Devices.
 * 
 * *	Any redistribution and use of any object code compiled from the source
 *      code and any resulting derivative works, are licensed by TI for use 
 *      only with TI Devices.
 * 
 * Neither the name of Texas Instruments Incorporated nor the names of its 
 * suppliers may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 * 
 * DISCLAIMER.
 * 
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS 
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
extern "C"
{

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef BUILD_FOR_TARGET
#include <edgeai_arm_neon_utils.h>
#endif

}

/* Standard headers */
#include <queue>
#include <mutex>
#include <chrono>
#include <condition_variable>

/* Gstreamer dependencies */
#include <glib.h>
#include <gst/video/video.h>
#include <gst/video/video-format.h>

/* ROS dependencies */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "gsttirosimgsrc.h"
#include "gsttirosutils.h"

#define MEMORY_ALIGNMENT 128
#define TIMEOUT_MSG_SEC 5
#define DEFAULT_MSG_QUEUE_MAX 1 // max ros msg to store in queue before dropping

/* Formats definition */
#define TI_ROS_IMG_SRC_SUPPORTED_FORMATS_SRC "{NV12, UYVY, RGB, GRAY8}"
#define TI_ROS_IMG_SRC_SUPPORTED_WIDTH "[1 , 8192]"
#define TI_ROS_IMG_SRC_SUPPORTED_HEIGHT "[1 , 8192]"

/* Src caps */
#define TI_ROS_IMG_SRC_STATIC_CAPS_SRC                           \
  "video/x-raw, "                                                \
  "format = (string) " TI_ROS_IMG_SRC_SUPPORTED_FORMATS_SRC ", " \
  "width = " TI_ROS_IMG_SRC_SUPPORTED_WIDTH ", "                 \
  "height = " TI_ROS_IMG_SRC_SUPPORTED_HEIGHT                    \
  "; "                                                           \
  "image/jpeg, "                                                 \
  "width = " TI_ROS_IMG_SRC_SUPPORTED_WIDTH ", "                 \
  "height = " TI_ROS_IMG_SRC_SUPPORTED_HEIGHT                    \

/* Properties definition */
enum
{
  PROP_0,
  PROP_CAPS,
  PROP_ROS_TOPIC,
  PROP_ROS_NODE_NAME,
  PROP_ROS_NODE_NAMESPACE,
  PROP
};

/* Pads definitions */
static
    GstStaticPadTemplate
    src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (TI_ROS_IMG_SRC_STATIC_CAPS_SRC)
    );

struct _GstTIRosImgSrc
{
  GstBaseSrc
      element;
  gchar *
      initial_caps;
  gboolean
      initalize_caps;
  gint
      width;
  gint
      height;
  gchar *
      ros_encoding;
  gsize
      buffer_size;
  gboolean
      ros_is_compressed_image;
  gchar *
      ros_topic;
  gchar *
      ros_node_name;
  gchar *
      ros_node_namespace;
  rclcpp::Context::SharedPtr
      ros_context;
  rclcpp::Executor::SharedPtr
      ros_executor;
  rclcpp::Node::SharedPtr
      ros_node;
  rclcpp::Logger
      ros_logger;
  rclcpp::Clock::SharedPtr
      ros_clock;
  std::thread
      ros_spin_thread;
  rclcpp::Time
      ros_stream_start;
  rcl_time_point_value_t
      ros_stream_start_prop;
  GstClockTimeDiff
      ros_clock_offset;
  gsize
      ros_msg_queue_max;
  std::mutex 
      ros_msg_queue_mtx;
  std::condition_variable
      ros_msg_queue_cv;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      ros_image_raw_subscriber;
  std::queue<sensor_msgs::msg::Image::ConstSharedPtr>
      ros_image_raw_msg_queue;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      ros_image_compressed_subscriber;
  std::queue<sensor_msgs::msg::CompressedImage::ConstSharedPtr>
      ros_image_compressed_msg_queue;
  gboolean
      stop;
};

GST_DEBUG_CATEGORY_STATIC (gst_ti_ros_img_src_debug);
#define GST_CAT_DEFAULT gst_ti_ros_img_src_debug

#define gst_ti_ros_img_src_parent_class parent_class
G_DEFINE_TYPE (GstTIRosImgSrc, gst_ti_ros_img_src, GST_TYPE_BASE_SRC);

static void
gst_ti_ros_img_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void
gst_ti_ros_img_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static
  GstCaps*
gst_ti_ros_img_src_get_caps (GstBaseSrc * src, GstCaps * filter);
static
  gboolean
gst_ti_ros_img_src_query (GstBaseSrc * src, GstQuery * query);
static
    gboolean
gst_ti_ros_img_src_decide_allocation (GstBaseSrc * src, GstQuery * query);
static
  GstFlowReturn
gst_ti_ros_img_src_create_output_buffer (GstTIRosImgSrc * self, GstBuffer ** outbuf);
static
  GstStateChangeReturn
gst_ti_ros_img_src_change_state (GstElement * element, GstStateChange transition);
static 
  GstFlowReturn
gst_ti_ros_img_src_create (GstBaseSrc * src, guint64 offset, guint size,
  GstBuffer **buf);
static
  sensor_msgs::msg::Image::ConstSharedPtr
gst_ti_ros_img_src_get_image_raw_msg (GstTIRosImgSrc * self);
static void
gst_ti_ros_img_src_image_raw_subscriber_callback (GstTIRosImgSrc * self,
  sensor_msgs::msg::Image::ConstSharedPtr msg);
static
  sensor_msgs::msg::CompressedImage::ConstSharedPtr
gst_ti_ros_img_src_get_image_compressed_msg (GstTIRosImgSrc * self);
static void
gst_ti_ros_img_src_image_compressed_subscriber_callback (GstTIRosImgSrc * self,
  sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
static
  gboolean
gst_ti_ros_img_src_open (GstTIRosImgSrc * self);
static void
gst_ti_ros_img_src_close (GstTIRosImgSrc * self);
static void
gst_ti_ros_img_src_spin_wrapper(GstTIRosImgSrc * self);
/* Initialize the plugin's class */
static void
gst_ti_ros_img_src_class_init (GstTIRosImgSrcClass * klass)
{
  GObjectClass *
      gobject_class = NULL;
  GstBaseSrcClass *
      gstbasesrc_class = NULL;
  GstElementClass *
      gstelement_class = NULL;

  gobject_class = G_OBJECT_CLASS (klass);
  gstbasesrc_class = GST_BASE_SRC_CLASS (klass);
  gstelement_class = GST_ELEMENT_CLASS (klass);

  gst_element_class_set_details_simple (gstelement_class,
      "TI Ros Img Src",
      "Source",
      "ROS2 source for Image/Compressed_Image msgs",
      "Abhay Chirania <a-chirania@ti.com>");

  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&src_template));

  gobject_class->set_property = gst_ti_ros_img_src_set_property;
  gobject_class->get_property = gst_ti_ros_img_src_get_property;

  g_object_class_install_property (gobject_class, PROP_CAPS,
      g_param_spec_string ("caps", "Intial Caps",
          "Fixed initial caps of the frame",
          NULL,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
              GST_PARAM_MUTABLE_READY)));
  g_object_class_install_property (gobject_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "ROS Topic",
          "ROS Topic to subscribe to",
          NULL,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
              GST_PARAM_MUTABLE_READY)));
  g_object_class_install_property (gobject_class, PROP_ROS_NODE_NAME,
      g_param_spec_string ("ros-node-name", "ROS Node Name",
          "Name of ROS NODE",
          "tirosimgsrc_node",
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
              GST_PARAM_MUTABLE_READY)));
  g_object_class_install_property (gobject_class, PROP_ROS_NODE_NAMESPACE,
      g_param_spec_string ("ros-node-namespace", "ROS Node Namespace",
          "Namespace of ROS NODE",
          "",
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
              GST_PARAM_MUTABLE_READY)));

  gstbasesrc_class->get_caps = GST_DEBUG_FUNCPTR (gst_ti_ros_img_src_get_caps);
  gstbasesrc_class->query = GST_DEBUG_FUNCPTR(gst_ti_ros_img_src_query);
  gstbasesrc_class->decide_allocation =
      GST_DEBUG_FUNCPTR(gst_ti_ros_img_src_decide_allocation);
  gstbasesrc_class->create = GST_DEBUG_FUNCPTR(gst_ti_ros_img_src_create);

  gstelement_class->change_state =
      GST_DEBUG_FUNCPTR (gst_ti_ros_img_src_change_state);

  GST_DEBUG_CATEGORY_INIT (gst_ti_ros_img_src_debug,
      "tirosimgsrc", 0, "TI Ros Img Src");
}

/* Initialize the new element
 * Initialize instance structure
 */
static void
gst_ti_ros_img_src_init (GstTIRosImgSrc * self)
{
  GST_LOG_OBJECT (self, "init");
  self->initial_caps = NULL;
  self->initalize_caps = TRUE;
  self->width = 0;
  self->height = 0;
  self->ros_encoding = NULL;
  self->buffer_size = 0;
  self->ros_is_compressed_image = FALSE;
  self->ros_topic = NULL;
  self->ros_node_name = g_strdup("tirosimgsrc_node");
  self->ros_node_namespace = g_strdup("");
  self->ros_msg_queue_max = DEFAULT_MSG_QUEUE_MAX;
  self->ros_image_raw_msg_queue =
      std::queue<sensor_msgs::msg::Image::ConstSharedPtr>();
  self->ros_image_compressed_msg_queue =
      std::queue<sensor_msgs::msg::CompressedImage::ConstSharedPtr>();
  self->stop = FALSE;

  gst_base_src_set_live (GST_BASE_SRC (self), TRUE);
  gst_base_src_set_format (GST_BASE_SRC (self), GST_FORMAT_TIME);
  gst_base_src_set_do_timestamp (GST_BASE_SRC (self), TRUE);

  return;
}

static void
gst_ti_ros_img_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstTIRosImgSrc *
      self = GST_TI_ROS_IMG_SRC (object);

  GST_LOG_OBJECT (self, "set_property");

  GST_OBJECT_LOCK (object);
  switch (prop_id) {
    case PROP_CAPS:
      if (!self->initalize_caps) {
        GST_WARNING_OBJECT (self, "can't initialize caps once set already.");
      } else {
        self->initial_caps = g_value_dup_string (value);
        self->initalize_caps = FALSE;
      }
      break;

    case PROP_ROS_TOPIC:
      if (self->ros_node) {
        RCLCPP_WARN(self->ros_logger, "can't change topic once opened.");
      } else {
        self->ros_topic = g_value_dup_string (value);
      }
      break;

    case PROP_ROS_NODE_NAME:
      if (self->ros_node) {
        RCLCPP_WARN(self->ros_logger, "can't change node name once opened.");
      } else {
        self->ros_node_name = g_value_dup_string (value);
      }
      break;

    case PROP_ROS_NODE_NAMESPACE:
      if (self->ros_node) {
        RCLCPP_WARN(self->ros_logger, "can't change node namespace once opened.");
      } else {
        self->ros_node_namespace = g_value_dup_string (value);
      }
      break;
  
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (object);
}

static void
gst_ti_ros_img_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstTIRosImgSrc *
      self = GST_TI_ROS_IMG_SRC (object);

  GST_LOG_OBJECT (self, "get_property");

  GST_OBJECT_LOCK (object);
  switch (prop_id) {
    case PROP_CAPS:
      g_value_set_string(value, self->initial_caps);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, self->ros_topic);
      break;

    case PROP_ROS_NODE_NAME:
      g_value_set_string(value, self->ros_node_name);
      break;

    case PROP_ROS_NODE_NAMESPACE:
      g_value_set_string(value, self->ros_node_namespace);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (object);
}

static
  gboolean
gst_ti_ros_img_src_query (GstBaseSrc * src, GstQuery * query)
{
  gboolean ret;

  switch (GST_QUERY_TYPE (query)) {
    case GST_QUERY_SCHEDULING:
    {
      gst_query_set_scheduling (query, GST_SCHEDULING_FLAG_SEQUENTIAL, 1, -1,
          0);
      gst_query_add_scheduling_mode (query, GST_PAD_MODE_PUSH);
      ret = TRUE;
      break;
    }
    default:
      ret = GST_BASE_SRC_CLASS (gst_ti_ros_img_src_parent_class)->query (src, query);
      break;
  }
  return ret;
}

static
  GstCaps*
gst_ti_ros_img_src_get_caps (GstBaseSrc * src, GstCaps * filter)
{
  GstTIRosImgSrc *
      self = GST_TI_ROS_IMG_SRC (src);

  GstCaps * caps;
  const gchar * caps_str;
  GstStructure * caps_struct;
  GstVideoInfo * video_info;
  const gchar * format_str;
  GstVideoFormat format_enum;

  static sensor_msgs::msg::Image::ConstSharedPtr msg;

  if(NULL != self->initial_caps) {

    GST_DEBUG_OBJECT (src, "Get Caps from caps property string");
    caps = gst_caps_from_string(self->initial_caps);
    caps_struct = gst_caps_get_structure (caps, 0);

    caps_str = gst_structure_get_name (caps_struct);

    if(!gst_structure_get_int (caps_struct, "width", &self->width)) {
      GST_ERROR_OBJECT (self, "caps property missing width");
      return NULL;
    }

    if(!gst_structure_get_int (caps_struct, "height", &self->height)) {
      GST_ERROR_OBJECT (self, "caps property missing height");
      return NULL;
    }

    if (0 == g_strcmp0(caps_str, "image/jpeg")) {

      self->ros_is_compressed_image = TRUE;

    } else {

      format_str = gst_structure_get_string (caps_struct, "format");
      if(NULL == format_str) {
        GST_ERROR_OBJECT (self, "caps property missing format");
        return NULL;
      }

      format_enum =  gst_video_format_from_string (format_str);

      self->ros_encoding = g_strdup(
        ti_ros_gst_plugins_utils::get_ros_encoding(format_enum).c_str()
        );

      if(0 == g_strcmp0(self->ros_encoding, "UNKNOWN")) {
        GST_ERROR_OBJECT (self,
          "format %s not supported", format_str);
        return NULL;
      }

    }

  } else {

    // Look for ros raw image msg, to get width and height
    GST_DEBUG_OBJECT (src, "Get Caps from ros msg");

    // return if ros_node isnt up
    if(NULL == self->ros_node)
    {
      GST_DEBUG_OBJECT (self, "Cannot get caps from ros msg, node is not up.");
      return gst_pad_get_pad_template_caps (GST_BASE_SRC (self)->srcpad);
    }

    do
    {
      RCLCPP_INFO (self->ros_logger,
          "waiting for first msg from [%s] to parse caps.", self->ros_topic);
      msg = gst_ti_ros_img_src_get_image_raw_msg(self);
      if (NULL == msg) {
        RCLCPP_WARN (self->ros_logger,
            "timeout after %d seconds.", TIMEOUT_MSG_SEC);
      }
    } while (NULL == msg);

    format_enum = ti_ros_gst_plugins_utils::get_gst_video_format \
                                              (std::string(self->ros_encoding));

    if (GST_VIDEO_FORMAT_UNKNOWN == format_enum) {
      RCLCPP_ERROR(self->ros_logger,
        "Ros encoding format %s not supported", self->ros_encoding);
      return NULL;
    }

    format_str = gst_video_format_to_string (format_enum);

    caps = gst_caps_new_simple ("video/x-raw",
                                "width", G_TYPE_INT, self->width,
                                "height", G_TYPE_INT, self->height,
                                "format", G_TYPE_STRING, format_str,
                                NULL);
  }

  if (filter) {
    GstCaps *tmp = caps;
    caps = gst_caps_intersect (caps, filter);
    gst_caps_unref (tmp);
  }

  video_info = gst_video_info_new();
  gst_video_info_from_caps(video_info,caps);

  // Get buffer size from caps in case allocation is needed.
  if (self->ros_is_compressed_image)
  {
    // Change this to allocate depending on format
    self->buffer_size = self->width * self->height * 3;
  }
  else
  {
    self->buffer_size = GST_VIDEO_INFO_SIZE (video_info);
  }

  gst_video_info_free(video_info);

  GST_DEBUG_OBJECT (self, "Resulting caps are %" GST_PTR_FORMAT, (void*) caps);
  GST_DEBUG_OBJECT (self, "Resulting buffer-size(if allocated) is %ld",
    self->buffer_size);
  
  return caps;
}

static
    gboolean
gst_ti_ros_img_src_decide_allocation (GstBaseSrc * src, GstQuery * query)
{
  GstTIRosImgSrc *
      self = GST_TI_ROS_IMG_SRC (src);
  GstBufferPool *
      pool = NULL;
  gboolean ret = TRUE;
  guint npool = 0;
  gboolean pool_needed = TRUE;

  GST_LOG_OBJECT (self, "Decide allocation");

  for (npool = 0; npool < gst_query_get_n_allocation_pools (query); ++npool) {
    GstBufferPool *pool = NULL;

    gst_query_parse_nth_allocation_pool (query, npool, &pool, NULL, NULL, NULL);

    if (NULL == pool) {
      GST_DEBUG_OBJECT (self, "No pool in query position: %d, ignoring", npool);
      gst_query_remove_nth_allocation_pool (query, npool);
      continue;
    }

    /* Use pool if found */
    if (pool) {
      pool_needed = FALSE;
    }

    gst_object_unref (pool);
    pool = NULL;
  }

  if (pool_needed) {

    GST_DEBUG_OBJECT (self, "Pool needed, allocating buffer\n");

    GstStructure *config;
    GstCaps *caps;
    GstAllocationParams alloc_params;

    gst_query_parse_allocation (query, &caps, NULL);
    pool = gst_buffer_pool_new ();
    config = gst_buffer_pool_get_config (pool);
    gst_buffer_pool_config_set_params (config,
                                       caps,
                                       self->buffer_size,
                                       1,
                                       4);

    gst_allocation_params_init(&alloc_params);
    alloc_params.align = MEMORY_ALIGNMENT - 1;

    gst_buffer_pool_config_set_allocator (config,
                                          NULL,
                                          &alloc_params);
    gst_buffer_pool_set_config(pool, config);
    gst_query_add_allocation_pool (query, pool, self->buffer_size, 1, 4);

    ret = gst_buffer_pool_set_active (GST_BUFFER_POOL (pool), TRUE);
    if (!ret) {
      GST_ERROR_OBJECT (self, "Failed to activate bufferpool");
      goto exit;
    }
    gst_object_unref (pool);
    pool = NULL;
  }

  gst_base_src_set_blocksize(src, self->buffer_size);

exit:
  return ret;
}

static
  GstFlowReturn
gst_ti_ros_img_src_create_output_buffer (GstTIRosImgSrc * self, GstBuffer ** outbuf)
{
  GstBaseSrc *src = NULL;
  GstBufferPool *pool;
  GstFlowReturn ret = GST_FLOW_ERROR;

  g_return_val_if_fail (self, ret);

  GST_LOG_OBJECT (self, "get output buffer");

  src = GST_BASE_SRC (self);

  pool = gst_base_src_get_buffer_pool  (src);

  if (pool) {
    if (!gst_buffer_pool_is_active (pool)) {
      if (!gst_buffer_pool_set_active (pool, TRUE)) {
        GST_ERROR_OBJECT (self, "Failed to activate bufferpool");
        goto exit;
      }
    }

    ret = gst_buffer_pool_acquire_buffer (pool, outbuf, NULL);
    gst_object_unref (pool);
    pool = NULL;
    ret = GST_FLOW_OK;
  } else {
    GST_ERROR_OBJECT (self,
        "An output buffer can only be created from a buffer pool");
  }

exit:
  return ret;
}

static
  GstStateChangeReturn
gst_ti_ros_img_src_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  GstTIRosImgSrc * self = GST_TI_ROS_IMG_SRC (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_NULL_TO_READY:
    {
      if (!gst_ti_ros_img_src_open(self)) {
        return GST_STATE_CHANGE_FAILURE;
      }
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    {
      self->stop = TRUE;
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    {
      self->ros_clock_offset =
        ti_ros_gst_plugins_utils::get_colck_offset (GST_ELEMENT_CLOCK(self),
                                                    self->ros_clock->now());
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (gst_ti_ros_img_src_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
    {
      gst_ti_ros_img_src_close(self);
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    {
      self->stop = FALSE;
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    default:
      break;
  }

  return ret;
}

static 
  GstFlowReturn
gst_ti_ros_img_src_create (GstBaseSrc * src, guint64 offset, guint size,
  GstBuffer **buf)
{
  GstTIRosImgSrc *
      self = GST_TI_ROS_IMG_SRC (src);
  GstFlowReturn ret = GST_FLOW_ERROR;
  GstMapInfo info;
  gsize ros_msg_size;

  (void)offset;
  (void)size;

  sensor_msgs::msg::CompressedImage::ConstSharedPtr image_compressed_msg;
  sensor_msgs::msg::Image::ConstSharedPtr image_raw_msg;

  GST_LOG_OBJECT (self, "create");

  if (self->stop) {
    goto exit;
  }

  /* Get ros msg */
  if (self->ros_is_compressed_image) {
    image_compressed_msg = gst_ti_ros_img_src_get_image_compressed_msg(self);
    if (NULL == image_compressed_msg)
    {
      RCLCPP_WARN (self->ros_logger,
        "timeout after %d seconds.", TIMEOUT_MSG_SEC);
      ret = GST_FLOW_EOS;
      goto exit;
    }
    {
      std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);
        self->ros_image_compressed_msg_queue.pop();
    }
    ros_msg_size = image_compressed_msg->data.size();
  } else {
    image_raw_msg = gst_ti_ros_img_src_get_image_raw_msg(self);
    if (NULL == image_raw_msg)
    {
      RCLCPP_WARN (self->ros_logger,
        "timeout after %d seconds.", TIMEOUT_MSG_SEC);
      ret = GST_FLOW_EOS;
      goto exit;
    }
    {
      std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);
        self->ros_image_raw_msg_queue.pop();
    }
    ros_msg_size = image_raw_msg->data.size();
  }

  if (self->stop) {
    goto exit;
  }

  ret = gst_ti_ros_img_src_create_output_buffer (self, buf);

  if (GST_FLOW_OK != ret) {
    GST_ERROR_OBJECT (self, "Unable to acquire output buffer");
    goto exit;
  }

  /* Map output Buffer */
  if (!gst_buffer_map (*buf, &info, GST_MAP_WRITE)) {
      GST_ERROR_OBJECT (self, "failed to map buffer");
      goto exit;
  }

  if(self->ros_is_compressed_image) {
    if (info.size < ros_msg_size) {
      GST_ERROR_OBJECT (self,
        "Skipping as allocated buffer size (%ld) "
        "less than ros msg data size (%ld)",
        info.size, ros_msg_size);
      goto skip;
    }
#ifdef BUILD_FOR_TARGET
    memcpy_neon (info.data, image_compressed_msg->data.data(), ros_msg_size);
#else
    memcpy (info.data, image_compressed_msg->data.data(), ros_msg_size);
#endif
  } else {
    if(info.size != ros_msg_size) {
      GST_ERROR_OBJECT (self,
        "Skipping as allocated buffer size (%ld) "
        "does not match ros msg data size (%ld)",
        info.size, ros_msg_size);
      goto skip;
    }
#ifdef BUILD_FOR_TARGET
    memcpy_neon (info.data, image_raw_msg->data.data(), info.size);
#else
    memcpy (info.data, image_raw_msg->data.data(), info.size);
#endif
  }

skip:
  gst_buffer_unmap (*buf, &info);

exit:
    return ret;
}

static
  sensor_msgs::msg::CompressedImage::ConstSharedPtr
gst_ti_ros_img_src_get_image_compressed_msg (GstTIRosImgSrc * self)
{
  std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);
  sensor_msgs::msg::CompressedImage::ConstSharedPtr msg;

  // Wait for msg if ros_image_compressed_msg_queue is empty
  while(self->ros_image_compressed_msg_queue.empty())
  {
    if (self->ros_msg_queue_cv.wait_for(lck,std::chrono::seconds(TIMEOUT_MSG_SEC))
        == std::cv_status::timeout) {
      return NULL;
    }
  }

  // Get the msg from front of msg_queue
  msg = self->ros_image_compressed_msg_queue.front();

  return msg;
}

static void
gst_ti_ros_img_src_image_compressed_subscriber_callback (GstTIRosImgSrc * self,
  sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);

  // Push msg to ros_image_compressed_msg_queue
  self->ros_image_compressed_msg_queue.push(msg);

  // Drop oldest msg if queue is completely filled
  while(self->ros_image_compressed_msg_queue.size() > self->ros_msg_queue_max)
  {
    self->ros_image_compressed_msg_queue.pop();
    RCLCPP_WARN(self->ros_logger,
      "Compressed Image message queue full, dropping oldest message");
  }

  self->ros_msg_queue_cv.notify_one();

}

static
  sensor_msgs::msg::Image::ConstSharedPtr
gst_ti_ros_img_src_get_image_raw_msg (GstTIRosImgSrc * self)
{
  std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);
  sensor_msgs::msg::Image::ConstSharedPtr msg;

  // Wait for msg if ros_image_raw_msg_queue is empty
  while(self->ros_image_raw_msg_queue.empty())
  {
    if (self->ros_msg_queue_cv.wait_for(lck,std::chrono::seconds(TIMEOUT_MSG_SEC))
        == std::cv_status::timeout) {
      return NULL;
    }
  }

  // Get the msg from front of msg_queue
  msg = self->ros_image_raw_msg_queue.front();

  return msg;
}

static void
gst_ti_ros_img_src_image_raw_subscriber_callback (GstTIRosImgSrc * self,
  sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (self->initalize_caps) {
    self->width = msg->width;
    self->height = msg->height;
    self->ros_encoding = g_strdup(msg->encoding.c_str());
    self->initalize_caps = FALSE;
  }

  if(self->width != (int) msg->width) {
    RCLCPP_ERROR(self->ros_logger,
                 "Image width changed, %d != %d",
                 self->width,
                 msg->width);
  }

  if(self->height != (int) msg->height) {
    RCLCPP_ERROR(self->ros_logger,
                 "Image height changed, %d != %d",
                 self->height,
                 msg->height);
  }

  if(0 != g_strcmp0(self->ros_encoding, msg->encoding.c_str())) {
    RCLCPP_ERROR(self->ros_logger,
                 "Image encoding changed, %s != %s",
                 self->ros_encoding,
                 msg->encoding.c_str());
  }

  std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);

  // Push msg to ros_image_raw_msg_queue
  self->ros_image_raw_msg_queue.push(msg);

  // Drop oldest msg if queue is completely filled
  while(self->ros_image_raw_msg_queue.size() > self->ros_msg_queue_max)
  {
    self->ros_image_raw_msg_queue.pop();
    RCLCPP_WARN(self->ros_logger,
      "Image Raw message queue full, dropping oldest message");
  }

  self->ros_msg_queue_cv.notify_one();

}

static
  gboolean
gst_ti_ros_img_src_open (GstTIRosImgSrc * self)
{

  GST_DEBUG_OBJECT (self, "open");
  
  using std::placeholders::_1;

  if (NULL == self->ros_topic) {
    GST_ERROR_OBJECT (self, "ROS Topic undefined.");
    return FALSE;
  }

  self->ros_context = std::make_shared<rclcpp::Context>();
  self->ros_context->init(0, NULL); 
  auto opts = rclcpp::NodeOptions();
  opts.context(self->ros_context);
  self->ros_node = std::make_shared<rclcpp::Node>(std::string(self->ros_node_name),
                                                  std::string(self->ros_node_namespace),
                                                  opts);

  auto ex_args = rclcpp::ExecutorOptions();
  ex_args.context = self->ros_context;
  self->ros_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(ex_args);
  self->ros_executor->add_node(self->ros_node);
  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  if (self->ros_is_compressed_image) {

    auto callback = [self] (sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
      gst_ti_ros_img_src_image_compressed_subscriber_callback(self, msg);
    };
    self->ros_image_compressed_subscriber =
      self->ros_node->create_subscription<sensor_msgs::msg::CompressedImage>(self->ros_topic,
                                                                             qos,
                                                                             callback);
    RCLCPP_INFO(self->ros_logger,
        "Subscribed to topic [%s] of type compressed image", self->ros_topic);

  } else {

    auto callback = [self] (sensor_msgs::msg::Image::ConstSharedPtr msg) {
      gst_ti_ros_img_src_image_raw_subscriber_callback(self, msg);
    };
    self->ros_image_raw_subscriber =
      self->ros_node->create_subscription<sensor_msgs::msg::Image>(self->ros_topic,
                                                                   qos,
                                                                   callback);
    RCLCPP_INFO(self->ros_logger,
        "Subscribed to topic [%s] of type raw image", self->ros_topic);

  }
  self->ros_logger = self->ros_node->get_logger();
  self->ros_clock = self->ros_node->get_clock();

  self->ros_spin_thread = std::thread{&gst_ti_ros_img_src_spin_wrapper, self};

  return TRUE;
}

static void
gst_ti_ros_img_src_close (GstTIRosImgSrc * self)
{
  GST_DEBUG_OBJECT (self, "close");

  if (NULL != self->ros_node) {
    self->ros_clock.reset();

    if (self->ros_is_compressed_image) {
      self->ros_image_compressed_subscriber.reset();
      std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);
      while(self->ros_image_compressed_msg_queue.size() > 0)
      {
        self->ros_image_compressed_msg_queue.pop();
      }
    } else {
      self->ros_image_raw_subscriber.reset();
      std::unique_lock<std::mutex> lck(self->ros_msg_queue_mtx);
      while(self->ros_image_raw_msg_queue.size() > 0)
      {
        self->ros_image_raw_msg_queue.pop();
      }
    }

    self->ros_executor->cancel();
    self->ros_spin_thread.join();
    self->ros_context->shutdown("gst closing tirosimgsrc");

    self->ros_context.reset();
    self->ros_executor.reset();
    self->ros_node.reset();
    self->ros_clock.reset();
  }
}

static void
gst_ti_ros_img_src_spin_wrapper (GstTIRosImgSrc * self)
{
  self->ros_executor->spin();
}