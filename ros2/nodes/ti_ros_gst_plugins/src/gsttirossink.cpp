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

/* Gstreamer dependencies */
#include <glib.h>
#include <gst/video/video.h>
#include <gst/video/video-format.h>

/* ROS dependencies */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

/* Common Msgs dependencies*/
#include <common_msgs/msg/detection2_d.hpp>
#include <common_msgs/msg/pose6_d.hpp>

#ifdef BUILD_FOR_TARGET
#include <yaml-cpp/yaml.h>
#endif

#include "gsttirossink.h"
#include "gsttirosutils.h"

/* Ros msg types*/
#define ROS_MSG_TYPE_UNDEFINED              -1
#define ROS_MSG_TYPE_RAW_IMAGE              0
#define ROS_MSG_TYPE_COMPRESSED_IMAGE_JPEG  1
#define ROS_MSG_TYPE_COMPRESSED_IMAGE_PNG   2
#define ROS_MSG_TYPE_STRING                 3
#define ROS_MSG_TYPE_DETECTION2D            4
#define ROS_MSG_TYPE_POSE6D                 5

#define GST_TYPE_PROP_ROS_MSG_TYPE (gst_ti_ros_sink_msg_get_type())

/* Sink caps */
#define TI_ROS_SINK_STATIC_CAPS_SINK                  \
  "video/x-raw, "                                     \
  "format = (string) {GRAY8, RGB, UYVY, NV12}"        \
  "; "                                                \
  "image/jpeg; "                                      \
  "image/png; "                                       \
  "text/x-raw"

/* Properties definition */
enum
{
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_NODE_NAME,
  PROP_ROS_NODE_NAMESPACE,
  PROP_ROS_MSG_TYPE,
  PROP
};

static GType
gst_ti_ros_sink_msg_get_type (void)
{
  static GType msg_type = 0;

  static const GEnumValue msg[] = {
    {ROS_MSG_TYPE_UNDEFINED, "Undefined", "undefined"},
    {ROS_MSG_TYPE_RAW_IMAGE, "Image", "image"},
    {ROS_MSG_TYPE_COMPRESSED_IMAGE_JPEG, "CompressedImageJpeg", "compressed-image-jpeg"},
    {ROS_MSG_TYPE_COMPRESSED_IMAGE_PNG, "CompressedImagePng", "compressed-image-png"},
    {ROS_MSG_TYPE_STRING, "String", "string"},
    {ROS_MSG_TYPE_DETECTION2D, "Detection2D", "detection2d"},
    {ROS_MSG_TYPE_POSE6D, "Pose6D", "pose6d"},
    {0, NULL, NULL},
  };
  if (!msg_type) {
    msg_type =
        g_enum_register_static ("GstTIRosSinkMsgType", msg);
  }
  return msg_type;
}

/* Pads definitions */
static
    GstStaticPadTemplate
    sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS (TI_ROS_SINK_STATIC_CAPS_SINK)
    );

struct _GstTIRosSink
{
  GstBaseSink
      element;
  gint
      ros_msg_type;
  gint
      prop_ros_msg_type;
  gint
      width;
  gint
      height;
  gchar *
      ros_encoding;
  gsize
      ros_step;
  gint
      ros_is_big_endian;
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
  GstClockTimeDiff
      ros_clock_offset;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      ros_image_raw_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      ros_image_compressed_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    ros_string_publisher;
  rclcpp::Publisher<common_msgs::msg::Detection2D>::SharedPtr
    ros_detection_2d_publisher;
  rclcpp::Publisher<common_msgs::msg::Pose6D>::SharedPtr
    ros_pose_6d_publisher;
};

GST_DEBUG_CATEGORY_STATIC (gst_ti_ros_sink_debug);
#define GST_CAT_DEFAULT gst_ti_ros_sink_debug

#define gst_ti_ros_sink_parent_class parent_class
G_DEFINE_TYPE (GstTIRosSink, gst_ti_ros_sink, GST_TYPE_BASE_SINK);

static void
gst_ti_ros_sink_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void
gst_ti_ros_sink_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static
  gboolean
gst_ti_ros_sink_set_caps (GstBaseSink * sink, GstCaps * caps);
static
  GstStateChangeReturn
gst_ti_ros_sink_change_state (GstElement * element,
    GstStateChange transition);
static
    GstFlowReturn
gst_ti_ros_sink_render (GstBaseSink * sink, GstBuffer * buffer);
static
  gboolean
gst_ti_ros_sink_open (GstTIRosSink * self);
static void
gst_ti_ros_sink_close (GstTIRosSink * self);
static void
gst_ti_ros_sink_spin_wrapper(GstTIRosSink * self);
static void
gst_ti_ros_sink_publish_msg (GstTIRosSink * self,
    guint ros_msg_type, GstMapInfo * info, rclcpp::Time msg_time);

/* Initialize the plugin's class */
static void
gst_ti_ros_sink_class_init (GstTIRosSinkClass * klass)
{
  GObjectClass *
      gobject_class = NULL;
  GstBaseSinkClass *
      gstbasesink_class = NULL;
  GstElementClass *
      gstelement_class = NULL;

  gobject_class = G_OBJECT_CLASS (klass);
  gstbasesink_class = GST_BASE_SINK_CLASS (klass);
  gstelement_class = GST_ELEMENT_CLASS (klass);

  gst_element_class_set_details_simple (gstelement_class,
      "TI Ros Sink",
      "Sink",
      "ROS2 sink",
      "Abhay Chirania <a-chirania@ti.com>");

  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&sink_template));

  gobject_class->set_property = gst_ti_ros_sink_set_property;
  gobject_class->get_property = gst_ti_ros_sink_get_property;

  g_object_class_install_property (gobject_class, PROP_ROS_TOPIC,
      g_param_spec_string ("ros-topic", "ROS Topic",
          "ROS Topic to publish to",
          NULL,
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
              GST_PARAM_MUTABLE_READY)));
  g_object_class_install_property (gobject_class, PROP_ROS_NODE_NAME,
      g_param_spec_string ("ros-node-name", "ROS Node Name",
          "Name of ROS NODE",
          "tirossink_node",
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
              GST_PARAM_MUTABLE_READY)));
  g_object_class_install_property (gobject_class, PROP_ROS_NODE_NAMESPACE,
      g_param_spec_string ("ros-node-namespace", "ROS Node Namespace",
          "Namespace of ROS NODE",
          "",
          (GParamFlags) (G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
              GST_PARAM_MUTABLE_READY)));

  g_object_class_install_property (gobject_class, PROP_ROS_MSG_TYPE,
    g_param_spec_enum ("ros-msg-type", "ROS Msg Type",
        "Type of ROS message to publish",
        GST_TYPE_PROP_ROS_MSG_TYPE,
        ROS_MSG_TYPE_UNDEFINED,
        (GParamFlags) (G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE | G_PARAM_STATIC_STRINGS)));

  gstbasesink_class->set_caps = GST_DEBUG_FUNCPTR (gst_ti_ros_sink_set_caps);
  gstbasesink_class->render = GST_DEBUG_FUNCPTR(gst_ti_ros_sink_render);

  gstelement_class->change_state =
      GST_DEBUG_FUNCPTR (gst_ti_ros_sink_change_state);

  GST_DEBUG_CATEGORY_INIT (gst_ti_ros_sink_debug,
      "tirossink", 0, "TI Ros Sink");
}

/* Initialize the new element
 * Initialize instance structure
 */
static void
gst_ti_ros_sink_init (GstTIRosSink * self)
{
  GST_LOG_OBJECT (self, "init");
  self->ros_msg_type = ROS_MSG_TYPE_UNDEFINED;
  self->prop_ros_msg_type = ROS_MSG_TYPE_UNDEFINED;
  self->width = 0;
  self->height = 0;
  self->ros_encoding = NULL;
  self->ros_topic = NULL;
  self->ros_node_name = g_strdup("tirossink_node");
  self->ros_node_namespace = g_strdup("");

  return;
}

static void
gst_ti_ros_sink_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstTIRosSink *
      self = GST_TI_ROS_SINK (object);

  GST_LOG_OBJECT (self, "set_property");

  GST_OBJECT_LOCK (object);
  switch (prop_id) {
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

    case PROP_ROS_MSG_TYPE:
      self->prop_ros_msg_type = g_value_get_enum (value);
      break;
  
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (object);
}

static void
gst_ti_ros_sink_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstTIRosSink *
      self = GST_TI_ROS_SINK (object);

  GST_LOG_OBJECT (self, "get_property");

  GST_OBJECT_LOCK (object);
  switch (prop_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, self->ros_topic);
      break;

    case PROP_ROS_NODE_NAME:
      g_value_set_string(value, self->ros_node_name);
      break;

    case PROP_ROS_NODE_NAMESPACE:
      g_value_set_string(value, self->ros_node_namespace);
      break;

    case PROP_ROS_MSG_TYPE:
      g_value_set_enum (value, self->prop_ros_msg_type);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (object);
}

static
  gboolean
gst_ti_ros_sink_set_caps (GstBaseSink * sink, GstCaps * caps)
{
  GstTIRosSink *
      self = GST_TI_ROS_SINK (sink);
  gboolean ret = TRUE;
  GstStructure * caps_struct;
  const gchar * caps_str;
  const gchar * format_str;
  GstVideoFormat format_enum;
  const GstVideoFormatInfo * format_info;
  
  GST_LOG_OBJECT (self, "set_caps");

  if(!gst_caps_is_fixed(caps))
  {
    GST_ERROR_OBJECT(self, "caps is not fixed");
    ret = FALSE;
    goto exit;
  }
  
  caps_struct = gst_caps_get_structure (caps, 0);
  caps_str = gst_structure_get_name (caps_struct);

  if (0 == g_strcmp0(caps_str, "video/x-raw")) {
    self->ros_msg_type = ROS_MSG_TYPE_RAW_IMAGE;

    if(!gst_structure_get_int (caps_struct, "width", &self->width)) {
      GST_ERROR_OBJECT(self, "caps is missing width");
      ret = FALSE;
      goto exit;
    }

    if(!gst_structure_get_int (caps_struct, "height", &self->height)) {
      GST_ERROR_OBJECT(self, "caps is missing height");
      ret = FALSE;
      goto exit;
    }

    format_str = gst_structure_get_string (caps_struct, "format");
    if(!format_str) {
      GST_ERROR_OBJECT (self, "caps is missing format");
      ret = FALSE;
      goto exit;
    }

    format_enum =  gst_video_format_from_string (format_str);

    self->ros_encoding = g_strdup(
      ti_ros_gst_plugins_utils::get_ros_encoding(format_enum).c_str()
      );

    if(0 == g_strcmp0(self->ros_encoding, "UNKNOWN")) {
      RCLCPP_ERROR (self->ros_logger,
          "GStreamer format %s not supported", format_str);
      ret = FALSE;
      goto exit;
    }

    format_info = gst_video_format_get_info (format_enum);
    self->ros_step = format_info->pixel_stride[0];

    if(format_info->bits < 8) {
      self->ros_step = self->ros_step/8;
      RCLCPP_ERROR(self->ros_logger, "low bits per pixel");
    }

    self->ros_step = self->ros_step * self->width;
    self->ros_is_big_endian = 
        GST_VIDEO_FORMAT_INFO_IS_LE(format_info) ? FALSE : TRUE;

  } else if (0 == g_strcmp0(caps_str, "image/jpeg")) {

    self->ros_msg_type = ROS_MSG_TYPE_COMPRESSED_IMAGE_JPEG;

  } else if (0 == g_strcmp0(caps_str, "image/png")) {

    self->ros_msg_type = ROS_MSG_TYPE_COMPRESSED_IMAGE_PNG;

  } else if (0 == g_strcmp0(caps_str, "text/x-raw")) {

    if (self->prop_ros_msg_type == ROS_MSG_TYPE_DETECTION2D) {
      self->ros_msg_type = ROS_MSG_TYPE_DETECTION2D;
    } else if (self->prop_ros_msg_type == ROS_MSG_TYPE_POSE6D) {
      self->ros_msg_type = ROS_MSG_TYPE_POSE6D;
    } else {
      self->ros_msg_type = ROS_MSG_TYPE_STRING;
    }

  } else {

    self->ros_msg_type = ROS_MSG_TYPE_UNDEFINED;
    RCLCPP_ERROR(self->ros_logger, "caps '%s' cannot be converted to ros msg",
      gst_caps_to_string(caps));
    goto exit;

  }
  
  RCLCPP_INFO(self->ros_logger, "preparing msg with caps '%s'",
    gst_caps_to_string(caps));
  GST_DEBUG_OBJECT (self, "Resulting caps are %" GST_PTR_FORMAT, (void*) caps);

exit:
  return ret;
}

static
    GstFlowReturn
gst_ti_ros_sink_render (GstBaseSink * sink, GstBuffer * buffer)
{
  GstTIRosSink *
      self = GST_TI_ROS_SINK (sink);
  GstFlowReturn ret = GST_FLOW_ERROR;
  GstMapInfo info;
  GstClockTimeDiff base_time;
  rclcpp::Time msg_time;

  GST_LOG_OBJECT (self, "render");

  base_time = gst_element_get_base_time(GST_ELEMENT(sink));
  msg_time = rclcpp::Time(GST_BUFFER_PTS(buffer) + base_time + self->ros_clock_offset,
    self->ros_clock->get_clock_type());

  /* Map output Buffer */
  if (!gst_buffer_map (buffer, &info, GST_MAP_READ)) {
      GST_ERROR_OBJECT (self, "failed to map buffer");
      goto exit;
  }

  gst_ti_ros_sink_publish_msg (self,self->ros_msg_type,&info,msg_time);

  gst_buffer_unmap (buffer, &info);

  ret = GST_FLOW_OK;

exit:
  return ret;
}

static
  GstStateChangeReturn
gst_ti_ros_sink_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  GstTIRosSink * self = GST_TI_ROS_SINK (element);

  switch (transition)
  {
    case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    {
      if (!gst_ti_ros_sink_open(self)) {
        return GST_STATE_CHANGE_FAILURE;
      }
      self->ros_clock_offset =
        ti_ros_gst_plugins_utils::get_colck_offset (GST_ELEMENT_CLOCK(self),
                                                    self->ros_clock->now());
      break;
    }
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    case GST_STATE_CHANGE_NULL_TO_READY:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (gst_ti_ros_sink_parent_class)->change_state (element, transition);

  switch (transition)
  {
    case GST_STATE_CHANGE_READY_TO_NULL:
    {
      gst_ti_ros_sink_close(self);
      break;
    }
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    case GST_STATE_CHANGE_PAUSED_TO_READY:
    case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    default:
      break;
  }

  return ret;
}

static
  gboolean
gst_ti_ros_sink_open (GstTIRosSink * self)
{
  GST_DEBUG_OBJECT (self, "open");
  
  using std::placeholders::_1;

  if (ROS_MSG_TYPE_UNDEFINED == self->ros_msg_type) {
    GST_ERROR_OBJECT (self, "ROS msg type undefined.");
    return FALSE;
  }

  if (NULL == self->ros_topic) {
    GST_ERROR_OBJECT (self, "ROS Topic undefined.");
    return FALSE;
  }

  if (ROS_MSG_TYPE_UNDEFINED != self->prop_ros_msg_type &&
    self->ros_msg_type != self->prop_ros_msg_type) {
    GST_ERROR_OBJECT (self, "Incoming caps is not supported by choosen msg type.");
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

  self->ros_logger = self->ros_node->get_logger();
  self->ros_clock = self->ros_node->get_clock();

  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();

  switch (self->ros_msg_type) {
    case ROS_MSG_TYPE_RAW_IMAGE:
    {
      self->ros_image_raw_publisher =
        self->ros_node->create_publisher<sensor_msgs::msg::Image>(self->ros_topic, qos);
      RCLCPP_INFO(self->ros_logger,
          "Publishing to topic [%s] of type Image", self->ros_topic);
      break;
    }
    case ROS_MSG_TYPE_COMPRESSED_IMAGE_JPEG:
    case ROS_MSG_TYPE_COMPRESSED_IMAGE_PNG:
    {
      self->ros_image_compressed_publisher =
        self->ros_node->create_publisher<sensor_msgs::msg::CompressedImage>(self->ros_topic, qos);
      RCLCPP_INFO(self->ros_logger,
          "Publishing to topic [%s] of type CompressedImage", self->ros_topic);
      break;
    }
    case ROS_MSG_TYPE_STRING:
    {
      self->ros_string_publisher =
        self->ros_node->create_publisher<std_msgs::msg::String>(self->ros_topic, qos);
      RCLCPP_INFO(self->ros_logger,
          "Publishing to topic [%s] of type String", self->ros_topic);
      break;
    }
    case ROS_MSG_TYPE_DETECTION2D:
    {
      self->ros_detection_2d_publisher =
        self->ros_node->create_publisher<common_msgs::msg::Detection2D>(self->ros_topic, qos);
      RCLCPP_INFO(self->ros_logger,
          "Publishing to topic [%s] of type Detection2D", self->ros_topic);
#ifndef BUILD_FOR_TARGET
      RCLCPP_WARN(self->ros_logger,
          "Empty Detection2D msg will be published, since text/x-raw in yaml "
          "format cannot be parsed in emulation mode");
#endif
      break;
    }
    case ROS_MSG_TYPE_POSE6D:
    {
      self->ros_pose_6d_publisher =
        self->ros_node->create_publisher<common_msgs::msg::Pose6D>(self->ros_topic, qos);
      RCLCPP_INFO(self->ros_logger,
          "Publishing to topic [%s] of type Pose6D", self->ros_topic);
#ifndef BUILD_FOR_TARGET
      RCLCPP_WARN(self->ros_logger,
          "Empty Pose6D msg will be published, since text/x-raw in yaml "
          "format cannot be parsed in emulation mode");
#endif
      break;
    }
    default:
    {
      return FALSE;
      break;
    }
  }

  self->ros_spin_thread = std::thread{&gst_ti_ros_sink_spin_wrapper, self};

  return TRUE;
}

static void
gst_ti_ros_sink_close (GstTIRosSink * self)
{
  GST_DEBUG_OBJECT (self, "close");

  if (NULL != self->ros_node) {
    self->ros_clock.reset();

    self->ros_image_raw_publisher.reset();
    self->ros_image_compressed_publisher.reset();
    self->ros_string_publisher.reset();
    self->ros_detection_2d_publisher.reset();
    self->ros_pose_6d_publisher.reset();

    self->ros_executor->cancel();
    self->ros_spin_thread.join();
    self->ros_context->shutdown("gst closing tirossink");

    self->ros_context.reset();
    self->ros_executor.reset();
    self->ros_node.reset();
    self->ros_clock.reset();
  }
}

static void
gst_ti_ros_sink_spin_wrapper (GstTIRosSink * self)
{
  self->ros_executor->spin();
}

static void
gst_ti_ros_sink_publish_msg (GstTIRosSink * self,
    guint ros_msg_type, GstMapInfo * info, rclcpp::Time msg_time)
{
  switch (ros_msg_type) {
    case ROS_MSG_TYPE_RAW_IMAGE:
    {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = msg_time;
      msg.width = self->width;
      msg.height = self->height;
      msg.encoding = self->ros_encoding;
      msg.is_bigendian = self->ros_is_big_endian;
      msg.step = self->ros_step;
      msg.data.assign (info->data, info->data + info->size);
      self->ros_image_raw_publisher->publish (msg);
      break;
    }
    case ROS_MSG_TYPE_COMPRESSED_IMAGE_JPEG:
    case ROS_MSG_TYPE_COMPRESSED_IMAGE_PNG:
    {
      sensor_msgs::msg::CompressedImage msg;
      msg.header.stamp = msg_time;
      msg.format = (ros_msg_type == ROS_MSG_TYPE_COMPRESSED_IMAGE_JPEG) ? "jpeg" : "png";
      msg.data.assign (info->data, info->data + info->size);
      self->ros_image_compressed_publisher->publish (msg);
      break;
    }
    case ROS_MSG_TYPE_STRING:
    {
      std_msgs::msg::String msg;
      msg.data.assign (info->data, info->data + info->size);
      self->ros_string_publisher->publish (msg);
      break;
    }
    case ROS_MSG_TYPE_DETECTION2D:
    {
      common_msgs::msg::Detection2D msg;
      msg.header.stamp = msg_time;

#ifdef BUILD_FOR_TARGET
      std::string      data;
      data.assign(info->data, info->data + info->size);

      const YAML::Node node = YAML::Load(data.c_str());

      const YAML::Node &labelIdNode   = node["label_ids"];
      const YAML::Node &socreNode     = node["scores"];
      const YAML::Node &boxNode       = node["bounding_boxes"];
      const YAML::Node &outWidthNode  = node["output_width"];
      const YAML::Node &outHeightNode = node["output_height"];

      if (node.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Cannot convert text/x-raw data to yaml node");
        return;
      }

      if (labelIdNode.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key label_ids missing from input data");
        return;
      } else if (labelIdNode.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "label_ids is not a sequence");
        return;
      }

      if (socreNode.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key scores missing from input data");
        return;
      } else if (socreNode.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "scores is not a sequence");
        return;
      }

      if (boxNode.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key bounding_boxes missing from input data");
        return;
      } else if (boxNode.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "bounding_boxes is not a sequence");
        return;
      }

      if (labelIdNode.size() != socreNode.size() ||
          socreNode.size() != boxNode.size()
          ) {
        RCLCPP_ERROR(self->ros_logger,
            "Size of label_ids, scores & bounding_boxes dont match");
        return;
      }

      if (!outWidthNode.IsNull() &&
          outWidthNode.Type() == YAML::NodeType::Scalar) {
        msg.img_width = outWidthNode.as<int32_t>();
      }

      if (!outHeightNode.IsNull() &&
          outHeightNode.Type() == YAML::NodeType::Scalar) {
        msg.img_height = outHeightNode.as<int32_t>();
      }

      msg.num_objects = labelIdNode.size();
      msg.bounding_boxes.assign(msg.num_objects,
          common_msgs::msg::BoundingBox2D{});

      std::vector<int32_t> labelId = labelIdNode.as<std::vector<int32_t>>();
      std::vector<float> scores = socreNode.as<std::vector<float>>();
      std::vector<std::vector<int32_t>> bBox = \
          boxNode.as<std::vector<std::vector<int32_t>>>();

      for (gint i = 0; i < msg.num_objects; i++)
      {
          auto &box      = msg.bounding_boxes[i];
          box.xmin       = bBox[i][0];
          box.ymin       = bBox[i][1];
          box.xmax       = bBox[i][2];
          box.ymax       = bBox[i][3];
          box.label_id   = labelId[i];
          box.confidence = scores[i];
      }
#endif
      self->ros_detection_2d_publisher->publish (msg);
      break;
    }
    case ROS_MSG_TYPE_POSE6D:
    {
      common_msgs::msg::Pose6D msg;
      msg.header.stamp = msg_time;

#ifdef BUILD_FOR_TARGET
      std::string      data;
      data.assign(info->data, info->data + info->size);

      const YAML::Node node = YAML::Load(data.c_str());

      const YAML::Node &labelIdNode   = node["label_ids"];
      const YAML::Node &socreNode     = node["scores"];
      const YAML::Node &boxNode       = node["bounding_boxes"];
      const YAML::Node &rot1Node      = node["rotation_1"];
      const YAML::Node &rot2Node      = node["rotation_2"];
      const YAML::Node &transNode     = node["translation"];
      const YAML::Node &inWidthNode   = node["input_width"];
      const YAML::Node &inHeightNode  = node["input_height"];

      if (node.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Cannot convert text/x-raw data to yaml node");
        return;
      }

      if (labelIdNode.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key label_ids missing from input data");
        return;
      } else if (labelIdNode.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "label_ids is not a sequence");
        return;
      }

      if (socreNode.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key scores missing from input data");
        return;
      } else if (socreNode.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "scores is not a sequence");
        return;
      }

      if (boxNode.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key bounding_boxes missing from input data");
        return;
      } else if (boxNode.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "bounding_boxes is not a sequence");
        return;
      }

      if (rot1Node.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key rotation_1 missing from input data");
        return;
      } else if (rot1Node.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "rotation_1 is not a sequence");
        return;
      }

      if (rot2Node.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key rotation_2 missing from input data");
        return;
      } else if (rot2Node.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "rotation_2 is not a sequence");
        return;
      }

      if (transNode.IsNull()) {
        RCLCPP_ERROR(self->ros_logger,
            "Key translation missing from input data");
        return;
      } else if (transNode.Type() != YAML::NodeType::Sequence) {
        RCLCPP_ERROR(self->ros_logger, "translation is not a sequence");
        return;
      }

      if (labelIdNode.size() != socreNode.size() ||
          socreNode.size() != boxNode.size() ||
          boxNode.size() != rot1Node.size() ||
          rot1Node.size() != rot2Node.size() ||
          rot2Node.size() != transNode.size()
          ) {
        RCLCPP_ERROR(self->ros_logger,
            "Size of label_ids, scores , bounding_boxes, rotation_1, rotation_2 and translation don't match");
        return;
      }

      if (!inWidthNode.IsNull() &&
          inWidthNode.Type() == YAML::NodeType::Scalar) {
        msg.img_width = inWidthNode.as<int32_t>();
      }

      if (!inHeightNode.IsNull() &&
          inHeightNode.Type() == YAML::NodeType::Scalar) {
        msg.img_height = inHeightNode.as<int32_t>();
      }

      msg.num_objects = labelIdNode.size();
      msg.bounding_boxes.assign(msg.num_objects,
          common_msgs::msg::BoundingBox2D{});
      msg.transform_matrix.assign(msg.num_objects,
          common_msgs::msg::Transform{});

      std::vector<int32_t> labelId = labelIdNode.as<std::vector<int32_t>>();

      std::vector<float> scores = socreNode.as<std::vector<float>>();

      std::vector<std::vector<int32_t>> bBox = \
          boxNode.as<std::vector<std::vector<int32_t>>>();

      std::vector<std::vector<float>> rot1 = \
          rot1Node.as<std::vector<std::vector<float>>>();

      std::vector<std::vector<float>> rot2 = \
          rot2Node.as<std::vector<std::vector<float>>>();

      std::vector<std::vector<float>> trans = \
          transNode.as<std::vector<std::vector<float>>>();

      for (gint i = 0; i < msg.num_objects; i++)
      {
          auto &box         = msg.bounding_boxes[i];
          box.xmin          = bBox[i][0];
          box.ymin          = bBox[i][1];
          box.xmax          = bBox[i][2];
          box.ymax          = bBox[i][3];
          box.label_id      = labelId[i];
          box.confidence    = scores[i];

          auto &t_matrix    = msg.transform_matrix[i];
          t_matrix.rot1_x   = rot1[i][0];
          t_matrix.rot1_y   = rot1[i][1];
          t_matrix.rot1_z   = rot1[i][2];
          t_matrix.rot2_x   = rot2[i][0];
          t_matrix.rot2_y   = rot2[i][1];
          t_matrix.rot2_z   = rot2[i][2];
          t_matrix.trans_x  = trans[i][0];
          t_matrix.trans_y  = trans[i][1];
          t_matrix.trans_z  = trans[i][2];
      }
#endif
      self->ros_pose_6d_publisher->publish (msg);
      break;
    }
    default:
      break;
  }
}