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

#include "gsttirosutils.h"

namespace ti_ros_gst_plugins_utils
{

GstClockTimeDiff get_colck_offset(GstClock * gst_clock,
                                  rclcpp::Time ros_clock)
{
  GstClockTime gst_time;
  GstClockTime ros_time;
  gst_time = gst_clock_get_time (gst_clock);
  ros_time = ros_clock.nanoseconds();
  return ros_time - gst_time;
}

GstVideoFormat get_gst_video_format(const std::string & ros_encoding)
{
  using namespace sensor_msgs::image_encodings;

  if (MONO8 == ros_encoding)
  {
    return GST_VIDEO_FORMAT_GRAY8;
  }
  else if (RGB8 == ros_encoding)
  {
    return GST_VIDEO_FORMAT_RGB;
  }
  else if (YUV422 == ros_encoding)
  {
    return GST_VIDEO_FORMAT_UYVY;
  }
  else if (YUV422_YUY2 == ros_encoding)
  {
    return GST_VIDEO_FORMAT_YUY2;
  }
  else if ("yuv420" == ros_encoding)
  {
    return GST_VIDEO_FORMAT_NV12;
  }

  return GST_VIDEO_FORMAT_UNKNOWN;
}

std::string get_ros_encoding(GstVideoFormat gst_format)
{
  using namespace sensor_msgs::image_encodings;

  switch (gst_format)
  {
    case GST_VIDEO_FORMAT_GRAY8:
    {
      return MONO8;
      break;
    }
    case GST_VIDEO_FORMAT_RGB:
    {
      return RGB8;
      break;
    }
    case GST_VIDEO_FORMAT_UYVY:
    {
      return YUV422;
      break;
    }
    case GST_VIDEO_FORMAT_NV12:
    {
      return "yuv420";
      break;
    }
    default:
      break;
  }

  return "UNKNOWN";
}

}  //namespace ti_ros_gst_plugins_utils

