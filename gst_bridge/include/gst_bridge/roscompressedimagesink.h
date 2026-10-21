/* gst_bridge
 * Copyright (C) 2020-2021 Brett Downing <brettrd@brettrd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _GST_ROSCOMPRESSEDIMAGESINK_H_
#define _GST_ROSCOMPRESSEDIMAGESINK_H_

#include <gst/base/gstbasesink.h>
#include <gst_bridge/camera_info.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesink.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>

G_BEGIN_DECLS

#define GST_TYPE_ROSCOMPRESSEDIMAGESINK (roscompressedimagesink_get_type())
#define GST_ROSCOMPRESSEDIMAGESINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSCOMPRESSEDIMAGESINK, Roscompressedimagesink))
#define GST_ROSCOMPRESSEDIMAGESINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSCOMPRESSEDIMAGESINK, RoscompressedimagesinkClass))
#define GST_IS_ROSCOMPRESSEDIMAGESINK(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROSCOMPRESSEDIMAGESINK))
#define GST_IS_ROSCOMPRESSEDIMAGESINK_CLASS(obj) \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROSCOMPRESSEDIMAGESINK))

typedef struct _Roscompressedimagesink Roscompressedimagesink;
typedef struct _RoscompressedimagesinkClass RoscompressedimagesinkClass;

// C++ members: GObject zero allocates the instance and runs no constructors, so they are
// constructed with one placement new in roscompressedimagesink_init and destroyed in
// roscompressedimagesink_finalize
struct RoscompressedimagesinkPrivate
{
  std::string pub_topic = "gst_image_pub";  //base image topic, published on <pub_topic>/<transport>
  std::string transport = "compressed";     //image_transport style topic suffix, none if empty
  std::string frame_id = "image_frame";
  std::string format;  //compressed image format string override, deduced from the caps if blank
  std::string caps_format;  //format string deduced from the caps

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;

  gst_bridge::CameraInfoSettings camera_info;

  int height = 0;  //from the caps if known, only used to fill the camera info
  int width = 0;
};

struct _Roscompressedimagesink
{
  RosBaseSink parent;
  RoscompressedimagesinkPrivate priv;
};

struct _RoscompressedimagesinkClass
{
  RosBaseSinkClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType roscompressedimagesink_get_type(void);

G_END_DECLS

#endif
