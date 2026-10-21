/* gst_bridge
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

#ifndef _GST_ROSCOMPRESSEDIMAGESRC_H_
#define _GST_ROSCOMPRESSEDIMAGESRC_H_

#include <gst/base/gstbasesrc.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesrc.h>

//include ROS and ROS message formats
#include <condition_variable>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>

G_BEGIN_DECLS

#define GST_TYPE_ROSCOMPRESSEDIMAGESRC (roscompressedimagesrc_get_type())
#define GST_ROSCOMPRESSEDIMAGESRC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROSCOMPRESSEDIMAGESRC, Roscompressedimagesrc))
#define GST_ROSCOMPRESSEDIMAGESRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROSCOMPRESSEDIMAGESRC, RoscompressedimagesrcClass))
#define GST_IS_ROSCOMPRESSEDIMAGESRC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROSCOMPRESSEDIMAGESRC))
#define GST_IS_ROSCOMPRESSEDIMAGESRC_CLASS(obj) \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROSCOMPRESSEDIMAGESRC))

typedef struct _Roscompressedimagesrc Roscompressedimagesrc;
typedef struct _RoscompressedimagesrcClass RoscompressedimagesrcClass;

// C++ members, defined in the .cpp: allocated with new in _init and deleted in _finalize
// (GObject zero allocates instances without running C++ constructors, only guarantees 16 byte
// alignment and limits instances to 64 kB)
struct RoscompressedimagesrcPrivate;

struct _Roscompressedimagesrc
{
  using MsgType = sensor_msgs::msg::CompressedImage;

  RosBaseSrc parent;
  RoscompressedimagesrcPrivate * priv;
};

struct _RoscompressedimagesrcClass
{
  RosBaseSrcClass parent_class;
};

GType roscompressedimagesrc_get_type(void);

G_END_DECLS

#endif  // _GST_ROSCOMPRESSEDIMAGESRC_H_
