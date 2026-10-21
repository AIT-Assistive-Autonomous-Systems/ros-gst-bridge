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

// C++ members: GObject zero allocates the instance and runs no constructors, so they are
// constructed with one placement new in roscompressedimagesrc_init and destroyed in
// roscompressedimagesrc_finalize
struct RoscompressedimagesrcPrivate
{
  using MsgType = sensor_msgs::msg::CompressedImage;

  std::string sub_topic =
    "gst_image_sub";                     //base image topic, subscribed on <sub_topic>/<transport>
  std::string transport = "compressed";  //image_transport style topic suffix, none if empty

  rclcpp::Subscription<MsgType>::SharedPtr sub;

  // latest messages, guarded by msg_queue_mtx
  std::deque<MsgType::ConstSharedPtr> msg_queue;
  std::mutex msg_queue_mtx;
  std::condition_variable msg_queue_cv;
  bool flushing = false;  //basesrc unlock, wake up and return from create
  std::string frame_id;   //of the last message
  std::string format;     //of the last message

  std::string media_type;  //media type of the caps currently set on the src pad
};

struct _Roscompressedimagesrc
{
  using MsgType = RoscompressedimagesrcPrivate::MsgType;

  RosBaseSrc parent;
  RoscompressedimagesrcPrivate priv;
};

struct _RoscompressedimagesrcClass
{
  RosBaseSrcClass parent_class;
};

GType roscompressedimagesrc_get_type(void);

G_END_DECLS

#endif  // _GST_ROSCOMPRESSEDIMAGESRC_H_
