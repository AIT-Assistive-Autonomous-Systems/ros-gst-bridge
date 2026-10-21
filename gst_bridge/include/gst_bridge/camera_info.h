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

#ifndef _GST_BRIDGE_CAMERA_INFO_H_
#define _GST_BRIDGE_CAMERA_INFO_H_

#include <gst_bridge/gst_bridge.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace gst_bridge
{
// Loads a calibration file and publishes it alongside the images of an image sink element.
// camera_info_manager is not used: its getCameraInfo() returns an empty message unless the
// default rclcpp context is ok(), and the elements run on their own context.
class CameraInfoPublisher
{
public:
  // url follows the camera_info_manager conventions (yaml or ini),
  //   e.g. file:///path/to/calibration.yaml, package://pkg/calib/camera.yaml or a plain path
  CameraInfoPublisher(
    std::shared_ptr<node_interface_collection> node_if, const std::string & topic,
    const std::string & url, const rclcpp::QoS & qos);

  // (re)load a calibration, keeps the previous one if the url could not be loaded
  bool load(const std::string & url);

  // frame_id override, used as is; if empty the frame_id comes from the calibration
  //   (a frame_id field as is, else <camera_name>_optical as in REP 103),
  //   or from the image header if the calibration names no camera
  void set_frame_id(const std::string & frame_id) { frame_id_ = frame_id; }

  // publish the camera info with the header of the matching image,
  // nothing is published until a calibration was loaded
  // width and height fill in an uncalibrated (zero size) camera info
  void publish(const std_msgs::msg::Header & header, uint32_t width, uint32_t height);

private:
  std::shared_ptr<node_interface_collection> node_if_;
  sensor_msgs::msg::CameraInfo info_;
  bool loaded_ = false;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
  std::string frame_id_;
  std::string calibration_frame_id_;
};

// camera info properties of an image sink element, allocated with new in the element's init
// and deleted in its finalize (GObject instances run no C++ constructors or destructors)
struct CameraInfoSettings
{
  std::string url;       // calibration url or path, camera info is only published if set
  std::string topic;     // defaults to camera_info next to the image topic
  std::string frame_id;  // camera info frame_id override, used as is
  std::shared_ptr<CameraInfoPublisher> publisher;
};

// copy a string GValue, NULL becomes an empty string
inline std::string value_to_string(const GValue * value)
{
  const gchar * str = g_value_get_string(value);
  return str ? str : "";
}

// camera_info topic next to the image topic, e.g. /cam/image_raw -> /cam/camera_info
std::string camera_info_topic_for(const std::string & image_topic);

}  // namespace gst_bridge

#endif  // _GST_BRIDGE_CAMERA_INFO_H_
