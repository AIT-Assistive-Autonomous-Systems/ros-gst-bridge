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

#include <gst_bridge/camera_info.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_calibration_parsers/parse.hpp>

namespace gst_bridge
{
// map a camera_info_manager style url to a file name, empty if it can't be resolved
static std::string calibration_file_for(const std::string & url, std::string & error)
{
  const std::string file_prefix = "file://";
  const std::string package_prefix = "package://";

  if (0 == url.compare(0, file_prefix.size(), file_prefix)) {
    return url.substr(file_prefix.size());
  }
  if (0 == url.compare(0, package_prefix.size(), package_prefix)) {
    const std::string rest = url.substr(package_prefix.size());
    const auto slash = rest.find('/');
    if (slash == std::string::npos) {
      error = "package url has no file path";
      return "";
    }
    try {
      return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) +
             rest.substr(slash);
    } catch (const ament_index_cpp::PackageNotFoundError & e) {
      error = e.what();
      return "";
    }
  }
  if (url.find("://") != std::string::npos) {
    error = "unsupported url scheme";
    return "";
  }
  return url;  // plain path
}

CameraInfoPublisher::CameraInfoPublisher(
  std::shared_ptr<node_interface_collection> node_if, const std::string & topic,
  const std::string & url, const rclcpp::QoS & qos)
: node_if_(node_if)
{
  pub_ = rclcpp::create_publisher<sensor_msgs::msg::CameraInfo>(
    node_if->parameters, node_if->topics, topic, qos);
  load(url);
}

bool CameraInfoPublisher::load(const std::string & url)
{
  std::string error;
  const std::string file_name = calibration_file_for(url, error);
  std::string camera_name;
  sensor_msgs::msg::CameraInfo info;

  if (
    file_name.empty() ||
    !camera_calibration_parsers::readCalibration(file_name, camera_name, info)) {
    RCLCPP_ERROR(
      node_if_->logging->get_logger(), "failed to load camera info from '%s' %s", url.c_str(),
      error.c_str());
    return false;
  }
  RCLCPP_INFO(
    node_if_->logging->get_logger(), "loaded camera info for '%s' from '%s'", camera_name.c_str(),
    file_name.c_str());
  info_ = info;
  loaded_ = true;

  // the calibration format has no frame_id field (yet), use it if the parser provides one
  calibration_frame_id_ = info.header.frame_id;
  if (calibration_frame_id_.empty() && !camera_name.empty() && camera_name != "unknown") {
    const std::string suffix = "_optical";
    const bool has_suffix =
      camera_name.size() >= suffix.size() &&
      0 == camera_name.compare(camera_name.size() - suffix.size(), suffix.size(), suffix);
    calibration_frame_id_ = has_suffix ? camera_name : camera_name + suffix;
  }
  return true;
}

void CameraInfoPublisher::publish(
  const std_msgs::msg::Header & header, uint32_t width, uint32_t height)
{
  if (!loaded_) return;
  sensor_msgs::msg::CameraInfo msg = info_;
  msg.header = header;
  if (!frame_id_.empty()) {
    msg.header.frame_id = frame_id_;
  } else if (!calibration_frame_id_.empty()) {
    msg.header.frame_id = calibration_frame_id_;
  }
  if (msg.width == 0 && msg.height == 0) {
    msg.width = width;
    msg.height = height;
  }
  pub_->publish(msg);
}

std::string camera_info_topic_for(const std::string & image_topic)
{
  const auto pos = image_topic.rfind('/');
  if (pos == std::string::npos) {
    return "camera_info";
  }
  return image_topic.substr(0, pos + 1) + "camera_info";
}

}  // namespace gst_bridge
