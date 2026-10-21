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

// roscompressedimagesink: camera info, formats, transport suffix, invalid topics

#include <gst_bridge/camera_info.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <set>

#include "test_utils.hpp"

using namespace gst_bridge_test;

struct CameraInfoCase
{
  std::string name;
  std::string camera_name;  // in the calibration file, empty: none
  std::string frame_override;
  std::string expected_frame;
};

class CompressedSinkCameraInfo : public ::testing::TestWithParam<CameraInfoCase>
{
};

TEST_P(CompressedSinkCameraInfo, frame_and_stamps)
{
  const auto & c = GetParam();
  const std::string base = test_topic("image_raw");
  const std::string info_topic = gst_bridge::camera_info_topic_for(base);
  Collector<sensor_msgs::msg::CompressedImage> images(base + "/compressed");
  Collector<sensor_msgs::msg::CameraInfo> infos(info_topic);

  Pipeline p(
    "appsrc name=src is-live=true format=time caps=image/jpeg,width=320,height=240 ! "
    "roscompressedimagesink name=sink sync=false ros-frame-id=drone1_down ros-topic=" +
    base + " camera-info-url=" + write_calibration(c.name + ".yaml", c.camera_name) +
    (c.frame_override.empty() ? "" : " camera-info-frame-id=" + c.frame_override));
  ASSERT_TRUE(p.valid());
  ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  ASSERT_TRUE(images.wait_for_publisher());
  ASSERT_TRUE(infos.wait_for_publisher());

  for (int i = 0; i < 20 && !(images.wait_for(3, 50ms) && infos.wait_for(3, 0ms)); i++) {
    push_buffer(p.element("src"), i * 100 * GST_MSECOND);
  }
  ASSERT_TRUE(images.wait_for(3));
  ASSERT_TRUE(infos.wait_for(3));

  std::set<int64_t> image_stamps;
  for (const auto & img : images.msgs()) {
    EXPECT_EQ(img.format, "jpeg");
    EXPECT_EQ(img.header.frame_id, "drone1_down");
    image_stamps.insert(to_ns(img.header.stamp));
  }
  for (const auto & info : infos.msgs()) {
    EXPECT_EQ(info.header.frame_id, c.expected_frame);
    EXPECT_EQ(info.k[0], 300.0);
    EXPECT_EQ(info.distortion_model, "plumb_bob");
    EXPECT_EQ(info.width, 320u);
    EXPECT_EQ(info.height, 240u);
    EXPECT_TRUE(image_stamps.count(to_ns(info.header.stamp)))
      << "camera info stamp not an image stamp";
  }
}

INSTANTIATE_TEST_SUITE_P(
  FrameRules, CompressedSinkCameraInfo,
  ::testing::Values(
    CameraInfoCase{"named", "testcam", "", "testcam_optical"},
    CameraInfoCase{"optical", "down_optical", "", "down_optical"},
    CameraInfoCase{"noname", "", "", "drone1_down"},
    CameraInfoCase{"override", "testcam", "my_frame", "my_frame"}),
  [](const auto & info) { return info.param.name; });

TEST(CompressedSink, no_camera_info_without_url)
{
  const std::string base = test_topic("image_raw");
  Collector<sensor_msgs::msg::CompressedImage> images(base + "/compressed");
  Pipeline p(
    "appsrc name=src is-live=true format=time caps=image/png ! roscompressedimagesink sync=false "
    "ros-topic=" +
    base);
  ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  ASSERT_TRUE(images.wait_for_publisher());
  for (int i = 0; i < 20 && !images.wait_for(2, 50ms); i++) {
    push_buffer(p.element("src"), i * 100 * GST_MSECOND);
  }
  ASSERT_TRUE(images.wait_for(2));
  EXPECT_EQ(images.msgs().front().format, "png");
  EXPECT_EQ(test_node()->count_publishers(gst_bridge::camera_info_topic_for(base)), 0u);
}

TEST(CompressedSink, no_camera_info_if_calibration_fails)
{
  const std::string base = test_topic("image_raw");
  Collector<sensor_msgs::msg::CompressedImage> images(base + "/compressed");
  Collector<sensor_msgs::msg::CameraInfo> infos(gst_bridge::camera_info_topic_for(base));
  Pipeline p(
    "appsrc name=src is-live=true format=time caps=image/jpeg ! roscompressedimagesink sync=false "
    "camera-info-url=package://gst_bridge/does_not_exist.yaml ros-topic=" +
    base);
  ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  ASSERT_TRUE(images.wait_for_publisher());
  for (int i = 0; i < 20 && !images.wait_for(3, 50ms); i++) {
    push_buffer(p.element("src"), i * 100 * GST_MSECOND);
  }
  ASSERT_TRUE(images.wait_for(3));
  std::this_thread::sleep_for(200ms);
  EXPECT_TRUE(infos.msgs().empty());
}

TEST(CompressedSink, transport_suffix)
{
  const std::string base = test_topic("image_raw");
  for (const std::string transport : {"jpeg", ""}) {
    const std::string topic = transport.empty() ? base : base + "/" + transport;
    Collector<sensor_msgs::msg::CompressedImage> images(topic);
    Pipeline p(
      "appsrc name=src is-live=true format=time caps=image/jpeg ! roscompressedimagesink "
      "name=sink sync=false ros-topic=" +
      base);
    g_object_set(p.element("sink"), "ros-transport", transport.c_str(), nullptr);
    ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
    ASSERT_TRUE(images.wait_for_publisher()) << "transport '" << transport << "'";
    for (int i = 0; i < 20 && !images.wait_for(1, 50ms); i++) {
      push_buffer(p.element("src"), i * 100 * GST_MSECOND);
    }
    EXPECT_TRUE(images.wait_for(1)) << "transport '" << transport << "'";
  }
}

TEST(CompressedSink, invalid_topic_fails_state_change)
{
  Pipeline p("appsrc caps=image/jpeg ! roscompressedimagesink name=sink");
  g_object_set(p.element("sink"), "ros-transport", "bad name", nullptr);
  EXPECT_EQ(p.set_state(GST_STATE_READY), GST_STATE_CHANGE_FAILURE);
}
