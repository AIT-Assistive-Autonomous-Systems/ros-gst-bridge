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

// header stamps of the sinks in all timestamp modes

#include <set>

#include "test_utils.hpp"

using namespace gst_bridge_test;

// header stamps a sink publishes for buffers with the given metas
std::vector<int64_t> sink_stamps(
  const std::string & mode, const std::vector<std::pair<std::string, GstClockTime>> & metas,
  std::vector<GstClockTime> * pts = nullptr)
{
  const std::string base = test_topic("image_raw");
  Collector<sensor_msgs::msg::CompressedImage> images(base + "/compressed");
  Pipeline p(
    "appsrc name=src is-live=true format=time caps=image/jpeg ! roscompressedimagesink sync=false "
    "timestamp-mode=" +
    mode + " ros-topic=" + base);
  EXPECT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  EXPECT_TRUE(images.wait_for_publisher());
  for (int i = 0; i < 20 && !images.wait_for(3, 50ms); i++) {
    GstClockTime t = (i + 1) * 100 * GST_MSECOND;
    if (pts) pts->push_back(t);
    push_buffer(p.element("src"), t, metas);
  }
  EXPECT_TRUE(images.wait_for(3));
  std::vector<int64_t> stamps;
  for (const auto & img : images.msgs()) stamps.push_back(to_ns(img.header.stamp));
  return stamps;
}

TEST(SinkTimestamps, reference_unix)
{
  for (auto s : sink_stamps("reference", {{"timestamp/x-unix", OLD_STAMP_NS}})) {
    EXPECT_EQ(s, OLD_STAMP_NS);
  }
}

TEST(SinkTimestamps, reference_ntp_converted)
{
  for (auto s :
       sink_stamps("reference", {{"timestamp/x-ntp", OLD_STAMP_NS + NTP_UNIX_OFFSET_NS}})) {
    EXPECT_EQ(s, OLD_STAMP_NS);
  }
}

TEST(SinkTimestamps, reference_prefers_reference_caps)
{
  for (auto s :
       sink_stamps("reference", {{"timestamp/x-ptp", 42}, {"timestamp/x-unix", OLD_STAMP_NS}})) {
    EXPECT_EQ(s, OLD_STAMP_NS);
  }
}

TEST(SinkTimestamps, reference_falls_back_to_ros_offset)
{
  for (auto s : sink_stamps("reference", {})) {
    EXPECT_NEAR(s, wall_now_ns(), 10e9) << "expected the ros-offset (wall) time, not 0";
  }
}

TEST(SinkTimestamps, pts)
{
  std::vector<GstClockTime> pts;
  auto stamps = sink_stamps("pts", {}, &pts);
  std::set<int64_t> pushed(pts.begin(), pts.end());
  for (auto s : stamps) EXPECT_TRUE(pushed.count(s)) << s;
}

TEST(SinkTimestamps, arrival_ros_and_realtime)
{
  for (const char * mode : {"arrival-ros", "arrival-realtime", "ros-offset"}) {
    for (auto s : sink_stamps(mode, {})) EXPECT_NEAR(s, wall_now_ns(), 10e9) << mode;
  }
}

TEST(SinkTimestamps, arrival_pipeline_is_pipeline_clock)
{
  // the default GstSystemClock is monotonic
  for (auto s : sink_stamps("arrival-pipeline", {})) EXPECT_NEAR(s, monotonic_now_ns(), 10e9);
}
