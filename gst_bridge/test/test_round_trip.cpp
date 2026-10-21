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

// bag playback -> processing -> back to ROS keeps the original header stamps

#include <set>

#include "test_utils.hpp"

using namespace gst_bridge_test;

TEST(RoundTrip, reference_stamps_survive_the_pipeline)
{
  const std::string in = test_topic("in");
  const std::string out = test_topic("out");
  Publisher<sensor_msgs::msg::CompressedImage> pub(in + "/compressed");
  Collector<sensor_msgs::msg::CompressedImage> relayed(out + "/compressed");
  Pipeline p(
    "roscompressedimagesrc ros-topic=" + in +
    " timestamp-mode=arrival attach-reference-timestamp=true time-caps=timestamp/x-unix ! "
    "roscompressedimagesink ros-topic=" +
    out + " timestamp-mode=reference reference-caps=timestamp/x-unix");
  ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  ASSERT_TRUE(pub.wait_for_subscriber());
  ASSERT_TRUE(relayed.wait_for_publisher());

  std::set<int64_t> published;
  for (int i = 0; i < 100 && !relayed.wait_for(5, 50ms); i++) {
    int64_t stamp = OLD_STAMP_NS + i * 100000000LL;
    published.insert(stamp);
    pub.publish(compressed_msg(stamp, "jpeg"));
  }
  ASSERT_TRUE(relayed.wait_for(5));
  for (const auto & msg : relayed.msgs()) {
    EXPECT_TRUE(published.count(to_ns(msg.header.stamp))) << to_ns(msg.header.stamp);
  }
}
