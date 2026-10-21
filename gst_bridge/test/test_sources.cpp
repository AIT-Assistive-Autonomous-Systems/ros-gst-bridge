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

// roscompressedimagesrc and rosimagesrc: timestamp modes, arrival fallback for old stamps
// (bag playback), header as reference meta, formats, shutdown

#include <future>
#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <set>

#include "test_utils.hpp"

using namespace gst_bridge_test;

class CompressedSrc : public ::testing::Test
{
protected:
  // run roscompressedimagesrc with the given properties into an appsink
  std::vector<GstSample *> run(
    const std::string & props, const std::string & format, bool sync, int64_t first_stamp_ns,
    std::vector<int64_t> * published = nullptr)
  {
    const std::string base = test_topic("image_raw");
    Publisher<sensor_msgs::msg::CompressedImage> pub(base + "/compressed");
    pipeline_ = std::make_unique<Pipeline>(
      "roscompressedimagesrc ros-topic=" + base + " " + props +
      " ! appsink name=sink max-buffers=100 sync=" + (sync ? "true" : "false"));
    EXPECT_NE(pipeline_->set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
    EXPECT_TRUE(pub.wait_for_subscriber());
    return pull_samples_while_publishing(
      pipeline_->element("sink"), pub, first_stamp_ns, format, 3, published);
  }

  std::unique_ptr<Pipeline> pipeline_;
};

TEST_F(CompressedSrc, old_stamps_fall_back_to_arrival_with_warning)
{
  // sync=true: a PTS from an underflowed stamp would block the sink
  auto samples = run("", "jpeg", true, OLD_STAMP_NS);
  ASSERT_EQ(samples.size(), 3u) << "source stalled";
  for (auto * s : samples) {
    GstClockTime pts = GST_BUFFER_PTS(gst_sample_get_buffer(s));
    ASSERT_TRUE(GST_CLOCK_TIME_IS_VALID(pts));
    EXPECT_LT(pts, 60 * GST_SECOND) << "expected the arrival running time";
  }
  GstStructure * caps_struct = gst_caps_get_structure(gst_sample_get_caps(samples[0]), 0);
  EXPECT_TRUE(gst_structure_has_name(caps_struct, "image/jpeg"));
  bool warned = false;
  for (const auto & text : pipeline_->bus_messages(GST_MESSAGE_WARNING)) {
    warned |= text.find("before the stream start") != std::string::npos;
  }
  EXPECT_TRUE(warned);
  unref_samples(samples);
}

TEST_F(CompressedSrc, arrival_mode_without_warning)
{
  auto samples = run("timestamp-mode=arrival", "jpeg", true, OLD_STAMP_NS);
  ASSERT_EQ(samples.size(), 3u);
  for (auto * s : samples) {
    EXPECT_LT(GST_BUFFER_PTS(gst_sample_get_buffer(s)), 60 * GST_SECOND);
  }
  EXPECT_TRUE(pipeline_->bus_messages(GST_MESSAGE_WARNING).empty());
  unref_samples(samples);
}

TEST_F(CompressedSrc, pts_mode_uses_header_stamp)
{
  std::vector<int64_t> published;
  auto samples = run("timestamp-mode=pts", "jpeg", false, OLD_STAMP_NS, &published);
  ASSERT_EQ(samples.size(), 3u);
  std::set<int64_t> stamps(published.begin(), published.end());
  for (auto * s : samples) {
    EXPECT_TRUE(stamps.count(GST_BUFFER_PTS(gst_sample_get_buffer(s))));
  }
  unref_samples(samples);
}

TEST_F(CompressedSrc, header_as_reference_meta)
{
  std::vector<int64_t> published;
  auto samples = run(
    "timestamp-mode=arrival attach-reference-timestamp=true time-caps=timestamp/x-unix", "jpeg",
    true, OLD_STAMP_NS, &published);
  ASSERT_EQ(samples.size(), 3u);
  std::set<int64_t> stamps(published.begin(), published.end());
  GstCaps * unix_caps = gst_caps_new_empty_simple("timestamp/x-unix");
  for (auto * s : samples) {
    auto * meta = gst_buffer_get_reference_timestamp_meta(gst_sample_get_buffer(s), unix_caps);
    ASSERT_NE(meta, nullptr);
    EXPECT_TRUE(stamps.count(meta->timestamp));
  }
  gst_caps_unref(unix_caps);
  unref_samples(samples);
}

TEST_F(CompressedSrc, png_and_image_transport_format_strings)
{
  auto samples = run("timestamp-mode=arrival", "png", false, OLD_STAMP_NS);
  ASSERT_FALSE(samples.empty());
  EXPECT_TRUE(gst_structure_has_name(
    gst_caps_get_structure(gst_sample_get_caps(samples[0]), 0), "image/png"));
  unref_samples(samples);

  samples = run("timestamp-mode=arrival", "bgr8; jpeg compressed bgr8", false, OLD_STAMP_NS);
  ASSERT_FALSE(samples.empty());
  EXPECT_TRUE(gst_structure_has_name(
    gst_caps_get_structure(gst_sample_get_caps(samples[0]), 0), "image/jpeg"));
  unref_samples(samples);
}

TEST(CompressedSrcFormat, compressed_depth_is_rejected)
{
  const std::string base = test_topic("image_raw");
  Publisher<sensor_msgs::msg::CompressedImage> pub(base + "/compressed");
  Pipeline p("roscompressedimagesrc timestamp-mode=arrival ros-topic=" + base + " ! fakesink");
  ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  ASSERT_TRUE(pub.wait_for_subscriber());

  bool error = false;
  for (int i = 0; i < 50 && !error; i++) {
    pub.publish(compressed_msg(OLD_STAMP_NS, "16UC1; compressedDepth png"));
    std::this_thread::sleep_for(100ms);
    for (const auto & text : p.bus_messages(GST_MESSAGE_ERROR)) {
      error |= text.find("unsupported compressed image format") != std::string::npos;
    }
  }
  EXPECT_TRUE(error);
}

TEST(CompressedSrcShutdown, stops_while_waiting_for_messages)
{
  Pipeline p("roscompressedimagesrc ros-topic=" + test_topic("nobody") + " ! fakesink");
  ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  std::this_thread::sleep_for(300ms);
  auto stopped = std::async(std::launch::async, [&] { return p.set_state(GST_STATE_NULL); });
  EXPECT_EQ(stopped.wait_for(5s), std::future_status::ready) << "create() did not unlock";
}

TEST(ImageSrc, old_stamps_do_not_stall)
{
  const std::string topic = test_topic("image_raw");
  Publisher<sensor_msgs::msg::Image> pub(topic);
  Pipeline p("rosimagesrc ros-topic=" + topic + " ! appsink name=sink max-buffers=100 sync=true");
  ASSERT_NE(p.set_state(GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
  ASSERT_TRUE(pub.wait_for_subscriber());

  sensor_msgs::msg::Image msg;
  msg.width = 4;
  msg.height = 2;
  msg.encoding = "rgb8";
  msg.step = 12;
  msg.data.assign(24, 0x80);

  std::vector<GstSample *> samples;
  auto end = std::chrono::steady_clock::now() + 15s;
  for (int i = 0; samples.size() < 3 && std::chrono::steady_clock::now() < end; i++) {
    msg.header.stamp = from_ns(OLD_STAMP_NS + i * 100000000LL);
    pub.publish(msg);
    if (
      GstSample * s =
        gst_app_sink_try_pull_sample(GST_APP_SINK(p.element("sink")), 100 * GST_MSECOND)) {
      samples.push_back(s);
    }
  }
  ASSERT_EQ(samples.size(), 3u) << "source stalled";
  for (auto * s : samples) {
    EXPECT_LT(GST_BUFFER_PTS(gst_sample_get_buffer(s)), 60 * GST_SECOND);
  }
  unref_samples(samples);
}
