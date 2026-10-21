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

// Shared helpers of the gst_bridge element tests: pipelines run in the test process (the
// elements use their own ROS contexts) and talk to a test node over DDS. appsrc feeds the sinks
// with crafted buffers, appsink checks the output of the sources. Messages with old stamps stand
// in for ros2 bag play.

#ifndef GST_BRIDGE_TEST_UTILS_HPP_
#define GST_BRIDGE_TEST_UTILS_HPP_

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace gst_bridge_test
{
using namespace std::chrono_literals;

// node of the test process, spun by test_main.cpp
rclcpp::Node::SharedPtr test_node();

// timeouts scale with GST_BRIDGE_TEST_TIME_SCALE, e.g. when running under valgrind
inline int time_scale()
{
  static const int scale = [] {
    const char * env = std::getenv("GST_BRIDGE_TEST_TIME_SCALE");
    int value = env ? std::atoi(env) : 1;
    return value > 0 ? value : 1;
  }();
  return scale;
}

inline std::chrono::milliseconds scaled(std::chrono::milliseconds timeout)
{
  return timeout * time_scale();
}

// 2026-01-01T00:00:00Z, a recording far before the stream start
inline constexpr int64_t OLD_STAMP_NS = 1767225600LL * 1000000000LL;
inline constexpr int64_t NTP_UNIX_OFFSET_NS = 2208988800LL * 1000000000LL;

inline int64_t to_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

inline builtin_interfaces::msg::Time from_ns(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

inline int64_t wall_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

inline int64_t monotonic_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

// unique topic per test, all tests share one ROS domain
inline std::string test_topic(const std::string & suffix)
{
  const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string name = std::string(info->test_suite_name()) + "_" + info->name();
  for (auto & c : name) {
    if (!isalnum(static_cast<unsigned char>(c))) c = '_';
  }
  return "/test/" + name + "/" + suffix;
}

// collects the messages of a topic from the test node
template <typename MsgT>
class Collector
{
public:
  explicit Collector(const std::string & topic)
  {
    sub_ = test_node()->create_subscription<MsgT>(
      topic, rclcpp::SensorDataQoS(), [this](typename MsgT::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lck(mtx_);
        msgs_.push_back(*msg);
        cv_.notify_all();
      });
  }

  bool wait_for(size_t n, std::chrono::milliseconds timeout = scaled(10s))
  {
    std::unique_lock<std::mutex> lck(mtx_);
    return cv_.wait_for(lck, timeout, [&] { return msgs_.size() >= n; });
  }

  bool wait_for_publisher(std::chrono::milliseconds timeout = scaled(10s))
  {
    auto end = std::chrono::steady_clock::now() + timeout;
    while (sub_->get_publisher_count() == 0) {
      if (std::chrono::steady_clock::now() > end) return false;
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  std::vector<MsgT> msgs()
  {
    std::lock_guard<std::mutex> lck(mtx_);
    return msgs_;
  }

private:
  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::vector<MsgT> msgs_;
};

// owns a parsed pipeline, stops it on destruction
class Pipeline
{
public:
  explicit Pipeline(const std::string & description)
  {
    GError * error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error) {
      ADD_FAILURE() << "pipeline '" << description << "': " << error->message;
      g_error_free(error);
    }
  }
  ~Pipeline()
  {
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
    }
  }
  Pipeline(const Pipeline &) = delete;
  Pipeline & operator=(const Pipeline &) = delete;

  bool valid() const { return pipeline_ != nullptr; }

  // borrowed, valid while the pipeline exists
  GstElement * element(const char * name)
  {
    GstElement * e = gst_bin_get_by_name(GST_BIN(pipeline_), name);
    if (e) gst_object_unref(e);  // the bin keeps it alive
    return e;
  }

  GstStateChangeReturn set_state(GstState state) { return gst_element_set_state(pipeline_, state); }

  // all messages of the given types posted so far
  std::vector<std::string> bus_messages(GstMessageType types)
  {
    std::vector<std::string> texts;
    GstBus * bus = gst_element_get_bus(pipeline_);
    while (GstMessage * msg = gst_bus_pop_filtered(bus, types)) {
      GError * err = nullptr;
      gchar * dbg = nullptr;
      if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) gst_message_parse_error(msg, &err, &dbg);
      if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_WARNING) gst_message_parse_warning(msg, &err, &dbg);
      texts.push_back(err ? err->message : "");
      if (err) g_error_free(err);
      g_free(dbg);
      gst_message_unref(msg);
    }
    gst_object_unref(bus);
    return texts;
  }

private:
  GstElement * pipeline_ = nullptr;
};

// push a buffer with the given PTS and optional reference timestamp metas into an appsrc
inline void push_buffer(
  GstElement * appsrc, GstClockTime pts,
  const std::vector<std::pair<std::string, GstClockTime>> & metas = {})
{
  static const guint8 payload[] = {0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0xff, 0xd9};
  GstBuffer * buf = gst_buffer_new_memdup(payload, sizeof(payload));
  GST_BUFFER_PTS(buf) = pts;
  for (const auto & meta : metas) {
    GstCaps * caps = gst_caps_new_empty_simple(meta.first.c_str());
    gst_buffer_add_reference_timestamp_meta(buf, caps, meta.second, GST_CLOCK_TIME_NONE);
    gst_caps_unref(caps);
  }
  ASSERT_EQ(gst_app_src_push_buffer(GST_APP_SRC(appsrc), buf), GST_FLOW_OK);
}

// a calibration file in the camera_calibration_parsers yaml format
inline std::string write_calibration(const std::string & file_name, const std::string & camera_name)
{
  auto dir =
    std::filesystem::temp_directory_path() / ("gst_bridge_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(dir);
  auto path = dir / file_name;
  std::ofstream f(path);
  f << "image_width: 320\nimage_height: 240\n";
  if (!camera_name.empty()) f << "camera_name: " << camera_name << "\n";
  f << "camera_matrix: {rows: 3, cols: 3, data: [300, 0, 160, 0, 300, 120, 0, 0, 1]}\n"
       "distortion_model: plumb_bob\n"
       "distortion_coefficients: {rows: 1, cols: 5, data: [0.1, 0, 0, 0, 0]}\n"
       "rectification_matrix: {rows: 3, cols: 3, data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}\n"
       "projection_matrix: {rows: 3, cols: 4, data: [300, 0, 160, 0, 0, 300, 120, 0, 0, 0, 1, "
       "0]}\n";
  return path.string();
}

// publishes messages from the test node until the test stops it
template <typename MsgT>
class Publisher
{
public:
  explicit Publisher(const std::string & topic)
  {
    pub_ = test_node()->create_publisher<MsgT>(topic, rclcpp::SensorDataQoS());
  }
  bool wait_for_subscriber(std::chrono::milliseconds timeout = scaled(10s))
  {
    auto end = std::chrono::steady_clock::now() + timeout;
    while (pub_->get_subscription_count() == 0) {
      if (std::chrono::steady_clock::now() > end) return false;
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }
  void publish(const MsgT & msg) { pub_->publish(msg); }

private:
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
};

inline sensor_msgs::msg::CompressedImage compressed_msg(
  int64_t stamp_ns, const std::string & format)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = from_ns(stamp_ns);
  msg.header.frame_id = "cam";
  msg.format = format;
  msg.data = {0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0xff, 0xd9};
  return msg;
}

// publish (with increasing stamps from first_stamp_ns) until the appsink got n samples
inline std::vector<GstSample *> pull_samples_while_publishing(
  GstElement * appsink, Publisher<sensor_msgs::msg::CompressedImage> & pub, int64_t first_stamp_ns,
  const std::string & format, size_t n, std::vector<int64_t> * published = nullptr)
{
  std::vector<GstSample *> samples;
  auto end = std::chrono::steady_clock::now() + scaled(15s);
  int64_t stamp = first_stamp_ns;
  while (samples.size() < n && std::chrono::steady_clock::now() < end) {
    pub.publish(compressed_msg(stamp, format));
    if (published) published->push_back(stamp);
    stamp += 100000000;  // 10 Hz
    if (GstSample * s = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 100 * GST_MSECOND)) {
      samples.push_back(s);
    }
  }
  return samples;
}

inline void unref_samples(std::vector<GstSample *> & samples)
{
  for (auto * s : samples) gst_sample_unref(s);
  samples.clear();
}

}  // namespace gst_bridge_test

#endif  // GST_BRIDGE_TEST_UTILS_HPP_
