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

// End-to-end tests of the gst_bridge elements: pipelines run in this process (the elements
// use their own ROS contexts) and talk to a test node over DDS. appsrc feeds the sinks with
// crafted buffers, appsink checks the output of the sources. Messages with old stamps stand in
// for ros2 bag play.

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst_bridge/camera_info.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosaudiosink.h>
#include <gst_bridge/rosaudiosrc.h>
#include <gst_bridge/roscompressedimagesink.h>
#include <gst_bridge/roscompressedimagesrc.h>
#include <gst_bridge/rosimagesink.h>
#include <gst_bridge/rosimagesrc.h>
#include <gst_bridge/rosrawsink.h>
#include <gst_bridge/rosrawsrc.h>
#include <gst_bridge/rostextsink.h>
#include <gst_bridge/rostextsrc.h>
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
// 2026-01-01T00:00:00Z, a recording far before the stream start
constexpr int64_t OLD_STAMP_NS = 1767225600LL * 1000000000LL;
constexpr int64_t NTP_UNIX_OFFSET_NS = 2208988800LL * 1000000000LL;

rclcpp::Node::SharedPtr g_node;

int64_t to_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

builtin_interfaces::msg::Time from_ns(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

int64_t wall_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

int64_t monotonic_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

// unique topic per test, all tests share one ROS domain
std::string test_topic(const std::string & suffix)
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
    sub_ = g_node->create_subscription<MsgT>(
      topic, rclcpp::SensorDataQoS(), [this](typename MsgT::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lck(mtx_);
        msgs_.push_back(*msg);
        cv_.notify_all();
      });
  }

  bool wait_for(size_t n, std::chrono::milliseconds timeout = 10s)
  {
    std::unique_lock<std::mutex> lck(mtx_);
    return cv_.wait_for(lck, timeout, [&] { return msgs_.size() >= n; });
  }

  bool wait_for_publisher(std::chrono::milliseconds timeout = 10s)
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
void push_buffer(
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
std::string write_calibration(const std::string & file_name, const std::string & camera_name)
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
    pub_ = g_node->create_publisher<MsgT>(topic, rclcpp::SensorDataQoS());
  }
  bool wait_for_subscriber(std::chrono::milliseconds timeout = 10s)
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

sensor_msgs::msg::CompressedImage compressed_msg(int64_t stamp_ns, const std::string & format)
{
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = from_ns(stamp_ns);
  msg.header.frame_id = "cam";
  msg.format = format;
  msg.data = {0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0xff, 0xd9};
  return msg;
}

// publish (with increasing stamps from first_stamp_ns) until the appsink got n samples
std::vector<GstSample *> pull_samples_while_publishing(
  GstElement * appsink, Publisher<sensor_msgs::msg::CompressedImage> & pub, int64_t first_stamp_ns,
  const std::string & format, size_t n, std::vector<int64_t> * published = nullptr)
{
  std::vector<GstSample *> samples;
  auto end = std::chrono::steady_clock::now() + 15s;
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

void unref_samples(std::vector<GstSample *> & samples)
{
  for (auto * s : samples) gst_sample_unref(s);
  samples.clear();
}

}  // namespace

// ---------------------------------------------------------------- helpers

TEST(Helpers, camera_info_topic_for)
{
  EXPECT_EQ(gst_bridge::camera_info_topic_for("/cam/image_raw"), "/cam/camera_info");
  EXPECT_EQ(gst_bridge::camera_info_topic_for("cam/image_raw"), "cam/camera_info");
  EXPECT_EQ(gst_bridge::camera_info_topic_for("image_raw"), "camera_info");
}

TEST(Helpers, caps_to_string)
{
  GstCaps * caps = gst_caps_from_string("image/jpeg, width=(int)320");
  EXPECT_EQ(gst_bridge::caps_to_string(caps), "image/jpeg, width=(int)320");
  gst_caps_unref(caps);
}

// ---------------------------------------------------------------- type registration

// GObject registers sizeof(instance) (guint16) and only guarantees 16 byte alignment
template <typename T>
void check_layout(const char * factory)
{
  GstElement * e = gst_element_factory_make(factory, nullptr);
  ASSERT_NE(e, nullptr) << factory;
  GTypeQuery q;
  g_type_query(G_OBJECT_TYPE(e), &q);
  EXPECT_EQ(q.instance_size, sizeof(T)) << factory;
  EXPECT_LE(alignof(T), 2 * sizeof(gsize)) << factory;
  EXPECT_EQ(reinterpret_cast<uintptr_t>(e) % alignof(T), 0u) << factory;
  gst_object_unref(e);
}

TEST(Elements, type_layout)
{
  check_layout<Rosaudiosink>("rosaudiosink");
  check_layout<Rosaudiosrc>("rosaudiosrc");
  check_layout<Roscompressedimagesink>("roscompressedimagesink");
  check_layout<Roscompressedimagesrc>("roscompressedimagesrc");
  check_layout<Rosimagesink>("rosimagesink");
  check_layout<Rosimagesrc>("rosimagesrc");
  check_layout<Rosrawsink>("rosrawsink");
  check_layout<Rosrawsrc>("rosrawsrc");
  check_layout<Rostextsink>("rostextsink");
  check_layout<Rostextsrc>("rostextsrc");
}

TEST(Elements, rostextsrc_topic_alias)
{
  GstElement * e = gst_element_factory_make("rostextsrc", nullptr);
  ASSERT_NE(e, nullptr);
  GParamSpec * old_spec = g_object_class_find_property(G_OBJECT_GET_CLASS(e), "topic");
  ASSERT_NE(old_spec, nullptr);
  EXPECT_TRUE(old_spec->flags & G_PARAM_DEPRECATED);

  g_object_set(e, "ros-topic", "a/b", nullptr);
  gchar * value = nullptr;
  g_object_get(e, "topic", &value, nullptr);
  EXPECT_STREQ(value, "a/b");
  g_free(value);
  gst_object_unref(e);
}

// ---------------------------------------------------------------- compressed sink, camera info

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
  EXPECT_EQ(g_node->count_publishers(gst_bridge::camera_info_topic_for(base)), 0u);
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

// ---------------------------------------------------------------- sink timestamp modes

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

// ---------------------------------------------------------------- compressed source

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

// ---------------------------------------------------------------- round trip

// bag playback -> processing -> back to ROS keeps the original header stamps
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

// ---------------------------------------------------------------- raw image source

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

// ---------------------------------------------------------------- main

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  g_node = std::make_shared<rclcpp::Node>("gst_bridge_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(g_node);
  std::thread spin([&] { executor.spin(); });

  int result = RUN_ALL_TESTS();

  executor.cancel();
  spin.join();
  g_node.reset();
  rclcpp::shutdown();
  return result;
}
