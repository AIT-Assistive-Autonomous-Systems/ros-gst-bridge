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
/**
 * SECTION:element-gstroscompressedimagesrc
 *
 * The roscompressedimagesrc element subscribes to jpeg or png sensor_msgs/CompressedImage
 * messages and pushes them into the pipeline, the return path of roscompressedimagesink.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 roscompressedimagesrc ros-topic="/cam/image_raw" use-msg-stamp=false ! jpegdec ! videoconvert ! autovideosink
 * ]|
 * Shows the images of /cam/image_raw/compressed, e.g. from ros2 bag play.
 * </refsect2>
 */

#include <gst_bridge/camera_info.h>
#include <gst_bridge/roscompressedimagesrc.h>

GST_DEBUG_CATEGORY_STATIC(roscompressedimagesrc_debug_category);
#define GST_CAT_DEFAULT roscompressedimagesrc_debug_category

/* prototypes */

static void roscompressedimagesrc_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void roscompressedimagesrc_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec);
static void roscompressedimagesrc_init(Roscompressedimagesrc * src);
static void roscompressedimagesrc_finalize(GObject * object);

static gboolean roscompressedimagesrc_open(RosBaseSrc * ros_base_src);
static gboolean roscompressedimagesrc_close(RosBaseSrc * ros_base_src);
static gboolean roscompressedimagesrc_notify_thread(RosBaseSrc * ros_base_src);

static gboolean roscompressedimagesrc_unlock(GstBaseSrc * base_src);
static gboolean roscompressedimagesrc_unlock_stop(GstBaseSrc * base_src);
static gboolean roscompressedimagesrc_query(GstBaseSrc * base_src, GstQuery * query);
static GstFlowReturn roscompressedimagesrc_create(
  GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer ** buf);

static void roscompressedimagesrc_sub_cb(
  Roscompressedimagesrc * src, Roscompressedimagesrc::MsgType::ConstSharedPtr msg);

enum {
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_TRANSPORT,
  PROP_USE_MSG_STAMP,
  PROP_ROS_FRAME_ID,
  PROP_ROS_FORMAT,
};

// messages kept while the pipeline is busy, older ones are dropped
static const size_t msg_queue_max = 1;

/* pad templates */

static GstStaticPadTemplate roscompressedimagesrc_src_template = GST_STATIC_PAD_TEMPLATE(
  "src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS("image/jpeg; image/png"));

/* class initialization */

G_DEFINE_TYPE_WITH_CODE(
  Roscompressedimagesrc, roscompressedimagesrc, GST_TYPE_ROS_BASE_SRC,
  GST_DEBUG_CATEGORY_INIT(
    roscompressedimagesrc_debug_category, "roscompressedimagesrc", 0,
    "debug category for roscompressedimagesrc element"))

static void roscompressedimagesrc_class_init(RoscompressedimagesrcClass * klass)
{
  GObjectClass * object_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSrcClass * basesrc_class = GST_BASE_SRC_CLASS(klass);
  RosBaseSrcClass * ros_base_src_class = GST_ROS_BASE_SRC_CLASS(klass);

  object_class->set_property = roscompressedimagesrc_set_property;
  object_class->get_property = roscompressedimagesrc_get_property;
  object_class->finalize = roscompressedimagesrc_finalize;

  gst_element_class_add_static_pad_template(element_class, &roscompressedimagesrc_src_template);

  gst_element_class_set_static_metadata(
    element_class, "roscompressedimagesrc", "Source/Video",
    "a gstreamer source that subscribes to jpeg or png compressed images from ROS",
    "Markus Hofstaetter");

  g_object_class_install_property(
    object_class, PROP_ROS_TOPIC,
    g_param_spec_string(
      "ros-topic", "sub-topic",
      "base image topic, the image is subscribed on <ros-topic>/<ros-transport>", "gst_image_sub",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_TRANSPORT,
    g_param_spec_string(
      "ros-transport", "transport",
      "image_transport style suffix appended to ros-topic, subscribe to ros-topic directly if "
      "empty",
      "compressed", (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_USE_MSG_STAMP,
    g_param_spec_boolean(
      "use-msg-stamp", "use-msg-stamp",
      "timestamp buffers from the message header stamp, else with the arrival time "
      "(use false for ros2 bag play without sim time)",
      TRUE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_FRAME_ID,
    g_param_spec_string(
      "ros-frame-id", "frame-id", "frame_id of the last message", "",
      (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_FORMAT,
    g_param_spec_string(
      "ros-format", "format-string", "format string of the last message", "",
      (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));

  ros_base_src_class->open = GST_DEBUG_FUNCPTR(roscompressedimagesrc_open);
  ros_base_src_class->close = GST_DEBUG_FUNCPTR(roscompressedimagesrc_close);
  ros_base_src_class->notify_thread = GST_DEBUG_FUNCPTR(roscompressedimagesrc_notify_thread);

  basesrc_class->create = GST_DEBUG_FUNCPTR(roscompressedimagesrc_create);
  basesrc_class->unlock = GST_DEBUG_FUNCPTR(roscompressedimagesrc_unlock);
  basesrc_class->unlock_stop = GST_DEBUG_FUNCPTR(roscompressedimagesrc_unlock_stop);
  basesrc_class->query = GST_DEBUG_FUNCPTR(roscompressedimagesrc_query);
}

static void roscompressedimagesrc_init(Roscompressedimagesrc * src)
{
  RosBaseSrc * ros_base_src = GST_ROS_BASE_SRC(src);
  ros_base_src->node_name = g_strdup("gst_compressed_image_src_node");

  // the instance is zero allocated by GObject, construct the C++ members in place
  new (&src->sub_topic) std::string("gst_image_sub");
  new (&src->transport) std::string("compressed");
  src->use_msg_stamp = true;
  new (&src->sub) rclcpp::Subscription<Roscompressedimagesrc::MsgType>::SharedPtr();
  new (&src->msg_queue) std::deque<Roscompressedimagesrc::MsgType::ConstSharedPtr>();
  new (&src->msg_queue_mtx) std::mutex();
  new (&src->msg_queue_cv) std::condition_variable();
  src->flushing = false;
  new (&src->frame_id) std::string();
  new (&src->format) std::string();
  new (&src->media_type) std::string();
  src->first_buffer = true;
  src->warned_stamp = false;

  /* configure basesrc to be a live source */
  gst_base_src_set_live(GST_BASE_SRC(src), TRUE);
  /* make basesrc output a segment in time */
  gst_base_src_set_format(GST_BASE_SRC(src), GST_FORMAT_TIME);
  /* buffers without a timestamp get the running time when they were captured */
  gst_base_src_set_do_timestamp(GST_BASE_SRC(src), TRUE);
}

static void roscompressedimagesrc_finalize(GObject * object)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(object);
  using std::condition_variable;
  using std::mutex;
  using std::string;
  using SubscriptionPtr = rclcpp::Subscription<Roscompressedimagesrc::MsgType>::SharedPtr;
  using Queue = std::deque<Roscompressedimagesrc::MsgType::ConstSharedPtr>;
  src->sub_topic.~string();
  src->transport.~string();
  src->sub.~SubscriptionPtr();
  src->msg_queue.~Queue();
  src->msg_queue_mtx.~mutex();
  src->msg_queue_cv.~condition_variable();
  src->frame_id.~string();
  src->format.~string();
  src->media_type.~string();
  G_OBJECT_CLASS(roscompressedimagesrc_parent_class)->finalize(object);
}

static void roscompressedimagesrc_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec)
{
  RosBaseSrc * ros_base_src = GST_ROS_BASE_SRC(object);
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(object);

  GST_DEBUG_OBJECT(src, "set_property");

  switch (property_id) {
    case PROP_ROS_TOPIC:
      if (src->sub) {
        RCLCPP_ERROR(
          ros_base_src->node_if->logging->get_logger(), "can't change topic name once opened");
      } else {
        src->sub_topic = gst_bridge::value_to_string(value);
      }
      break;

    case PROP_ROS_TRANSPORT:
      if (src->sub) {
        RCLCPP_ERROR(
          ros_base_src->node_if->logging->get_logger(), "can't change transport once opened");
      } else {
        src->transport = gst_bridge::value_to_string(value);
      }
      break;

    case PROP_USE_MSG_STAMP:
      src->use_msg_stamp = g_value_get_boolean(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

static void roscompressedimagesrc_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(object);

  GST_DEBUG_OBJECT(src, "get_property");

  switch (property_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, src->sub_topic.c_str());
      break;

    case PROP_ROS_TRANSPORT:
      g_value_set_string(value, src->transport.c_str());
      break;

    case PROP_USE_MSG_STAMP:
      g_value_set_boolean(value, src->use_msg_stamp);
      break;

    case PROP_ROS_FRAME_ID: {
      std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
      g_value_set_string(value, src->frame_id.c_str());
      break;
    }

    case PROP_ROS_FORMAT: {
      std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
      g_value_set_string(value, src->format.c_str());
      break;
    }

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

/* open the subscription with given specs */
static gboolean roscompressedimagesrc_open(RosBaseSrc * ros_base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(ros_base_src);

  GST_DEBUG_OBJECT(src, "open");

  auto cb = [src](Roscompressedimagesrc::MsgType::ConstSharedPtr msg) {
    roscompressedimagesrc_sub_cb(src, msg);
  };
  rclcpp::QoS qos = rclcpp::SensorDataQoS();  //XXX add a parameter for overrides

  const std::string topic =
    src->transport.empty() ? src->sub_topic : src->sub_topic + "/" + src->transport;

  // e.g. invalid topic names throw, fail the state change instead of aborting the process
  try {
    src->sub = rclcpp::create_subscription<Roscompressedimagesrc::MsgType>(
      ros_base_src->node_if->parameters, ros_base_src->node_if->topics, topic, qos, cb);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_base_src->node_if->logging->get_logger(), "can't subscribe to '%s': %s", topic.c_str(),
      e.what());
    return FALSE;
  }

  return TRUE;
}

/* close the subscription */
static gboolean roscompressedimagesrc_close(RosBaseSrc * ros_base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(ros_base_src);

  GST_DEBUG_OBJECT(src, "close");

  src->sub.reset();

  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  src->msg_queue.clear();
  src->media_type.clear();
  src->first_buffer = true;

  return TRUE;
}

static gboolean roscompressedimagesrc_notify_thread(RosBaseSrc * ros_base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(ros_base_src);
  GST_DEBUG_OBJECT(src, "notify_thread");
  src->msg_queue_cv.notify_all();
  return TRUE;
}

/* basesrc asks us to return from create, e.g. on flushing seeks and state changes to READY */
static gboolean roscompressedimagesrc_unlock(GstBaseSrc * base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(base_src);
  GST_DEBUG_OBJECT(src, "unlock");
  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  src->flushing = true;
  src->msg_queue_cv.notify_all();
  return TRUE;
}

static gboolean roscompressedimagesrc_unlock_stop(GstBaseSrc * base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(base_src);
  GST_DEBUG_OBJECT(src, "unlock_stop");
  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  src->flushing = false;
  return TRUE;
}

static gboolean roscompressedimagesrc_query(GstBaseSrc * base_src, GstQuery * query)
{
  switch (GST_QUERY_TYPE(query)) {
    case GST_QUERY_SCHEDULING:
      /* push mode only */
      gst_query_set_scheduling(query, GST_SCHEDULING_FLAG_SEQUENTIAL, 1, -1, 0);
      gst_query_add_scheduling_mode(query, GST_PAD_MODE_PUSH);
      return TRUE;
    case GST_QUERY_LATENCY:
      /* live, messages are pushed on arrival and wait in the queue, no upper bound */
      gst_query_set_latency(query, TRUE, 0, GST_CLOCK_TIME_NONE);
      return TRUE;
    default:
      return GST_BASE_SRC_CLASS(roscompressedimagesrc_parent_class)->query(base_src, query);
  }
}

// media type of a CompressedImage format string, empty if unsupported
//   image_transport writes e.g. "jpeg", "png" or "bgr8; jpeg compressed bgr8"
static std::string media_type_for(const std::string & format)
{
  if (format.find("compressedDepth") != std::string::npos) {
    return "";  // prefixed with a depth quantization header, not a plain png
  }
  if (format.find("png") != std::string::npos) {
    return "image/png";
  }
  if (format.find("jpeg") != std::string::npos || format.find("jpg") != std::string::npos) {
    return "image/jpeg";
  }
  return "";
}

/*
 * Wait for a message and push its data, the caps follow the format of the message
 */
static GstFlowReturn roscompressedimagesrc_create(
  GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer ** buf)
{
  RosBaseSrc * ros_base_src = GST_ROS_BASE_SRC(base_src);
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(base_src);
  Roscompressedimagesrc::MsgType::ConstSharedPtr msg;

  (void)offset;
  (void)size;

  GST_DEBUG_OBJECT(src, "create");

  {
    std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
    src->msg_queue_cv.wait(lck, [src] { return src->flushing || !src->msg_queue.empty(); });
    if (src->flushing) {
      return GST_FLOW_FLUSHING;
    }
    msg = src->msg_queue.front();
    src->msg_queue.pop_front();
  }

  const std::string media_type = media_type_for(msg->format);
  if (media_type.empty()) {
    GST_ELEMENT_ERROR(
      src, STREAM, FORMAT, ("unsupported compressed image format '%s'", msg->format.c_str()),
      (NULL));
    return GST_FLOW_NOT_NEGOTIATED;
  }
  if (media_type != src->media_type) {
    GstCaps * caps = gst_caps_new_empty_simple(media_type.c_str());
    gboolean ok = gst_base_src_set_caps(base_src, caps);
    gst_caps_unref(caps);
    if (!ok) {
      GST_ELEMENT_ERROR(
        src, CORE, NEGOTIATION, ("downstream does not accept %s", media_type.c_str()), (NULL));
      return GST_FLOW_NOT_NEGOTIATED;
    }
    src->media_type = media_type;
  }

  *buf = gst_buffer_new_memdup(msg->data.data(), msg->data.size());

  GstClockTime msg_time = rclcpp::Time(msg->header.stamp).nanoseconds();
  // also attaches the reference timestamp meta if enabled
  set_timestamps(buf, ros_base_src, GST_ELEMENT(src), msg_time);

  // set_timestamps underflows for stamps before the stream start (e.g. bag playback),
  // leave those buffers without a PTS so basesrc stamps them with the arrival time
  GstClockTimeDiff stream_pts = (GstClockTimeDiff)msg_time - ros_base_src->ros_clock_offset -
                                (GstClockTimeDiff)gst_element_get_base_time(GST_ELEMENT(src));
  if (!src->use_msg_stamp || stream_pts < 0) {
    // the first message may be received during preroll, only later ones indicate old stamps
    if (src->use_msg_stamp && !src->first_buffer && !src->warned_stamp) {
      GST_ELEMENT_WARNING(
        src, STREAM, FAILED, ("message stamps are before the stream start"),
        ("timestamping with the arrival time instead, set use-msg-stamp=false to silence this"));
      src->warned_stamp = true;
    }
    GST_BUFFER_PTS(*buf) = GST_CLOCK_TIME_NONE;
  }
  src->first_buffer = false;

  return GST_FLOW_OK;
}

static void roscompressedimagesrc_sub_cb(
  Roscompressedimagesrc * src, Roscompressedimagesrc::MsgType::ConstSharedPtr msg)
{
  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  src->frame_id = msg->header.frame_id;
  src->format = msg->format;
  src->msg_queue.push_back(msg);
  while (src->msg_queue.size() > msg_queue_max) {
    src->msg_queue.pop_front();  // keep the latest
    GST_DEBUG_OBJECT(src, "dropping message");
  }
  src->msg_queue_cv.notify_one();
}
