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
 * gst-launch-1.0 roscompressedimagesrc ros-topic="/cam/image_raw" timestamp-mode=arrival ! jpegdec ! videoconvert ! autovideosink
 * ]|
 * Shows the images of /cam/image_raw/compressed, e.g. from ros2 bag play.
 * Add attach-reference-timestamp=true time-caps=timestamp/x-unix to keep the original header
 * stamps as reference timestamp meta (e.g. for a sink with timestamp-mode=reference).
 * </refsect2>
 */

#include <gst_bridge/camera_info.h>
#include <gst_bridge/roscompressedimagesrc.h>

GST_DEBUG_CATEGORY_STATIC(roscompressedimagesrc_debug_category);
#define GST_CAT_DEFAULT roscompressedimagesrc_debug_category

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
  g_free(ros_base_src->node_name);  // set by the base class init
  ros_base_src->node_name = g_strdup("gst_compressed_image_src_node");

  src->priv = new RoscompressedimagesrcPrivate();

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
  delete src->priv;
  src->priv = nullptr;
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
      if (src->priv->sub) {
        RCLCPP_ERROR(
          ros_base_src->node_if->logging->get_logger(), "can't change topic name once opened");
      } else {
        src->priv->sub_topic = gst_bridge::value_to_string(value);
      }
      break;

    case PROP_ROS_TRANSPORT:
      if (src->priv->sub) {
        RCLCPP_ERROR(
          ros_base_src->node_if->logging->get_logger(), "can't change transport once opened");
      } else {
        src->priv->transport = gst_bridge::value_to_string(value);
      }
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
      g_value_set_string(value, src->priv->sub_topic.c_str());
      break;

    case PROP_ROS_TRANSPORT:
      g_value_set_string(value, src->priv->transport.c_str());
      break;

    case PROP_ROS_FRAME_ID: {
      std::unique_lock<std::mutex> lck(src->priv->msg_queue_mtx);
      g_value_set_string(value, src->priv->frame_id.c_str());
      break;
    }

    case PROP_ROS_FORMAT: {
      std::unique_lock<std::mutex> lck(src->priv->msg_queue_mtx);
      g_value_set_string(value, src->priv->format.c_str());
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

  const std::string topic = src->priv->transport.empty()
                              ? src->priv->sub_topic
                              : src->priv->sub_topic + "/" + src->priv->transport;

  // e.g. invalid topic names throw, fail the state change instead of aborting the process
  try {
    src->priv->sub = rclcpp::create_subscription<Roscompressedimagesrc::MsgType>(
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

  src->priv->sub.reset();

  std::unique_lock<std::mutex> lck(src->priv->msg_queue_mtx);
  src->priv->msg_queue.clear();
  src->priv->media_type.clear();

  return TRUE;
}

static gboolean roscompressedimagesrc_notify_thread(RosBaseSrc * ros_base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(ros_base_src);
  GST_DEBUG_OBJECT(src, "notify_thread");
  src->priv->msg_queue_cv.notify_all();
  return TRUE;
}

/* basesrc asks us to return from create, e.g. on flushing seeks and state changes to READY */
static gboolean roscompressedimagesrc_unlock(GstBaseSrc * base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(base_src);
  GST_DEBUG_OBJECT(src, "unlock");
  std::unique_lock<std::mutex> lck(src->priv->msg_queue_mtx);
  src->priv->flushing = true;
  src->priv->msg_queue_cv.notify_all();
  return TRUE;
}

static gboolean roscompressedimagesrc_unlock_stop(GstBaseSrc * base_src)
{
  Roscompressedimagesrc * src = GST_ROSCOMPRESSEDIMAGESRC(base_src);
  GST_DEBUG_OBJECT(src, "unlock_stop");
  std::unique_lock<std::mutex> lck(src->priv->msg_queue_mtx);
  src->priv->flushing = false;
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
    std::unique_lock<std::mutex> lck(src->priv->msg_queue_mtx);
    src->priv->msg_queue_cv.wait(
      lck, [src] { return src->priv->flushing || !src->priv->msg_queue.empty(); });
    if (src->priv->flushing) {
      return GST_FLOW_FLUSHING;
    }
    msg = src->priv->msg_queue.front();
    src->priv->msg_queue.pop_front();
  }

  const std::string media_type = media_type_for(msg->format);
  if (media_type.empty()) {
    GST_ELEMENT_ERROR(
      src, STREAM, FORMAT, ("unsupported compressed image format '%s'", msg->format.c_str()),
      (NULL));
    return GST_FLOW_NOT_NEGOTIATED;
  }
  if (media_type != src->priv->media_type) {
    GstCaps * caps = gst_caps_new_empty_simple(media_type.c_str());
    gboolean ok = gst_base_src_set_caps(base_src, caps);
    gst_caps_unref(caps);
    if (!ok) {
      GST_ELEMENT_ERROR(
        src, CORE, NEGOTIATION, ("downstream does not accept %s", media_type.c_str()), (NULL));
      return GST_FLOW_NOT_NEGOTIATED;
    }
    src->priv->media_type = media_type;
  }

  *buf = gst_buffer_new_memdup(msg->data.data(), msg->data.size());

  GstClockTime msg_time = rclcpp::Time(msg->header.stamp).nanoseconds();
  // PTS per timestamp-mode, attaches the reference timestamp meta if enabled
  set_timestamps(buf, ros_base_src, GST_ELEMENT(src), msg_time);

  return GST_FLOW_OK;
}

static void roscompressedimagesrc_sub_cb(
  Roscompressedimagesrc * src, Roscompressedimagesrc::MsgType::ConstSharedPtr msg)
{
  std::unique_lock<std::mutex> lck(src->priv->msg_queue_mtx);
  src->priv->frame_id = msg->header.frame_id;
  src->priv->format = msg->format;
  src->priv->msg_queue.push_back(msg);
  while (src->priv->msg_queue.size() > msg_queue_max) {
    src->priv->msg_queue.pop_front();  // keep the latest
    GST_DEBUG_OBJECT(src, "dropping message");
  }
  src->priv->msg_queue_cv.notify_one();
}
