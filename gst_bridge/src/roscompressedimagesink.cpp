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
 * SECTION:element-gstroscompressedimagesink
 *
 * The roscompressedimagesink element pipes jpeg or png images into ROS2
 * as sensor_msgs/CompressedImage.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v videotestsrc ! jpegenc ! roscompressedimagesink ros-topic="/cam/image_raw" camera-info-url="file:///path/to/calibration.yaml"
 * ]|
 * Streams jpeg images on /cam/image_raw/compressed and the calibration on /cam/camera_info.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst_bridge/roscompressedimagesink.h>

GST_DEBUG_CATEGORY_STATIC(roscompressedimagesink_debug_category);
#define GST_CAT_DEFAULT roscompressedimagesink_debug_category

struct RoscompressedimagesinkPrivate
{
  std::string pub_topic = "gst_image_pub";  //base image topic, published on <pub_topic>/<transport>
  std::string transport = "compressed";     //image_transport style topic suffix, none if empty
  std::string frame_id = "image_frame";
  std::string format;  //compressed image format string override, deduced from the caps if blank
  std::string caps_format;  //format string deduced from the caps

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;

  gst_bridge::CameraInfoSettings camera_info;

  int height = 0;  //from the caps if known, only used to fill the camera info
  int width = 0;
};

/* prototypes */

static void roscompressedimagesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void roscompressedimagesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static void roscompressedimagesink_init(Roscompressedimagesink * sink);
static void roscompressedimagesink_finalize(GObject * object);

static gboolean roscompressedimagesink_open(RosBaseSink * sink);
static gboolean roscompressedimagesink_close(RosBaseSink * sink);
static gboolean roscompressedimagesink_setcaps(GstBaseSink * gst_base_sink, GstCaps * caps);
static GstFlowReturn roscompressedimagesink_render(
  RosBaseSink * base_sink, GstBuffer * buffer, rclcpp::Time msg_time);

enum {
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_TRANSPORT,
  PROP_ROS_FORMAT,
  PROP_CAMERA_INFO_URL,
  PROP_CAMERA_INFO_TOPIC,
  PROP_CAMERA_INFO_FRAME_ID,
};

/* pad templates */

static GstStaticPadTemplate roscompressedimagesink_sink_template = GST_STATIC_PAD_TEMPLATE(
  "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS("image/jpeg; image/png"));

/* class initialization */

G_DEFINE_TYPE_WITH_CODE(
  Roscompressedimagesink, roscompressedimagesink, GST_TYPE_ROS_BASE_SINK,
  GST_DEBUG_CATEGORY_INIT(
    roscompressedimagesink_debug_category, "roscompressedimagesink", 0,
    "debug category for roscompressedimagesink element"))

static void roscompressedimagesink_class_init(RoscompressedimagesinkClass * klass)
{
  GObjectClass * object_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSinkClass * basesink_class = GST_BASE_SINK_CLASS(klass);
  RosBaseSinkClass * ros_base_sink_class = GST_ROS_BASE_SINK_CLASS(klass);

  object_class->set_property = roscompressedimagesink_set_property;
  object_class->get_property = roscompressedimagesink_get_property;
  object_class->finalize = roscompressedimagesink_finalize;

  gst_element_class_add_static_pad_template(element_class, &roscompressedimagesink_sink_template);

  gst_element_class_set_static_metadata(
    element_class, "roscompressedimagesink", "Sink",
    "a gstreamer sink that publishes jpeg or png images into ROS", "Markus Hofstaetter");

  g_object_class_install_property(
    object_class, PROP_ROS_TOPIC,
    g_param_spec_string(
      "ros-topic", "pub-topic",
      "base image topic, the image is published on <ros-topic>/<ros-transport>", "gst_image_pub",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_TRANSPORT,
    g_param_spec_string(
      "ros-transport", "transport",
      "image_transport style suffix appended to ros-topic, publish on ros-topic directly if empty",
      "compressed", (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_FRAME_ID,
    g_param_spec_string(
      "ros-frame-id", "frame-id", "frame_id of the image message", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_FORMAT,
    g_param_spec_string(
      "ros-format", "format-string",
      "format string of the compressed image message, deduced from the caps (jpeg, png) if blank",
      "", (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_CAMERA_INFO_URL,
    g_param_spec_string(
      "camera-info-url", "camera-info-url",
      "calibration yaml/ini (file://, package:// url or a path), "
      "camera info is published with each image if set",
      "", (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_CAMERA_INFO_TOPIC,
    g_param_spec_string(
      "camera-info-topic", "camera-info-topic",
      "ROS topic for the camera info, defaults to camera_info next to ros-topic", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_CAMERA_INFO_FRAME_ID,
    g_param_spec_string(
      "camera-info-frame-id", "camera-info-frame-id",
      "frame_id of the camera info message used as is, defaults to <camera_name>_optical "
      "from the calibration, or ros-frame-id if the calibration names no camera",
      "", (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  basesink_class->set_caps = GST_DEBUG_FUNCPTR(roscompressedimagesink_setcaps);

  ros_base_sink_class->open = GST_DEBUG_FUNCPTR(roscompressedimagesink_open);
  ros_base_sink_class->close = GST_DEBUG_FUNCPTR(roscompressedimagesink_close);
  ros_base_sink_class->render = GST_DEBUG_FUNCPTR(roscompressedimagesink_render);
}

static void roscompressedimagesink_init(Roscompressedimagesink * sink)
{
  RosBaseSink * ros_base_sink GST_ROS_BASE_SINK(sink);
  g_free(ros_base_sink->node_name);  // set by the base class init
  ros_base_sink->node_name = g_strdup("gst_compressed_image_sink_node");
  sink->priv = new RoscompressedimagesinkPrivate();
}

static void roscompressedimagesink_finalize(GObject * object)
{
  Roscompressedimagesink * sink = GST_ROSCOMPRESSEDIMAGESINK(object);
  delete sink->priv;
  sink->priv = nullptr;
  G_OBJECT_CLASS(roscompressedimagesink_parent_class)->finalize(object);
}

void roscompressedimagesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec)
{
  RosBaseSink * ros_base_sink = GST_ROS_BASE_SINK(object);
  Roscompressedimagesink * sink = GST_ROSCOMPRESSEDIMAGESINK(object);

  GST_DEBUG_OBJECT(sink, "set_property");

  switch (property_id) {
    case PROP_ROS_TOPIC:
      if (sink->priv->pub) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(), "can't change topic name once opened");
      } else {
        sink->priv->pub_topic = gst_bridge::value_to_string(value);
      }
      break;

    case PROP_ROS_TRANSPORT:
      if (sink->priv->pub) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(), "can't change transport once opened");
      } else {
        sink->priv->transport = gst_bridge::value_to_string(value);
      }
      break;

    case PROP_ROS_FRAME_ID:
      sink->priv->frame_id = gst_bridge::value_to_string(value);
      break;

    case PROP_ROS_FORMAT:
      sink->priv->format = gst_bridge::value_to_string(value);
      break;

    case PROP_CAMERA_INFO_URL:
      sink->priv->camera_info.url = gst_bridge::value_to_string(value);
      if (sink->priv->camera_info.publisher) {
        sink->priv->camera_info.publisher->load(sink->priv->camera_info.url);
      } else if (sink->priv->pub && !sink->priv->camera_info.url.empty()) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(),
          "can't enable camera info once opened, set camera-info-url before starting");
      }
      break;

    case PROP_CAMERA_INFO_TOPIC:
      if (sink->priv->pub) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(),
          "can't change camera info topic once opened");
      } else {
        sink->priv->camera_info.topic = gst_bridge::value_to_string(value);
      }
      break;

    case PROP_CAMERA_INFO_FRAME_ID:
      sink->priv->camera_info.frame_id = gst_bridge::value_to_string(value);
      if (sink->priv->camera_info.publisher)
        sink->priv->camera_info.publisher->set_frame_id(sink->priv->camera_info.frame_id);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

void roscompressedimagesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec)
{
  Roscompressedimagesink * sink = GST_ROSCOMPRESSEDIMAGESINK(object);

  GST_DEBUG_OBJECT(sink, "get_property");
  switch (property_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, sink->priv->pub_topic.c_str());
      break;

    case PROP_ROS_TRANSPORT:
      g_value_set_string(value, sink->priv->transport.c_str());
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, sink->priv->frame_id.c_str());
      break;

    case PROP_ROS_FORMAT:
      g_value_set_string(value, sink->priv->format.c_str());
      break;

    case PROP_CAMERA_INFO_URL:
      g_value_set_string(value, sink->priv->camera_info.url.c_str());
      break;

    case PROP_CAMERA_INFO_TOPIC:
      g_value_set_string(value, sink->priv->camera_info.topic.c_str());
      break;

    case PROP_CAMERA_INFO_FRAME_ID:
      g_value_set_string(value, sink->priv->camera_info.frame_id.c_str());
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

/* open the device with given specs */
static gboolean roscompressedimagesink_open(RosBaseSink * ros_base_sink)
{
  Roscompressedimagesink * sink = GST_ROSCOMPRESSEDIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "open");
  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();  //XXX add a parameter for overrides

  // like image_transport, ros-topic is the base topic and the transport name is appended,
  // remapping (--ros-args -r) applies to the full <ros-topic>/<ros-transport> name
  const std::string topic = sink->priv->transport.empty()
                              ? sink->priv->pub_topic
                              : sink->priv->pub_topic + "/" + sink->priv->transport;
  const std::string info_topic = !sink->priv->camera_info.topic.empty()
                                   ? sink->priv->camera_info.topic
                                   : gst_bridge::camera_info_topic_for(sink->priv->pub_topic);

  // e.g. invalid topic names throw, fail the state change instead of aborting the process
  try {
    sink->priv->pub = rclcpp::create_publisher<sensor_msgs::msg::CompressedImage>(
      ros_base_sink->node_if->parameters, ros_base_sink->node_if->topics, topic, qos);

    if (!sink->priv->camera_info.url.empty()) {
      sink->priv->camera_info.publisher = std::make_shared<gst_bridge::CameraInfoPublisher>(
        ros_base_sink->node_if, info_topic, sink->priv->camera_info.url, qos);
      sink->priv->camera_info.publisher->set_frame_id(sink->priv->camera_info.frame_id);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_base_sink->node_if->logging->get_logger(), "can't publish on '%s': %s", topic.c_str(),
      e.what());
    sink->priv->camera_info.publisher.reset();
    sink->priv->pub.reset();
    return FALSE;
  }

  return TRUE;
}

/* close the device */
static gboolean roscompressedimagesink_close(RosBaseSink * ros_base_sink)
{
  Roscompressedimagesink * sink = GST_ROSCOMPRESSEDIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "close");
  sink->priv->camera_info.publisher.reset();
  sink->priv->pub.reset();
  return TRUE;
}

/* the compressed message has no size fields, width and height only fill the camera info */
static gboolean roscompressedimagesink_setcaps(GstBaseSink * gst_base_sink, GstCaps * caps)
{
  Roscompressedimagesink * sink = GST_ROSCOMPRESSEDIMAGESINK(gst_base_sink);
  GstStructure * caps_struct = gst_caps_get_structure(caps, 0);

  GST_DEBUG_OBJECT(sink, "setcaps");

  sink->priv->caps_format = gst_structure_has_name(caps_struct, "image/png") ? "png" : "jpeg";

  if (!gst_structure_get_int(caps_struct, "width", &sink->priv->width)) sink->priv->width = 0;
  if (!gst_structure_get_int(caps_struct, "height", &sink->priv->height)) sink->priv->height = 0;

  return TRUE;
}

static GstFlowReturn roscompressedimagesink_render(
  RosBaseSink * ros_base_sink, GstBuffer * buf, rclcpp::Time msg_time)
{
  GstMapInfo info;
  sensor_msgs::msg::CompressedImage msg;

  Roscompressedimagesink * sink = GST_ROSCOMPRESSEDIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "render");

  msg.header.stamp = msg_time;
  msg.header.frame_id = sink->priv->frame_id;
  msg.format = !sink->priv->format.empty() ? sink->priv->format : sink->priv->caps_format;

  if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
    GST_ERROR_OBJECT(sink, "Failed to map GstBuffer");
    return GST_FLOW_ERROR;
  }
  msg.data.assign(info.data, info.data + info.size);
  gst_buffer_unmap(buf, &info);

  sink->priv->pub->publish(msg);
  if (sink->priv->camera_info.publisher)
    sink->priv->camera_info.publisher->publish(msg.header, sink->priv->width, sink->priv->height);

  return GST_FLOW_OK;
}
