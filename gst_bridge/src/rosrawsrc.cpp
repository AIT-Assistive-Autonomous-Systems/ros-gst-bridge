/* gst_bridge
 * Copyright (C) 2021 Clyde McQueen <clyde@mcqueen.net>
 * Copyright (C) 2020-2021 Brett Downing <brettrd@brettrd.com>
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

#include <gst_bridge/rosrawsrc.h>

GST_DEBUG_CATEGORY_STATIC(rosrawsrc_debug_category);
#define GST_CAT_DEFAULT rosrawsrc_debug_category

static void rosrawsrc_set_property(
  GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec);
static void rosrawsrc_get_property(
  GObject * object, guint prop_id, GValue * value, GParamSpec * pspec);

static void rosrawsrc_init(Rosrawsrc * src);
static gboolean rosrawsrc_open(RosBaseSrc * ros_base_src);
static gboolean rosrawsrc_close(RosBaseSrc * ros_base_src);
static GstFlowReturn rosrawsrc_create(
  GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer ** buf);

static gboolean rosrawsrc_query(GstBaseSrc * base_src, GstQuery * query);
static void rosrawsrc_sub_cb(Rosrawsrc * src, std_msgs::msg::ByteMultiArray::ConstSharedPtr msg);
static std_msgs::msg::ByteMultiArray::ConstSharedPtr rosrawsrc_wait_for_msg(Rosrawsrc * src);

enum {
  PROP_0,
  PROP_SILENT,
  PROP_ROS_TOPIC,
  PROP_CAPS,
};

static GstStaticPadTemplate rosrawsrc_src_template = GST_STATIC_PAD_TEMPLATE(
  "src", GST_PAD_SRC, GST_PAD_ALWAYS,
  GST_STATIC_CAPS_ANY);  // Allow any caps, to be refined at runtime

G_DEFINE_TYPE_WITH_CODE(
  Rosrawsrc, rosrawsrc, GST_TYPE_ROS_BASE_SRC,
  GST_DEBUG_CATEGORY_INIT(rosrawsrc_debug_category, "rosrawsrc", 0, "debug category for rosrawsrc"))

static void rosrawsrc_class_init(RosrawsrcClass * klass)
{
  GObjectClass * object_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSrcClass * basesrc_class = GST_BASE_SRC_CLASS(klass);
  RosBaseSrcClass * ros_base_src_class = GST_ROS_BASE_SRC_CLASS(klass);

  object_class->set_property = rosrawsrc_set_property;
  object_class->get_property = rosrawsrc_get_property;

  gst_element_class_add_pad_template(
    element_class, gst_static_pad_template_get(&rosrawsrc_src_template));

  gst_element_class_set_static_metadata(
    element_class, "rosrawsrc", "Source/Binary",
    "A GStreamer source that receives ROS2 ByteMultiArray messages",
    "Guilherme Rodrigues <guilherme.rodrigues@ait.ac.at>");

  g_object_class_install_property(
    object_class, PROP_SILENT,
    g_param_spec_boolean("silent", "Silent", "Produce verbose output ?", FALSE, G_PARAM_READWRITE));

  g_object_class_install_property(
    object_class, PROP_ROS_TOPIC,
    g_param_spec_string(
      "topic", "Topic", "ROS topic to subscribe to", "raw",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_CAPS,
    g_param_spec_string(
      "caps", "Caps", "Output caps (e.g., application/octet-stream)",
      "application/octet-stream",  // default
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  ros_base_src_class->open = GST_DEBUG_FUNCPTR(rosrawsrc_open);
  ros_base_src_class->close = GST_DEBUG_FUNCPTR(rosrawsrc_close);
  basesrc_class->create = GST_DEBUG_FUNCPTR(rosrawsrc_create);

  basesrc_class->query = GST_DEBUG_FUNCPTR(rosrawsrc_query);  //set the scheduling modes
}

static void rosrawsrc_init(Rosrawsrc * src)
{
  RosBaseSrc * ros_base_src = GST_ROS_BASE_SRC(src);
  ros_base_src->node_name = g_strdup("gst_raw_src_node");

  src->silent = FALSE;
  src->sub_topic = g_strdup("raw");
  src->caps_string = g_strdup("application/octet-stream");  // Default caps

  src->msg_queue_max = 1;
  src->msg_queue = std::queue<std_msgs::msg::ByteMultiArray::ConstSharedPtr>();

  // Configure base src behavior
  gst_base_src_set_live(GST_BASE_SRC(src), TRUE);
  gst_base_src_set_format(GST_BASE_SRC(src), GST_FORMAT_TIME);
  gst_base_src_set_do_timestamp(GST_BASE_SRC(src), TRUE);

  // Set up dynamic caps pad
  GstCaps * caps = gst_caps_from_string(src->caps_string);
  src->srcpad = gst_pad_new_from_static_template(&rosrawsrc_src_template, "src");
  gst_pad_set_caps(src->srcpad, caps);
  gst_caps_unref(caps);
  gst_element_add_pad(GST_ELEMENT(src), src->srcpad);
}

static void rosrawsrc_set_property(
  GObject * object, guint prop_id, const GValue * value, GParamSpec * pspec)
{
  RosBaseSrc * ros_base_src = GST_ROS_BASE_SRC(object);
  Rosrawsrc * src = GST_ROSRAWSRC(object);

  switch (prop_id) {
    case PROP_SILENT:
      src->silent = g_value_get_boolean(value);
      break;

    case PROP_ROS_TOPIC:
      if (ros_base_src->node_if) {
        RCLCPP_ERROR(
          ros_base_src->node_if->logging->get_logger(), "can't change topic name once opened");
      } else {
        g_free(src->sub_topic);
        src->sub_topic = g_value_dup_string(value);
      }
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void rosrawsrc_get_property(
  GObject * object, guint prop_id, GValue * value, GParamSpec * pspec)
{
  Rosrawsrc * src = GST_ROSRAWSRC(object);

  switch (prop_id) {
    case PROP_SILENT:
      g_value_set_boolean(value, src->silent);
      break;

    case PROP_ROS_TOPIC:
      g_value_set_string(value, src->sub_topic);
      break;

    case PROP_CAPS:
      g_value_set_string(value, src->caps_string);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

/* open the subscription with given specs */
static gboolean rosrawsrc_open(RosBaseSrc * ros_base_src)
{
  Rosrawsrc * src = GST_ROSRAWSRC(ros_base_src);

  using std::placeholders::_1;

  GST_DEBUG_OBJECT(src, "open");

  // ROS can't cope with some forms of std::bind being passed as subscriber callbacks,
  // lambdas seem to be the preferred case for these instances
  auto cb = [src](std_msgs::msg::ByteMultiArray::ConstSharedPtr msg) {
    rosrawsrc_sub_cb(src, msg);
  };
  rclcpp::QoS qos = rclcpp::SensorDataQoS();  //XXX add a parameter for overrides

  src->sub = rclcpp::create_subscription<std_msgs::msg::ByteMultiArray>(
    ros_base_src->node_if->parameters, ros_base_src->node_if->topics, src->sub_topic, qos, cb);

  return TRUE;
}

static gboolean rosrawsrc_close(RosBaseSrc * ros_base_src)
{
  Rosrawsrc * src = GST_ROSRAWSRC(ros_base_src);
  src->sub.reset();
  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  while (!src->msg_queue.empty()) {
    src->msg_queue.pop();
  }

  return TRUE;
}

static gboolean rosrawsrc_query(GstBaseSrc * base_src, GstQuery * query)
{
  gboolean ret;

  switch (GST_QUERY_TYPE(query)) {
    case GST_QUERY_SCHEDULING: {
      /* a pushsrc can by default never operate in pull mode override
       * if you want something different. */
      gst_query_set_scheduling(query, GST_SCHEDULING_FLAG_SEQUENTIAL, 1, -1, 0);
      gst_query_add_scheduling_mode(query, GST_PAD_MODE_PUSH);

      ret = TRUE;
      break;
    }
    default:
      ret = GST_BASE_SRC_CLASS(rosrawsrc_parent_class)->query(base_src, query);
      break;
  }
  return ret;
}

/*
 * Wait for a message to be published, then load the contents into buf
 * Also update frame_id and encoding
 * Error if the number of channels or encoding changes at runtime
 */
static GstFlowReturn rosrawsrc_create(
  GstBaseSrc * base_src, guint64 offset, guint size, GstBuffer ** buf)
{
  RosBaseSrc * ros_base_src = GST_ROS_BASE_SRC(base_src);
  Rosrawsrc * src = GST_ROSRAWSRC(base_src);

  GstMapInfo info;
  GstClockTimeDiff base_time;
  size_t length;
  GstFlowReturn ret = GST_FLOW_OK;
  GstBuffer * res_buf;

  GST_DEBUG_OBJECT(src, "create");

  if (!ros_base_src->node_if) {
    GST_DEBUG_OBJECT(src, "ros raw src creating buffer before node init");
  } else if (false /* src->msg_init */) {
    GST_DEBUG_OBJECT(src, "ros raw src creating buffer before receiving first message");
  }

  auto msg = rosrawsrc_wait_for_msg(src);
  {  //scope the mutex lock
    std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
    src->msg_queue.pop();  // XXX we can stop dropping the first message during preroll now
  }

  // XXX check message contains anything

  length = msg->data.size();
  if (*buf == NULL) {
    /* downstream did not provide us with a buffer to fill, allocate one
     * ourselves
     * XXX pass the vector memory on directly */
    ret = GST_BASE_SRC_CLASS(rosrawsrc_parent_class)->alloc(base_src, offset, length, &res_buf);
    if (G_UNLIKELY(ret != GST_FLOW_OK)) {
      GST_DEBUG_OBJECT(src, "Failed to allocate buffer of %lu bytes", length);
    }
    *buf = res_buf;
    size = length;
  } else {
    /* downstream provided a buffer to fill
     * XXX pass the buffer to the ros subscription allocator */
    res_buf = *buf;
  }

  if (length != size) GST_DEBUG_OBJECT(src, "size mismatch, %ld, %d", length, size);

  // XXX check the buffer exists, and check info.size > length
  gst_buffer_map(*buf, &info, GST_MAP_READ);
  info.size = length;
  memcpy(info.data, msg->data.data(), length);
  gst_buffer_unmap(*buf, &info);

  base_time = gst_element_get_base_time(GST_ELEMENT(src));
  // String message does not have a header, use node->now() TODO call now() in the cb and save in the queue
  // GST_BUFFER_PTS (*buf) = rclcpp::Time(msg->header.stamp).nanoseconds() - ros_base_src->ros_clock_offset - base_time;
  GST_BUFFER_PTS(*buf) = ros_base_src->node_if->clock->get_clock()->now().nanoseconds() -
                         ros_base_src->ros_clock_offset - base_time;

  // TODO explore configurable message types
  //GST_BUFFER_DURATION (*buf) = GST_CLOCK_TIME_NONE;
  //GST_BUFFER_DURATION (*buf) = 0;
  GST_BUFFER_DURATION(*buf) = 1000000000L;

  GST_DEBUG_OBJECT(
    src, "Sending %lu bytes from ByteMultiArray, %" GST_TIME_FORMAT " + %" GST_TIME_FORMAT, length,
    GST_TIME_ARGS(GST_BUFFER_PTS(*buf)), GST_TIME_ARGS(GST_BUFFER_DURATION(*buf)));

  return ret;
}

static void rosrawsrc_sub_cb(Rosrawsrc * src, std_msgs::msg::ByteMultiArray::ConstSharedPtr msg)
{
  RosBaseSrc * ros_base_src = GST_ROS_BASE_SRC(src);
  GST_DEBUG_OBJECT(src, "ros cb called");
  RCLCPP_DEBUG(ros_base_src->node_if->logging->get_logger(), "ros cb called");

  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  src->msg_queue.push(msg);
  while (src->msg_queue.size() > src->msg_queue_max) {
    src->msg_queue.pop();
    RCLCPP_WARN(ros_base_src->node_if->logging->get_logger(), "dropping message");
  }
  src->msg_queue_cv.notify_one();
}

static std_msgs::msg::ByteMultiArray::ConstSharedPtr rosrawsrc_wait_for_msg(Rosrawsrc * src)
{
  //RosBaseSrc *ros_base_src = GST_ROS_BASE_SRC (src);

  std::unique_lock<std::mutex> lck(src->msg_queue_mtx);
  while (src->msg_queue.empty()) {
    src->msg_queue_cv.wait(lck);
  }
  auto msg = src->msg_queue.front();

  return msg;
}
