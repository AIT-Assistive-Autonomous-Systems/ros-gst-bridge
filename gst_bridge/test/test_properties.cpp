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

// helpers and element properties (no ROS traffic)

#include <gst_bridge/camera_info.h>
#include <gst_bridge/gst_bridge.h>

#include "test_utils.hpp"

using namespace gst_bridge_test;

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
