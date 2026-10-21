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

// helpers, type registration and properties of the elements (no ROS traffic)

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
