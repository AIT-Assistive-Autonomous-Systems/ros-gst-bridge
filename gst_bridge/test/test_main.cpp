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

// main of the gst_bridge element tests: initialises GStreamer and ROS, spins the test node

#include <gst/gst.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "test_utils.hpp"

namespace
{
rclcpp::Node::SharedPtr node;
}  // namespace

rclcpp::Node::SharedPtr gst_bridge_test::test_node() { return node; }

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("gst_bridge_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });

  int result = RUN_ALL_TESTS();

  executor.cancel();
  spin.join();
  node.reset();
  rclcpp::shutdown();
  return result;
}
