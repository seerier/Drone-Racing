/**
 * CrazyradioDriver standalone application.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * August 21, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <cstdlib>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <crazyradio_driver/crazyradio_driver.hpp>

using namespace CrazyradioDriver;

int main(int argc, char** argv)
{
  // Disable I/O buffering
  if (setvbuf(stdout, NULL, _IONBF, 0)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("crazyradio_driver_app"),
      "Failed to set I/O buffering");
      exit(EXIT_FAILURE);
  }

  // Create and initialize ROS 2 context
  rclcpp::init(argc, argv);

  // Initialize ROS 2 node
  auto ro_slam_node = std::make_shared<CrazyradioDriverNode>();

  // Create and configure executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(ro_slam_node);

  RCLCPP_WARN(
    rclcpp::get_logger("crazyradio_driver_app"),
    "(%d) " "crazyradio_driver_app" " online",
    getpid());

  // Spin the executor
  executor->spin();

  // Destroy ROS 2 node and context
  ro_slam_node.reset();
  rclcpp::shutdown();

  exit(EXIT_SUCCESS);
}
