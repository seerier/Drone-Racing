/**
 * CrazyradioDriver headers.
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

#ifndef CRAZYDRIVER__CRAZYDRIVER_HPP
#define CRAZYDRIVER__CRAZYDRIVER_HPP

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <crazyradio_driver/crazyflie_driver.hpp>

#include <jirl_interfaces/msg/command_ctbr.hpp>

#define UNUSED(arg) (void)(arg)
#define LINE std::cout << __FUNCTION__ << ", LINE: " << __LINE__ << std::endl;

using namespace std::chrono_literals;
using namespace rcl_interfaces::msg;
using namespace jirl_interfaces::msg;

namespace CrazyradioDriver
{

/**
 * @brief CrazyradioDriver node class.
 */
class CrazyradioDriverNode : public rclcpp::Node
{
public:
  CrazyradioDriverNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~CrazyradioDriverNode();

private:
  /* Node init functions */
  void init_parameters();
  void init_crazyflie();
  void init_callback_groups();
  void init_subscribers();
  void init_timers();

  /* Callback groups */
  std::vector<rclcpp::CallbackGroup::SharedPtr> cmd_cgroup_vec_;
  rclcpp::CallbackGroup::SharedPtr reconnect_cgroup_;

  /* Subscribers */
  std::vector<rclcpp::Subscription<CommandCTBR>::SharedPtr> cmd_sub_array_;

  /* Timers */
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  /* Callbacks */
  void reconnect_timer_clbk();

  /* Utility routines */

  /* Node parameters */
  std::vector<std::string> crazyflie_names_;
  std::vector<std::string> crazyflie_uris_;
  int reconnection_period_ms_;

  std::vector<std::unique_ptr<CrazyflieDriver>> crazyflie_drivers_;
};

} // namespace CrazyradioDriver

#endif // CRAZYDRIVER__CRAZYDRIVER_HPP