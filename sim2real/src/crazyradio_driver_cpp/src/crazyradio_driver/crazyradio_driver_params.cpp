/**
 * CrazyradioDriver node initialization and parameters routines.
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

#include <crazyradio_driver/crazyradio_driver.hpp>

namespace CrazyradioDriver
{

/**
 * @brief Routine to initialize parameters.
 */
void CrazyradioDriverNode::init_parameters()
{
  // Declare parameters
  this->declare_parameter("crazyflie_names", rclcpp::ParameterValue(std::vector<std::string>()));
  this->declare_parameter("crazyflie_uris", rclcpp::ParameterValue(std::vector<std::string>()));
  this->declare_parameter("reconnection_period_ms", rclcpp::ParameterValue(int64_t(0)));

  // Get parameters
  crazyflie_names_ = this->get_parameter("crazyflie_names").as_string_array();
  crazyflie_uris_ = this->get_parameter("crazyflie_uris").as_string_array();
  reconnection_period_ms_ = this->get_parameter("reconnection_period_ms").as_int();

  // Print parameters
  RCLCPP_INFO(this->get_logger(), "crazyflie_name:");
  for (const auto & name : crazyflie_names_) {
    RCLCPP_INFO(this->get_logger(), "\t- %s", name.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "crazyflie_uris:");
  for (const auto & uri : crazyflie_uris_) {
    RCLCPP_INFO(this->get_logger(), "\t- %s", uri.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "reconnection_period_ms: %d", reconnection_period_ms_);
}

} // namespace CrazyradioDriver
