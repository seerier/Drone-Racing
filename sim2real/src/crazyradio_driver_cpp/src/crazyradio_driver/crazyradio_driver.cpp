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
 * @brief CrazyradioDriver node constructor.
 *
 * @param node_opts Options for the base node.
 */
CrazyradioDriverNode::CrazyradioDriverNode(const rclcpp::NodeOptions & node_options)
: Node("crazyradio_driver", node_options)
{
  init_parameters();
  init_crazyflie();
  init_callback_groups();
  init_subscribers();
  init_timers();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief CrazyradioDriver node destructor.
 */
CrazyradioDriverNode::~CrazyradioDriverNode()
{
  RCLCPP_INFO(this->get_logger(), "Node destroyed");
}

/**
 * @brief Routine to initialize crazyflie libraries.
 */
void CrazyradioDriverNode::init_crazyflie()
{
  assert(crazyflie_names_.size() == crazyflie_uris_.size());

  for (size_t i = 0; i < crazyflie_names_.size(); i++) {
    std::string uri = crazyflie_uris_[i];
    std::string name = crazyflie_names_[i];
    RCLCPP_INFO(this->get_logger(), "Initializing Crazyflie %s with URI %s", name.c_str(), uri.c_str());

    auto cf_driver_ptr = std::make_unique<CrazyflieDriver>(uri);
    if (!cf_driver_ptr->connect()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Crazyflie '%s' (URI: %s). Skipping this drone.", name.c_str(), uri.c_str());
        // Pointer goes out of scope here, driver is automatically deleted. Nothing else needed.
        continue;
    }

    crazyflie_drivers_.push_back(std::move(cf_driver_ptr));
    // RCLCPP_INFO(this->get_logger(), "Connected to Crazyflie %s with URI %s", crazyflie_names_[i].c_str(), uri.c_str());
  }
}

/**
 * @brief Routine to initialize callback groups.
 */
void CrazyradioDriverNode::init_callback_groups()
{
  // Subscribers
  for (size_t i = 0; i < crazyflie_names_.size(); i++) {
    auto cmd_cgroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cmd_cgroup_vec_.push_back(cmd_cgroup);
  }

  // Timers
  reconnect_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize subscribers.
 */
void CrazyradioDriverNode::init_subscribers()
{
  // CTBR command
  for (size_t i = 0; i < crazyflie_names_.size(); i++) {
    std::string topic_name = crazyflie_names_[i] + "/ctbr_cmd";
    RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic_name.c_str());

    CrazyflieDriver* const driver_instance_ptr = crazyflie_drivers_[i].get();

    auto cmd_opts = rclcpp::SubscriptionOptions();
    cmd_opts.callback_group = cmd_cgroup_vec_[i];
    cmd_sub_array_.push_back(
      this->create_subscription<CommandCTBR>(
        topic_name,
        rclcpp::QoS(1).best_effort(),
        std::bind(
          &CrazyflieDriver::ctbr_clbk,
          driver_instance_ptr,
          std::placeholders::_1
        ),
        cmd_opts));
  }
}

/**
 * @brief Routine to initialize timers.
 */
void CrazyradioDriverNode::init_timers()
{
  // Reconnection
  reconnect_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(reconnection_period_ms_),
    std::bind(
      &CrazyradioDriverNode::reconnect_timer_clbk,
      this),
    reconnect_cgroup_);
}

} // namespace CrazyradioDriver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CrazyradioDriver::CrazyradioDriverNode)
