/**
 * CrazyradioDriver module auxiliary routines.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * January 13, 2024
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

  void CrazyradioDriverNode::reconnect_timer_clbk()
  {
    // Reconnect to the Crazyradio
    //
    RCLCPP_INFO(this->get_logger(), "Reconnecting to Crazyradio");
    RCLCPP_INFO(this->get_logger(), "is_connected: %d", crazyflie_drivers_[0].get()->isConnected());
    // crazyflie_driver_->reconnect();
  }

} // namespace CrazyradioDriver
