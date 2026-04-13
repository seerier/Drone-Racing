#ifndef CRAZYFLIE_DRIVER_HPP
#define CRAZYFLIE_DRIVER_HPP

#include <string>
#include <memory>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

// Forward declare library classes if possible, otherwise include headers.
// Including headers is often simpler unless compile times are critical.
#include "crazyflieLinkCpp/Connection.h"
#include "crazyflieLinkCpp/Packet.hpp"

#include <jirl_interfaces/msg/command_ctbr.hpp>

using namespace bitcraze;
using namespace crazyflieLinkCpp;
using namespace jirl_interfaces::msg;

// --- Data Structure ---
// Keep the struct definition here as it's needed by the public interface indirectly
// Or move to a separate common types header if used elsewhere.
#pragma pack(push, 1)
struct crtpRateSetpoint { float rollRate, pitchRate, yawRate; uint16_t thrust; };
#pragma pack(pop)


// --- CrazyflieDriver Class Declaration ---

class CrazyflieDriver {
public:
  // Constructor: Takes the URI of the Crazyflie
  explicit CrazyflieDriver(const std::string& uri);

  // Destructor: Ensures disconnection
  ~CrazyflieDriver();

  // Prevent copying and assignment
  CrazyflieDriver(const CrazyflieDriver&) = delete;
  CrazyflieDriver& operator=(const CrazyflieDriver&) = delete;

  // Connect to the Crazyflie and send initial zero commands
  // Returns true on success, false on failure.
  bool connect();

  // Disconnect from the Crazyflie and send final zero commands
  void disconnect();

  // Send a rate command (roll rate, pitch rate, yaw rate, thrust)
  // Returns true on success, false on failure (e.g., not connected, send error)
  bool sendRateCommand(float rollRate, float pitchRate, float yawRate, uint16_t thrust);

  void ctbr_clbk(CommandCTBR::SharedPtr msg);

  // Send a "stop" command (zero rates and thrust)
  // Returns true on success, false on failure
  bool sendStopCommand();

  // Check if the driver is currently connected
  bool isConnected() const;

private:
  // Internal helper to send rate commands without the is_connected_ check
  bool sendRateCommandInternal(float rollRate, float pitchRate, float yawRate, uint16_t thrust);

  // Internal helper to send a packet and handle exceptions
  bool sendPacket(Packet& p);

  std::string uri_;
  std::unique_ptr<Connection> connection_; // Use unique_ptr for RAII
  bool is_connected_;

  // --- Constants specific to the implementation detail ---
  // Could be defined here or in the cpp file. Keeping them close to Packet usage.
  static constexpr uint8_t CRTP_PORT_COMMANDER_ = 3;
  static constexpr uint8_t CRTP_CH_COMMANDER_SETPOINT_ = 0;
};

#endif // CRAZYFLIE_DRIVER_HPP