#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>   // For memcpy
#include <stdexcept> // For standard exceptions

// Include library headers needed for implementation details
#include "crazyflieLinkCpp/Connection.h"
#include "crazyflieLinkCpp/Packet.hpp"

#include <crazyradio_driver/crazyflie_driver.hpp>
#include <jirl_interfaces/msg/command_ctbr.hpp>

// namespace bitcraze {
// namespace crazyflieLinkCpp {

using namespace bitcraze;
using namespace crazyflieLinkCpp;

// --- Constant Definitions (if not in header) ---
// constexpr uint8_t CrazyflieDriver::CRTP_PORT_COMMANDER_; // Definition needed if static constexpr in class
// constexpr uint8_t CrazyflieDriver::CRTP_CH_COMMANDER_SETPOINT_;

// --- Constructor ---
CrazyflieDriver::CrazyflieDriver(const std::string& uri)
  : uri_(uri),
    connection_(nullptr), // Initialize unique_ptr to nullptr
    is_connected_(false)
{
  // Runtime size check (could also use static_assert in header if CRTP_MAXSIZE is constexpr)
  if (sizeof(crtpRateSetpoint) > (CRTP_MAXSIZE - 1)) {
    // Throwing an exception might be better than just printing an error
    throw std::length_error("crtpRateSetpoint size exceeds maximum CRTP payload size!");
  }
  std::cout << "CrazyflieDriver initialized for URI: " << uri_ << std::endl;
}

// --- Destructor ---
CrazyflieDriver::~CrazyflieDriver() {
  std::cout << "CrazyflieDriver destructor called for " << uri_ << "." << std::endl;
  disconnect(); // Ensure cleanup happens
}

// --- connect() Method ---
bool CrazyflieDriver::connect() {
  if (is_connected_) {
    std::cout << "Already connected to " << uri_ << std::endl;
    return true;
  }

  std::cout << "Connecting to " << uri_ << "..." << std::endl;
  try {
    connection_ = std::make_unique<Connection>(uri_);
    std::cout << "Connection object created. Waiting for stabilization (1s)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "Sending initial zero commands..." << std::endl;
    bool initial_zeros_ok = true;
    for (int i = 0; i < 10; ++i) {
      if (!sendRateCommandInternal(0.0f, 0.0f, 0.0f, 0)) {
        initial_zeros_ok = false; // Error logged internally
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!initial_zeros_ok) {
      std::cerr << "Warning: Failed to send some initial zero commands." << std::endl;
      // Continue connection attempt despite warnings? Or return false? Let's continue for now.
    } else {
      std::cout << "Initial zero commands sent." << std::endl;
    }

    is_connected_ = true;
    std::cout << "Successfully connected and initialized." << std::endl;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "Connection failed: " << e.what() << std::endl;
    connection_.reset(); // Ensure unique_ptr is null
    is_connected_ = false;
    return false;
  } catch (...) {
    std::cerr << "Connection failed due to an unknown exception." << std::endl;
    connection_.reset();
    is_connected_ = false;
    return false;
  }
}

// --- disconnect() Method ---
void CrazyflieDriver::disconnect() {
  if (!is_connected_) {
    return; // Nothing to do if not connected
  }

  std::cout << "Disconnecting from " << uri_ << "..." << std::endl;
  if (connection_) { // Check if connection object exists
    std::cout << "Sending final zero commands..." << std::endl;
    for (int i = 0; i < 20; ++i) {
      // Best effort: Ignore return value, errors logged internally
      sendRateCommandInternal(0.0f, 0.0f, 0.0f, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Final zero commands sent." << std::endl;
    try {
      connection_->close();
      std::cout << "Connection closed." << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Exception during connection close: " << e.what() << std::endl;
    } catch(...) {
      std::cerr << "Unknown exception during connection close." << std::endl;
    }
    connection_.reset(); // Release the connection object
  }
  is_connected_ = false; // Mark as disconnected
  std::cout << "Disconnected." << std::endl;
}

// --- sendRateCommand() Method ---
bool CrazyflieDriver::sendRateCommand(float rollRate, float pitchRate, float yawRate, uint16_t thrust) {
  if (!is_connected_) {
    // Avoid spamming errors if called rapidly while disconnected
    // Consider logging level or throttling if this becomes an issue
    // static bool disconnected_error_logged = false;
    // if (!disconnected_error_logged) {
    //    std::cerr << "Error: Cannot send command, not connected." << std::endl;
    //    disconnected_error_logged = true;
    // }
    std::cerr << "Error: Cannot send command, not connected." << std::endl;
    return false;
  }
  // If connected, reset the hypothetical disconnected error flag
  // disconnected_error_logged = false;
  return sendRateCommandInternal(rollRate, pitchRate, yawRate, thrust);
}

void CrazyflieDriver::ctbr_clbk(CommandCTBR::SharedPtr msg) {
  // Convert the message to the appropriate types
  float rollRate = static_cast<float>(msg->roll_rate);
  float pitchRate = static_cast<float>(-msg->pitch_rate);
  float yawRate = static_cast<float>(-msg->yaw_rate);
  uint16_t thrust = static_cast<uint16_t>(msg->thrust_pwm);

  // Call the existing sendRateCommand method
  sendRateCommand(rollRate, pitchRate, yawRate, thrust);
}

// --- sendStopCommand() Method ---
bool CrazyflieDriver::sendStopCommand() {
  // Still need connection check, done by sendRateCommand
  return sendRateCommand(0.0f, 0.0f, 0.0f, 0);
}

// --- isConnected() Method ---
bool CrazyflieDriver::isConnected() const {
  // Could add an extra check: return is_connected_ && (connection_ != nullptr);
  return is_connected_;
}

// --- sendRateCommandInternal() Private Method ---
bool CrazyflieDriver::sendRateCommandInternal(float rollRate, float pitchRate, float yawRate, uint16_t thrust) {
  if (!connection_) {
    // This check is crucial for the disconnect() method where is_connected_ might still be true
    // briefly while connection_ is being reset.
    std::cerr << "Internal Error: Attempted to send command with null connection." << std::endl;
    return false;
  }

  crtpRateSetpoint rate_data;
  rate_data.rollRate = rollRate;
  rate_data.pitchRate = pitchRate;
  rate_data.yawRate = yawRate;
  rate_data.thrust = thrust;

  Packet p;
  p.setPort(CRTP_PORT_COMMANDER_); // Use the class constant
  p.setChannel(CRTP_CH_COMMANDER_SETPOINT_); // Use the class constant
  p.setPayloadSize(sizeof(rate_data));
  std::memcpy(p.payload(), &rate_data, sizeof(rate_data));

  return sendPacket(p); // Delegate sending to the helper
}

// --- sendPacket() Private Method ---
bool CrazyflieDriver::sendPacket(Packet& p) {
  // Assumes connection_ is valid (checked by callers like sendRateCommandInternal)
  try {
    connection_->send(p);
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error sending packet: " << e.what() << std::endl;
    // Consider more drastic action? e.g., set is_connected_ = false;
    return false;
  } catch (...) {
    std::cerr << "Unknown error sending packet." << std::endl;
    // Consider more drastic action? e.g., set is_connected_ = false;
    return false;
  }
}


// } // namespace crazyflieLinkCpp
// } // namespace bitcraze