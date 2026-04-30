#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>        // For memcpy
#include <memory>         // For std::unique_ptr
#include <stdexcept>      // For standard exceptions
#include <cmath>          // For M_PI (potentially used in command logic)
#include <vector>         // Keep for potential future use or complex commands

#include "crazyflieLinkCpp/Connection.h"
#include "crazyflieLinkCpp/Packet.hpp" // Includes CRTP definitions

namespace bitcraze {
namespace crazyflieLinkCpp {

// --- Constants ---
constexpr uint8_t CRTP_PORT_COMMANDER = 3;
constexpr uint8_t CRTP_CH_COMMANDER_SETPOINT = 0;

// --- Data Structure ---
#pragma pack(push, 1)
struct crtpRateSetpoint { float rollRate, pitchRate, yawRate; uint16_t thrust; };
#pragma pack(pop)

// --- CrazyflieDriver Class ---

class CrazyflieDriver {
public:
    // Constructor: Takes the URI of the Crazyflie
    explicit CrazyflieDriver(const std::string& uri)
        : uri_(uri),
          is_connected_(false)
    {
        // Basic size check on construction
        static_assert(sizeof(crtpRateSetpoint) <= (CRTP_MAXSIZE - 1), "crtpRateSetpoint size exceeds maximum payload size!");
    }

    // Destructor: Ensures disconnection
    ~CrazyflieDriver() {
        disconnect(); // Automatically disconnect when the object is destroyed
    }

    // Prevent copying and assignment
    CrazyflieDriver(const CrazyflieDriver&) = delete;
    CrazyflieDriver& operator=(const CrazyflieDriver&) = delete;

    // Connect to the Crazyflie and send initial zero commands
    bool connect() {
        if (is_connected_) {
            std::cout << "Already connected to " << uri_ << std::endl;
            return true;
        }

        std::cout << "Connecting to " << uri_ << "..." << std::endl;
        try {
            connection_ = std::make_unique<Connection>(uri_);
            std::cout << "Connection object created. Waiting for stabilization..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << "Sending initial zero commands..." << std::endl;
            bool initial_zeros_ok = true;
            for (int i = 0; i < 10; ++i) { // Send a few initial zeros
                if (!sendRateCommandInternal(0.0f, 0.0f, 0.0f, 0)) {
                    initial_zeros_ok = false; // Logged internally
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (!initial_zeros_ok) {
                 std::cerr << "Warning: Failed to send some initial zero commands." << std::endl;
                 // Decide if this is critical - perhaps still proceed?
            } else {
                std::cout << "Initial zero commands sent." << std::endl;
            }

            is_connected_ = true;
            std::cout << "Successfully connected and initialized." << std::endl;
            return true;

        } catch (const std::exception& e) {
            std::cerr << "Connection failed: " << e.what() << std::endl;
            connection_.reset(); // Ensure unique_ptr is cleared
            is_connected_ = false;
            return false;
        } catch (...) {
            std::cerr << "Connection failed due to an unknown exception." << std::endl;
             connection_.reset();
            is_connected_ = false;
            return false;
        }
    }

    // Disconnect from the Crazyflie and send final zero commands
    void disconnect() {
        if (!is_connected_) {
            // Optional: std::cout << "Already disconnected." << std::endl;
            return;
        }

        std::cout << "Disconnecting from " << uri_ << "..." << std::endl;
        if (connection_) {
             std::cout << "Sending final zero commands..." << std::endl;
            for (int i = 0; i < 20; ++i) { // Send more final zeros
                sendRateCommandInternal(0.0f, 0.0f, 0.0f, 0); // Ignore failures here, best effort
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
        is_connected_ = false;
        std::cout << "Disconnected." << std::endl;
    }

    // Send a rate command (roll rate, pitch rate, yaw rate, thrust)
    // Returns true on success, false on failure (e.g., not connected, send error)
    bool sendRateCommand(float rollRate, float pitchRate, float yawRate, uint16_t thrust) {
        if (!is_connected_) {
            std::cerr << "Error: Cannot send command, not connected." << std::endl;
            return false;
        }
        return sendRateCommandInternal(rollRate, pitchRate, yawRate, thrust);
    }

    // Send a "stop" command (zero rates and thrust)
    // Returns true on success, false on failure
    bool sendStopCommand() {
        return sendRateCommand(0.0f, 0.0f, 0.0f, 0);
    }

    // Check if the driver is currently connected
    bool isConnected() const {
        return is_connected_;
    }

private:
    // Internal helper to send rate commands without the is_connected_ check
    // (used by connect/disconnect for initial/final zeros)
    bool sendRateCommandInternal(float rollRate, float pitchRate, float yawRate, uint16_t thrust) {
         if (!connection_) {
             // This check is mainly for the disconnect case where connection might be closing
             // but is_connected_ might not be false yet.
             return false;
         }

        crtpRateSetpoint rate_data;
        rate_data.rollRate = rollRate;
        rate_data.pitchRate = pitchRate;
        rate_data.yawRate = yawRate;
        rate_data.thrust = thrust;

        // Create and populate the packet
        Packet p;
        p.setPort(CRTP_PORT_COMMANDER);
        p.setChannel(CRTP_CH_COMMANDER_SETPOINT);
        p.setPayloadSize(sizeof(rate_data));
        std::memcpy(p.payload(), &rate_data, sizeof(rate_data));

        // Send the packet
        return sendPacket(p);
    }


    // Internal helper to send a packet and handle exceptions
    bool sendPacket(Packet& p) {
        // Assumes connection_ is valid and checked by caller if needed
        try {
            connection_->send(p);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error sending packet: " << e.what() << std::endl;
            // Consider if disconnection is needed on send failure
            // disconnect(); // Or maybe just return false and let caller decide
            return false;
        } catch (...) {
            std::cerr << "Unknown error sending packet." << std::endl;
            // disconnect();
            return false;
        }
    }

    std::string uri_;
    std::unique_ptr<Connection> connection_;
    bool is_connected_;
};


// --- Example Usage ---
int main() {
    std::string uri = "radio://0/93/2M/E7E7E70106"; // Your URI
    CrazyflieDriver cf_driver(uri);

    if (!cf_driver.connect()) {
        std::cerr << "Failed to connect to Crazyflie. Exiting." << std::endl;
        return 1;
    }

    // Control Loop
    std::cout << "Starting main control loop..." << std::endl;
    float controlFrequency = 100.0f; // Hz
    auto controlDt = std::chrono::milliseconds(static_cast<long long>(1000.0f / controlFrequency));
    auto nextControlSendTime = std::chrono::steady_clock::now();
    int loopDurationSeconds = 10;
    auto startTime = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - startTime < std::chrono::seconds(loopDurationSeconds)) {
        auto now = std::chrono::steady_clock::now();

        // Send Rate Commands (Timed)
        if (now >= nextControlSendTime) {
            if (!cf_driver.isConnected()) {
                std::cerr << "Connection lost during loop. Exiting." << std::endl;
                break; // Exit loop if connection drops
            }

            auto elapsed_time_s = std::chrono::duration<double>(now - startTime).count();

            // --- Calculate desired commands ---
            float targetRollRate = 0.0f;
            float targetPitchRate = 0.0f;
            float targetYawRate = 0.0f;
            uint16_t targetThrust = 0;

            if (elapsed_time_s > 5.0 && elapsed_time_s <= 8.0) {
                targetThrust = 0; // Stop thrust
            } else if (elapsed_time_s > 0.5 && elapsed_time_s <= 5.0) {
                targetThrust = 15000; // Example hover thrust - TUNE THIS!
            } else {
                targetThrust = 0; // Zero thrust otherwise
            }
            // --- End Calculation ---

            // Send the command using the driver
            if (!cf_driver.sendRateCommand(targetRollRate, targetPitchRate, targetYawRate, targetThrust)) {
                 std::cerr << "Failed to send command at t=" << elapsed_time_s << "s. Continuing..." << std::endl;
                 // Decide if you want to break or continue on send failure
            }

            nextControlSendTime += controlDt; // Schedule next send
        }

        // Efficient Sleep/Yield
        auto time_until_next_event = nextControlSendTime - std::chrono::steady_clock::now();
        if (time_until_next_event > std::chrono::milliseconds(1)) {
            std::this_thread::sleep_for(time_until_next_event);
        } else if (time_until_next_event > std::chrono::nanoseconds(0)) {
            std::this_thread::yield();
        }
        // If time_until_next_event <= 0, loop immediately
    }

    std::cout << "Control loop finished." << std::endl;

    // Disconnection is handled automatically by the destructor when cf_driver goes out of scope
    // Or you can explicitly call: cf_driver.disconnect();
    // The destructor will send final zero commands.

    std::cout << "Program finished successfully." << std::endl;
    return 0;
}


} // namespace crazyflieLinkCpp
} // namespace bitcraze

// --- Global main ---
int main() {
    try {
        return bitcraze::crazyflieLinkCpp::main();
    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception in global main: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unhandled unknown exception in global main." << std::endl;
        return 1;
    }
}