#include <iostream>
#include <vector>
#include <string>
#include <list>       // For std::list (required by LogBlock constructor)
#include <utility>    // For std::pair (required by LogBlock constructor)
#include <thread>
#include <chrono>
#include <functional> // For std::function
#include <memory>     // For std::unique_ptr
#include <cmath>      // For M_PI, sin, cos
#include <iomanip>    // For std::fixed, std::setprecision
#include <atomic>

// Include the crazyflie_cpp header
// Ensure this path is correct relative to your build setup
#include "crazyflie_cpp/Crazyflie.h"

// --- Struct to hold the desired log data ---
struct LogDataStruct {
    float roll;
    float pitch;
    float yaw;
    float vbat;
    uint8_t pmState;
};

// --- Global Variables ---
LogDataStruct latestLogData;
std::atomic<bool> logDataReceived(false);

// --- Callback function for Log Data ---
// Signature matches the LogBlock constructor in the provided Crazyflie.h
// Takes a non-const pointer LogDataStruct*
void onLogDataReceived(uint32_t time_in_ms, LogDataStruct* data) {
    latestLogData = *data; // Copy data
    logDataReceived = true;

    // Print received data
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "LOG T=" << time_in_ms << "ms -> "
              << "State(r,p,y): " << data->roll << ", " << data->pitch << ", " << data->yaw
              << " | Batt(V,St): " << data->vbat << ", " << (int)data->pmState
              << std::endl;
}

// --- Main Function ---
int main() {
    std::string uri = "radio://0/125/2M/E7E7E70102"; // Your drone's URI
    std::unique_ptr<LogBlock<LogDataStruct>> logBlock; // Manages the log block

    try {
        // --- 1. Connection ---
        std::cout << "Connecting to Crazyflie at: " << uri << std::endl;
        Crazyflie cf(uri); // Handles connection
        std::cout << "Connection established." << std::endl;

        // --- 2. Logging Setup ---
        std::cout << "Requesting Log Table of Contents (TOC)..." << std::endl;
        // Call requestLogToc (returns void as per header)
        cf.requestLogToc();
        // Add delay since we can't use .get()
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Log TOC hopefully received." << std::endl;

        // Define variables using std::list<std::pair> as required by header
        std::list<std::pair<std::string, std::string>> logVariables = {
            {"stabilizer", "roll"},
            {"stabilizer", "pitch"},
            {"stabilizer", "yaw"},
            {"pm", "vbat"},
            {"pm", "state"},
        };

        // Define the callback function object with the matching signature (non-const ptr)
        std::function<void(uint32_t, LogDataStruct*)> logCallback = onLogDataReceived;

        // Create the LogBlock - passes the non-const lvalue reference callback
        logBlock.reset(new LogBlock<LogDataStruct>(&cf, logVariables, logCallback));

        // Start the logging
        int logFrequencyHz = 20;
        // Calculate period in 10ms units (e.g., 20Hz -> 50ms -> period 5)
        uint8_t logPeriod_10ms = static_cast<uint8_t>(100 / logFrequencyHz);
        logBlock->start(logPeriod_10ms);
        std::cout << "Log start command sent for requested variables at " << logFrequencyHz << " Hz (Period: " << (int)logPeriod_10ms <<")." << std::endl;


        // --- 3. Initial Zero Commands ---
        std::cout << "Sending initial zero thrust commands..." << std::endl;
        for (int i = 0; i < 10; ++i) {
            cf.sendSetpoint(0.0f, 0.0f, 0.0f, 0); // RollRate, PitchRate, YawRate, Thrust
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "Initial zero commands sent." << std::endl;

        // --- 4. Control Loop ---
        std::cout << "Starting main control loop..." << std::endl;

        // Frequencies and timing (using nanoseconds for extPosDt)
        float extPosFrequency = 50.0f;
        long long extPosPeriod_ns = static_cast<long long>((1.0 / extPosFrequency) * 1e9);
        auto extPosDt = std::chrono::nanoseconds(extPosPeriod_ns);
        auto nextExtPosSendTime = std::chrono::steady_clock::now();

        float controlFrequency = 100.0f;
        auto controlDt = std::chrono::milliseconds(static_cast<long long>(1000.0f / controlFrequency));
        auto nextControlSendTime = std::chrono::steady_clock::now();

        int loopDurationSeconds = 10; // Duration for the timed sequence
        auto startTime = std::chrono::steady_clock::now(); // Start timer AFTER setup

        // External position simulation variables
        float current_x = 0.0f, current_y = 0.0f, current_z = 0.5f;
        float radius = 0.5f, angular_velocity = 2.0f * M_PI / 10.0f;

        // --- Main While Loop ---
        while (std::chrono::steady_clock::now() - startTime < std::chrono::seconds(loopDurationSeconds)) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_time_s = std::chrono::duration<double>(now - startTime).count();

            // --- Process Incoming Packets ---
            // ** IMPORTANT: Need to process packets for callbacks to fire **
            // Call a function like processAllPackets if it exists, otherwise hope
            // that send/receive calls internally process the queue. If logs don't
            // print, lack of packet processing is a likely cause *after* linking is fixed.
            // cf.processAllPackets(); // Uncomment if this function exists in your Crazyflie.h

            // --- Send External Position Data periodically ---
            if (now >= nextExtPosSendTime) {
                // ** REPLACE simulation with actual data **
                current_x = radius * std::cos(angular_velocity * elapsed_time_s);
                current_y = radius * std::sin(angular_velocity * elapsed_time_s);

                // Use the correct function name from header
                cf.sendExternalPositionUpdate(current_x, current_y, current_z);

                nextExtPosSendTime += extPosDt; // Use nanoseconds duration
            }

            // --- Send Control Commands (Rates/Thrust) periodically with TIMED SEQUENCE ---
             if (now >= nextControlSendTime) {
                float rollRate_cmd = 0.0f;
                float pitchRate_cmd = 0.0f;
                float yawRate_cmd = 0.0f;
                uint16_t thrust_cmd = 0;

                // Timed thrust sequence: 0 -> 10k (5s) -> 0 (3s) -> 0
                if (elapsed_time_s > 5.0 && elapsed_time_s <= 8.0) { thrust_cmd = 0; } // Phase 3
                else if (elapsed_time_s > 0.0 && elapsed_time_s <= 5.0) { thrust_cmd = 10000; } // Phase 2
                else { thrust_cmd = 0; } // Phase 1 & 4

                cf.sendSetpoint(rollRate_cmd, pitchRate_cmd, yawRate_cmd, thrust_cmd);

                nextControlSendTime += controlDt;
            }

            // --- Yield/Sleep ---
            auto next_event_time = std::min(nextExtPosSendTime, nextControlSendTime);
            auto sleep_duration = next_event_time - std::chrono::steady_clock::now();
             if (sleep_duration > std::chrono::milliseconds(1)) {
                 std::this_thread::sleep_for(sleep_duration);
            } else if (sleep_duration > std::chrono::nanoseconds(0)){
                 std::this_thread::yield(); // Yield if next event is very soon
            }
        } // End of while loop

        // --- 5. Cleanup ---
        std::cout << "Control loop finished." << std::endl;

        // Send final zero thrust command multiple times for safety
        std::cout << "Sending final zero thrust commands..." << std::endl;
        for (int i = 0; i < 10; ++i) {
            cf.sendSetpoint(0.0f, 0.0f, 0.0f, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cout << "Final zero commands sent." << std::endl;

        // LogBlock stops automatically when logBlock unique_ptr goes out of scope
        std::cout << "Stopping logging (via LogBlock destructor)..." << std::endl;
        // logBlock->stop(); // Can call explicitly if preferred

        // Connection closes automatically when cf goes out of scope (RAII)
        std::cout << "Disconnecting (via Crazyflie destructor)..." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        // Destructors should still handle cleanup
        return 1;
    } catch (...) {
        std::cerr << "An unknown error occurred." << std::endl;
        return 1;
    }

    std::cout << "Program finished successfully." << std::endl;
    return 0;
}