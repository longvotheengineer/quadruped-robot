/**
 * @file change_servo_id.cpp
 * @brief Change the ID of an ST3215 servo.
 *
 * IMPORTANT: Only ONE servo must be connected when running this tool.
 *
 * Usage:
 *   ./change_servo_id /dev/ttyACM0 <old_id> <new_id>
 *   e.g:  ./change_servo_id /dev/ttyACM0 1 2
 */

#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>
#include "SCServo.h"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0] << " <port> <old_id> <new_id>" << std::endl;
        std::cout << "  e.g: " << argv[0] << " /dev/ttyACM0 1 2" << std::endl;
        std::cout << std::endl;
        std::cout << "  WARNING: Only ONE servo must be connected!" << std::endl;
        return 1;
    }

    const char* port = argv[1];
    int old_id = std::atoi(argv[2]);
    int new_id = std::atoi(argv[3]);

    if (new_id < 1 || new_id > 253) {
        std::cout << "ERROR: new_id must be 1-253." << std::endl;
        return 1;
    }

    SMS_STS servo;
    if (!servo.begin(1000000, port)) {
        std::cout << "FAILED to open " << port << std::endl;
        return 1;
    }

    // Verify old ID responds
    if (servo.Ping(old_id) == -1) {
        std::cout << "Servo ID " << old_id << " did NOT respond." << std::endl;
        std::cout << "Is the servo connected? Is the ID correct?" << std::endl;
        servo.end();
        return 1;
    }
    std::cout << "OK: Servo ID " << old_id << " is alive." << std::endl;

    // Unlock EEPROM
    std::cout << "Unlocking EEPROM..." << std::endl;
    servo.unLockEprom(old_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Write new ID
    std::cout << "Writing new ID: " << old_id << " -> " << new_id << std::endl;
    servo.writeByte(old_id, SMS_STS_ID, new_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Lock EEPROM
    std::cout << "Locking EEPROM..." << std::endl;
    servo.LockEprom(new_id);  // Must address with NEW id now
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Verify new ID
    if (servo.Ping(new_id) != -1) {
        std::cout << std::endl;
        std::cout << "SUCCESS: Servo now responds as ID " << new_id << std::endl;
    } else {
        std::cout << std::endl;
        std::cout << "WARNING: Servo did not respond at new ID " << new_id << std::endl;
        std::cout << "Try power-cycling the servo and ping again." << std::endl;
    }

    // Verify old ID no longer responds
    if (servo.Ping(old_id) == -1) {
        std::cout << "CONFIRMED: ID " << old_id << " no longer responds." << std::endl;
    }

    servo.end();
    return 0;
}
