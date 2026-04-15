/*
 * Phase 0 — ST3215 Servo Verification
 * ====================================
 * Build:
 *   g++ -std=c++11 -O2 -I<SCLIB> -o test_servo test_servo_phase0.cpp <SCLIB>/build/libSCServo.a
 * Run:
 *   ./test_servo /dev/ttyACM0 1
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include "SCServo.h"

static const char* mode_name(int mode) {
    switch (mode) {
        case 0: return "Position Servo (CORRECT)";
        case 1: return "Motor Speed    (WRONG - will spin!)";
        case 2: return "PWM Open-Loop  (WRONG)";
        case 3: return "Step Servo     (multi-turn)";
        default: return "UNKNOWN";
    }
}

static const char* baud_name(int code) {
    switch (code) {
        case 0: return "1,000,000 bps";
        case 1: return "500,000 bps";
        case 2: return "250,000 bps";
        case 3: return "128,000 bps";
        case 4: return "115,200 bps";
        default: return "UNKNOWN";
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <port> <servo_id>" << std::endl;
        std::cout << "  e.g: " << argv[0] << " /dev/ttyACM0 1"   << std::endl;
        return 1;
    }

    const char* port = argv[1];
    int servo_id     = std::atoi(argv[2]);

    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  ST3215 Servo Verification - Phase 0" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  Port:     " << port     << std::endl;
    std::cout << "  Servo ID: " << servo_id << std::endl;
    std::cout << std::endl;

    // -- Open serial port at 1 Mbps --
    SMS_STS servo;
    bool opened_1m = servo.begin(1000000, port);

    if (!opened_1m) {
        std::cout << "  x FAILED to open " << port << " at 1 Mbps" << std::endl;
        std::cout << "  Trying 115200 baud..." << std::endl;

        if (!servo.begin(115200, port)) {
            std::cout << "  x FAILED at 115200 too. Check:" << std::endl;
            std::cout << "    - USB cable connected?" << std::endl;
            std::cout << "    - Jumper set to 'B'?" << std::endl;
            std::cout << "    - sudo usermod -a -G dialout $USER" << std::endl;
            return 1;
        }
        std::cout << "  OK: Opened at 115200 baud" << std::endl;
    } else {
        std::cout << "  OK: Serial port opened at 1,000,000 bps" << std::endl;
    }

    // -- Ping --
    std::cout << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "  PING TEST" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    int ping_id = servo.Ping(servo_id);
    if (ping_id != -1) {
        std::cout << "  OK: Servo ID " << servo_id
                  << " is ALIVE (returned ID=" << ping_id << ")" << std::endl;
    } else {
        std::cout << "  FAIL: Servo ID " << servo_id << " did NOT respond." << std::endl;
        std::cout << "    - Is 12V power connected to the adapter?"     << std::endl;
        std::cout << "    - Is the servo cable plugged in (3-pin)?"     << std::endl;
        std::cout << "    - Is the servo ID actually " << servo_id << "?" << std::endl;

        // Try scanning 0-20
        std::cout << std::endl;
        std::cout << "  AUTO-SCANNING IDs 0..20:" << std::endl;
        for (int i = 0; i <= 20; i++) {
            int r = servo.Ping(i);
            if (r != -1) {
                std::cout << "    FOUND: ID " << i << " responded!" << std::endl;
            }
        }
        servo.end();
        return 1;
    }

    // -- Read EEPROM --
    std::cout << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  EEPROM CONFIGURATION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    int fw_ver     = servo.readWord(servo_id, 0);
    int id_read    = servo.readByte(servo_id, SMS_STS_ID);
    int baud_code  = servo.readByte(servo_id, SMS_STS_BAUD_RATE);
    int min_angle  = servo.readWord(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L);
    int max_angle  = servo.readWord(servo_id, SMS_STS_MAX_ANGLE_LIMIT_L);
    int max_temp   = servo.readByte(servo_id, 13);
    int max_volt   = servo.readByte(servo_id, 14);
    int min_volt   = servo.readByte(servo_id, 15);
    int max_torque = servo.readWord(servo_id, 16);
    int p_gain     = servo.readByte(servo_id, 21);
    int d_gain     = servo.readByte(servo_id, 22);
    int i_gain     = servo.readByte(servo_id, 23);
    int mode       = servo.readByte(servo_id, SMS_STS_MODE);
    int lock       = servo.readByte(servo_id, SMS_STS_LOCK);

    std::cout << std::endl;
    std::cout << "  Firmware Version  : " << fw_ver    << std::endl;
    std::cout << "  Servo ID          : " << id_read   << std::endl;
    std::cout << "  Baud Rate         : " << baud_code
              << " -> " << baud_name(baud_code)        << std::endl;
    std::cout << "  Min Angle Limit   : " << min_angle
              << " (" << std::fixed << std::setprecision(1)
              << (min_angle * 360.0 / 4096) << " deg)" << std::endl;
    std::cout << "  Max Angle Limit   : " << max_angle
              << " (" << std::fixed << std::setprecision(1)
              << (max_angle * 360.0 / 4096) << " deg)" << std::endl;
    std::cout << "  Max Temperature   : " << max_temp  << " C" << std::endl;
    std::cout << "  Max Voltage       : " << std::fixed << std::setprecision(1)
              << (max_volt * 0.1) << " V"              << std::endl;
    std::cout << "  Min Voltage       : " << std::fixed << std::setprecision(1)
              << (min_volt * 0.1) << " V"              << std::endl;
    std::cout << "  Max Torque        : " << max_torque << std::endl;
    std::cout << "  PID Gains (P/D/I) : " << p_gain << " / "
              << d_gain << " / " << i_gain             << std::endl;
    std::cout << "  Operating Mode    : " << mode
              << " -> " << mode_name(mode)              << std::endl;
    std::cout << "  EEPROM Lock       : " << lock
              << " -> " << (lock ? "LOCKED" : "UNLOCKED") << std::endl;

    // -- Read live status (SRAM) --
    std::cout << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "  LIVE STATUS (SRAM)" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    int pos   = servo.ReadPos(servo_id);
    int speed = servo.ReadSpeed(servo_id);
    int load  = servo.ReadLoad(servo_id);
    int volt  = servo.ReadVoltage(servo_id);
    int temp  = servo.ReadTemper(servo_id);

    std::cout << "  Position    : " << pos << " ticks ("
              << std::fixed << std::setprecision(1)
              << (pos * 360.0 / 4096) << " deg)"       << std::endl;
    std::cout << "  Speed       : " << speed            << std::endl;
    std::cout << "  Load        : " << load             << std::endl;
    std::cout << "  Voltage     : " << std::fixed << std::setprecision(1)
              << (volt * 0.1) << " V"                   << std::endl;
    std::cout << "  Temperature : " << temp << " C"     << std::endl;

    // -- Diagnostic summary --
    std::cout << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  DIAGNOSTIC SUMMARY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    bool all_ok = true;

    if (mode != 0) {
        std::cout << "  FAIL: Mode is " << mode << " (" << mode_name(mode) << ")" << std::endl;
        all_ok = false;
    } else {
        std::cout << "  OK: Mode = 0 (Position Servo)" << std::endl;
    }

    if (baud_code != 0) {
        std::cout << "  WARN: Baud rate is NOT 1 Mbps (code=" << baud_code << ")" << std::endl;
    } else {
        std::cout << "  OK: Baud = 1,000,000 bps" << std::endl;
    }

    if (min_angle == 0 && max_angle == 4095) {
        std::cout << "  OK: Angle limits = full range (0-4095)" << std::endl;
    } else {
        std::cout << "  INFO: Angle limits: " << min_angle << "-" << max_angle
                  << " (customized)" << std::endl;
    }

    if (pos >= 0 && pos <= 4095) {
        std::cout << "  OK: Current position = " << pos << " ("
                  << std::fixed << std::setprecision(1)
                  << (pos * 360.0 / 4096) << " deg)"   << std::endl;
    } else {
        std::cout << "  WARN: Position read returned " << pos << std::endl;
        all_ok = false;
    }

    std::cout << std::endl;
    if (all_ok) {
        std::cout << "  >>> ALL CHECKS PASSED - Servo is ready <<<" << std::endl;
    } else {
        std::cout << "  >>> Issues found - fix before proceeding <<<" << std::endl;
    }

    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  Copy/paste ENTIRE output and send it to me." << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    servo.end();
    return 0;
}
