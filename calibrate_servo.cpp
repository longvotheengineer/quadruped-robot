/**
 * @file calibrate_servo.cpp
 * @brief Interactive ST3215 servo calibration tool (Method A: CalibrationOfs)
 *
 * This tool walks you through calibrating one servo at a time:
 *   0. Asks what URDF angle you'll position the horn at (e.g., 0 or -90)
 *   1. Disables torque so you can rotate the horn by hand
 *   2. Live-displays the current tick value as you move the shaft
 *   3. On your command, calls CalibrationOfs() to set current position = tick 2048
 *   4. Verifies the calibration
 *   5. Tests direction SAFELY (by hand, no motor movement)
 *   6. Tests angle limits
 *   7. Computes the correct tick_offset for URDF 0 degrees
 *
 * Build:
 *   g++ -std=c++11 -O2 -I<SCLIB> -o calibrate_servo calibrate_servo.cpp <SCLIB>/build/libSCServo.a -lpthread
 *
 * Usage:
 *   ./calibrate_servo /dev/ttyACM0 1
 *                     ^port        ^servo_id
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "SCServo.h"

// ─── Terminal raw mode helpers ──────────────────────────────────────────────

static struct termios orig_termios;

static void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

static void enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 1;  // 100ms timeout
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

static bool keyPressed(char& c) {
    int n = read(STDIN_FILENO, &c, 1);
    return n > 0;
}


// ─── Utility ────────────────────────────────────────────────────────────────

static double tickToDeg(int tick, int offset = 2048) {
    return (tick - offset) * 360.0 / 4096.0;
}


// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <port> <servo_id>" << std::endl;
        return 1;
    }

    const char* port = argv[1];
    int servo_id     = std::atoi(argv[2]);

    // ── Open serial port ────────────────────────────────────────────────
    SMS_STS servo;
    if (!servo.begin(1000000, port)) {
        std::cout << "FAILED to open " << port << std::endl;
        return 1;
    }

    // ── Ping ────────────────────────────────────────────────────────────
    if (servo.Ping(servo_id) == -1) {
        std::cout << "Servo ID " << servo_id << " did not respond!" << std::endl;
        servo.end();
        return 1;
    }

    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  ST3215 Calibration Tool — Servo ID " << servo_id << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << std::endl;

    // ── Read current mode ───────────────────────────────────────────────
    int mode = servo.readByte(servo_id, SMS_STS_MODE);
    if (mode != 0) {
        std::cout << "  WARNING: Mode is " << mode << " (not Position Servo)." << std::endl;
        std::cout << "  Calibration requires Mode 0. Abort." << std::endl;
        servo.end();
        return 1;
    }

    // ── Read position BEFORE calibration ────────────────────────────────
    int pos_before = servo.ReadPos(servo_id);
    std::cout << "  Current position: " << pos_before << " ticks ("
              << std::fixed << std::setprecision(1)
              << tickToDeg(pos_before) << " deg from midpoint)" << std::endl;
    std::cout << std::endl;

    // ════════════════════════════════════════════════════════════════════
    //  STEP 0: Ask reference angle
    // ════════════════════════════════════════════════════════════════════
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "  STEP 0: Reference angle" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << std::endl;
    std::cout << "  What URDF angle (degrees) will you position the horn at?" << std::endl;
    std::cout << "  Examples:" << std::endl;
    std::cout << "    0    = horn at URDF zero (joint 1)" << std::endl;
    std::cout << "    -90  = horn at URDF -pi/2 (joint 2)" << std::endl;
    std::cout << std::endl;
    std::cout << "  Enter reference angle in degrees: ";
    std::cout.flush();

    double ref_angle_deg = 0.0;
    std::cin >> ref_angle_deg;
    std::cin.ignore();  // consume newline

    std::cout << std::endl;
    std::cout << "  Reference angle: " << ref_angle_deg << " deg" << std::endl;
    std::cout << std::endl;

    // ════════════════════════════════════════════════════════════════════
    //  STEP 1: Free the servo — disable torque
    // ════════════════════════════════════════════════════════════════════
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "  STEP 1: Free the servo" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << std::endl;

    servo.EnableTorque(servo_id, 0);
    std::cout << "  Torque DISABLED. You can now rotate the horn by hand." << std::endl;
    std::cout << std::endl;
    std::cout << "  WHAT TO DO:" << std::endl;
    std::cout << "    Rotate the horn to the position corresponding to" << std::endl;
    std::cout << "    URDF angle = " << ref_angle_deg << " degrees." << std::endl;
    std::cout << std::endl;
    std::cout << "  Live position display starting..." << std::endl;
    std::cout << "  Press [ENTER] when the horn is at the target position." << std::endl;
    std::cout << "  Press [q] to abort." << std::endl;
    std::cout << std::endl;

    // ── Live position display ───────────────────────────────────────────
    enableRawMode();

    bool confirmed = false;
    while (true) {
        int pos = servo.ReadPos(servo_id);
        double deg = tickToDeg(pos);

        // Print live position (overwrite same line)
        std::cout << "\r  >>> Tick: " << std::setw(5) << pos
                  << "  |  Deg from mid: " << std::setw(7) << std::fixed
                  << std::setprecision(1) << deg << "°"
                  << "  |  Press ENTER to confirm <<<" << std::flush;

        char c;
        if (keyPressed(c)) {
            if (c == '\n' || c == '\r') {
                confirmed = true;
                break;
            } else if (c == 'q' || c == 'Q') {
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    disableRawMode();
    std::cout << std::endl << std::endl;

    if (!confirmed) {
        std::cout << "  Aborted by user." << std::endl;
        servo.end();
        return 0;
    }

    // ════════════════════════════════════════════════════════════════════
    //  STEP 2: Call CalibrationOfs — "Set Middle Position"
    // ════════════════════════════════════════════════════════════════════
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "  STEP 2: Setting middle position (CalibrationOfs)" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    int pos_at_ref = servo.ReadPos(servo_id);
    std::cout << "  Position at reference: " << pos_at_ref << " ticks" << std::endl;
    std::cout << "  Calling CalibrationOfs()..." << std::endl;

    // Unlock EEPROM, calibrate, lock EEPROM
    servo.unLockEprom(servo_id);
    servo.CalibrationOfs(servo_id);

    // Wait for EEPROM write
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    servo.LockEprom(servo_id);

    // Verify: position should now read ~2048
    int pos_after = servo.ReadPos(servo_id);
    std::cout << "  Position after calibration: " << pos_after << " ticks" << std::endl;

    if (std::abs(pos_after - 2048) < 10) {
        std::cout << "  OK: Midpoint calibrated successfully (2048 +/- 10)" << std::endl;
    } else {
        std::cout << "  WARNING: Expected ~2048, got " << pos_after << std::endl;
        std::cout << "  The calibration may not have taken effect." << std::endl;
        std::cout << "  Try power-cycling the servo and re-running." << std::endl;
    }

    std::cout << std::endl;

    // ════════════════════════════════════════════════════════════════════
    //  STEP 3: Determine direction (SAFE — manual rotation, no motor)
    // ════════════════════════════════════════════════════════════════════
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "  STEP 3: Determine direction (SAFE — by hand)" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << std::endl;
    std::cout << "  Torque is still OFF." << std::endl;
    std::cout << "  Gently rotate the horn in the URDF-POSITIVE direction" << std::endl;
    std::cout << "  (the direction where the joint angle INCREASES)." << std::endl;
    std::cout << std::endl;
    std::cout << "  Watch the tick value on screen:" << std::endl;
    std::cout << "    - If tick INCREASES → press [+]" << std::endl;
    std::cout << "    - If tick DECREASES → press [-]" << std::endl;
    std::cout << std::endl;
    std::cout << "  Press [s] to skip (default +1)." << std::endl;
    std::cout << std::endl;

    // Live tick display while user manually rotates
    enableRawMode();

    int direction = 1;
    int baseline_tick = servo.ReadPos(servo_id);

    while (true) {
        int pos = servo.ReadPos(servo_id);
        int delta = pos - baseline_tick;

        std::cout << "\r  >>> Tick: " << std::setw(5) << pos
                  << "  |  Delta: " << std::setw(5)
                  << (delta >= 0 ? "+" : "") << delta
                  << "  |  Press [+] [-] or [s] <<<" << std::flush;

        char c;
        if (keyPressed(c)) {
            if (c == '+' || c == '=') { direction = +1; break; }
            if (c == '-' || c == '_') { direction = -1; break; }
            if (c == 's' || c == 'S') { direction = +1; break; }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    disableRawMode();

    std::cout << std::endl;
    std::cout << "  Direction set to: " << (direction > 0 ? "+1" : "-1") << std::endl;
    std::cout << std::endl;

    // ════════════════════════════════════════════════════════════════════
    //  STEP 4: Find angle limits (manual, torque off)
    // ════════════════════════════════════════════════════════════════════
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "  STEP 4: Find angle limits" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    std::cout << std::endl;

    std::cout << "  Torque is OFF. Manually rotate the horn to the" << std::endl;
    std::cout << "  MINIMUM angle limit (mechanical stop / safe limit)." << std::endl;
    std::cout << "  Press [ENTER] when at the minimum position." << std::endl;
    std::cout << "  Press [s] to skip limit detection." << std::endl;
    std::cout << std::endl;

    enableRawMode();
    int tick_min = 0;
    bool limits_set = true;
    while (true) {
        int pos = servo.ReadPos(servo_id);
        double deg = tickToDeg(pos);
        std::cout << "\r  >>> MIN: Tick=" << std::setw(5) << pos
                  << " (" << std::setw(7) << std::fixed << std::setprecision(1)
                  << deg << " deg)  |  ENTER=confirm  s=skip <<<" << std::flush;

        char c;
        if (keyPressed(c)) {
            if (c == '\n' || c == '\r') {
                tick_min = pos;
                break;
            }
            if (c == 's' || c == 'S') {
                tick_min = 0;
                limits_set = false;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    disableRawMode();
    std::cout << std::endl;

    int tick_max = 4095;
    if (limits_set) {
        std::cout << "  Min limit: " << tick_min << " ticks" << std::endl;
        std::cout << std::endl;
        std::cout << "  Now rotate to the MAXIMUM angle limit." << std::endl;
        std::cout << "  Press [ENTER] when at the maximum position." << std::endl;
        std::cout << std::endl;

        enableRawMode();
        while (true) {
            int pos = servo.ReadPos(servo_id);
            double deg = tickToDeg(pos);
            std::cout << "\r  >>> MAX: Tick=" << std::setw(5) << pos
                      << " (" << std::setw(7) << std::fixed << std::setprecision(1)
                      << deg << " deg)  |  ENTER=confirm <<<" << std::flush;

            char c;
            if (keyPressed(c) && (c == '\n' || c == '\r')) {
                tick_max = pos;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        disableRawMode();
        std::cout << std::endl;
        std::cout << "  Max limit: " << tick_max << " ticks" << std::endl;
    }

    // Make sure min < max
    if (tick_min > tick_max) {
        std::swap(tick_min, tick_max);
        std::cout << "  (Swapped min/max to maintain order)" << std::endl;
    }

    std::cout << std::endl;

    // ════════════════════════════════════════════════════════════════════
    //  SUMMARY — compute the real tick_offset
    // ════════════════════════════════════════════════════════════════════
    //
    // Formula: tick = direction * angle_deg * (4096/360) + tick_offset
    // At calibration: 2048 = direction * ref_angle_deg * (4096/360) + tick_offset
    // Therefore: tick_offset = 2048 - direction * ref_angle_deg * (4096/360)
    //
    double ticks_per_deg = 4096.0 / 360.0;
    int computed_offset = static_cast<int>(
        std::round(2048.0 - direction * ref_angle_deg * ticks_per_deg));

    std::cout << std::string(60, '=') << std::endl;
    std::cout << "  CALIBRATION RESULT — Servo ID " << servo_id << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << std::endl;
    std::cout << "  tick_offset : 2048  (set via CalibrationOfs)" << std::endl;
    std::cout << "  direction   : " << (direction > 0 ? "+1" : "-1") << std::endl;
    std::cout << "  tick_min    : " << tick_min << " (" << std::fixed
              << std::setprecision(1) << tickToDeg(tick_min) << " deg)" << std::endl;
    std::cout << "  tick_max    : " << tick_max << " (" << std::fixed
              << std::setprecision(1) << tickToDeg(tick_max) << " deg)" << std::endl;
    std::cout << std::endl;
    std::cout << "  Copy these values into your ROS parameters:" << std::endl;
    std::cout << std::endl;
    std::cout << "    ros2 run serial_driver serial_driver_node \\" << std::endl;
    std::cout << "      --ros-args \\" << std::endl;
    std::cout << "      -p tick_offsets:=[2048, ...]  \\" << std::endl;
    std::cout << "      -p directions:=[" << direction << ", ...]  \\" << std::endl;
    std::cout << "      -p tick_min:=[" << tick_min << ", ...]  \\" << std::endl;
    std::cout << "      -p tick_max:=[" << tick_max << ", ...]" << std::endl;
    std::cout << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    // Disable torque and close
    servo.EnableTorque(servo_id, 0);
    servo.end();
    return 0;
}
