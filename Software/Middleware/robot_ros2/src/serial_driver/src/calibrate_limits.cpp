/**
 * @file calibrate_limits.cpp
 * @brief Interactive CLI tool to set EEPROM angle limits on ST3215 servos.
 *
 * Features:
 *   - Real-time position display (ticks, degrees, radians) while you move the joint
 *   - Records MIN and MAX positions on Enter
 *   - Writes limits to servo EEPROM
 *
 * Usage:
 *   ./calibrate_limits /dev/ttyACM1 1       (calibrate servo ID 1)
 *   ./calibrate_limits /dev/ttyACM1 1 2 3   (calibrate servos 1, 2, 3)
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "SCServo.h"

static constexpr double TICKS_TO_DEG = 360.0 / 4096.0;
static constexpr double DEG_TO_RAD = M_PI / 180.0;

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Non-blocking key check
static bool key_pressed() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        return (ch == '\n' || ch == '\r');
    }
    return false;
}

// Show live position until Enter is pressed, return final tick
static int live_read_until_enter(SMS_STS& servo, int id, const char* label) {
    std::cout << "\n  >>> Move the joint to the " << label << " position." << std::endl;
    std::cout << "  >>> Press ENTER to record...\n" << std::endl;

    int last_tick = 0;
    while (true) {
        int tick = servo.ReadPos(id);
        if (tick != -1) {
            last_tick = tick;
            double deg = tick * TICKS_TO_DEG;
            double rad = deg * DEG_TO_RAD;
            // Overwrite the same line
            std::cout << "\r  Position:  "
                      << std::setw(5) << tick << " ticks  |  "
                      << std::fixed << std::setprecision(1) << std::setw(7) << deg << " deg  |  "
                      << std::setprecision(3) << std::setw(7) << rad << " rad   "
                      << std::flush;
        }

        if (key_pressed()) {
            std::cout << std::endl;  // Move to next line after Enter
            break;
        }
        sleep_ms(50);  // ~20 Hz refresh
    }
    return last_tick;
}

static void calibrate_one(SMS_STS& servo, int id) {
    std::cout << "\n══════════════════════════════════════════" << std::endl;
    std::cout << "  Calibrating Servo ID " << id << std::endl;
    std::cout << "══════════════════════════════════════════" << std::endl;

    // Ping
    if (servo.Ping(id) == -1) {
        std::cout << "  ERROR: Servo ID " << id << " did NOT respond. Skipping." << std::endl;
        return;
    }
    std::cout << "  Servo ID " << id << " is ONLINE." << std::endl;

    // Read current limits
    int cur_min = servo.readWord(id, 9);
    int cur_max = servo.readWord(id, 11);
    int cur_pos = servo.ReadPos(id);
    double cur_deg = cur_pos * TICKS_TO_DEG;
    double cur_rad = cur_deg * DEG_TO_RAD;
    std::cout << "  Current EEPROM limits: min=" << cur_min << ", max=" << cur_max << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  Current position: " << cur_pos << " ticks ("
              << cur_deg << " deg / " << std::setprecision(3) << cur_rad << " rad)" << std::endl;

    // Disable torque
    std::cout << "\n  Disabling torque — move the joint freely." << std::endl;
    servo.EnableTorque(id, 0);
    sleep_ms(100);

    // ── MIN position (live) ─────────────────────────────────────────
    int min_tick = live_read_until_enter(servo, id, "MIN (limit)");
    double min_deg = min_tick * TICKS_TO_DEG;
    double min_rad = min_deg * DEG_TO_RAD;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  ✓ MIN recorded: " << min_tick << " ticks ("
              << min_deg << " deg / " << std::setprecision(3) << min_rad << " rad)" << std::endl;

    // ── MAX position (live) ─────────────────────────────────────────
    int max_tick = live_read_until_enter(servo, id, "MAX (limit)");
    double max_deg = max_tick * TICKS_TO_DEG;
    double max_rad = max_deg * DEG_TO_RAD;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  ✓ MAX recorded: " << max_tick << " ticks ("
              << max_deg << " deg / " << std::setprecision(3) << max_rad << " rad)" << std::endl;

    // Swap if needed
    if (min_tick > max_tick) {
        std::swap(min_tick, max_tick);
        std::swap(min_deg, max_deg);
        std::swap(min_rad, max_rad);
        std::cout << "  (Swapped so min < max)" << std::endl;
    }

    // ── Confirm ─────────────────────────────────────────────────────
    std::cout << "\n  ┌──────────────────────────────────────┐" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  │  MIN = " << std::setw(4) << min_tick << " ticks  ("
              << std::setw(6) << min_deg << "° / " << std::setprecision(3) << std::setw(6) << min_rad << " rad)" << std::endl;
    std::cout << std::setprecision(1);
    std::cout << "  │  MAX = " << std::setw(4) << max_tick << " ticks  ("
              << std::setw(6) << max_deg << "° / " << std::setprecision(3) << std::setw(6) << max_rad << " rad)" << std::endl;
    std::cout << "  └──────────────────────────────────────┘" << std::endl;
    std::cout << "  Press ENTER to write to EEPROM, or Ctrl+C to abort..." << std::endl;

    // Wait for Enter (blocking)
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    std::cin.get();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    // ── Write to EEPROM ─────────────────────────────────────────────
    std::cout << "  Writing to EEPROM..." << std::endl;
    servo.unLockEprom(id);
    sleep_ms(100);

    servo.writeWord(id, 9, min_tick);   // SMS_STS_MIN_ANGLE_LIMIT_L
    sleep_ms(50);

    servo.writeWord(id, 11, max_tick);  // SMS_STS_MAX_ANGLE_LIMIT_L
    sleep_ms(50);

    servo.LockEprom(id);
    sleep_ms(100);

    // ── Verify ──────────────────────────────────────────────────────
    int v_min = servo.readWord(id, 9);
    int v_max = servo.readWord(id, 11);
    std::cout << "  Verify: min=" << v_min << (v_min == min_tick ? " ✓" : " ✗ MISMATCH!")
              << "  max=" << v_max << (v_max == max_tick ? " ✓" : " ✗ MISMATCH!") << std::endl;

    // Re-enable torque
    servo.EnableTorque(id, 1);
    std::cout << "  Torque re-enabled. Done.\n" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <port> <servo_id> [id2] [id3] ..." << std::endl;
        std::cout << "  e.g: " << argv[0] << " /dev/ttyACM1 1 2 3" << std::endl;
        return 1;
    }

    const char* port = argv[1];
    std::vector<int> ids;
    for (int i = 2; i < argc; ++i) {
        ids.push_back(std::atoi(argv[i]));
    }

    SMS_STS servo;
    if (!servo.begin(1000000, port)) {
        std::cout << "FAILED to open " << port << std::endl;
        return 1;
    }
    std::cout << "Serial port " << port << " opened at 1000000 baud.\n" << std::endl;

    for (int id : ids) {
        calibrate_one(servo, id);
    }

    std::cout << "═══ All calibration complete ═══" << std::endl;
    servo.end();
    return 0;
}
