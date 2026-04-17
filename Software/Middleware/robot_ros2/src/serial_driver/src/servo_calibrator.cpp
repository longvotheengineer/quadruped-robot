/**
 * @file servo_calibrator.cpp
 * @brief Comprehensive CLI calibration tool for ST3215/STS servos.
 *
 * Features:
 *   1. Scan bus for online servos
 *   2. Read servo info (ID, position, limits, offset, voltage, temp)
 *   3. Change servo ID
 *   4. Calibrate center (set current position as center=2048)
 *   5. Set min/max angle limits with live position display
 *   6. Live position monitor
 *   7. Calibrate direction (determine +1/-1 multiplier for serial_driver)
 *
 * Usage:
 *   ./servo_calibrator /dev/ttyACM1
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "SCServo.h"

static constexpr double TICKS_TO_DEG = 360.0 / 4096.0;
static constexpr double DEG_TO_RAD  = M_PI / 180.0;

static void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ─── Non-blocking key check ──────────────────────────────────────────────
static bool key_pressed() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    return (ch == '\n' || ch == '\r');
}

// ─── Restore terminal to canonical mode ──────────────────────────────────
static void restore_terminal() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

// ─── Wait for Enter (blocking) ──────────────────────────────────────────
static void wait_enter() {
    restore_terminal();
    std::cin.get();
}

// ─── Read position with live display until Enter ─────────────────────────
static int live_read(SMS_STS& servo, int id, const char* label) {
    std::cout << "\n  >>> Move joint to " << label << " position." << std::endl;
    std::cout << "  >>> Press ENTER to record...\n" << std::endl;
    int last = 0;
    while (true) {
        int tick = servo.ReadPos(id);
        if (tick != -1) {
            last = tick;
            double deg = tick * TICKS_TO_DEG;
            double rad = deg * DEG_TO_RAD;
            std::cout << "\r  Pos: "
                      << std::setw(5) << tick << " ticks | "
                      << std::fixed << std::setprecision(1) << std::setw(7) << deg << "° | "
                      << std::setprecision(3) << std::setw(7) << rad << " rad   "
                      << std::flush;
        }
        if (key_pressed()) { std::cout << std::endl; break; }
        sleep_ms(50);
    }
    return last;
}

// ═══════════════════════════════════════════════════════════════════════════
//  MENU FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

static void scan_bus(SMS_STS& servo, std::vector<int>& found) {
    found.clear();
    std::cout << "\n  Scanning IDs 1-20..." << std::endl;
    for (int id = 1; id <= 20; ++id) {
        if (servo.Ping(id) != -1) {
            int pos = servo.ReadPos(id);
            int min = servo.readWord(id, 9);
            int max = servo.readWord(id, 11);
            int ofs = servo.readWord(id, 31);
            std::cout << "  ✓ ID " << std::setw(2) << id
                      << "  pos=" << std::setw(4) << pos
                      << "  min=" << std::setw(4) << min
                      << "  max=" << std::setw(4) << max
                      << "  ofs=" << std::setw(4) << ofs
                      << std::endl;
            found.push_back(id);
        }
    }
    if (found.empty()) {
        std::cout << "  No servos found!" << std::endl;
    } else {
        std::cout << "  Found " << found.size() << " servo(s)." << std::endl;
    }
}

static void read_info(SMS_STS& servo, int id) {
    std::cout << "\n  ╔══ Servo ID " << id << " ══════════════════════════╗" << std::endl;
    int pos   = servo.ReadPos(id);
    int speed = servo.ReadSpeed(id);
    int load  = servo.ReadLoad(id);
    int volt  = servo.ReadVoltage(id);
    int temp  = servo.ReadTemper(id);
    int cur   = servo.ReadCurrent(id);
    int minl  = servo.readWord(id, 9);
    int maxl  = servo.readWord(id, 11);
    int ofs   = servo.readWord(id, 31);
    int mode  = servo.readByte(id, 33);
    int model = servo.readWord(id, 3);

    double deg = pos * TICKS_TO_DEG;
    double rad = deg * DEG_TO_RAD;

    std::cout << std::fixed;
    std::cout << "  ║  Model:      " << model << std::endl;
    std::cout << "  ║  Mode:       " << mode << " (" << (mode == 0 ? "Position" : "Wheel") << ")" << std::endl;
    std::cout << "  ║  Position:   " << pos << " ticks ("
              << std::setprecision(1) << deg << "° / "
              << std::setprecision(3) << rad << " rad)" << std::endl;
    std::cout << "  ║  Speed:      " << speed << std::endl;
    std::cout << "  ║  Load:       " << load << " /1000" << std::endl;
    std::cout << "  ║  Voltage:    " << std::setprecision(1) << (volt / 10.0) << " V" << std::endl;
    std::cout << "  ║  Temp:       " << temp << " °C" << std::endl;
    std::cout << "  ║  Current:    " << cur << " mA" << std::endl;
    std::cout << "  ║  Offset:     " << ofs << " ticks" << std::endl;
    std::cout << "  ║  Min Limit:  " << minl << " ticks ("
              << std::setprecision(1) << (minl * TICKS_TO_DEG) << "°)" << std::endl;
    std::cout << "  ║  Max Limit:  " << maxl << " ticks ("
              << std::setprecision(1) << (maxl * TICKS_TO_DEG) << "°)" << std::endl;
    std::cout << "  ╚══════════════════════════════════════╝" << std::endl;
}

static void change_id(SMS_STS& servo, int old_id) {
    std::cout << "\n  Current ID: " << old_id << std::endl;
    std::cout << "  Enter new ID (1-253): ";
    int new_id;
    std::cin >> new_id;
    std::cin.ignore();
    if (new_id < 1 || new_id > 253) {
        std::cout << "  Invalid ID!" << std::endl;
        return;
    }
    // Check if new ID is already in use
    if (servo.Ping(new_id) != -1) {
        std::cout << "  WARNING: ID " << new_id << " is already in use!" << std::endl;
        std::cout << "  Continue anyway? (y/n): ";
        char c; std::cin >> c; std::cin.ignore();
        if (c != 'y' && c != 'Y') return;
    }

    std::cout << "  Changing " << old_id << " → " << new_id << "..." << std::endl;
    servo.unLockEprom(old_id);
    sleep_ms(100);
    servo.writeByte(old_id, SMS_STS_ID, new_id);
    sleep_ms(100);
    servo.LockEprom(new_id);
    sleep_ms(100);

    if (servo.Ping(new_id) != -1) {
        std::cout << "  ✓ SUCCESS: Now responds as ID " << new_id << std::endl;
    } else {
        std::cout << "  ✗ FAILED: No response at ID " << new_id << ". Try power-cycling." << std::endl;
    }
}

static void calibrate_center(SMS_STS& servo, int id) {
    std::cout << "\n  ═══ Center Calibration (ID " << id << ") ═══" << std::endl;
    std::cout << "  This sets the CURRENT position as center (tick 2048)." << std::endl;
    std::cout << "\n  Disabling torque — move joint to desired center..." << std::endl;
    servo.EnableTorque(id, 0);
    sleep_ms(100);

    int tick = live_read(servo, id, "CENTER");
    double deg = tick * TICKS_TO_DEG;

    std::cout << "\n  Center will be set at: " << tick << " ticks ("
              << std::fixed << std::setprecision(1) << deg << "°)" << std::endl;
    std::cout << "  After calibration, this position will read as 2048." << std::endl;
    std::cout << "  Press ENTER to confirm, Ctrl+C to abort..." << std::endl;
    wait_enter();

    std::cout << "  Calibrating..." << std::endl;
    servo.CalibrationOfs(id);
    sleep_ms(200);

    int new_pos = servo.ReadPos(id);
    int new_ofs = servo.readWord(id, 31);
    std::cout << "  ✓ Done. New position reads: " << new_pos
              << " (offset=" << new_ofs << ")" << std::endl;

    servo.EnableTorque(id, 1);
    std::cout << "  Torque re-enabled." << std::endl;
}

static void set_limits(SMS_STS& servo, int id) {
    std::cout << "\n  ═══ Limit Calibration (ID " << id << ") ═══" << std::endl;

    int cur_min = servo.readWord(id, 9);
    int cur_max = servo.readWord(id, 11);
    std::cout << "  Current limits: min=" << cur_min << " max=" << cur_max << std::endl;

    std::cout << "\n  Disabling torque — move joint freely." << std::endl;
    servo.EnableTorque(id, 0);
    sleep_ms(100);

    // MIN
    int min_tick = live_read(servo, id, "MIN (limit)");
    double min_deg = min_tick * TICKS_TO_DEG;
    std::cout << "  ✓ MIN: " << min_tick << " ticks ("
              << std::fixed << std::setprecision(1) << min_deg << "°)" << std::endl;

    // MAX
    int max_tick = live_read(servo, id, "MAX (limit)");
    double max_deg = max_tick * TICKS_TO_DEG;
    std::cout << "  ✓ MAX: " << max_tick << " ticks ("
              << std::fixed << std::setprecision(1) << max_deg << "°)" << std::endl;

    if (min_tick > max_tick) {
        std::swap(min_tick, max_tick);
        std::swap(min_deg, max_deg);
        std::cout << "  (Swapped)" << std::endl;
    }

    std::cout << "\n  ┌────────────────────────────────┐" << std::endl;
    std::cout << "  │  MIN = " << std::setw(4) << min_tick << " ("
              << std::setw(6) << min_deg << "°)" << std::endl;
    std::cout << "  │  MAX = " << std::setw(4) << max_tick << " ("
              << std::setw(6) << max_deg << "°)" << std::endl;
    std::cout << "  └────────────────────────────────┘" << std::endl;
    std::cout << "  Press ENTER to write, Ctrl+C to abort..." << std::endl;
    wait_enter();

    servo.unLockEprom(id);
    sleep_ms(100);
    servo.writeWord(id, 9, min_tick);
    sleep_ms(50);
    servo.writeWord(id, 11, max_tick);
    sleep_ms(50);
    servo.LockEprom(id);
    sleep_ms(100);

    int v_min = servo.readWord(id, 9);
    int v_max = servo.readWord(id, 11);
    std::cout << "  Verify: min=" << v_min << (v_min == min_tick ? " ✓" : " ✗")
              << "  max=" << v_max << (v_max == max_tick ? " ✓" : " ✗") << std::endl;

    servo.EnableTorque(id, 1);
    std::cout << "  Torque re-enabled. Done." << std::endl;
}

static void live_monitor(SMS_STS& servo, int id) {
    std::cout << "\n  ═══ Live Monitor (ID " << id << ") ═══" << std::endl;
    std::cout << "  Press ENTER to stop.\n" << std::endl;
    while (true) {
        int pos   = servo.ReadPos(id);
        int speed = servo.ReadSpeed(id);
        int load  = servo.ReadLoad(id);
        int volt  = servo.ReadVoltage(id);
        int temp  = servo.ReadTemper(id);
        if (pos != -1) {
            double deg = pos * TICKS_TO_DEG;
            std::cout << "\r  "
                      << std::setw(5) << pos << " ticks | "
                      << std::fixed << std::setprecision(1) << std::setw(7) << deg << "° | "
                      << "spd=" << std::setw(5) << speed << " | "
                      << "load=" << std::setw(4) << load << " | "
                      << std::setprecision(1) << (volt / 10.0) << "V | "
                      << temp << "°C    "
                      << std::flush;
        }
        if (key_pressed()) { std::cout << std::endl; break; }
        sleep_ms(50);
    }
}

static void calibrate_direction(SMS_STS& servo, int id) {
    static constexpr int TEST_DELTA  = 200;   // ticks to move for the test
    static constexpr int TEST_SPEED  = 500;   // slow speed for safety
    static constexpr int TEST_ACC    = 20;

    std::cout << "\n  ═══ Direction Calibration (ID " << id << ") ═══" << std::endl;
    std::cout << "  This determines the software direction multiplier (+1 / -1)" << std::endl;
    std::cout << "  for the serial_driver 'directions' parameter.\n" << std::endl;
    std::cout << "  The servo will move a small amount in the +tick direction." << std::endl;
    std::cout << "  After observing the motion, you decide if that matches" << std::endl;
    std::cout << "  the POSITIVE joint direction on your robot.\n" << std::endl;

    // Read current position
    int start_pos = servo.ReadPos(id);
    if (start_pos == -1) {
        std::cout << "  ✗ Could not read position!" << std::endl;
        return;
    }
    double start_deg = start_pos * TICKS_TO_DEG;
    std::cout << "  Current position: " << start_pos << " ticks ("
              << std::fixed << std::setprecision(1) << start_deg << "°)" << std::endl;

    // Check if we have room to move
    int min_lim = servo.readWord(id, 9);
    int max_lim = servo.readWord(id, 11);
    std::cout << "  EEPROM limits: min=" << min_lim << "  max=" << max_lim << std::endl;

    int target = start_pos + TEST_DELTA;
    bool moved_positive = true;

    // If moving +delta would exceed the max limit, move -delta instead
    if (max_lim > 0 && target > max_lim) {
        target = start_pos - TEST_DELTA;
        moved_positive = false;
        if (min_lim >= 0 && target < min_lim) {
            std::cout << "  ✗ Not enough range to test! Widen the limits first." << std::endl;
            return;
        }
    }

    std::cout << "\n  Will move: " << start_pos << " → " << target
              << " (" << (moved_positive ? "+" : "-") << TEST_DELTA << " ticks)" << std::endl;
    std::cout << "  Press ENTER to move, Ctrl+C to abort..." << std::endl;
    wait_enter();

    // Move the servo
    servo.WritePosEx(id, target, TEST_SPEED, TEST_ACC);
    sleep_ms(1500);  // wait for motion to complete

    int actual_pos = servo.ReadPos(id);
    double actual_deg = actual_pos * TICKS_TO_DEG;
    std::cout << "  Moved to: " << actual_pos << " ticks ("
              << std::fixed << std::setprecision(1) << actual_deg << "°)" << std::endl;

    // Ask user about the direction
    std::cout << "\n  ┌─────────────────────────────────────────────────┐" << std::endl;
    std::cout << "  │  Did the joint move in the POSITIVE direction   │" << std::endl;
    std::cout << "  │  for your robot? (the direction you want when   │" << std::endl;
    std::cout << "  │  the IK sends a positive angle change)          │" << std::endl;
    std::cout << "  └─────────────────────────────────────────────────┘" << std::endl;
    std::cout << "  (y/n): ";

    char answer;
    std::cin >> answer;
    std::cin.ignore();

    int direction;
    if (moved_positive) {
        direction = (answer == 'y' || answer == 'Y') ? 1 : -1;
    } else {
        // We actually moved in -tick direction, so invert the logic
        direction = (answer == 'y' || answer == 'Y') ? -1 : 1;
    }

    // Move back to start
    std::cout << "\n  Returning to start position..." << std::endl;
    servo.WritePosEx(id, start_pos, TEST_SPEED, TEST_ACC);
    sleep_ms(1500);

    int returned_pos = servo.ReadPos(id);
    std::cout << "  Returned to: " << returned_pos << " ticks" << std::endl;

    // Display result
    std::cout << "\n  ╔══════════════════════════════════════════════════╗" << std::endl;
    std::cout << "  ║  DIRECTION RESULT for servo ID " << std::setw(3) << id << "               ║" << std::endl;
    std::cout << "  ╠══════════════════════════════════════════════════╣" << std::endl;
    std::cout << "  ║                                                  ║" << std::endl;
    std::cout << "  ║   direction = " << std::setw(2) << direction
              << "                                 ║" << std::endl;
    std::cout << "  ║                                                  ║" << std::endl;
    std::cout << "  ║  Use this value in serial_driver's 'directions'  ║" << std::endl;
    std::cout << "  ║  parameter array for this servo's joint index.   ║" << std::endl;
    std::cout << "  ╚══════════════════════════════════════════════════╝" << std::endl;

    // Offer to test the other direction
    std::cout << "\n  Test with reversed direction? (y/n): ";
    std::cin >> answer;
    std::cin.ignore();

    if (answer == 'y' || answer == 'Y') {
        int rev_target = start_pos - (target - start_pos);  // opposite direction

        // Check limits
        if ((rev_target < min_lim && min_lim >= 0) || (rev_target > max_lim && max_lim > 0)) {
            std::cout << "  ✗ Reverse target (" << rev_target << ") exceeds limits!" << std::endl;
            return;
        }

        std::cout << "  Moving: " << start_pos << " → " << rev_target
                  << " (reverse direction)" << std::endl;
        servo.WritePosEx(id, rev_target, TEST_SPEED, TEST_ACC);
        sleep_ms(1500);

        int rev_pos = servo.ReadPos(id);
        std::cout << "  Moved to: " << rev_pos << " ticks ("
                  << std::fixed << std::setprecision(1) << (rev_pos * TICKS_TO_DEG) << "°)" << std::endl;

        std::cout << "  Press ENTER to return to start..." << std::endl;
        wait_enter();

        servo.WritePosEx(id, start_pos, TEST_SPEED, TEST_ACC);
        sleep_ms(1500);
        std::cout << "  ✓ Returned to start." << std::endl;
    }

    std::cout << "\n  ✓ Direction calibration complete. direction = "
              << direction << " for ID " << id << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN MENU
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <port>" << std::endl;
        std::cout << "  e.g: " << argv[0] << " /dev/ttyACM1" << std::endl;
        return 1;
    }

    SMS_STS servo;
    if (!servo.begin(1000000, argv[1])) {
        std::cout << "FAILED to open " << argv[1] << std::endl;
        return 1;
    }
    std::cout << "  Port " << argv[1] << " opened at 1000000 baud.\n" << std::endl;

    std::vector<int> found;
    int selected_id = -1;

    while (true) {
        std::cout << "\n  ╔══════════════════════════════════════╗" << std::endl;
        std::cout << "  ║     ST3215 SERVO CALIBRATOR          ║" << std::endl;
        std::cout << "  ╠══════════════════════════════════════╣" << std::endl;
        std::cout << "  ║  1. Scan bus                         ║" << std::endl;
        std::cout << "  ║  2. Select servo (current: ";
        if (selected_id > 0) std::cout << "ID " << std::setw(2) << selected_id;
        else std::cout << "none";
        std::cout << ")   ║" << std::endl;
        std::cout << "  ║  3. Read servo info                  ║" << std::endl;
        std::cout << "  ║  4. Change servo ID                  ║" << std::endl;
        std::cout << "  ║  5. Calibrate center                 ║" << std::endl;
        std::cout << "  ║  6. Set angle limits                 ║" << std::endl;
        std::cout << "  ║  7. Live position monitor            ║" << std::endl;
        std::cout << "  ║  8. Calibrate direction              ║" << std::endl;
        std::cout << "  ║  0. Exit                             ║" << std::endl;
        std::cout << "  ╚══════════════════════════════════════╝" << std::endl;
        std::cout << "  Choice: ";

        int choice;
        std::cin >> choice;
        std::cin.ignore();

        switch (choice) {
            case 0:
                std::cout << "  Bye!" << std::endl;
                servo.end();
                return 0;

            case 1:
                scan_bus(servo, found);
                break;

            case 2: {
                std::cout << "  Enter servo ID: ";
                int id;
                std::cin >> id;
                std::cin.ignore();
                if (servo.Ping(id) != -1) {
                    selected_id = id;
                    std::cout << "  ✓ ID " << id << " selected." << std::endl;
                } else {
                    std::cout << "  ✗ ID " << id << " not responding!" << std::endl;
                }
                break;
            }

            case 3:
                if (selected_id < 0) { std::cout << "  Select a servo first (option 2)." << std::endl; break; }
                read_info(servo, selected_id);
                break;

            case 4:
                if (selected_id < 0) { std::cout << "  Select a servo first (option 2)." << std::endl; break; }
                change_id(servo, selected_id);
                selected_id = -1; // Reset since ID changed
                break;

            case 5:
                if (selected_id < 0) { std::cout << "  Select a servo first (option 2)." << std::endl; break; }
                calibrate_center(servo, selected_id);
                break;

            case 6:
                if (selected_id < 0) { std::cout << "  Select a servo first (option 2)." << std::endl; break; }
                set_limits(servo, selected_id);
                break;

            case 7:
                if (selected_id < 0) { std::cout << "  Select a servo first (option 2)." << std::endl; break; }
                live_monitor(servo, selected_id);
                break;

            case 8:
                if (selected_id < 0) { std::cout << "  Select a servo first (option 2)." << std::endl; break; }
                calibrate_direction(servo, selected_id);
                break;

            default:
                std::cout << "  Invalid choice." << std::endl;
        }
    }
}
