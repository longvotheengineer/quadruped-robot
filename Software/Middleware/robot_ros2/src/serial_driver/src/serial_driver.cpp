/**
 * @file serial_driver.cpp
 * @brief ROS 2 driver node for Waveshare ST3215 serial bus servos.
 *
 * Wraps the official Feetech SCServo_Linux (SMS_STS) library to provide:
 *   - SyncWrite position commands to 12 servos via /servo_commands topic
 *   - Single-servo position write via /servo_single_command topic
 *   - Periodic position feedback published on /joint_states_real
 *   - Startup servo ping & torque enable
 *
 * Architecture:
 *   This node is Layer 3 (ROS Interface) in the serial_driver package.
 *   Layer 2 (SCS Protocol) and Layer 1 (Serial Transport) are provided
 *   by the official SCServo_Linux library.
 *
 * @author lvdaengineer
 * @date   2026-04-14
 */

// ─── ROS 2 Headers ─────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

// ─── Vendor Library ─────────────────────────────────────────────────────────
#include "SCServo.h"

// ─── Standard Library ───────────────────────────────────────────────────────
#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>


// ═══════════════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════════════

static constexpr int    kNumServos        = 12;
static constexpr int    kDefaultBaudRate  = 1000000;
static constexpr double kTicksPerDegree   = 4096.0 / 360.0;   // ≈ 11.378
static constexpr double kDegreesPerTick   = 360.0 / 4096.0;   // ≈ 0.0879°
static constexpr int    kFeedbackPeriodMs = 10;               // 100 Hz


// ═══════════════════════════════════════════════════════════════════════════
//  Per-Joint Calibration Data
// ═══════════════════════════════════════════════════════════════════════════

struct JointCalibration {
    uint8_t  servo_id;    ///< SCS bus ID (1–12)
    int16_t  tick_offset; ///< Tick value when joint is at 0° (URDF zero)
    int8_t   direction;   ///< +1 or -1 (maps IK sign to servo direction)
    uint16_t tick_min;    ///< Software lower limit (protection)
    uint16_t tick_max;    ///< Software upper limit (protection)
};


// ═══════════════════════════════════════════════════════════════════════════
//  SerialDriverNode
// ═══════════════════════════════════════════════════════════════════════════

class SerialDriverNode : public rclcpp::Node {
public:
    // ── Constructor & Destructor ────────────────────────────────────────

    SerialDriverNode() : Node("serial_driver_node") {
        declareParameters();
        loadParameters();
        buildCalibrationTable();
        openSerialPort();
        pingAllServos();
        readEepromLimits();
        enableAllTorque();
        createSubscribers();
        createPublisher();
        createFeedbackTimer();

        RCLCPP_INFO(this->get_logger(), "═══ serial_driver_node ready ═══");
    }

    ~SerialDriverNode() {
        for (int i = 0; i < num_servos_; ++i) {
            servo_bus_.EnableTorque(calibration_[i].servo_id, 0);
        }
        RCLCPP_INFO(this->get_logger(), "Torque disabled. Shutting down.");
        servo_bus_.end();
    }

private:
    // ── Member Variables ────────────────────────────────────────────────

    // Hardware
    SMS_STS     servo_bus_;
    std::string port_name_;
    int         baud_rate_      = 0;
    int         num_servos_     = 0;
    uint16_t    default_speed_  = 0;
    uint8_t     default_acc_    = 0;
    int         array_offset_   = 0;

    // Calibration
    std::vector<JointCalibration> calibration_;
    std::vector<std::string>      joint_names_;

    // Feedback
    bool enable_feedback_ = false;
    int  feedback_ms_     = 0;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_commands_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_single_command_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              sub_feedback_toggle_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        pub_joint_states_;
    rclcpp::TimerBase::SharedPtr                                      feedback_timer_;


    // ── Initialization Helpers ──────────────────────────────────────────

    /** @brief Declare all ROS parameters with their default values. */
    void declareParameters() {
        this->declare_parameter<std::string>("port", "/dev/ttyACM1");
        this->declare_parameter<int>("baud_rate",          kDefaultBaudRate);
        this->declare_parameter<int>("num_servos",         kNumServos);
        this->declare_parameter<int>("default_speed",      1500);
        this->declare_parameter<int>("default_acc",        50);
        this->declare_parameter<bool>("enable_feedback",   false);
        this->declare_parameter<int>("feedback_period_ms", kFeedbackPeriodMs);

        // Servo ID list (default: LF, LB, RF, RB ordering)
        std::vector<int64_t> default_ids = {7, 8, 9, 4, 5, 6, 1, 2, 3, 10, 11, 12};
        this->declare_parameter<std::vector<int64_t>>("servo_ids", default_ids);

        // Joint names for /joint_states_real (matches URDF joint names)
        std::vector<std::string> default_joint_names = {
            "joint_lf_1", "joint_lf_2", "joint_lf_3",
            "joint_lb_1", "joint_lb_2", "joint_lb_3",
            "joint_rf_1", "joint_rf_2", "joint_rf_3",
            "joint_rb_1", "joint_rb_2", "joint_rb_3"
        };
        this->declare_parameter<std::vector<std::string>>("joint_names", default_joint_names);

        // Per-joint calibration defaults
        //   Indices: LF1 LF2 LF3  LB1 LB2 LB3  RF1 RF2 RF3  RB1 RB2 RB3
        //   LB joint2 (index 4): direction=-1, offset=3144
        //     — servo physically rotates same as right legs (opposite to LF)
        //     — driver inverts + shifts offset to stay within EEPROM [2046,3763]
        std::vector<int64_t> default_directions =
            {1, 1, 1,  1, -1, 1,  1, 1, 1,  1, 1, 1};
        std::vector<int64_t> default_offsets =
            {2048, 2048, 2048,  2048, 3144, 2048,  2048, 2048, 2048,  2048, 2048, 2048};
        std::vector<int64_t> default_tick_min(kNumServos, 0);
        std::vector<int64_t> default_tick_max(kNumServos, 4095);
        this->declare_parameter<std::vector<int64_t>>("tick_offsets", default_offsets);
        this->declare_parameter<std::vector<int64_t>>("directions",  default_directions);
        this->declare_parameter<std::vector<int64_t>>("tick_min",    default_tick_min);
        this->declare_parameter<std::vector<int64_t>>("tick_max",    default_tick_max);

        // Array offset: starting index in the 12-element command array
        //   LF=0, LB=3, RF=6, RB=9 (for single-leg testing)
        this->declare_parameter<int>("array_offset", 0);
    }

    /** @brief Load all ROS parameters into member variables. */
    void loadParameters() {
        port_name_       = this->get_parameter("port").as_string();
        baud_rate_       = this->get_parameter("baud_rate").as_int();
        num_servos_      = this->get_parameter("num_servos").as_int();
        default_speed_   = static_cast<uint16_t>(this->get_parameter("default_speed").as_int());
        default_acc_     = static_cast<uint8_t>(this->get_parameter("default_acc").as_int());
        enable_feedback_ = this->get_parameter("enable_feedback").as_bool();
        feedback_ms_     = this->get_parameter("feedback_period_ms").as_int();
        joint_names_     = this->get_parameter("joint_names").as_string_array();
        array_offset_    = this->get_parameter("array_offset").as_int();
    }

    /** @brief Build the per-joint calibration table from ROS parameters. */
    void buildCalibrationTable() {
        auto servo_ids = this->get_parameter("servo_ids").as_integer_array();
        auto offsets   = this->get_parameter("tick_offsets").as_integer_array();
        auto dirs      = this->get_parameter("directions").as_integer_array();
        auto tick_mins = this->get_parameter("tick_min").as_integer_array();
        auto tick_maxs = this->get_parameter("tick_max").as_integer_array();

        calibration_.resize(num_servos_);
        for (int i = 0; i < num_servos_; ++i) {
            calibration_[i].servo_id    = static_cast<uint8_t>(servo_ids[i]);
            calibration_[i].tick_offset = static_cast<int16_t>(offsets[i]);
            calibration_[i].direction   = static_cast<int8_t>(dirs[i]);
            calibration_[i].tick_min    = static_cast<uint16_t>(tick_mins[i]);
            calibration_[i].tick_max    = static_cast<uint16_t>(tick_maxs[i]);
        }
    }

    /** @brief Open the serial port and verify the connection. */
    void openSerialPort() {
        if (!servo_bus_.begin(baud_rate_, port_name_.c_str())) {
            RCLCPP_FATAL(this->get_logger(),
                "Failed to open serial port '%s' at %d baud",
                port_name_.c_str(), baud_rate_);
            throw std::runtime_error("Serial port open failed");
        }
        RCLCPP_INFO(this->get_logger(),
            "Serial port '%s' opened at %d baud", port_name_.c_str(), baud_rate_);
    }

    /** @brief Ping every servo on the bus and report connectivity. */
    void pingAllServos() {
        int alive_count = 0;
        for (int i = 0; i < num_servos_; ++i) {
            int response = servo_bus_.Ping(calibration_[i].servo_id);
            if (response != -1) {
                RCLCPP_INFO(this->get_logger(),
                    "Servo ID %d is ONLINE", calibration_[i].servo_id);
                ++alive_count;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Servo ID %d did NOT respond", calibration_[i].servo_id);
            }
        }
        RCLCPP_INFO(this->get_logger(),
            "Ping complete: %d / %d servos online", alive_count, num_servos_);
    }

    /** @brief Read and log the EEPROM angle limits from every servo. */
    void readEepromLimits() {
        for (int i = 0; i < num_servos_; ++i) {
            int id        = calibration_[i].servo_id;
            int min_limit = servo_bus_.readWord(id, 9);   // SMS_STS_MIN_ANGLE_LIMIT_L
            int max_limit = servo_bus_.readWord(id, 11);  // SMS_STS_MAX_ANGLE_LIMIT_L
            if (min_limit != -1 && max_limit != -1) {
                RCLCPP_INFO(this->get_logger(),
                    "Servo ID %d EEPROM limits: min=%d, max=%d (ticks)",
                    id, min_limit, max_limit);
            }
        }
    }

    /** @brief Enable torque on all servos in the calibration table. */
    void enableAllTorque() {
        for (int i = 0; i < num_servos_; ++i) {
            servo_bus_.EnableTorque(calibration_[i].servo_id, 1);
        }
        RCLCPP_INFO(this->get_logger(), "Torque enabled on all servos");
    }

    /** @brief Create all ROS topic subscribers. */
    void createSubscribers() {
        // Main control topic: receives 12 position values in degrees
        sub_commands_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/servo_commands", 10,
            std::bind(&SerialDriverNode::onServoCommandsReceived, this, std::placeholders::_1));

        // Single-servo test topic: [servo_index, position_degrees]
        sub_single_command_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/servo_single_command", 10,
            std::bind(&SerialDriverNode::onSingleServoCommandReceived, this, std::placeholders::_1));

        // Feedback on/off toggle topic
        sub_feedback_toggle_ = this->create_subscription<std_msgs::msg::Bool>(
            "/feedback_enable", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data && feedback_timer_) {
                    feedback_timer_->cancel();
                    feedback_timer_.reset();
                    RCLCPP_INFO(this->get_logger(), "Feedback DISABLED via topic");
                }
            });
    }

    /** @brief Create the joint-state feedback publisher. */
    void createPublisher() {
        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states_real", 10);
    }

    /** @brief Start the periodic feedback timer if enabled via parameter. */
    void createFeedbackTimer() {
        if (!enable_feedback_) {
            return;
        }
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(feedback_ms_),
            std::bind(&SerialDriverNode::onFeedbackTimerTick, this));
        RCLCPP_INFO(this->get_logger(),
            "Feedback timer started (%d ms period)", feedback_ms_);
    }


    // ── Angle ↔ Tick Conversion ─────────────────────────────────────────

    /**
     * @brief Convert a joint angle in degrees to a servo tick value.
     * @param degrees Joint angle from IK (degrees)
     * @param cal     Calibration data for this joint
     * @return Clamped servo tick value (0–4095)
     */
    int16_t degreesToTicks(double degrees, const JointCalibration& cal) const {
        int32_t raw_tick = static_cast<int32_t>(
            cal.direction * degrees * kTicksPerDegree) + cal.tick_offset;

        raw_tick = std::clamp(raw_tick,
            static_cast<int32_t>(cal.tick_min),
            static_cast<int32_t>(cal.tick_max));

        return static_cast<int16_t>(raw_tick);
    }

    /**
     * @brief Convert a servo tick value to a joint angle in degrees.
     * @param tick Servo position tick (0–4095)
     * @param cal  Calibration data for this joint
     * @return Joint angle in degrees
     */
    double ticksToDegrees(int16_t tick, const JointCalibration& cal) const {
        return cal.direction * (tick - cal.tick_offset) * kDegreesPerTick;
    }


    // ── ROS Callbacks ───────────────────────────────────────────────────

    /**
     * @brief Handle incoming multi-servo position commands.
     *
     * The leg_controller publishes 12 values (θ1, θ2, θ3 for LF, LB, RF, RB)
     * in degrees. This callback converts each to servo ticks using the
     * calibration table and issues a single SyncWritePosEx command.
     */
    void onServoCommandsReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (static_cast<int>(msg->data.size()) < array_offset_ + num_servos_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Expected at least %d values (offset=%d, servos=%d), got %zu",
                array_offset_ + num_servos_, array_offset_, num_servos_,
                msg->data.size());
            return;
        }

        std::vector<uint8_t>  ids(num_servos_);
        std::vector<int16_t>  positions(num_servos_);
        std::vector<uint16_t> speeds(num_servos_, default_speed_);
        std::vector<uint8_t>  accs(num_servos_, default_acc_);

        for (int i = 0; i < num_servos_; ++i) {
            ids[i]       = calibration_[i].servo_id;
            positions[i] = degreesToTicks(msg->data[array_offset_ + i], calibration_[i]);
        }

        servo_bus_.SyncWritePosEx(
            ids.data(),
            static_cast<uint8_t>(num_servos_),
            positions.data(),
            speeds.data(),
            accs.data());
    }

    /**
     * @brief Handle incoming single-servo test commands.
     *
     * Message format: [servo_index, degrees].
     * servo_index is 0-based (index into the calibration table).
     */
    void onSingleServoCommandReceived(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(),
                "Single command needs [servo_index, degrees], got %zu values",
                msg->data.size());
            return;
        }

        int    servo_index   = static_cast<int>(msg->data[0]);
        double angle_degrees = msg->data[1];

        if (servo_index < 0 || servo_index >= num_servos_) {
            RCLCPP_WARN(this->get_logger(),
                "Servo index %d out of range [0, %d)", servo_index, num_servos_);
            return;
        }

        const auto& cal = calibration_[servo_index];
        int16_t tick    = degreesToTicks(angle_degrees, cal);

        servo_bus_.WritePosEx(cal.servo_id, tick, default_speed_, default_acc_);

        RCLCPP_DEBUG(this->get_logger(),
            "Single write: index=%d, ID=%d, %.1f° → tick %d",
            servo_index, cal.servo_id, angle_degrees, tick);
    }

    /**
     * @brief Periodic callback that reads all servo positions and publishes
     *        a JointState message on /joint_states_real.
     */
    void onFeedbackTimerTick() {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name         = joint_names_;
        joint_state_msg.position.resize(num_servos_);
        joint_state_msg.velocity.resize(num_servos_);
        joint_state_msg.effort.resize(num_servos_);

        for (int i = 0; i < num_servos_; ++i) {
            int pos  = servo_bus_.ReadPos(calibration_[i].servo_id);
            int load = servo_bus_.ReadLoad(calibration_[i].servo_id);

            if (pos != -1) {
                joint_state_msg.position[i] = ticksToDegrees(
                    static_cast<int16_t>(pos), calibration_[i]);
            } else {
                joint_state_msg.position[i] = 0.0;
            }

            joint_state_msg.velocity[i] = 0.0;  // Not read every cycle for speed
            joint_state_msg.effort[i]   = static_cast<double>(load);
        }

        pub_joint_states_->publish(joint_state_msg);
    }
};


// ═══════════════════════════════════════════════════════════════════════════
//  Main Entry Point
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<SerialDriverNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("serial_driver"),
            "Node terminated: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}