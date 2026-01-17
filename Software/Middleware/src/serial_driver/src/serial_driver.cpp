#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> // Listening for Text now

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <sstream>
#include <iomanip>

class SerialDriver : public rclcpp::Node
{
public:
    SerialDriver() : Node("serial_driver_node")
    {
        // 1. Open Serial Port
        serial_port_ = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully opened /dev/ttyUSB1");
            configure_serial_port();
        }

        // 2. Subscriber: Listens for a STRING of Hex (e.g. "AA 55 FF")
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "serial_write_hex", 10, std::bind(&SerialDriver::topic_callback, this, std::placeholders::_1));
    }

    ~SerialDriver() { close(serial_port_); }

private:
    int serial_port_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void configure_serial_port()
    {
        struct termios tty;
        tcgetattr(serial_port_, &tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        
        tty.c_cflag &= ~PARENB; 
        tty.c_cflag &= ~CSTOPB; 
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;     
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
        tty.c_lflag &= ~ICANON; 
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ISIG;

        tcsetattr(serial_port_, TCSANOW, &tty);
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::vector<uint8_t> bytes_to_send;
        std::stringstream ss(msg->data);
        std::string segment;

        // 3. Parser: Split string by spaces and convert Hex to Byte
        while (std::getline(ss, segment, ' ')) {
            if (segment.empty()) continue;
            
            // strtoul converts string to unsigned long. 16 means Base-16 (Hex)
            try {
                uint8_t byte = (uint8_t)std::strtoul(segment.c_str(), nullptr, 16);
                bytes_to_send.push_back(byte);
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Invalid Hex: %s", segment.c_str());
                return;
            }
        }

        // 4. Send Raw Bytes
        if (!bytes_to_send.empty()) {
            ssize_t written = write(serial_port_, bytes_to_send.data(), bytes_to_send.size());
            if (written > 0) {
                RCLCPP_INFO(this->get_logger(), "Sent %ld bytes: %s", written, msg->data.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Write failed");
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialDriver>());
    rclcpp::shutdown();
    return 0;
}