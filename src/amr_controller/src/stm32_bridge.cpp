#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>

// ============================================================
//  AMR - STM32 Bridge Node
//  Steering Mode : Ackermann 2WS (front 2 wheels only)
//  Servo Output  : 1 signal (mechanically synchronized)
//  Author        : Muhammad Al Azhar Faradis
//  Institution   : Automation Engineering, ITS Surabaya
//
//  Features:
//  - Publishes /joy commands to STM32 via USB Serial
//  - Reads encoder feedback from STM32 (E:{delta}\n)
//  - Publishes encoder data to /encoder topic
//  - Uses stable port /dev/serial/by-id/ to avoid port changes
// ============================================================

// --- IMPORTANT: Change this to your STM32 serial ID ---
// Run: ls -l /dev/serial/by-id/
// Then copy the full name of the STM32 Virtual ComPort entry
#define SERIAL_PORT  "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_206833894152-if00"
#define BAUD_RATE    B115200
#define MAX_PWM      4000             // Max PWM value for traction motors
#define MAX_STEER    45
#define STEER_TRIM   -5               // Max steering angle in degrees
#define DEADMAN_BTN  5                // R1 button on PS4/PS5 DualShock Bluetooth joystick
#define AXIS_VEL     1                // Left analog stick (up/down) -> velocity
#define AXIS_STEER   3                // Right analog stick (left/right) -> steering
// NOTE: Joystick is PS4/PS5 DualShock via Bluetooth (MAC 8C:41:F2:D6:9D:7F).
// Verified Day 2: detected as "Wireless Controller" by Linux kernel.
// Button/axis mapping is compatible with PS4 BT default layout.

class STM32Bridge : public rclcpp::Node
{
public:
  STM32Bridge() : Node("stm32_bridge"), serial_fd_(-1), running_(true)
  {
    // Open serial port to STM32
    serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(),
        "[ERROR] Failed to open serial port!\n"
        "  Run: ls -l /dev/serial/by-id/\n"
        "  Then update SERIAL_PORT in stm32_bridge.cpp");
    } else {
      configure_serial(serial_fd_);
      RCLCPP_INFO(this->get_logger(), "[OK] STM32 connected!");
    }

    // Subscribe to joystick topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&STM32Bridge::joy_callback, this, std::placeholders::_1));

    // Publisher: encoder feedback -> /encoder topic
    encoder_pub_ = this->create_publisher<std_msgs::msg::Int32>("/encoder", 10);

    // Start encoder reader thread (runs in parallel)
    if (serial_fd_ >= 0) {
      read_thread_ = std::thread(&STM32Bridge::read_encoder_loop, this);
    }

    RCLCPP_INFO(this->get_logger(),
      "[OK] Ready! Hold R1 + move analog sticks to drive the robot.");
    RCLCPP_INFO(this->get_logger(),
      "[INFO] Steering mode: Ackermann 2WS - front 2 wheels only");
    RCLCPP_INFO(this->get_logger(),
      "[INFO] Encoder feedback publishing to /encoder");
  }

  ~STM32Bridge()
  {
    // Stop encoder reader thread
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }

    // Stop motors and close serial port
    if (serial_fd_ >= 0) {
      send_command(0, 0);
      close(serial_fd_);
      RCLCPP_INFO(this->get_logger(), "[OK] Motors stopped. Serial port closed.");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_pub_;
  std::thread read_thread_;
  int serial_fd_;
  bool running_;

  // --------------------------------------------------
  // Joystick callback: reads /joy and sends to STM32
  // --------------------------------------------------
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[WARN] Serial port not open! Connect STM32 USB cable.");
      return;
    }

    // Deadman switch: R1 must be held for robot to move (safety)
    bool deadman = (msg->buttons.size() > DEADMAN_BTN &&
                    msg->buttons[DEADMAN_BTN] == 1);
    if (!deadman) {
      send_command(0, 0);
      return;
    }

    float vel_raw   = (msg->axes.size() > AXIS_VEL)
                      ? msg->axes[AXIS_VEL]   : 0.0f;
    float steer_raw = (msg->axes.size() > AXIS_STEER)
                      ? msg->axes[AXIS_STEER] : 0.0f;

    // Negate velocity: analog up (+1.0) = forward on hardware
    int velocity = static_cast<int>(vel_raw   * -MAX_PWM);

    // Negate steering: analog right (+1.0) = turn right on hardware
    int steering = static_cast<int>(steer_raw * -MAX_STEER) + STEER_TRIM;

    // Clamp values to valid range
    velocity = std::max(-MAX_PWM,  std::min(MAX_PWM,  velocity));
    steering = std::max(-MAX_STEER, std::min(MAX_STEER, steering));

    send_command(velocity, steering);
  }

  // --------------------------------------------------
  // Send command to STM32
  // Format: "V:2000,S:30\n"
  // --------------------------------------------------
  void send_command(int velocity, int steering)
  {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "V:%d,S:%d\n", velocity, steering);
    ssize_t n = write(serial_fd_, buffer, strlen(buffer));
    if (n < 0) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to write to serial port!");
    } else {
      RCLCPP_INFO(this->get_logger(), "[TX] %s", buffer);
    }
  }

  // --------------------------------------------------
  // Encoder reader thread
  // Reads "E:{delta}\n" from STM32 continuously
  // Publishes delta to /encoder topic
  // --------------------------------------------------
  void read_encoder_loop()
  {
    char line[128];
    int  line_pos = 0;

    while (running_ && serial_fd_ >= 0) {
      char c;
      ssize_t n = read(serial_fd_, &c, 1);
      if (n <= 0) continue;

      if (c == '\n') {
        line[line_pos] = '\0';
        line_pos = 0;

        // Parse encoder format: "E:{delta}"
        int delta = 0;
        if (sscanf(line, "E:%d", &delta) == 1) {
          auto msg = std_msgs::msg::Int32();
          msg.data = delta;
          encoder_pub_->publish(msg);
          RCLCPP_DEBUG(this->get_logger(), "[RX] Encoder delta: %d", delta);
        }
      } else {
        if (line_pos < 127) line[line_pos++] = c;
      }
    }
  }

  // --------------------------------------------------
  // Configure serial port settings
  // --------------------------------------------------
  void configure_serial(int fd)
  {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit characters
    tty.c_cflag |= (CLOCAL | CREAD);               // Enable receiver
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS); // No parity, no flow control
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag  = 0;   // No echo, no canonical mode
    tty.c_oflag  = 0;   // No output processing
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tcsetattr(fd, TCSANOW, &tty);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<STM32Bridge>());
  rclcpp::shutdown();
  return 0;
}
