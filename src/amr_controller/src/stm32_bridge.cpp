#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cmath>
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
#define WHEELBASE    0.5f             // m, jarak sumbu roda (Ackermann)
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

    // ---- Autonomous mode (cmd_vel) parameters ----
    // autonomous_enabled : false = perilaku lama (joystick-only).
    //   Aktifkan runtime: ros2 param set /stm32_bridge autonomous_enabled true
    // max_speed_mps      : kecepatan (m/s) yang dipetakan ke MAX_PWM.
    //   KALIBRASI di lapangan! cmd_vel 0.3 m/s -> PWM = 0.3/max_speed*4000.
    // cmd_vel_timeout_ms : watchdog — tanpa cmd_vel baru selama ini, STOP.
    this->declare_parameter("autonomous_enabled", false);
    this->declare_parameter("max_speed_mps", 1.0);
    this->declare_parameter("cmd_vel_timeout_ms", 500);

    // Subscribe to joystick topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&STM32Bridge::joy_callback, this, std::placeholders::_1));

    // Subscribe to cmd_vel (Nav2 / autonomous) — aktif hanya jika
    // autonomous_enabled=true DAN R1 tidak ditekan (R1 = manual override)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&STM32Bridge::cmd_vel_callback, this, std::placeholders::_1));

    // Watchdog 100ms: stop motor jika cmd_vel berhenti datang (autonomous)
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&STM32Bridge::watchdog_check, this));
    last_cmd_vel_time_ = this->now();

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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_pub_;
  rclcpp::Time last_cmd_vel_time_;
  std::thread read_thread_;
  int serial_fd_;
  bool running_;
  bool manual_override_ = false;   // true saat R1 ditekan (joystick pegang kendali)
  bool autonomous_active_ = false; // true saat cmd_vel sedang menggerakkan robot

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
    manual_override_ = deadman;
    if (!deadman) {
      // R1 lepas: kalau autonomous sedang aktif, JANGAN kirim stop di sini —
      // biarkan cmd_vel yang pegang kendali (watchdog tetap menjaga).
      if (!autonomous_active_) {
        send_command(0, 0);
      }
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
  // cmd_vel callback: jalur AUTONOMOUS (Nav2 / patrol)
  // Konversi Twist -> (PWM, steering deg) kinematika Ackermann:
  //   steer = atan(WHEELBASE * angular.z / linear.x)
  // Aktif hanya jika autonomous_enabled=true dan R1 TIDAK ditekan.
  // --------------------------------------------------
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (serial_fd_ < 0) return;

    bool enabled = this->get_parameter("autonomous_enabled").as_bool();
    if (!enabled || manual_override_) {
      autonomous_active_ = false;
      return;  // joystick (R1) selalu menang
    }

    last_cmd_vel_time_ = this->now();

    double v = msg->linear.x;    // m/s
    double w = msg->angular.z;   // rad/s
    double max_speed = this->get_parameter("max_speed_mps").as_double();
    if (max_speed < 0.1) max_speed = 0.1;

    int velocity = static_cast<int>((v / max_speed) * MAX_PWM);

    // Ackermann: butuh kecepatan maju untuk belok (tidak bisa putar di tempat)
    int steering = STEER_TRIM;
    if (std::fabs(v) > 0.05) {
      double steer_rad = std::atan(WHEELBASE * w / v);
      // Konvensi REP-103: +angular.z = belok kiri. Di hardware ini nilai
      // steering POSITIF = kiri (lihat negasi joystick di joy_callback).
      // VERIFIKASI sekali di bench: roda harus belok kiri saat angular.z > 0.
      steering = static_cast<int>(steer_rad * 180.0 / M_PI) + STEER_TRIM;
    }

    velocity = std::max(-MAX_PWM,   std::min(MAX_PWM,   velocity));
    steering = std::max(-MAX_STEER, std::min(MAX_STEER, steering));

    autonomous_active_ = (velocity != 0);
    send_command(velocity, steering);
  }

  // --------------------------------------------------
  // Watchdog: kalau autonomous aktif tapi cmd_vel berhenti
  // datang > timeout, STOP motor (cegah robot kabur saat
  // Nav2/ROS mati mendadak)
  // --------------------------------------------------
  void watchdog_check()
  {
    if (!autonomous_active_ || manual_override_) return;
    int timeout_ms = this->get_parameter("cmd_vel_timeout_ms").as_int();
    auto elapsed_ms =
      (this->now() - last_cmd_vel_time_).nanoseconds() / 1000000;
    if (elapsed_ms > timeout_ms) {
      autonomous_active_ = false;
      send_command(0, 0);
      RCLCPP_WARN(this->get_logger(),
        "[WATCHDOG] cmd_vel timeout (%ld ms) — motor STOP.", elapsed_ms);
    }
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
