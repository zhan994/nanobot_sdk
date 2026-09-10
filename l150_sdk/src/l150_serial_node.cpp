#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

const uint8_t FRAME_HEAD = 0x7B;
const uint8_t FRAME_TAIL = 0x7D;
const int SEND_SIZE = 11;
const int RECEIVE_SIZE = 24;

// 计算 BCC：把指定范围内的字节依次异或。
uint8_t calculateBcc(const uint8_t *data, int length) {
  uint8_t result = 0;
  for (int i = 0; i < length; ++i) {
    result ^= data[i];
  }
  return result;
}

// 将 int16_t 拆成“高字节 + 低字节”。
void int16ToBytes(int16_t value, uint8_t &high, uint8_t &low) {
  const uint16_t raw = static_cast<uint16_t>(value);
  high = static_cast<uint8_t>(raw >> 8);
  low = static_cast<uint8_t>(raw & 0xFF);
}

// 将“高字节 + 低字节”还原成 int16_t。
int16_t bytesToInt16(uint8_t high, uint8_t low) {
  const uint16_t raw = (static_cast<uint16_t>(high) << 8) | low;
  return (raw & 0x8000)
             ? static_cast<int16_t>(static_cast<int32_t>(raw) - 65536)
             : static_cast<int16_t>(raw);
}

// ROS 速度乘以 1000 后放入 int16_t，并防止数值溢出。
int16_t speedToProtocol(double speed) {
  long value = std::lround(speed * 1000.0);
  value = std::max(-32768L, std::min(32767L, value));
  return static_cast<int16_t>(value);
}

class L150SerialNode {
 public:
  L150SerialNode() : nh_(), private_nh_("~") {
    private_nh_.param("port", port_, std::string("/dev/ttyUSB0"));
    private_nh_.param("imu_frame", imu_frame_, std::string("imu_link"));
    private_nh_.param("command_timeout", command_timeout_, 0.5);

    if (!openSerial()) {
      ros::shutdown();
      return;
    }

    cmd_vel_sub_ =
        nh_.subscribe("cmd_vel", 10, &L150SerialNode::cmdVelCallback, this);

    velocity_pub_ =
        nh_.advertise<geometry_msgs::TwistStamped>("l150/velocity", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    battery_pub_ =
        nh_.advertise<std_msgs::Float32>("l150/battery_voltage", 10);
    stopped_pub_ = nh_.advertise<std_msgs::Bool>("l150/stopped", 10);

    // 每 0.05 秒执行一次发送和接收，即 20 Hz。
    timer_ = nh_.createTimer(ros::Duration(0.05),
                             &L150SerialNode::timerCallback, this);

    last_cmd_time_ = ros::Time::now();
    ROS_INFO("L150 node started, port=%s, baud=115200", port_.c_str());
    ROS_WARN("The car ignores commands for about 10 seconds after power-on");
  }

  ~L150SerialNode() {
    if (serial_fd_ < 0) {
      return;
    }
    // 停止定时器，防止继续发送旧的速度指令。
    timer_.stop();

    // 强制目标速度清零。
    target_x_ = 0.0;
    target_z_ = 0.0;

    // 连续发送多帧停止指令，降低单帧丢失风险。
    for (int i = 0; i < 3; ++i) {
      sendCommand();

      // 等待串口发送缓冲区中的数据真正发完。
      if (tcdrain(serial_fd_) != 0) {
        ROS_WARN("Failed to drain serial port while stopping: %s",
                strerror(errno));
      }

      usleep(20000);  // 间隔 20ms
    }

    close(serial_fd_);
    serial_fd_ = -1;
  }

 private:
  // 打开并配置串口：115200、8 数据位、无校验、1 停止位。
  bool openSerial() {
    //允许读写、不成为进程控制终端、非阻塞
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      ROS_ERROR("Cannot open %s: %s", port_.c_str(), strerror(errno));
      return false;
    }

    termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
      ROS_ERROR("Cannot read serial settings: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    cfmakeraw(&options); //原始模式
    cfsetispeed(&options, B115200); //输入输出波特率
    cfsetospeed(&options, B115200);
    options.c_cflag |= CLOCAL | CREAD; //忽略载波、启用接收
    options.c_cflag &= ~CSTOPB; //设置一个停止位
    options.c_cflag &= ~CRTSCTS; //关闭硬件流控
    options.c_cc[VMIN] = 0; //read() 立即返回，不等待任何字节，也不设置超时
    options.c_cc[VTIME] = 0;
    //TCSANOW：立即生效
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      ROS_ERROR("Cannot configure serial port: %s", strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    tcflush(serial_fd_, TCIOFLUSH); //清空串口缓存区
    return true;
  }

  // 收到 /cmd_vel 后，只保存需要的前进速度和转向速度。
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    target_x_ = msg->linear.x;
    target_z_ = msg->angular.z;
    last_cmd_time_ = ros::Time::now();
  }

  void timerCallback(const ros::TimerEvent &) {
    sendCommand();
    receiveData();
  }

  // 生成并发送 11 字节控制帧。
  void sendCommand() {
    double x = target_x_;
    double z = target_z_;

    // 长时间没有收到新指令时，发送零速度。
    if ((ros::Time::now() - last_cmd_time_).toSec() > command_timeout_) {
      x = 0.0;
      z = 0.0;
    }

    std::array<uint8_t, SEND_SIZE> frame{}; //创建发送帧，固定长度字节数组
    frame[0] = FRAME_HEAD;
    frame[1] = 0x00;  // 预留位
    frame[2] = 0x00;  // 预留位

    int16ToBytes(speedToProtocol(x), frame[3], frame[4]); //
    int16ToBytes(0, frame[5], frame[6]);  // L150 不支持 Y 轴平移
    // 底盘固件约定：正 z = 右转，与 ROS REP-103（正 z = 左转）相反，
    // 这里取反再下发，保证 /cmd_vel 遵循 REP-103。
    int16ToBytes(speedToProtocol(-z), frame[7], frame[8]);

    frame[9] = calculateBcc(frame.data(), 9);
    frame[10] = FRAME_TAIL;

    const ssize_t count = write(serial_fd_, frame.data(), frame.size()); //发送
    if (count < 0 && errno != EAGAIN && errno != EINTR) {
      ROS_ERROR_THROTTLE(1.0, "Serial write failed: %s", strerror(errno));
    } else if (count >= 0 && count != static_cast<ssize_t>(frame.size())) {
      ROS_WARN_THROTTLE(1.0, "Only part of the command frame was sent");
    }
  }

  // 读取串口。数据可能不足一帧，所以先放入接收缓存。
  void receiveData() {
    std::array<uint8_t, 256> new_data{};
    const ssize_t count =
        read(serial_fd_, new_data.data(), new_data.size());

    if (count > 0) {
      receive_buffer_.insert(receive_buffer_.end(), new_data.begin(),
                             new_data.begin() + count);
    } else if (count < 0 && errno != EAGAIN && errno != EINTR) {
      ROS_ERROR_THROTTLE(1.0, "Serial read failed: %s", strerror(errno));
      return;
    }

    parseReceiveBuffer();
  }

  // 从缓存中寻找并解析完整的 24 字节反馈帧。
  void parseReceiveBuffer() {
    while (true) {
      // 丢弃帧头 0x7B 之前的无效数据。
      const auto head = std::find(receive_buffer_.begin(),
                                  receive_buffer_.end(), FRAME_HEAD);
      receive_buffer_.erase(receive_buffer_.begin(), head);

      // 数据不足一帧，等待下一次接收。
      if (receive_buffer_.size() < RECEIVE_SIZE) {
        return;
      }

      const bool tail_ok = receive_buffer_[23] == FRAME_TAIL;
      const bool bcc_ok =
          calculateBcc(receive_buffer_.data(), 22) == receive_buffer_[22];

      if (tail_ok && bcc_ok) {
        publishFeedback(receive_buffer_.data());
        receive_buffer_.erase(receive_buffer_.begin(),
                              receive_buffer_.begin() + RECEIVE_SIZE);
      } else {
        // 当前帧错误，只删除帧头，然后继续寻找下一个帧头。
        receive_buffer_.erase(receive_buffer_.begin());
        ROS_WARN_THROTTLE(1.0, "Invalid L150 feedback frame");
      }
    }
  }

  // 将反馈帧转换成 ROS 消息并发布。
  void publishFeedback(const uint8_t *data) {
    const ros::Time now = ros::Time::now();

    geometry_msgs::TwistStamped velocity;
    velocity.header.stamp = now;
    velocity.twist.linear.x = bytesToInt16(data[2], data[3]) / 1000.0;
    velocity.twist.linear.y = bytesToInt16(data[4], data[5]) / 1000.0;
    // 与命令下发一致：底盘反馈正 z = 右转，取反为 REP-103（正 z = 左转）。
    velocity.twist.angular.z = -bytesToInt16(data[6], data[7]) / 1000.0;
    velocity_pub_.publish(velocity);

    sensor_msgs::Imu imu;
    imu.header.stamp = now;
    imu.header.frame_id = imu_frame_;
    imu.orientation_covariance[0] = -1.0;  // 协议中没有姿态数据
    imu.linear_acceleration.x = bytesToInt16(data[8], data[9]) / 1672.0; //16384 LSB/g，目标m/s² g=9.8
    imu.linear_acceleration.y = bytesToInt16(data[10], data[11]) / 1672.0;
    imu.linear_acceleration.z = bytesToInt16(data[12], data[13]) / 1672.0;
    imu.angular_velocity.x = bytesToInt16(data[14], data[15]) / 3753.0;//65.5 LSB/(°/s)  目标rad/s 65.5*180/pi
    imu.angular_velocity.y = bytesToInt16(data[16], data[17]) / 3753.0;
    imu.angular_velocity.z = bytesToInt16(data[18], data[19]) / 3753.0;
    imu_pub_.publish(imu);

    std_msgs::Float32 battery;
    battery.data = bytesToInt16(data[20], data[21]) / 1000.0;
    battery_pub_.publish(battery);

    std_msgs::Bool stopped;
    stopped.data = data[1] != 0;
    stopped_pub_.publish(stopped);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher velocity_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher battery_pub_;
  ros::Publisher stopped_pub_;
  ros::Timer timer_;

  std::string port_;
  std::string imu_frame_;
  double command_timeout_ = 0.5;
  double target_x_ = 0.0;
  double target_z_ = 0.0;
  ros::Time last_cmd_time_;

  int serial_fd_ = -1;
  std::vector<uint8_t> receive_buffer_;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "l150_serial_node");
  L150SerialNode node;
  ros::spin();
  return 0;
}
