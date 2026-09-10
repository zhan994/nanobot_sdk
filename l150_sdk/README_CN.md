# L150 ROS SDK

`l150_sdk` 将原来的 `l150_base`、`l150_bringup` 和 `l150_odometry`
合并为一个 ROS1 catkin 包，统一提供底盘串口、手柄遥控和轮式里程计功能。

## 目录

```text
l150_sdk/
├── config/                 # 串口与里程计参数
├── launch/                 # 分模块及总启动文件
├── node/                   # Python 节点
├── src/                    # C++ 节点
├── CMakeLists.txt
├── package.xml
├── README.md
└── README_CN.md
```

## 编译

```bash
cd <catkin_ws>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 启动

启动串口、里程计和手柄：

```bash
roslaunch l150_sdk l150_bringup.launch
```

实机自动控制时通常关闭手柄并启动外接 IMU：

```bash
roslaunch l150_sdk l150_bringup.launch \
  start_joystick:=false start_imu:=true
```

也可单独启动：

```bash
roslaunch l150_sdk l150_serial.launch port:=/dev/l150
roslaunch l150_sdk l150_odometry.launch start_imu:=true
roslaunch l150_sdk l150_joystart.launch
```

## ROS 接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |
| 发布 | `/l150/velocity` | `geometry_msgs/TwistStamped` | 底盘速度反馈 |
| 发布 | `/wheel_odom` | `nav_msgs/Odometry` | 积分轮式里程计 |
| 发布 | `/imu/data_raw` | `sensor_msgs/Imu` | 底盘板载 IMU 原始数据 |
| 发布 | `/l150/battery_voltage` | `std_msgs/Float32` | 电池电压 |
| 发布 | `/l150/stopped` | `std_msgs/Bool` | 电机失能状态 |


