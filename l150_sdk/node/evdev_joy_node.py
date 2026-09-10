#!/usr/bin/env python3

import select

import rospy
from evdev import InputDevice, ecodes
from sensor_msgs.msg import Joy


def code_name(event_type, code):
    name = ecodes.bytype.get(event_type, {}).get(code, str(code))
    return "/".join(name) if isinstance(name, list) else name


class EvdevJoyNode:
    def __init__(self):
        device_path = rospy.get_param("~device", "/dev/input/event0")
        self.deadzone = max(0.0, min(0.99, rospy.get_param("~deadzone", 0.1)))
        self.publish_rate = max(1.0, rospy.get_param("~publish_rate", 20.0))
        self.device = InputDevice(device_path)

        capabilities = self.device.capabilities(absinfo=True)
        absolute_axes = capabilities.get(ecodes.EV_ABS, [])
        keys = capabilities.get(ecodes.EV_KEY, [])

        self.axis_codes = sorted(code for code, _ in absolute_axes)
        self.axis_info = {code: info for code, info in absolute_axes}
        self.axis_index = {
            code: index for index, code in enumerate(self.axis_codes)
        }

        # 忽略 event0 可能包含的键盘键，只发布 BTN_* 范围内的手柄按键。
        self.button_codes = sorted(code for code in keys if code >= ecodes.BTN_MISC)
        self.button_index = {
            code: index for index, code in enumerate(self.button_codes)
        }

        if not self.axis_codes:
            raise RuntimeError("{} has no absolute axes".format(device_path))

        self.message = Joy()
        self.message.axes = [0.0] * len(self.axis_codes)
        self.message.buttons = [0] * len(self.button_codes)
        self.publisher = rospy.Publisher("joy", Joy, queue_size=10)

        rospy.loginfo("Using evdev joystick: %s", self.device.name)
        rospy.loginfo(
            "Joy axes: %s",
            ", ".join(
                "{}={}".format(index, code_name(ecodes.EV_ABS, code))
                for index, code in enumerate(self.axis_codes)
            ),
        )
        rospy.loginfo(
            "Joy buttons: %s",
            ", ".join(
                "{}={}".format(index, code_name(ecodes.EV_KEY, code))
                for index, code in enumerate(self.button_codes)
            ),
        )

    def normalize_axis(self, code, value):
        info = self.axis_info[code]
        minimum = float(info.min)
        maximum = float(info.max)
        if maximum <= minimum:
            return 0.0

        center = (minimum + maximum) / 2.0
        normalized = -(float(value) - center) / ((maximum - minimum) / 2.0)
        normalized = max(-1.0, min(1.0, normalized))
        return 0.0 if abs(normalized) < self.deadzone else normalized

    def update(self, event):
        if event.type == ecodes.EV_ABS and event.code in self.axis_index:
            self.message.axes[self.axis_index[event.code]] = self.normalize_axis(
                event.code, event.value
            )
        elif event.type == ecodes.EV_KEY and event.code in self.button_index:
            self.message.buttons[self.button_index[event.code]] = (
                1 if event.value else 0
            )

    def run(self):
        timeout = 1.0 / self.publish_rate
        while not rospy.is_shutdown():
            try:
                readable, _, _ = select.select([self.device.fd], [], [], timeout)
                if readable:
                    for event in self.device.read():
                        self.update(event)
            except BlockingIOError:
                pass
            except OSError as error:
                rospy.logerr("Joystick disconnected: %s", error)
                return

            self.message.header.stamp = rospy.Time.now()
            self.publisher.publish(self.message)


if __name__ == "__main__":
    rospy.init_node("evdev_joy")
    try:
        EvdevJoyNode().run()
    except (RuntimeError, FileNotFoundError, PermissionError, OSError) as error:
        rospy.logfatal("Cannot start evdev joystick: %s", error)
