#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import BatteryState


class EmergencyManager(Node):

    def __init__(self):
        super().__init__('emergency_manager')

        self.current_state = State()
        self.battery = BatteryState()
        self.emergency_triggered = False

        self.low_battery_threshold = 0.25  # 25%

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_cb,
            10
        )

        self.mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        self.timer = self.create_timer(0.1, self.monitor)

        self.get_logger().info("Emergency Manager running")

    def state_cb(self, msg):
        self.current_state = msg

    def battery_cb(self, msg):
        self.battery = msg

    def monitor(self):
        if self.emergency_triggered:
            return

        if not self.current_state.connected:
            self.trigger_emergency("MAVROS disconnected")
            return

        if self.current_state.armed and self.current_state.mode != "OFFBOARD":
            self.trigger_emergency("OFFBOARD lost")
            return

        if self.battery.percentage > 0 and self.battery.percentage < self.low_battery_threshold:
            self.trigger_emergency("Low battery")
            return

    def trigger_emergency(self, reason):
        self.emergency_triggered = True
        self.get_logger().warn(f"EMERGENCY: {reason} → LAND")

        if not self.mode_client.service_is_ready():
            self.get_logger().error("SetMode service not available")
            return

        req = SetMode.Request()
        req.custom_mode = "LAND"
        self.mode_client.call_async(req)


def main():
    rclpy.init()
    node = EmergencyManager()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
