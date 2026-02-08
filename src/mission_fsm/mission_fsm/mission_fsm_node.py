#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum
import time
import signal
import sys

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger


# ───────────────────────── FSM STATES ─────────────────────────

class MissionState(Enum):
    IDLE = 0
    BOOT = 1
    STREAM_SETPOINTS = 2
    SET_OFFBOARD = 3
    ARM = 4
    TAKEOFF = 5
    HOVER = 6
    LAND = 7


# ───────────────────────── FSM NODE ─────────────────────────

class MissionFSM(Node):

    def __init__(self):
        super().__init__('mission_fsm')

        # FSM state
        self.state = MissionState.IDLE
        self.start_mission = False
        self.shutdown_requested = False

        self.setpoint_counter = 0
        self.hover_start_time = None

        # Takeoff parameters
        self.takeoff_altitude = 2.5
        self.takeoff_rate = 0.01
        self.current_altitude = 0.0

        # PX4 state
        self.current_state = State()
        self.battery = BatteryState()

        # Publisher
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Subscribers
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

        # MAVROS services
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        self.mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        # START service
        self.create_service(
            Trigger,
            '/start_mission',
            self.start_cb
        )

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.run_fsm)

        # Target pose (ground)
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.0

        # Register SIGINT handler (Ctrl+C)
        signal.signal(signal.SIGINT, self.sigint_handler)

        self.get_logger().info("Mission FSM ready — waiting for START")

    # ───────────────────────── CALLBACKS ─────────────────────────

    def state_cb(self, msg):
        self.current_state = msg

    def battery_cb(self, msg):
        self.battery = msg

    def start_cb(self, request, response):
        self.get_logger().info("START pressed")
        self.start_mission = True
        response.success = True
        response.message = "Mission started"
        return response

    def sigint_handler(self, signum, frame):
        self.get_logger().warn("Ctrl+C received → initiating safe LAND")
        self.shutdown_requested = True

    # ───────────────────────── FSM CORE ─────────────────────────

    def run_fsm(self):

        # Always stream setpoints
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target_pose)

        # Graceful shutdown handling
        if self.shutdown_requested:
            self.set_mode("LAND")
            return

        # ───────── IDLE ─────────
        if self.state == MissionState.IDLE:
            if self.start_mission:
                self.get_logger().info("IDLE → BOOT")
                self.state = MissionState.BOOT
            return

        # ───────── BOOT ─────────
        elif self.state == MissionState.BOOT:
            self.get_logger().info("BOOT → STREAM_SETPOINTS")
            self.state = MissionState.STREAM_SETPOINTS

        # ───────── STREAM SETPOINTS ─────────
        elif self.state == MissionState.STREAM_SETPOINTS:
            self.setpoint_counter += 1
            if self.setpoint_counter > 40:
                self.get_logger().info("SETPOINTS → SET_OFFBOARD")
                self.state = MissionState.SET_OFFBOARD

        # ───────── SET OFFBOARD ─────────
        elif self.state == MissionState.SET_OFFBOARD:
            self.set_mode("OFFBOARD")
            if self.current_state.mode == "OFFBOARD":
                self.get_logger().info("OFFBOARD → ARM")
                self.state = MissionState.ARM

        # ───────── ARM ─────────
        elif self.state == MissionState.ARM:
            if not self.current_state.armed:
                self.arm(True)
            else:
                self.get_logger().info("ARMED → TAKEOFF")
                self.state = MissionState.TAKEOFF

        # ───────── TAKEOFF ─────────
        elif self.state == MissionState.TAKEOFF:
            if self.current_altitude < self.takeoff_altitude:
                self.current_altitude += self.takeoff_rate
                self.target_pose.pose.position.z = self.current_altitude
            else:
                self.target_pose.pose.position.z = self.takeoff_altitude
                self.hover_start_time = time.time()
                self.get_logger().info("TAKEOFF → HOVER")
                self.state = MissionState.HOVER

        # ───────── HOVER ─────────
        elif self.state == MissionState.HOVER:
            if time.time() - self.hover_start_time > 10.0:
                self.get_logger().info("HOVER → LAND")
                self.state = MissionState.LAND

        # ───────── LAND ─────────
        elif self.state == MissionState.LAND:
            self.set_mode("LAND")
            if not self.current_state.armed:
                self.get_logger().info("Mission complete")
                rclpy.shutdown()

    # ───────────────────────── SERVICES ─────────────────────────

    def arm(self, value: bool):
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = value
        self.arming_client.call_async(req)

    def set_mode(self, mode: str):
        if not self.mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)


# ───────────────────────── MAIN ─────────────────────────

def main():
    rclpy.init()
    node = MissionFSM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
