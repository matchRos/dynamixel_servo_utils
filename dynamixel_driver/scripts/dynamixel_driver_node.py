#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
from std_msgs.msg import Float32, String
from dynamixel_sdk import PortHandler, PacketHandler


class DynamixelDriverNode:
    def __init__(self):
        # Parameters
        self.device_name = rospy.get_param("~device_name", "/dev/ttyUSB1")
        self.baudrate = rospy.get_param("~baudrate", 57600)
        self.protocol_version = rospy.get_param("~protocol_version", 2.0)
        self.dxl_id = rospy.get_param("~dxl_id", 1)
        self.read_rate = rospy.get_param("~read_rate", 20.0)

        self.max_current_ma = rospy.get_param("~max_current_ma", 50.0)
        self.kp_rpm_to_current = rospy.get_param("~kp_rpm_to_current", 2.0)

        # XL330 registers
        self.addr_operating_mode = 11
        self.addr_torque_enable = 64
        self.addr_goal_current = 102
        self.addr_goal_velocity = 104
        self.addr_present_current = 126
        self.addr_present_velocity = 128

        self.mode_current = 0
        self.mode_velocity = 1

        self.torque_enable = 1
        self.torque_disable = 0

        # Internal state
        self.current_mode = None
        self.is_shutting_down = False
        self.dxl_lock = threading.Lock()

        self.last_present_current_ma = 0.0
        self.last_present_rpm = 0.0

        self.active_mode_name = "unknown"

        self.goal_rpm_limited_active = False
        self.goal_rpm_limited = 0.0

        # SDK
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        # Publishers
        self.pub_present_current = rospy.Publisher("/present_current", Float32, queue_size=10)
        self.pub_present_rpm = rospy.Publisher("/present_rpm", Float32, queue_size=10)
        self.pub_active_mode = rospy.Publisher("/active_mode", String, queue_size=10)

        # Subscribers
        self.sub_goal_current = rospy.Subscriber("/goal_current", Float32, self.goal_current_cb)
        self.sub_goal_rpm = rospy.Subscriber("/goal_rpm", Float32, self.goal_rpm_cb)
        self.sub_goal_rpm_limited = rospy.Subscriber("/goal_rpm_limited", Float32, self.goal_rpm_limited_cb)
        self.sub_max_current = rospy.Subscriber("/max_current", Float32, self.max_current_cb)

        self.init_dynamixel()

    def dxl_to_signed(self, val, bits):
        if val >= (1 << (bits - 1)):
            val -= (1 << bits)
        return val

    def signed_to_dxl(self, val, bits):
        if val < 0:
            val += (1 << bits)
        return val

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def check_result(self, dxl_comm_result, dxl_error, action_name):
        if dxl_comm_result != 0:
            rospy.logerr("%s failed: %s", action_name, self.packet_handler.getTxRxResult(dxl_comm_result))
            return False
        if dxl_error != 0:
            rospy.logerr("%s DXL error: %s", action_name, self.packet_handler.getRxPacketError(dxl_error))
            return False
        return True

    def init_dynamixel(self):
        with self.dxl_lock:
            if not self.port_handler.openPort():
                rospy.logfatal("Failed to open port: %s", self.device_name)
                raise RuntimeError("Failed to open port")

            rospy.loginfo("Opened port: %s", self.device_name)

            if not self.port_handler.setBaudRate(self.baudrate):
                rospy.logfatal("Failed to set baudrate: %d", self.baudrate)
                raise RuntimeError("Failed to set baudrate")

            rospy.loginfo("Set baudrate: %d", self.baudrate)

            mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_operating_mode
            )
            if not self.check_result(dxl_comm_result, dxl_error, "Read operating mode"):
                raise RuntimeError("Failed to read operating mode")

            self.current_mode = mode
            self.active_mode_name = self.mode_to_name(mode)
            rospy.loginfo("Initial operating mode: %d", self.current_mode)

            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_enable
            )
            if not self.check_result(dxl_comm_result, dxl_error, "Enable torque"):
                raise RuntimeError("Failed to enable torque")

            rospy.loginfo("Torque enabled")

    def mode_to_name(self, mode):
        if mode == self.mode_current:
            return "current"
        if mode == self.mode_velocity:
            return "velocity"
        return str(mode)

    def set_operating_mode(self, new_mode):
        with self.dxl_lock:
            if self.current_mode == new_mode:
                return True

            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_disable
            )
            if not self.check_result(dxl_comm_result, dxl_error, "Disable torque for mode switch"):
                return False

            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_operating_mode, new_mode
            )
            if not self.check_result(dxl_comm_result, dxl_error, "Write operating mode"):
                return False

            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_enable
            )
            if not self.check_result(dxl_comm_result, dxl_error, "Re-enable torque after mode switch"):
                return False

            self.current_mode = new_mode
            self.active_mode_name = self.mode_to_name(new_mode)
            rospy.loginfo("Switched operating mode to %s", self.active_mode_name)
            return True

    def write_goal_current_ma(self, current_ma):
        current_ma_int = int(round(current_ma))
        current_raw = self.signed_to_dxl(current_ma_int, 16)

        with self.dxl_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_goal_current, current_raw
            )
        return self.check_result(dxl_comm_result, dxl_error, "Write goal current")

    def write_goal_velocity_rpm(self, rpm):
        velocity_raw_signed = int(round(rpm / 0.229))
        velocity_raw = self.signed_to_dxl(velocity_raw_signed, 32)

        with self.dxl_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_goal_velocity, velocity_raw
            )
        return self.check_result(dxl_comm_result, dxl_error, "Write goal velocity")

    def goal_current_cb(self, msg):
        if self.is_shutting_down:
            return

        self.goal_rpm_limited_active = False

        if not self.set_operating_mode(self.mode_current):
            return

        goal_current_ma = msg.data
        goal_current_ma = self.clamp(goal_current_ma, -self.max_current_ma, self.max_current_ma)
        self.write_goal_current_ma(goal_current_ma)

    def goal_rpm_cb(self, msg):
        if self.is_shutting_down:
            return

        self.goal_rpm_limited_active = False

        if not self.set_operating_mode(self.mode_velocity):
            return

        self.write_goal_velocity_rpm(msg.data)

    def goal_rpm_limited_cb(self, msg):
        if self.is_shutting_down:
            return

        self.goal_rpm_limited = msg.data
        self.goal_rpm_limited_active = True

        if not self.set_operating_mode(self.mode_current):
            return

        self.active_mode_name = "rpm_limited"

    def max_current_cb(self, msg):
        if self.is_shutting_down:
            return

        self.max_current_ma = abs(msg.data)
        rospy.loginfo("Updated max_current_ma: %.2f", self.max_current_ma)

        if self.goal_rpm_limited_active:
            if not self.set_operating_mode(self.mode_current):
                return
            self.apply_limited_rpm_control()

    def read_present_state(self):
        with self.dxl_lock:
            present_current_raw, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_present_current
            )
        if self.check_result(dxl_comm_result, dxl_error, "Read present current"):
            self.last_present_current_ma = float(self.dxl_to_signed(present_current_raw, 16))

        with self.dxl_lock:
            present_velocity_raw, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.dxl_id, self.addr_present_velocity
            )
        if self.check_result(dxl_comm_result, dxl_error, "Read present velocity"):
            present_velocity_signed = self.dxl_to_signed(present_velocity_raw, 32)
            self.last_present_rpm = float(present_velocity_signed) * 0.229

    def apply_limited_rpm_control(self):
        if not self.goal_rpm_limited_active:
            return

        rpm_error = self.goal_rpm_limited - self.last_present_rpm
        commanded_current = self.kp_rpm_to_current * rpm_error
        commanded_current = self.clamp(
            commanded_current,
            -self.max_current_ma,
            self.max_current_ma,
        )

        self.write_goal_current_ma(commanded_current)

    def read_and_publish(self):
        if self.is_shutting_down:
            return

        self.read_present_state()

        if self.goal_rpm_limited_active:
            if self.current_mode != self.mode_current:
                if not self.set_operating_mode(self.mode_current):
                    return
            self.active_mode_name = "rpm_limited"
            self.apply_limited_rpm_control()
        else:
            self.active_mode_name = self.mode_to_name(self.current_mode)

        self.pub_present_current.publish(self.last_present_current_ma)
        self.pub_present_rpm.publish(self.last_present_rpm)
        self.pub_active_mode.publish(self.active_mode_name)

    def shutdown(self):
        self.is_shutting_down = True
        rospy.loginfo("Shutting down Dynamixel driver node")

        with self.dxl_lock:
            try:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, self.dxl_id, self.addr_goal_current, 0
                )
            except Exception as e:
                rospy.logwarn("Could not reset goal current: %s", str(e))

            try:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, self.dxl_id, self.addr_goal_velocity, 0
                )
            except Exception as e:
                rospy.logwarn("Could not reset goal velocity: %s", str(e))

            try:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, self.dxl_id, self.addr_torque_enable, self.torque_disable
                )
            except Exception as e:
                rospy.logwarn("Could not disable torque: %s", str(e))

            try:
                self.port_handler.closePort()
            except Exception as e:
                rospy.logwarn("Could not close port: %s", str(e))

        rospy.loginfo("Torque disabled and port closed")


if __name__ == "__main__":
    rospy.init_node("dynamixel_driver_node")

    node = DynamixelDriverNode()
    rospy.on_shutdown(node.shutdown)

    rate = rospy.Rate(node.read_rate)
    while not rospy.is_shutdown():
        node.read_and_publish()
        rate.sleep()