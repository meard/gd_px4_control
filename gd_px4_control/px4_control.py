#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import Float32, Int16


# utility functions to transform different PX4 and ROS frames
def px4_2_ros(position_px4):
    # first a pi/2 rotation around the Z-axis (down),
    rotation_matrix_z = np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]
    ])
    # then a pi rotation around the X-axis (old North/new East). Note that the two resulting operations are mathematically equivalent.
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])

    # rotation
    position_temp = np.dot(rotation_matrix_z, position_px4)
    position_ros = np.dot(rotation_matrix_x, position_temp)
    return position_ros


def ros_2_px4(position_ros):
    position_px4 = px4_2_ros(position_ros)
    # Note that the two resulting operations are mathematically equivalent.
    return position_px4


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_px4_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.next_search_position_subscriber = self.create_subscription(
            PoseStamped, '/fv/next_search_position', self.next_search_position_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.next_search_position = np.array([0.0, 0.0, -self.takeoff_height])
        self.search_length = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # mini state machine
        self.search = True
        self.airborne = False

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        pose_msg = PoseStamped()
        # TODO: get timestamp from PX4 massage
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'  # Adjust frame_id to your needs

        # TODO: rotation between PX4 and ROS https://docs.px4.io/main/en/ros/ros2_comm.html#ros-2-px4-frame-conventions
        position_px4 = np.array(
            [vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z])
        position_ros = px4_2_ros(position_px4)

        # TODO: integrate uncertainty
        pose_msg.pose.position.x = position_ros[0]
        pose_msg.pose.position.y = position_ros[1]
        pose_msg.pose.position.z = position_ros[2]

        # TODO: integrate attitude
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.drone_position_publisher.publish(pose_msg)

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def next_search_position_callback(self, next_search_position):
        """Callback function for next_search_position topic subscriber."""
        self.next_search_position = np.array(
            [next_search_position.pose.position.x, next_search_position.pose.position.y, next_search_position.pose.position.z])

    def search_length_callback(self, search_length):
        """Callback function for next_search_position topic subscriber."""
        self.search_length = search_length.data

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """ 
            Publish a vehicle command.
            https://docs.px4.io/main/en/msg_docs/VehicleCommand.html#vehiclecommand-uorb-message 
            Vehicle Command uORB message. Used for commanding a mission / action / etc. 
            Follows the MAVLink COMMAND_INT / COMMAND_LONG definition
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        # self.get_logger().info(('self.vehicle_local_position.z %d' % (self.vehicle_local_position.z)))

        # arms the drone with a delay of 10*0.1 sec
        if self.offboard_setpoint_counter == 10:
            self.get_logger().info(('ARM'))
            self.engage_offboard_mode()
            self.arm()

        # not airborn -> take off
        if (not self.airborne) and self.search and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # self.get_logger().info(('TAKEOFF'))
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

            # reached hover attiture = airborn
            if (np.abs(self.takeoff_height - self.vehicle_local_position.z) <= 1.0):
                self.airborne = True
                self.get_logger().info(('AIRBORNE'))

        # airborn and search -> search
        if self.search and self.airborne and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # self.get_logger().info(('SEARCHING'))
            position_ros = self.next_search_position
            px4_pos = ros_2_px4(position_ros)
            self.publish_position_setpoint(px4_pos[0], px4_pos[1], px4_pos[2])

        if self.search_length > 5:
            self.search = False

        # search finished but airborne and not above landing -> retun home
        if (not self.search) and self.airborne and ((np.abs(self.vehicle_local_position.x) >= 1.0) or (np.abs(self.vehicle_local_position.y) >= 1.0)) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info(('RETURN HOME'))
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        # search finished, airborne and above landing -> land
        if (not self.search) and self.airborne and (np.abs(self.vehicle_local_position.x) <= 1.0) and (np.abs(self.vehicle_local_position.y) <= 1.0):
            self.get_logger().info(('LAND'))
            self.land()
            exit(0)

        # only relevant for delay???
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
