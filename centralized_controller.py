import rclpy
import argparse
from rclpy.node import Node
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleOdometry,
    VehicleAttitude
)
from std_msgs.msg import Float32MultiArray, Float32,Bool
import numpy as np
from math import cos, exp, pi, sin
import time

import coverage_test


way = coverage_test.planned_path_in_1st_quad
print(way)

initial_drone_way = [x for x in way if way.index(x)%2 != 0]
angles = []

for i in range(len(initial_drone_way) - 1):
    x1, y1 = initial_drone_way[i]
    x2, y2 = initial_drone_way[i + 1]
    angle = math.atan2(y2 - y1, x2 - x1)
    angles.append(angle)

# Append 0 for the last element
angles.append(0)

# Combine the points with their corresponding angles
drone_way = [(initial_drone_way[i][0], initial_drone_way[i][1], angles[i]) for i in range(len(initial_drone_way))]

def R_x(theta):
    R_phi = np.vstack(
        ([1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)])
    )

    return R_phi


def R_y(theta):
    R_theta = np.vstack(
        ([cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)])
    )

    return R_theta


def R_z(theta):
    R_psi = np.vstack(
        ([cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1])
    )

    return R_psi


def euler_from_quaternion(q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        w,x,y,z = q

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def invert_quaternion(q):
    return [q[0], -q[1], -q[2], -q[3]]

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, takeoff, no_ibvs) -> None:
        super().__init__("offboard_control_takeoff_and_land")
        self._logger = self.get_logger()

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.DO_TAKEOFF = takeoff
        self.NO_IBVS = no_ibvs

        self.error = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        self.velocity = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        self.vehicle_odom = VehicleOdometry()
        self.estimated_velocity = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        self.cu = 640 / 2
        self.cv = 480 / 2
        self.f = 215.4
        # pt_star
        # 693.9468994140625
        # 267.8990173339844
        # 1233.7037353515625
        # 268.4234313964844
        # 1233.422119140625
        # 808.4156494140625
        # 693.327880859375
        # 807.75146484375
        self.h_d = 0.236
        self.error = [float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf')]
        self.pt_star = np.array(
            [
                179.2055,
                99.2273,
                459.6814,
                101.1491,
                457.6014,
                381.6531,
                177.1040,
                379.4215,
            ],
            dtype=np.float32,
        )

        self.K_near = [float(0.3), float(0.3), float(0.15)]
        self.K=self.K_far = [float(0.053), float(0.053), float(0.15)]
        self.gain_switching_threshold = 2.0
        self.marker_visibility = False
        self.SATURATION_XY = 0.6
        self.SATURATION_Z = 0.5
        self.SATURATION_YAWRATE = 3
        self.k_yaw = 1
        self.VQ = np.array([0, 0, 0, 0, 0, 0])
        self.VI = np.array(
            [float(0), float(0), float(0), float(0), float(0), float(0)],
            dtype=np.float32,
        )

        self.offboard_msg = OffboardControlMode()
        self.offboard_msg.position = False
        self.offboard_msg.velocity = True
        self.offboard_msg.acceleration = False
        self.offboard_msg.attitude = False
        self.offboard_msg.body_rate = False
        self.q = np.array([0, 0, 0, 0], dtype=np.float32)
        self.error_out = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )
        self.error_pub = self.create_publisher(
            Float32MultiArray, "/error", qos_profile=qos_profile
        )
        self.attitude_pub = self.create_publisher(
            Float32MultiArray, "/attitude", qos_profile=qos_profile
        )

        # Create subscribers
        self.visibility_subscriber = self.create_subscription(
            Bool,
            "/marker_visibility",
            self.visibility_callback,
            qos_profile=qos_profile,
        )

        # Below vehicle_local_position_subscriber has been replace by vehicle_odometry 
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )
        if not self.NO_IBVS:
            self.aruco_subscriber = self.create_subscription(
                Float32MultiArray,
                "/float_arr",
                self.image_coord_callback,
                qos_profile=qos_profile,
            )
        self.odom_subscriber = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos_profile=qos_profile,
        )
        self.attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos_profile=qos_profile,
        )

        # Initialize variables
        if self.DO_TAKEOFF:
            self.STATE = "SETUP"
        else:
            self.STATE = "IBVS"
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -7.0
        # self.waypoint = (-10.0, -10.0, self.takeoff_height)

        # Define waypoints as a list of tuples (x, y, z)
        self.waypoints = [(float(waypoint[0]), float(waypoint[1]), self.takeoff_height, float(waypoint[2])) for waypoint in drone_way ]
        self.waypoints = [(0.0, 0.0, self.takeoff_height, 0.0)] + self.waypoints
        print(self.waypoints)
        # self.waypoints = [
        #     (0.0, 0.0, self.takeoff_height),  # Takeoff position
        #     (48.0, 0.0, self.takeoff_height),  # First waypoint
        #     (48.0, 48.0, self.takeoff_height),  # Second waypoint
        #     (0.0, 48.0, self.takeoff_height),  # Third waypoint
        #     (0.0, 0.0, self.takeoff_height)  # Back to initial position
        # ]
        self.current_waypoint_index = 0  # Start with the first waypoint

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.04, self.timer_callback)

    def visibility_callback(self, msg):
        self.marker_visibility = msg.data


    def attitude_callback(self, msg):
        q = msg.q
        self.q = q
        euler_quad = euler_from_quaternion(q)
        attitude_msg = Float32MultiArray()
        attitude_msg.data = euler_quad
        self.attitude_pub.publish(attitude_msg)
    def odom_callback(self, msg):
        self.vehicle_odom = msg
        self.velocity[5] = msg.angular_velocity[2]

    def image_coord_callback(self, msg):
        self.image_coord = msg.data
        if msg.data[0] == -1.0:
            return

        # quaternion from NED to drone FRD
        
        q = self.q
        euler_quad = euler_from_quaternion(q)  # Use something like this
        # yaw at 0 rad points at south, make it point at north
        #euler_quad[2] = euler_quad[2] + pi
        euler_quad = [float(euler_quad[0]), float(euler_quad[1]), float(euler_quad[2])]
        #pub 


        #self._logger.info(f"euler_quad: {euler_quad}")
        # cam to body conversion

        # comparing image frame axis to body frame axis
        # +x in image frame (u) is  +y in body frame
        # +y in image frame (v) is  -x in body frame
        # +z in image frame (f) is  +z in body frame

        R_C_B = R_z(pi/2)  # R_C_B

        R_euler = np.matmul(R_y(euler_quad[1]), R_x(euler_quad[0]))  ## euler_quad[1] = pitch angle , euler_quad[0] = roll angle

        R_virtual = np.matmul(np.transpose(R_C_B), np.matmul(R_euler, R_C_B))
        
        #self._logger.info(f"R_y(euler_quad[1]): {R_y(euler_quad[1])}")
        #self._logger.info(f"R_x(euler_quad[0]): {R_x(euler_quad[0])}")
        #self._logger.info(f"R_euler: {R_euler}")
        #self._logger.info(f"R_C_B: {R_C_B}")
        #
        #self._logger.info(f"R_virtual: {R_virtual}")
        R_V_I = np.matmul(R_z(euler_quad[2]), R_C_B)

        s = np.array(msg.data, dtype=np.float32)
        s_star = self.pt_star - np.array(
            [self.cu, self.cv, self.cu, self.cv, self.cu, self.cv, self.cu, self.cv],
            dtype=np.float32,
        )

        # Virtual Plane Conversion
        s1 = np.array([s[0] - self.cu, s[1] - self.cv])
        s2 = np.array([s[2] - self.cu, s[3] - self.cv])
        s3 = np.array([s[4] - self.cu, s[5] - self.cv])
        s4 = np.array([s[6] - self.cu, s[7] - self.cv])

        p_1_bar = np.vstack(([s1[0]], [s1[1]], [self.f]))
        p_2_bar = np.vstack(([s2[0]], [s2[1]], [self.f]))
        p_3_bar = np.vstack(([s3[0]], [s3[1]], [self.f]))
        p_4_bar = np.vstack(([s4[0]], [s4[1]], [self.f]))

        p_1_bar_star = np.vstack(([s_star[0]], [s_star[1]], [self.f]))
        p_2_bar_star = np.vstack(([s_star[2]], [s_star[3]], [self.f]))
        p_3_bar_star = np.vstack(([s_star[4]], [s_star[5]], [self.f]))
        p_4_bar_star = np.vstack(([s_star[6]], [s_star[7]], [self.f]))

        vs1 = (self.f / np.matmul(R_virtual[2, :], p_1_bar)) * (
            np.matmul(R_virtual[0, :], p_1_bar)
        )
        vs2 = (self.f / np.matmul(R_virtual[2, :], p_1_bar)) * (
            np.matmul(R_virtual[1, :], p_1_bar)
        )
        vs3 = (self.f / np.matmul(R_virtual[2, :], p_2_bar)) * (
            np.matmul(R_virtual[0, :], p_2_bar)
        )
        vs4 = (self.f / np.matmul(R_virtual[2, :], p_2_bar)) * (
            np.matmul(R_virtual[1, :], p_2_bar)
        )
        vs5 = (self.f / np.matmul(R_virtual[2, :], p_3_bar)) * (
            np.matmul(R_virtual[0, :], p_3_bar)
        )
        vs6 = (self.f / np.matmul(R_virtual[2, :], p_3_bar)) * (
            np.matmul(R_virtual[1, :], p_3_bar)
        )
        vs7 = (self.f / np.matmul(R_virtual[2, :], p_4_bar)) * (
            np.matmul(R_virtual[0, :], p_4_bar)
        )
        vs8 = (self.f / np.matmul(R_virtual[2, :], p_4_bar)) * (
            np.matmul(R_virtual[1, :], p_4_bar)
        )

        vs1_star = (self.f / np.matmul(R_virtual[2, :], p_1_bar_star)) * (
            np.matmul(R_virtual[0, :], p_1_bar_star)
        )
        vs2_star = (self.f / np.matmul(R_virtual[2, :], p_1_bar_star)) * (
            np.matmul(R_virtual[1, :], p_1_bar_star)
        )
        vs3_star = (self.f / np.matmul(R_virtual[2, :], p_2_bar_star)) * (
            np.matmul(R_virtual[0, :], p_2_bar_star)
        )
        vs4_star = (self.f / np.matmul(R_virtual[2, :], p_2_bar_star)) * (
            np.matmul(R_virtual[1, :], p_2_bar_star)
        )
        vs5_star = (self.f / np.matmul(R_virtual[2, :], p_3_bar_star)) * (
            np.matmul(R_virtual[0, :], p_3_bar_star)
        )
        vs6_star = (self.f / np.matmul(R_virtual[2, :], p_3_bar_star)) * (
            np.matmul(R_virtual[1, :], p_3_bar_star)
        )
        vs7_star = (self.f / np.matmul(R_virtual[2, :], p_4_bar_star)) * (
            np.matmul(R_virtual[0, :], p_4_bar_star)
        )
        vs8_star = (self.f / np.matmul(R_virtual[2, :], p_4_bar_star)) * (
            np.matmul(R_virtual[1, :], p_4_bar_star)
        )


        vs = np.array([vs1, vs2, vs3, vs4, vs5, vs6, vs7, vs8])

        s_star1 = np.array(
            [
                vs1_star,
                vs2_star,
                vs3_star,
                vs4_star,
                vs5_star,
                vs6_star,
                vs7_star,
                vs8_star,
            ]
        )

        # vs = np.array([vs1, vs2, vs3, vs4], dtype=np.float32)

        x_g = (vs[0] + vs[2] + vs[4] + vs[6]) / 4

        y_g = (vs[1] + vs[3] + vs[5] + vs[7]) / 4

        x_g_star = (s_star1[0] + s_star1[2] + s_star1[4] + s_star1[6]) / 4
        y_g_star = (s_star1[1] + s_star1[3] + s_star1[5] + s_star1[7]) / 4

        a = (
            (np.square(vs[0] - x_g) + np.square(vs[1] - y_g))
            + (np.square(vs[2] - x_g) + np.square(vs[3] - y_g))
            + (np.square(vs[4] - x_g) + np.square(vs[5] - y_g))
            + (np.square(vs[6] - x_g) + np.square(vs[7] - y_g))
        )
        a_d = (
            (np.square(s_star1[0] - x_g_star) + np.square(s_star1[1] - y_g_star))
            + (np.square(s_star1[2] - x_g_star) + np.square(s_star1[3] - y_g_star))
            + (np.square(s_star1[4] - x_g_star) + np.square(s_star1[5] - y_g_star))
            + (np.square(s_star1[6] - x_g_star) + np.square(s_star1[7] - y_g_star))
        )

        a_n = (self.h_d) * np.sqrt(a_d / a)
        x_n = (a_n / self.f) * x_g
        y_n = (a_n / self.f) * y_g
        s_v = np.array([x_n, y_n, a_n], dtype=np.float32)
        a_n_star = (self.h_d) * np.sqrt(a_d / a_d)
        x_n_star = (a_n_star / self.f) * x_g_star
        y_n_star = (a_n_star / self.f) * y_g_star
        s_v_star = np.array([x_n_star, y_n_star, a_n_star], dtype=np.float32)

        e_v = np.subtract(s_v, s_v_star)

        alpha = np.arctan2(vs[1] - vs[5], vs[0] - vs[4])
        alpha_star = np.arctan2(s_star1[1] - s_star1[5], s_star1[0] - s_star1[4])
        heading_error = np.arctan2(
            np.sin(alpha - alpha_star), np.cos(alpha - alpha_star)
        )
        
        self._logger.info(f"e_v[2]: {e_v[2]}")
        # gain switching
        if e_v[2] < self.gain_switching_threshold:
            self.K = self.K_near
        else:
            self.K = self.K_far

        
        scaling_mat = np.array(
            [[self.K[0], 0, 0], [0, self.K[1], 0], [0, 0, self.K[2]]]
        )

        V_c_body = np.matmul(scaling_mat, e_v)

        # apply rotation matrix to V_c_body
        V_I = np.matmul(R_V_I, V_c_body)

        V_omega = np.array(
            [float(0), float(0), (self.k_yaw * heading_error)[0]], dtype=np.float32
        )

        V_I = np.append(V_I, V_omega)

        # V_Q = np.array([V_I[0], V_I[1], V_I[2], 0, 0, V_I[5]])

        # make 6x1 array for errors

        self.error = [float(e_v[0][0]), float(e_v[1][0]), float(e_v[2][0]), float(0.0), float(0.0), float(heading_error[0])]
        self._logger.info(f"{self.error}")

        error_msg = Float32MultiArray()
        error_msg.data = self.error
        self.error_pub.publish(error_msg)

        #self._logger.info(f"xg: {x_g} : x_g_star: {x_g_star}")


        self.VI = V_I

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        # self.velocity = np.array([    float(vehicle_local_position.vx),
        #    float(vehicle_local_position.vy),
        #    float(vehicle_local_position.vz),
        # ])
        self.velocity[0] = vehicle_local_position.velocity[0] / 2
        self.velocity[1] = vehicle_local_position.velocity[1] / 2
        self.velocity[2] = vehicle_local_position.velocity[2] / 2
        self.vehicle_local_position = vehicle_local_position

        # self.orientation = vehicle_local_position.q

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def compute_error_norm(self):
        """Compute the error percentage."""
        error_norm = np.linalg.norm(self.error)
        #self.get_logger().info(f"Error norm: {error_norm}")
        return error_norm

    def compute_land_condition(self):

        if self.error[2] < 0.05 and self.error[0] < 0.01 and self.error[1] < 0.01:
            return True
        
        if self.error[2] < 0.1 and self.marker_visibility == False :
            return True

        if self.error[2] < 0.01 and self.marker_visibility == True :
            return True


        return False
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()

        if self.STATE == "SETUP":
            msg.position = True
            msg.velocity = False
        else:
            msg.position = True
            msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_visual_servoing_setpoint():
        pass

    def publish_position_setpoint(self, x: float, y: float, z: float, angle: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # msg.yaw = 0.0 # (90 degree)
        msg.yaw = angle
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, wz: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        if abs(vx) > self.SATURATION_XY:
            vx = self.SATURATION_XY * np.sign(vx)
        if abs(vy) > self.SATURATION_XY:
            vy = self.SATURATION_XY * np.sign(vy)
        if abs(vz) > self.SATURATION_Z:
            vz = self.SATURATION_Z * np.sign(vz)
        if abs(wz) > self.SATURATION_YAWRATE:
            wz = self.SATURATION_YAWRATE * np.sign(wz)
        msg.velocity = [vx, vy, vz]
        #self._logger.info(f"Velocity setpoint: {[vx, vy, vz]}")
        msg.yawspeed =  wz
        msg.yaw = float("nan")
        # set position to NaN
        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[vx, vy, vz]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
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

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()

            self.arm()
        if self.STATE == "SETUP":
            print(f"self.STATE is SETUP now")
            print(f"Z: {self.vehicle_local_position.position[-1]}")
            if (
                self.vehicle_local_position.position[-1] > self.takeoff_height
                and self.vehicle_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                print(self.waypoints)
                print(f"publishing position setpoint {self.waypoints[self.current_waypoint_index]}")
                self.publish_position_setpoint(*self.waypoints[self.current_waypoint_index])
                if abs(self.vehicle_local_position.position[-1] - self.takeoff_height) < 1.0:
                    print(f"self.STATE is IBVS now")
                    self.STATE = "IBVS"
                # pass
                # self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

            # if abs(self.vehicle_local_position.z - self.takeoff_height) < 0.5:
            #     if self.NO_IBVS:
            #         self.get_logger().info("TAKEOFF COMPLETED")
            #         exit()
            #     self.STATE = "IBVS"

        # if self.STATE == "IBVS" and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

        #     if self.compute_land_condition():
        #         self.land()
        #         self.get_logger().info("Landing completed")
        #         exit()
        #     self.publish_velocity_setpoint(
        #         float(self.VI[0]),
        #         float(self.VI[1]),
        #         float(self.VI[2]),
        #         float(self.VI[5]),
        #     )

        if self.STATE == "IBVS" and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.compute_land_condition():
                self.land()
                self.get_logger().info("Landing completed")
                exit()

            # Navigate through waypoints
            if self.current_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_index]
                self.publish_position_setpoint(*target)
                self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index + 1}: {target}")

                # Check if the vehicle is close to the target waypoint
                if (abs(self.vehicle_local_position.position[0] - target[0]) < 0.2 and
                    abs(self.vehicle_local_position.position[1] - target[1]) < 0.2 and
                    abs(self.vehicle_local_position.position[2] - target[2]) < 0.2):
                    self.current_waypoint_index += 1
                    print(f"current waypoint index: {self.current_waypoint_index} and length of waypoints: {len(self.waypoints)}")

                    if self.current_waypoint_index == (len(self.waypoints)):
                        self.get_logger().info("All waypoints reached!")
                        self.land()
                        exit()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="parser for controller, checks if takeoff is required"
    )
    parser.add_argument(
        "--takeoff", action="store_true", help="Takeoff flag (default: false)"
    )
    parser.add_argument(
        "--no_ibvs", action="store_true", help="IBVS flag (default: false)"
    )
    args = parser.parse_args()
    print("Starting offboard control node...")

    rclpy.init()
    offboard_control = OffboardControl(takeoff=args.takeoff, no_ibvs=args.no_ibvs)
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)