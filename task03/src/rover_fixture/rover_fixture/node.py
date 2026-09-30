"""Steady-driven simulation clock and independently calibrated sensor fixture."""
import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_srvs.srv import SetBool, Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .truth import (LANDMARKS, MOUNTS, PRACTICE_LANDMARK, Pose, cart_pose,
                    cart_probe_pose, compose, local_point, quaternion, rotate, rover_pose)


def time_message(seconds):
    ns = round(seconds*1e9)
    return Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


def set_quaternion(out, value):
    out.x, out.y, out.z, out.w = value


class Fixture(Node):
    def __init__(self):
        super().__init__('fixture')
        for name, value in [('scene', 'homework'), ('frame_prefix', 'rover/'),
                            ('moving', True), ('delay', 0.2), ('joint_angle', 0.4),
                            ('mount_x', 0.05), ('arm_axis', '0 0 1'),
                            ('imu_frame', 'imu_link'), ('odom_child_frame', 'base_link')]:
            self.declare_parameter(name, value)
        self.prefix = self.get_parameter('frame_prefix').value
        if self.prefix.startswith('/') or (self.prefix and not self.prefix.endswith('/')):
            raise ValueError('frame_prefix must be empty or end in /, without a leading /')
        self.scene = self.get_parameter('scene').value
        self.practice = self.scene == 'practice'
        self.sim_ns = 10_000_000_000
        self.paused = False
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.clock_pub = self.create_publisher(ClockMessage, '/clock', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'scene_markers', 10)
        self.pending = []
        if self.practice:
            self.probe_pub = self.create_publisher(PointStamped, 'probe', 10)
        else:
            self.probes = {name: self.create_publisher(PointStamped, f'observation/{name}', 10)
                           for name in LANDMARKS}
            self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
            self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
            self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
            self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
            self.imu_pub = self.create_publisher(Imu, 'imu', 10)
            self.imu_marker_pub = self.create_publisher(Marker, 'imu_marker', 10)
            self.static.sendTransform(self.transform('map', 'odom', Pose((0., 0., 0.), quaternion()), 10.))
        self.create_service(SetBool, 'pause', self.pause_callback)
        self.create_service(Trigger, 'reset', self.reset_callback)
        # ROS-time timers would freeze before the first /clock and during pause.
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'{self.scene}: clock starts at 10 s; frames={self.prefix!r}; synthetic, no physics')

    def frame(self, name):
        return self.prefix + name

    def transform(self, parent, child, pose, stamp):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = self.frame(parent), self.frame(child)
        msg.header.stamp = time_message(stamp)
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = pose.xyz
        set_quaternion(msg.transform.rotation, pose.q)
        return msg

    def pause_callback(self, request, response):
        self.paused = request.data
        response.success, response.message = True, 'paused' if self.paused else 'running'
        return response

    def reset_callback(self, request, response):
        self.sim_ns = 10_000_000_000
        self.pending.clear()
        response.success = True
        response.message = 'Clock reset to 10 s; pending measurements cleared. Restart consumers with epoch-specific state.'
        self.clock_pub.publish(ClockMessage(clock=time_message(10.)))
        return response

    def point(self, frame, stamp, xyz):
        msg = PointStamped()
        msg.header.frame_id, msg.header.stamp = self.frame(frame), time_message(stamp)
        msg.point.x, msg.point.y, msg.point.z = xyz
        return msg

    def tick(self):
        now = self.sim_ns / 1e9
        self.clock_pub.publish(ClockMessage(clock=time_message(now)))
        if self.paused:
            return
        moving = self.get_parameter('moving').value
        delay = float(self.get_parameter('delay').value)
        delay = max(0., min(delay, 5.0))
        if self.practice:
            self.practice_sample(now, moving, delay)
        else:
            self.homework_sample(now, moving, delay)
        # Store observations when measured, including joint state at that time.
        # A changed live joint_angle never rewrites a delayed old observation.
        remaining = []
        for due, publisher, message in self.pending:
            if due <= now + 1e-9:
                publisher.publish(message)
            else:
                remaining.append((due, publisher, message))
        self.pending = remaining
        if self.sim_ns % 500_000_000 == 0:
            self.publish_scene(now)
        self.sim_ns += 50_000_000

    def later(self, publisher, message, now, delay):
        self.pending.append((now+delay, publisher, message))

    def practice_sample(self, now, moving, delay):
        angle = float(self.get_parameter('joint_angle').value)
        mount_x = float(self.get_parameter('mount_x').value)
        axis = tuple(float(v) for v in self.get_parameter('arm_axis').value.split())
        pose = cart_pose(now, moving)
        self.dynamic.sendTransform(self.transform('world', 'cart_base', pose, now))
        joints = JointState()
        joints.header.stamp = time_message(now)
        joints.name, joints.position = ['arm_joint'], [angle]
        self.joint_pub.publish(joints)
        xyz = local_point(cart_probe_pose(now, moving, angle, mount_x, axis), PRACTICE_LANDMARK)
        self.later(self.probe_pub, self.point('probe_link', now, xyz), now, delay)

    def homework_sample(self, now, moving, delay):
        pose = rover_pose(now, moving)
        self.dynamic.sendTransform(self.transform('odom', 'base_link', pose, now))
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp = self.frame('odom'), time_message(now)
        odom.child_frame_id = self.frame(self.get_parameter('odom_child_frame').value)
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pose.xyz
        set_quaternion(odom.pose.pose.orientation, pose.q)
        # Twist is expressed in the physical base frame, independently of labels.
        t = now-10.
        if moving:
            velocity = rotate((-pose.q[0], -pose.q[1], -pose.q[2], pose.q[3]),
                              (0.20, 0.06*math.cos(.4*t), 0.))
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = velocity
            odom.twist.twist.angular.z = .06*math.cos(.3*t)
        self.odom_pub.publish(odom)
        joints = JointState()
        joints.header.stamp = time_message(now)
        joints.name = [f'{side}_wheel_joint' for side in ('front_left', 'front_right', 'rear_left', 'rear_right')]
        joints.position = [2.*(now-10.) if moving else 0.] * 4
        self.joint_pub.publish(joints)
        sensor_poses = {key: compose(pose, mount) for key, mount in MOUNTS.items()}
        labels = {'lidar': 'lidar_link', 'camera': 'camera_optical_frame',
                  'imu': self.get_parameter('imu_frame').value}
        for name in LANDMARKS:
            point = self.point(labels[name], now, local_point(sensor_poses[name], LANDMARKS[name]))
            self.later(self.probes[name], point, now, delay)
        self.scan_sample(now, delay, sensor_poses['lidar'])
        self.camera_sample(now, delay, sensor_poses['camera'])
        self.imu_sample(now, delay, sensor_poses['imu'], labels['imu'], moving)

    def scan_sample(self, now, delay, pose):
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = self.frame('lidar_link'), time_message(now)
        scan.angle_min, scan.angle_max = -math.pi/2, math.pi/2
        scan.angle_increment = math.pi/180
        scan.scan_time, scan.time_increment = 0.05, 0.
        scan.range_min, scan.range_max = 0.05, 20.
        for i in range(181):
            angle = -math.pi/2 + i*math.pi/180
            ray = rotate(pose.q, (math.cos(angle), math.sin(angle), 0.))
            distance = (3.-pose.xyz[0])/ray[0] if abs(ray[0]) > 1e-9 else math.inf
            hit_y = pose.xyz[1]+distance*ray[1] if math.isfinite(distance) else math.inf
            scan.ranges.append(distance if 0.05 < distance < 20. and abs(hit_y)<2. else math.inf)
        self.later(self.scan_pub, scan, now, delay)

    def camera_sample(self, now, delay, pose):
        image = Image()
        image.header.frame_id, image.header.stamp = self.frame('camera_optical_frame'), time_message(now)
        image.width, image.height, image.encoding, image.step = 320, 240, 'rgb8', 960
        pixels = bytearray(bytes((227, 236, 240)) * (320*240))
        # Simple calibrated target, not a renderer or a source of pose estimates.
        p = local_point(pose, LANDMARKS['camera'])
        if p[2] > 0.01:
            u, v = round(200*p[0]/p[2]+160), round(200*p[1]/p[2]+120)
            for y in range(max(0, v-7), min(240, v+8)):
                for x in range(max(0, u-7), min(320, u+8)):
                    at = (y*320+x)*3
                    pixels[at:at+3] = bytes((238, 107, 63))
        image.data = bytes(pixels)
        info = CameraInfo()
        info.header = image.header
        info.width, info.height, info.distortion_model = 320, 240, 'plumb_bob'
        info.d = [0.] * 5
        info.k = [200., 0., 160., 0., 200., 120., 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [200., 0., 160., 0., 0., 200., 120., 0., 0., 0., 1., 0.]
        self.later(self.image_pub, image, now, delay)
        self.later(self.info_pub, info, now, delay)

    def imu_sample(self, now, delay, pose, label, moving):
        imu = Imu()
        imu.header.frame_id, imu.header.stamp = self.frame(label), time_message(now)
        set_quaternion(imu.orientation, pose.q)
        imu.angular_velocity.z = .06*math.cos(.3*(now-10.)) if moving else 0.
        imu.linear_acceleration.z = 9.81  # Static gravity convention; no dynamics model.
        for covariance in (imu.orientation_covariance, imu.angular_velocity_covariance, imu.linear_acceleration_covariance):
            covariance[0] = covariance[4] = covariance[8] = 1e-4
        self.later(self.imu_pub, imu, now, delay)
        marker = Marker()
        marker.header = imu.header
        marker.ns, marker.id, marker.type, marker.action = 'imu_axis', 0, Marker.ARROW, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale.x, marker.scale.y, marker.scale.z = .25, .025, .025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .55, .15, 1.
        self.later(self.imu_marker_pub, marker, now, delay)

    def publish_scene(self, now):
        result = MarkerArray()
        root = 'world' if self.practice else 'odom'
        points = [PRACTICE_LANDMARK] if self.practice else list(LANDMARKS.values())
        for index, xyz in enumerate(points):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.frame(root), time_message(now)
            marker.ns, marker.id, marker.type = 'landmarks', index, Marker.SPHERE
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .08
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .9, .25, .12, 1.
            result.markers.append(marker)
        if not self.practice:
            wall = Marker()
            wall.header.frame_id, wall.header.stamp = self.frame(root), time_message(now)
            wall.ns, wall.id, wall.type = 'wall', 0, Marker.CUBE
            wall.pose.position.x, wall.pose.position.z, wall.pose.orientation.w = 3., .4, 1.
            wall.scale.x, wall.scale.y, wall.scale.z = .025, 4., .8
            wall.color.r, wall.color.g, wall.color.b, wall.color.a = .35, .55, .65, .35
            result.markers.append(wall)
        self.marker_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = Fixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
