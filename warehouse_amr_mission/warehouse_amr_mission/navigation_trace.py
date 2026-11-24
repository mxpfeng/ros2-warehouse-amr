"""Report planned paths and compare them with Gazebo's robot ground truth.

Run alongside the simulation and Nav2:

    ros2 run warehouse_amr_mission navigation_trace
"""

from __future__ import annotations

import json
import math
from collections import deque
from typing import Dict, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

Point2D = Tuple[float, float]
Pose2D = Tuple[float, float, float]


def _yaw(pose: Pose) -> float:
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _rotate(quaternion, vector):
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _multiply_quaternions(left, right):
    return (
        left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1],
        left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0],
        left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3],
        left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2],
    )


def _wrapped_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance(left: Point2D, right: Point2D) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


class NavigationTrace(Node):
    def __init__(self) -> None:
        super().__init__('navigation_trace')
        self.declare_parameter('robot_model_name', 'warehouse_amr')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('global_path_topic', '/plan')
        self.declare_parameter('local_path_topic', '/local_plan')
        self.declare_parameter('truth_topic', '/gazebo/model_states')
        self.declare_parameter('status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('output_rate_hz', 1.0)

        self._robot_model_name = str(self.get_parameter('robot_model_name').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._truth: Optional[Pose2D] = None
        self._goal: Optional[Pose2D] = None
        self._global_path = []
        self._local_path = []
        self._global_path_dirty = False
        self._local_path_dirty = False
        self._warned_frames = set()
        self._goal_by_uuid: Dict[bytes, Optional[Pose2D]] = {}
        self._seen_terminal_uuids = set()
        self._terminal_uuid_order = deque()

        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self._on_goal, 10)
        self.create_subscription(
            Path, self.get_parameter('global_path_topic').value,
            self._on_global_path, 10)
        self.create_subscription(
            Path, self.get_parameter('local_path_topic').value,
            self._on_local_path, 10)
        self.create_subscription(
            ModelStates, self.get_parameter('truth_topic').value,
            self._on_model_states, 10)
        self.create_subscription(
            GoalStatusArray, self.get_parameter('status_topic').value,
            self._on_status, 10)

        rate = float(self.get_parameter('output_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('output_rate_hz must be greater than zero')
        self._timer = self.create_timer(1.0 / rate, self._report_tracking)
        self.get_logger().info(
            'monitoring RViz goals, Nav2 paths/status, and Gazebo model truth')

    def _emit(self, record: dict) -> None:
        self.get_logger().info(json.dumps(record, ensure_ascii=False, separators=(',', ':')))

    def _pose_in_map(self, pose: Pose, source_frame: str) -> Optional[Pose2D]:
        if not source_frame:
            source_frame = self._map_frame
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        if source_frame != self._map_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._map_frame, source_frame, Time())
            except TransformException as exc:
                key = (self._map_frame, source_frame)
                if key not in self._warned_frames:
                    self.get_logger().warning(
                        f'cannot transform {source_frame} to {self._map_frame}: {exc}')
                    self._warned_frames.add(key)
                return None

            tf = transform.transform
            tf_rotation = (
                tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
            rotated = _rotate(tf_rotation, (x, y, z))
            x = tf.translation.x + rotated[0]
            y = tf.translation.y + rotated[1]
            orientation = _multiply_quaternions(tf_rotation, orientation)

        return (x, y, math.atan2(
            2.0 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]),
            1.0 - 2.0 * (orientation[1] ** 2 + orientation[2] ** 2),
        ))

    def _on_goal(self, message: PoseStamped) -> None:
        self._goal = self._pose_in_map(
            message.pose, message.header.frame_id or self._map_frame)
        if self._goal is None:
            return
        self._emit({
            'event': 'goal_received',
            'frame': self._map_frame,
            'expected_goal': self._goal,
        })

    def _on_global_path(self, message: Path) -> None:
        points = self._path_points(message)
        if points is not None:
            self._global_path = points
            self._global_path_dirty = True

    def _on_local_path(self, message: Path) -> None:
        points = self._path_points(message)
        if points is not None:
            self._local_path = points
            self._local_path_dirty = True

    def _path_points(self, message: Path):
        frame = message.header.frame_id or self._map_frame
        points = []
        for stamped_pose in message.poses:
            pose_frame = stamped_pose.header.frame_id or frame
            point = self._pose_in_map(stamped_pose.pose, pose_frame)
            if point is None:
                return None
            points.append(point)
        return points

    def _on_model_states(self, message: ModelStates) -> None:
        try:
            index = message.name.index(self._robot_model_name)
        except ValueError:
            return
        self._truth = self._pose_in_map(message.pose[index], self._map_frame)

    def _on_status(self, message: GoalStatusArray) -> None:
        for item in message.status_list:
            goal_uuid = bytes(item.goal_info.goal_id.uuid)
            if not any(goal_uuid):
                continue

            if item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                if goal_uuid not in self._goal_by_uuid or (
                    self._goal_by_uuid[goal_uuid] is None and self._goal is not None
                ):
                    self._goal_by_uuid[goal_uuid] = self._goal
                continue

            if item.status not in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_CANCELED,
            ):
                continue
            if goal_uuid in self._seen_terminal_uuids:
                continue

            self._seen_terminal_uuids.add(goal_uuid)
            self._terminal_uuid_order.append(goal_uuid)
            if len(self._terminal_uuid_order) > 256:
                expired_uuid = self._terminal_uuid_order.popleft()
                self._seen_terminal_uuids.discard(expired_uuid)

            expected = self._goal_by_uuid.pop(goal_uuid, self._goal)
            if item.status == GoalStatus.STATUS_SUCCEEDED:
                self._report_arrival(expected)

    def _report_arrival(self, expected: Optional[Pose2D]) -> None:
        if self._truth is None:
            self.get_logger().warning(
                'Nav2 reported goal success, but no Gazebo ground-truth pose is available')
            return
        if expected is None:
            self.get_logger().warning(
                'Nav2 reported goal success, but no RViz goal pose is available')
            return

        delta_x = self._truth[0] - expected[0]
        delta_y = self._truth[1] - expected[1]
        delta_yaw = _wrapped_angle(self._truth[2] - expected[2])
        self._emit({
            'event': 'goal_succeeded',
            'frame': self._map_frame,
            'expected_goal': expected,
            'actual_truth': self._truth,
            'error': {
                'dx': delta_x,
                'dy': delta_y,
                'distance_xy': math.hypot(delta_x, delta_y),
                'dyaw': delta_yaw,
            },
        })

    def _path_report(self, points):
        if not points:
            return {
                'point_count': 0,
                'points': [],
                'errors_to_truth': [] if self._truth is not None else None,
                'nearest_point_to_truth': None,
            }
        if self._truth is None:
            errors = None
            nearest = None
        else:
            errors = [
                {
                    'dx': self._truth[0] - point[0],
                    'dy': self._truth[1] - point[1],
                    'distance_xy': _distance(self._truth[:2], point[:2]),
                }
                for point in points
            ]
            nearest_index = min(
                range(len(points)),
                key=lambda index: errors[index]['distance_xy'])
            nearest = {
                'index': nearest_index,
                'point': points[nearest_index],
                **errors[nearest_index],
            }
        return {
            'point_count': len(points),
            'points': points,
            'errors_to_truth': errors,
            'nearest_point_to_truth': nearest,
        }

    def _report_tracking(self) -> None:
        if self._truth is None:
            return

        record = {
            'event': 'robot_truth',
            'frame': self._map_frame,
            'actual_truth': self._truth,
            'nearest_global_path_point': self._path_report(
                self._global_path)['nearest_point_to_truth'],
            'nearest_local_path_point': self._path_report(
                self._local_path)['nearest_point_to_truth'],
        }
        if self._global_path_dirty:
            record['global_path'] = self._path_report(self._global_path)
            self._global_path_dirty = False
        if self._local_path_dirty:
            record['local_path'] = self._path_report(self._local_path)
            self._local_path_dirty = False
        self._emit(record)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationTrace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
