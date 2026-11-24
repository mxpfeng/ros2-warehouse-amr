#!/usr/bin/env python3

"""Publish Gazebo's model pose as the robot's map-frame localization."""

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TruthLocalization(Node):
    def __init__(self):
        super().__init__('truth_localization')
        self.declare_parameter('robot_model_name', 'warehouse_amr')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.robot_model_name = self.get_parameter('robot_model_name').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self._warned_missing_model = False
        self._publish_map_to_odom()
        self.create_subscription(ModelStates, '/gazebo/model_states', self._on_model_states, 10)

    def _publish_map_to_odom(self):
        transform = TransformStamped()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(transform)

    def _on_model_states(self, message):
        try:
            model_pose = message.pose[message.name.index(self.robot_model_name)]
        except ValueError:
            if not self._warned_missing_model:
                self.get_logger().warning(
                    f'Gazebo model {self.robot_model_name!r} is not present in /gazebo/model_states')
                self._warned_missing_model = True
            return
        self._warned_missing_model = False

        stamp = self.get_clock().now().to_msg()
        pose_message = PoseWithCovarianceStamped()
        pose_message.header.frame_id = self.map_frame
        pose_message.header.stamp = stamp
        pose_message.pose.pose = model_pose
        self._pose_pub.publish(pose_message)

        transform = TransformStamped()
        transform.header.frame_id = self.odom_frame
        transform.header.stamp = stamp
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = model_pose.position.x
        transform.transform.translation.y = model_pose.position.y
        transform.transform.translation.z = model_pose.position.z
        transform.transform.rotation = model_pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = TruthLocalization()
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
