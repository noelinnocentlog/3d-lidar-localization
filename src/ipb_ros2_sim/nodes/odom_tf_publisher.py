#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.declare_parameter('odom_topic', 'wheel_odom')
        self.declare_parameter('parent_frame', '')
        self.declare_parameter('child_frame', '')

        odom_topic = self.get_parameter('odom_topic').value
        self._parent_frame = self.get_parameter('parent_frame').value
        self._child_frame = self.get_parameter('child_frame').value
        self._got_odom = False

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._cb, 50)

        # Publish identity TF at 10 Hz until first real odom arrives so the
        # frame exists in the TF tree immediately (prevents costmap startup hang).
        if self._parent_frame and self._child_frame:
            self.create_timer(0.1, self._warmup_timer)

        self.get_logger().info(
            f'odom_tf_publisher: {odom_topic} -> '
            f'{self._parent_frame or "msg.frame_id"} -> {self._child_frame or "msg.child_frame_id"}'
        )

    def _warmup_timer(self):
        if self._got_odom:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent_frame
        t.child_frame_id = self._child_frame
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _cb(self, msg: Odometry):
        self._got_odom = True
        t = TransformStamped()
        t.header = msg.header
        if self._parent_frame:
            t.header.frame_id = self._parent_frame
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
