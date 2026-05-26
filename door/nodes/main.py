import rclpy
from rclpy.executors import MultiThreadedExecutor
from nodes.door_node import DoorNode
from nodes.urdf_publisher_node import UrdfPublisherNode
from nodes.lidar_localizer_node import LidarLocalizer
from nodes.astar_planner_node import AStarPlannerNode
from nodes.control_node import ControlNode
from nodes.pointcloud_obstacle_map_node import PointCloudObstacleMapNode
from nodes.fused_obstacle_map_node import FusedObstacleMapNode

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticMapNode(Node):
    """Broadcasts a static identity map → car_base so Foxglove has a map frame
    even before the lidar localizer receives its first scan."""
    def __init__(self):
        super().__init__('static_map_tf')
        br = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id  = 'car_base'
        tf.transform.rotation.w = 1.0
        br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)

    nodes = [
        StaticMapNode(),       # map → car_base identity (placeholder until lidar kicks in)
        UrdfPublisherNode(),   # /robot_description + TF from URDF
        LidarLocalizer(),      # map → car_base TF (live, overrides static once scan arrives)
        DoorNode(),            # /door/entrance pose
        AStarPlannerNode(),    # path_goal → Nav2 → /astar_path
        ControlNode(),         # manual/auto driving to door entrance
        PointCloudObstacleMapNode(),  # point cloud → obstacle map
        FusedObstacleMapNode(),        # fuse lidar+camera obstacle maps
    ]

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
