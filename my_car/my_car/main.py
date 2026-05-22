
import rclpy
from rclpy.executors import MultiThreadedExecutor
from my_car.my_yolo_node import MyYoloNode
from my_car.my_control_node import MyControlNode
from my_car.urdf_publisher_node import UrdfPublisherNode
from my_car.map_publisher_node import MapPublisherNode
from my_car.astar_planner_node import AStarPlannerNode
from my_car.rrt_star_planner_node import RRTStarPlannerNode
from my_car.bear_map_node import BearMapNode
from my_car.lidar_localizer_node import LidarLocalizer

def main(args=None):
    rclpy.init(args=args)

    nodes = []
    nodes.append(MyYoloNode())
    nodes.append(MyControlNode())
    nodes.append(UrdfPublisherNode())
    nodes.append(MapPublisherNode())
    nodes.append(AStarPlannerNode())
    # nodes.append(RRTStarPlannerNode())
    nodes.append(BearMapNode())
    # nodes.append(LidarLocalizer())

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