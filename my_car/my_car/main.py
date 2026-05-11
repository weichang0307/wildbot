import rclpy
from rclpy.executors import MultiThreadedExecutor
from my_car.my_yolo_node import MyYoloNode
from my_car.my_control_node import MyControlNode
from my_car.urdf_publisher_node import UrdfPublisherNode

def main(args=None):
    rclpy.init(args=args)

    nodes = []
    nodes.append(MyYoloNode())
    nodes.append(MyControlNode())
    nodes.append(UrdfPublisherNode())

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