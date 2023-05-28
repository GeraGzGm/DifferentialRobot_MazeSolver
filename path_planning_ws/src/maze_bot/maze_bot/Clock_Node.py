import rclpy
from rclpy.node import Node

class Clock(Node):

    def __init__(self):
        super().__init__("Clock_Node")

    def Get_time(self):
        t = self.get_clock().now()
        
        return t.nanoseconds*1e-9

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Clock()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()