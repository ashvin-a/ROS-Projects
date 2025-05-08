import rclpy
from rclpy.node import Node


class Thala(Node):

    def __init__(self):
        super().__init__("thala_node")
        self.get_logger().info("Im alive!")
        self.create_timer(timer_period_sec=1, callback=self.timer)
    
    def timer(self):
        self.get_logger().info("Hello human!")

def main(args=None):

    rclpy.init(args=args)

    node = Thala()
    rclpy.spin(node=node) # Continues to execute the node till its killed
    # (Not like an infinite loop though)

    rclpy.shutdown()


if __name__ == "__main__":
    main()