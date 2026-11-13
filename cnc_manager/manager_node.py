#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class cnc_manager_node(Node):

    def __init__(self):
        super().__init__("cnc_manager")
        self.get_logger().info("CNC Manager Node is running.")
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)






    def timer_callback(self):
        self.counter +=1
        self.get_logger().info("Node is running since " + str(self.counter)+ " seconds")









def main(args=None):
    rclpy.init(args=args)
    node = cnc_manager_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()