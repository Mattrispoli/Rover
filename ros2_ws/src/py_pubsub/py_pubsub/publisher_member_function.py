import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3



class Publisher(Node):

    def __init__(self):
        super().__init__("publisher")
        self.publisher_ = self.create_publisher(Vector3, 'direction', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i =0

    def timer_callback(self):
        msg = Vector3()
        msg.x = 0.7
        msg.y = 0.9
        msg.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%f, %f, %f"' % (msg.x, msg.y, msg.z))
        self.i += 1
def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()