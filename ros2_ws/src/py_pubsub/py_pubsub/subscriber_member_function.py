import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3



class Subscriber(Node):

    def __init__(self):
        super().__init__("subscriber")
        self.subscription = self.create_subscription(
        Vector3, 
        'direction', 
        self.listener_callback,
        10)
        self.subscription  

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%f, %f, %f"' % (msg.x, msg.y, msg.z))
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()