import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'information', 10)
        timerPeriod = 2
        self.timer = self.create_timer(timerPeriod, self.timerCallback)
        self.i = 0

    def timerCallback(self):
        msg = String()
        msg.data = 'I am an info: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    myPublisher = MyPublisher()

    rclpy.spin(myPublisher)

    # Explucitly destroy node
    myPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
