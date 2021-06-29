import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/tracking_pose', 10)
        #self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        if(self.i % 3 == 0):
            print(self.i %3)
            msg.pose.position.x = 1.0
            msg.pose.position.y = 0.5
        elif (self.i % 3 == 1):
            print(self.i % 3)
            msg.pose.position.x = 2.0
            msg.pose.position.y = -0.5
        elif (self.i % 3 == 2):
            print(self.i %3)
            msg.pose.position.x = -1.5
            msg.pose.position.y = -0.5
        else:
            print("???")

        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
