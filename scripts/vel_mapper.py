import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRepublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_republisher')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    def cmd_vel_callback(self, msg):
        # Simply republish the received message to the new topic
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

