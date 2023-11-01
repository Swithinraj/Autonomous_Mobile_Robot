import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist

class CmdVelocityPublisher(Node):

    def __init__(self):
        super().__init__('square_motion_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        # creating a timer to publish the velocity at a fixed rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg1 = Twist()
        msg2 = Twist()

        """
        set the linear and angular velocity of the robot according to required behavior
        """
        # example: linear velocity in 'x' direction can be set as: "msg.linear.x = 0.5"
        msg1.linear.x = 0.3
        msg1.angular.z= 0.0
        msg2.linear.x=-0.365
        msg2.angular.z=90*(3.14/180) 
        t=5  
        while t:
            time.sleep(2)
            t -= 0.5
            if (t==0):
                self.publisher_.publish(msg2)
            else:
                self.publisher_.publish(msg1)

        #self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    cmd_velocity_publisher = CmdVelocityPublisher()

    rclpy.spin(cmd_velocity_publisher)

    cmd_velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()