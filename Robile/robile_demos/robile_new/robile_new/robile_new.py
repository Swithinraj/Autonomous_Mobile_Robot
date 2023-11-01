import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelocityPublisher(Node):

    def __init__(self):
        super().__init__('motion_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        # creating a timer to publish the velocity at a fixed rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.move_forward()
        self.stop_moving()
        self.turn_left()
        self.stop_moving()
        self.turn_left()
        self.stop_moving()

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5      
        msg.angular.z = 0.0
        # publish the first velocity command to move forward     
        self.publisher_.publish(msg)
        # wait for the robot to move forward a certain distance
        rclpy.spin_once(self)
        rclpy.sleep(1)

    def stop_moving(self):
        # stop the robot by setting all velocities to zero     
        msg = Twist()
        msg.linear.x = 0.0      
        msg.angular.z = 0.0      
        self.publisher_.publish(msg)      
        # wait for the robot to stop moving      
        rclpy.spin_once(self)
        rclpy.sleep(1)

    def turn_left(self):
        # turn the robot by setting angular velocity to turn   
        msg = Twist()
        msg.linear.x = 0.0      
        msg.angular.z = 90*(3.14/180) # rad/s      
        self.publisher_.publish(msg)      
        rclpy.spin_once(self)
        rclpy.sleep(3)

def main(args=None):
    rclpy.init(args=args)

    cmd_velocity_publisher = CmdVelocityPublisher()

    rclpy.spin(cmd_velocity_publisher)

    cmd_velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
