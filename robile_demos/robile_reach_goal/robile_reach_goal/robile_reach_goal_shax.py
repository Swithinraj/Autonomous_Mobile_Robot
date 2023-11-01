# ros2 run autonomous_map_navigate move_to_goal 1.0 0.4

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener
from tf2_ros import Buffer


class MoveToGoal(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('move_to_goal')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.distance_threshold = 0.05
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_reached = False

    def odom_callback(self, msg):
        # Update current position and orientation from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, self.current_theta = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        distance_to_goal = self.get_distance_to_goal()

        if self.goal_reached or distance_to_goal < self.distance_threshold :  # Adjust the distance threshold as needed
            print("executed")
            self.stop_robot()
            self.goal_reached = True
        else:
            self.goal_reached = False

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def get_distance_to_goal(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance

    # def rotate_to_goal(self):
    #     angular_speed = 0.5  # Adjust the angular speed as needed
    #     tolerance = math.radians(0.1)  # Adjust the tolerance as needed

    #     target_angle = self.get_angle_to_goal()
    #     angular_error = target_angle - self.current_theta

    #     while math.radians(abs(angular_error)) > tolerance and not self.goal_reached:
    #         twist_msg = Twist()
    #         print(angular_speed * math.copysign(1, angular_error)*-1,"  ",angular_error)
    #         twist_msg.angular.z = angular_speed * math.copysign(1, angular_error)
    #         # twist_msg.angular.z = angular_speed * -angular_error
    #         self.cmd_vel_pub.publish(twist_msg)
    #         rclpy.spin_once(self)
    #         angular_error = target_angle - self.current_theta

    #     self.stop_robot()

    def get_angle_to_goal(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        angle = math.atan2(dy, dx)
        return angle

    '''
    Implementing a proportional control strategy for both the linear and angular velocities. 
    Proportional control adjusts the velocities based on the error between the current state and the desired state.
    '''
    def move_to_goal(self):
        print("moving to goal")
        max_linear_velocity = 0.6  # Adjust the linear speed as needed
        angular_speed = 1.0  # Adjust the angular speed as needed
        angle_tolerance = math.radians(0.1)  # Adjust the angle tolerance as needed
        min_velocity_scale = 0.4
        max_velocity_scale = 1.0
    
        '''
        Implementing a velocity scaling factor that adjusts the linear and angular velocities based on the distance to the goal.
        '''
        distance_to_goal = self.get_distance_to_goal()

        velocity_scale_factor = max(min_velocity_scale, min(max_velocity_scale, distance_to_goal / self.distance_threshold))
    
        while distance_to_goal > self.distance_threshold and not self.goal_reached:
            twist_msg = Twist()
            print("moving to goal")
            print("Distance to goal INSIDE WHILE:", self.get_distance_to_goal())

            # Calculate the desired angle
            desired_angle = self.get_angle_to_goal()
            angular_error = desired_angle - self.current_theta
            # print("this is angular error",angular_error)
            #     
            '''
            Prop control adjusts the angular velocity based on the error.
            A larger error results in a higher angular velocity to rotate the robot towards the goal direction.
            '''
            # Apply proportional control to angular velocity
            twist_msg.angular.z = angular_speed * angular_error
    
            # Adjust linear velocity based on angular error
            # With the (max_linear_vel= variable, we limit the linear velocity to a maximum value

            linear_velocity = max_linear_velocity * (1 - abs(angular_error) / math.pi) 
            # linear_velocity = max_linear_velocity * (1 - abs(distance_to_goal) / math.pi)
            '''
            Here we are calculating a proportional scaling factor for linear velocity.
            The closer the robot gets to the goal, the slower it moves,
            due to the ratio of (Distance to goal) to (Distance tolerance).
            It provides a trade-off between speed and stability.
            '''
            # Apply proportional control to linear velocity
            twist_msg.linear.x = linear_velocity * velocity_scale_factor
            twist_msg.linear.x = 0.5
            # Publish the twist message
            self.cmd_vel_pub.publish(twist_msg)
    
            # Check if the goal is reached
            # if distance_to_goal < self.distance_threshold:
            #     print("break condition")
            #     break
    
            rclpy.spin_once(self)

        self.stop_robot()

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x =0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Please provide the goal coordinates as command-line arguments: python3 move_to_goal.py <goal_x> <goal_y>")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])

    move_to_goal = MoveToGoal(goal_x, goal_y)

    try:
        # move_to_goal.rotate_to_goal()
        move_to_goal.move_to_goal()
        print("goal reached")
        move_to_goal.stop_robot()

    except KeyboardInterrupt:
        move_to_goal.stop_robot()

    move_to_goal.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

