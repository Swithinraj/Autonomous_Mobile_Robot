import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import math


class MoveToGoal(Node):
    def __init__(self, args=None):
        if args is None:
            args = [1.45, 0.00]
        super().__init__('move_to_goal')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # save the input goal location with respect to base_laser
        self.goal_wrt_base_laser = PoseStamped()
        self.goal_wrt_base_laser.pose.position.x = args[0]
        self.goal_wrt_base_laser.pose.position.y = args[1]

        # Initialise velocity in x and y direction and set direction_found_bool to False
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.direction_found_bool = False

        # Initialise the goal location with respect to base_link
        self.goal_wrt_base_link = PoseStamped()

        # Initialise goal location with respect to odom
        self.goal_wrt_odom = PoseStamped()

        print("Initialisation completed. Calling publisher callback ...")
        """
        Initialise a publisher to publish to the cmd_vel topic (refer to the first exercise)
        The callback of this publisher will trigger the functions to find the direction and monitor the movement of the robot
        """
        ### YOUR CODE HERE ###
        # Initialise a publisher to publish to the cmd_vel topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)   # message type has to be Twist here for cmd_vel

        # Use a timer to publish commands to the cmd_vel topic
        self.timer = self.create_timer(0.1, self.timer_callback)    # Timer -> for 0.1 secs, it will keep calling timer_callback        
        self.transform_available = False

    def timer_callback(self):
        """
        Publish command to the cmd_vel topic in the direction of the goal until the goal is reached
        Hint: goal_wrt_odom can be used to check if the base_laser_front_link is at the goal
        """
        print("In timer_callback")
        if not self.direction_found_bool:
            msg = Twist()
            self.get_direction('odom', 'base_link', 'base_laser_front_link')
            if self.transform_available:
                print("Direction found. Publishing velocity command ...")
                """
                Determine the direction in which the robot should move and publish to cmd_vel topic
                """
                ### YOUR CODE HERE ###
                msg.linear.x = self.x_vel
                msg.linear.y = self.y_vel
                self.publisher.publish(msg)
                self.direction_found_bool = True            
            
        else:
            msg = Twist()
            distance = self.get_distance('base_link', 'odom')
            print("Distance to goal: ", distance)
            if distance < 0.05:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                self.publisher.publish(msg)

    def get_direction(self, odom_frame, base_link_frame, base_laser_frame):
        """
        Function to get the direction towards goal from base_link
        """
        try:
            transform_laser_wrt_base_link = self.get_transform(base_laser_frame, base_link_frame)
            transform_laser_wrt_odom = self.get_transform(base_laser_frame, odom_frame)

            if self.transform_available:
                # For better intuition, print different transformations between different frames
                print("\ntransform_laser_wrt_base_link:\n", transform_laser_wrt_base_link)
                
                self.goal_wrt_base_link.pose.position.x = transform_laser_wrt_base_link.translation.x + self.goal_wrt_base_laser.pose.position.x
                self.goal_wrt_base_link.pose.position.y = transform_laser_wrt_base_link.translation.y + self.goal_wrt_base_laser.pose.position.y

                """
                Get `goal_wrt_odom` and set the desired velocity (x_vel, y_vel) 
                """
                ### YOUR CODE HERE ###
                self.goal_wrt_odom.pose.position.x = transform_laser_wrt_odom.translation.x + self.goal_wrt_base_laser.pose.position.x
                self.goal_wrt_odom.pose.position.y = transform_laser_wrt_odom.translation.y + self.goal_wrt_base_laser.pose.position.y
                if self.goal_wrt_base_link.pose.position.x==0.0 and self.goal_wrt_base_link.pose.position.y==0.0:
                    self.x_vel = 0.0
                    self.y_vel = 0.0
                else:
                    self.x_vel = self.goal_wrt_base_link.pose.position.x/math.sqrt((self.goal_wrt_base_link.pose.position.x)**2 + (self.goal_wrt_base_link.pose.position.y)**2)
                    self.y_vel = self.goal_wrt_base_link.pose.position.y/math.sqrt((self.goal_wrt_base_link.pose.position.x)**2 + (self.goal_wrt_base_link.pose.position.y)**2)
                print("self.x_vel: ", self.x_vel )
                print("self.y_vel: ", self.y_vel )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            pass
            self.get_logger().error('Failed to get transform: %s' % str(e))

    def get_distance(self, target_frame, source_frame):
        """
        Get the distance between the target_frame (base_link) and the goal with respect to source_frame (odom frame)
        """
        try:
            transform = self.get_transform(target_frame, source_frame)
            if self.transform_available:
                print("base_link wrt odom: ", transform.translation.x, transform.translation.y)
                print("self.goal_wrt_odom.pose.position.x: ", self.goal_wrt_odom.pose.position.x)
                print("self.goal_wrt_odom.pose.position.y: ", self.goal_wrt_odom.pose.position.y)
                distance = math.sqrt((transform.translation.x-self.goal_wrt_odom.pose.position.x)**2
                                    + (transform.translation.y-self.goal_wrt_odom.pose.position.y)**2)
                return distance
            else:
                return False

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Failed to get transform: %s' % str(e))

    def get_transform(self, target_frame, source_frame):
        """
        Get the transform from the source_frame to the target_frame
        """
        try:
            while not self.tf_buffer.can_transform(target_frame, source_frame, tf2_ros.Time()):
                self.get_logger().warn('Waiting for transform...')
                self.transform_available = False
                return False
            self.transform_available = True
            transform = self.tf_buffer.lookup_transform(source_frame, target_frame, tf2_ros.Time())
            return transform.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(str(e))
            return None


def main(args=None):
    rclpy.init(args=args)

    # Create a node with input arguments
    move_to_goal_node = MoveToGoal(args)

    # Use spin to execute the node
    rclpy.spin(move_to_goal_node)

    # Shutdown the node once it is killed
    move_to_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
