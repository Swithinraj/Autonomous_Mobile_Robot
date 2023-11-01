#!/usr/bin/env python3

'''
IMPORTING LIBRARIES
'''
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener
from tf2_ros import Buffer
import numpy as np
import time
import py_trees as pt
import py_trees_ros as ptr
import rclpy
from geometry_msgs.msg import Twist
from tf2_ros.transform_broadcaster import TransformBroadcaster
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data 
from sensor_msgs.msg import Joy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
'''
IMPORTING utilities file
'''

from autonomous_map_navigate.utils import *

'''
Values for testing on ROBILE
'''

ALIGN_THRES=0.05                # Threshold for align behaviour
ROT_THRES=0.45                  # Threshold for angular velocities while following the wall

rotation_value=0.50             # Angular veloctiy   
max_velocity=0.30               # Maximum linear velocity 
lscan="/scan"      # Topic for getting laser data 

min_angle_rad = -1.5700000524520874
max_angle_rad = 1.5700000524520874 

'''
Values for testing in Simulation 
'''
# rotation_value=1.50 
# max_velocity=.55
# lscan="/scan" 


class battery_status2bb(ptr.subscribers.ToBlackboard): 
    '''
    Checking battery status
    '''
    def __init__(self, 
                 topic_name: str="/battery_voltage",
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 threshold: float=30.0):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=Float32,
                        blackboard_variables={'battery': 'data'},
                        initialise_variables={'battery': 100.0},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        qos_profile=ptr.utilities.qos_profile_unlatched()
                        )
        self.blackboard.register_key('battery_low_warning',access=pt.common.Access.WRITE) 
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold

    def update(self):
        '''
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        '''
        self.logger.info('[BATTERY] update: running batter_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(battery_status2bb, self).update()
        
        '''
        check battery voltage level stored in self.blackboard.battery. By comparing with threshold value, update the value of 
        self.blackboad.battery_low_warning
        '''
       
        if self.blackboard.battery < self.threshold:
            self.blackboard.battery_low_warning=True
        else:
            self.blackboard.battery_low_warning=False
        
        return pt.common.Status.SUCCESS
    
class laser_scan_2bb(ptr.subscribers.ToBlackboard):

    '''
    Checking laser_scan to avoid possible collison
    '''
    def __init__(self, 
                 topic_name: str=lscan,
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 safe_range: float=0.4):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=LaserScan,
                        blackboard_variables={'laser_scan':'ranges'},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        # qos_profile=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                        qos_profile=QoSProfile(
                                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    depth=10
                                )
        )
        self.blackboard.register_key('collision_warning',access=pt.common.Access.WRITE)
        self.blackboard.register_key('point_at_min_dist',access=pt.common.Access.WRITE)
        self.blackboard.register_key('counter',access=pt.common.Access.WRITE)
        self.blackboard.register_key('wall_reached',access=pt.common.Access.WRITE)
        self.blackboard.register_key("slope", access=pt.common.Access.WRITE)
        self.blackboard.register_key("dist", access=pt.common.Access.WRITE)
        self.blackboard.register_key("process_scan", access=pt.common.Access.WRITE)
        self.blackboard.register_key("position", access=pt.common.Access.WRITE)
        self.blackboard.register_key("aligned", access=pt.common.Access.WRITE)
        self.blackboard.register_key("align", access=pt.common.Access.WRITE)
        self.blackboard.register_key("rotation_amount", access=pt.common.Access.WRITE)
        self.blackboard.register_key("follow", access=pt.common.Access.WRITE)
        self.blackboard.register_key("rotation_direction", access=pt.common.Access.WRITE)
        self.blackboard.register_key("slope_angle", access=pt.common.Access.WRITE)
        self.blackboard.register_key("wall_found", access=pt.common.Access.WRITE)
        self.blackboard.register_key("left_dist", access=pt.common.Access.WRITE)
        self.blackboard.register_key("right_dist", access=pt.common.Access.WRITE)
        self.blackboard.register_key("front_dist", access=pt.common.Access.WRITE)
        self.blackboard.align=0.
        self.blackboard.left_dist= 0.
        self.blackboard.right_dist = 0.
        self.blackboard.front_dist = 0.
        self.blackboard.counter = 0.
        self.safe_min_range = safe_range
        self.blackboard.point_at_min_dist = 0.0      
        self.blackboard.slope_angle=0.0
        self.blackboard.rotation_direction=0.0
        self.blackboard.process_scan = 0.0
        self.blackboard.aligned=True
        self.blackboard.slope = 0.0
        self.blackboard.dist = 0.0  
        self.blackboard.rotation_amount = 0.0
        '''
        Decision making variables: These trigger respective behaviours
        '''
        self.blackboard.collision_warning = False 
        self.blackboard.wall_found = False 
        self.blackboard.follow= False
        self.blackboard.position = False 
        self.blackboard.wall_reached = False 

        
    def update(self):
        '''
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the ROBILE is close to any obstacle.
        '''
        self.logger.info("[LASER SCAN] update: running laser_scan_2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(laser_scan_2bb, self).update()

        '''
        Based on the closeness of laser scan points (any of them) to ROBILE, update the value of self.blackboard.collision_warning.
        Assign the minimum value of laser scan to self.blackboard.point_at_min_dist. 
        Note: The min() function can be used to find the minimum value in a list.
        '''

        self.blackboard.counter += 1
        if self.blackboard.exists("laser_scan"):
            laser=np.array(self.blackboard.laser_scan)
            laser=np.nan_to_num(laser,nan=0.0)
            laser[laser<=0.02]=1.0
            self.blackboard.process_scan=laser
            self.blackboard.set("laser_scan", laser)
            process_laser = np.array(self.blackboard.process_scan)
            self.blackboard.set("process_scan", self.blackboard.process_scan)
            '''
            Parsing the [laser_scan] data for left, right and front
            '''
            self.blackboard.set("left_dist",np.mean(np.array(process_laser[509:512])))
            self.blackboard.set("front_dist",np.mean(np.array(process_laser[253:256])))
            self.blackboard.set("right_dist",np.mean(np.array(process_laser[0:3])))

            # self.blackboard.set("left_dist",np.mean(np.array(process_laser[355:360])))
            # self.blackboard.set("front_dist",np.mean(np.array(process_laser[175:180])))
            # self.blackboard.set("right_dist",np.mean(np.array(process_laser[0:5])))

            if run_online(process_laser,0.0005,5) == 0:
                '''
                ----------When NO WALL is Detected----------------
                '''
                self.blackboard.wall_found=True
                self.logger.info("[no_wall] Behaviour is triggered")
                return pt.common.Status.RUNNING

            else:
                self.blackboard.dist,self.blackboard.slope = run_online(process_laser,0.0005,5)
                slope_angle=math.atan(self.blackboard.slope)
                align=np.deg2rad(90)-abs(math.atan(self.blackboard.slope))
                self.blackboard.point_at_min_dist= min(laser)

                print("Nearest distance Point:",self.blackboard.point_at_min_dist)
                print("Perpendicular Dist of ROBILE: ",self.blackboard.dist)
                print("Collision Safety threshold value is :",self.safe_min_range)
                print("Angle left to align",align)
                print("Current Slope Angle", slope_angle)

                '''
                Updating blackboard variables with current values
                '''
                self.blackboard.set("dist", self.blackboard.dist)
                self.blackboard.set("align",align)
                self.blackboard.set("slope_angle", slope_angle)
                self.blackboard.set("point_at_min_dist",self.blackboard.point_at_min_dist)

                self.blackboard.follow=self.blackboard.get("aligned")
                
                '''
                --------------COLLISION AVOIDANCE CONDITION [stop_motion]------
                '''
                if self.blackboard.point_at_min_dist < self.safe_min_range:
                    self.blackboard.collision_warning= True

                    '''
                    ----------MOVE TO WALL CONDITION [move_2bb]----------------
                    '''
                elif self.blackboard.point_at_min_dist > self.safe_min_range+0.19:
                    self.blackboard.wall_reached=True
    
                    '''
                    ----------ALIGN TO WALL CONDITION [align]------------------
                    '''
                elif 1.0 > self.blackboard.point_at_min_dist > self.safe_min_range and align > ALIGN_THRES*1.2:
                    self.blackboard.position=True
                    self.blackboard.wall_reached=False

                else:
                    self.blackboard.collision_warning= False
                return pt.common.Status.SUCCESS  
        else:
            return pt.common.Status.RUNNING
        
class rotate(pt.behaviour.Behaviour):
    '''
    Rotates the ROBILE about z-axis 
    '''

    def __init__(self, name="rotate platform", topic_name="/cmd_vel", direction=1):

        # Set up topic name publish rotation commands
        self.topic_name = topic_name
        self.direction = direction
        # become a behaviour
        super(rotate, self).__init__(name)

    def setup(self, **kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        '''
        Primary function of the behavior is implemented in this method

        Rotating the ROBILE at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        '''
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = np.deg2rad(90)
        self.cmd_vel_pub.publish(twist_msg)
               
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        '''
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        
        return super().terminate(new_status)

class stop_motion(pt.behaviour.Behaviour):

    '''
    Stops the ROBILE when it is controlled using joystick or by cmd_vel command
    '''

    def __init__(self, 
                 name: str="stop platform", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(stop_motion, self).__init__(name)
        # Set up topic name publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.blackboard = pt.blackboard.Blackboard()


    def setup(self, **kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        self.feedback_message = "setup"
        return True

    def update(self):
        '''
        Primary function of the behavior is implemented in this method

        Rotating the ROBILE at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        '''
        self.logger.info("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        point_at_min_dist=self.blackboard.get("point_at_min_dist")
        print("Closest distance",point_at_min_dist)
        self.left_dist = self.blackboard.get("left_dist")
        self.front_dist = self.blackboard.get("front_dist")
        self.right_dist = self.blackboard.get("right_dist")
        self.rot=self.blackboard.get("rotation_direction")

        print("This is front dist:", self.front_dist)
        print("This is left dist:", self.left_dist)
        print("This is right dist:", self.right_dist)

        '''
        Send the zero rotation command to self.cmd_vel_topic
        '''
        if (self.right_dist > self.left_dist) and self.front_dist<1.2:
            twist_msg = Twist()

            twist_msg.linear.x = -0.04
            twist_msg.linear.y = -0.2
            twist_msg.angular.z = -0.1
            self.cmd_vel_pub.publish(twist_msg)

            self.logger.info("[Stop Motion]: Found clear path- Moving RIGHT ")
            return pt.common.Status.RUNNING
        
        if (self.right_dist < self.left_dist) and self.front_dist<1.2:
            twist_msg = Twist()

            twist_msg.linear.x = -0.04
            twist_msg.linear.y = 0.2
            twist_msg.angular.z = 0.1
            self.cmd_vel_pub.publish(twist_msg)

            self.logger.info("[Stop Motion]: Found clear path- Moving LEFT")
            return pt.common.Status.RUNNING

        if (self.left_dist < 1.2 and self.right_dist < 1.2) and self.front_dist <1.2:
            twist_msg = Twist()

            twist_msg.linear.x = -0.08
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg)

            self.logger.info("[Stop Motion]: Stuck- Moving backwards")
            return pt.common.Status.RUNNING
        
        if (self.left_dist>1. and self.right_dist>1.) and self.front_dist>1. and point_at_min_dist>0.2:
            twist_msg = Twist()

            twist_msg.linear.x = 0.07
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg)
            self.logger.info("[Stop Motion]: Front Clear found")
            self.blackboard.set("collision_warning",False)
            return pt.common.Status.SUCCESS

        '''
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        '''
        ## Uncomment the following lines to publish Joy() message when running on the ROBILE ##
        joyMessage = Joy()
        joyMessage.header.frame_id = "/dev/input/js0"
        joyMessage.axes = [0., 0., 0., 0., 0., 0.]
        joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        self.joy_pub.publish(joyMessage)        
        return pt.common.Status.SUCCESS


    def terminate(self, new_status):
        '''
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        
        return super().terminate(new_status)        
        
class move_2bb(pt.behaviour.Behaviour):

    def __init__(self, 
                 name: str="move to wall", 
                 topic_name: str="/cmd_vel"):
        super(move_2bb, self).__init__(name)

        self.wall_reached = False
        # Set up topic name publish rotation commands
        self.cmd_vel_topic = topic_name
    
    def setup(self,**kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        self.logger.info("[move_2bb] setting up Move_2bb motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        self.feedback_message = "setup"
        return True 
   
    def update(self):
        '''
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the ROBILE is close to any obstacle.
        '''
        self.logger.info("[move_2bb] update: running move update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(move_2bb, self).update()
        '''
        Based on the closeness of laser scan points (any of them) to ROBILE, update the value of self.blackboard.points.
        Assign the minimum value of laser scan to self.blackboard.point_at_min_dist. 
        Note: The min() function can be used to find the minimum value in a list.
        '''
        blackboard = pt.blackboard.Blackboard()     
        self.logger.info("[move_2bb] IS WORKING")
        slope_angle=blackboard.get("slope_angle")
        perp_ang=abs(slope_angle)
        print("perp_ang",perp_ang)

        if perp_ang <=ALIGN_THRES:
            self.logger.info("Reached near a wall")
            blackboard.set("aligned",True) # Triggering [follow_wall] behaviour
            return pt.common.Status.SUCCESS 
        else:            
            if slope_angle > 0.0:
                rotation_direction = rotation_value # Clockwise rotation
            else:
                rotation_direction = -rotation_value # Counter-clockwise rotation
            angular_velocity = max_velocity
            twist_msg = Twist()
            twist_msg.linear.x = 0.04
            twist_msg.linear.y = 0.
            twist_msg.angular.z = -angular_velocity * rotation_direction
            blackboard.set("rotation_direction",rotation_direction)

            self.cmd_vel_pub.publish(twist_msg)

            return pt.common.Status.RUNNING     

    def terminate(self, new_status):
        '''
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[move]_2bb terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = max_velocity
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        return super().terminate(new_status)
      
class align(pt.behaviour.Behaviour):

    def __init__(self, name="Align Platform", topic_name1="/cmd_vel", direction: int=1):

        # Set up topic name publish rotation commands
        self.topic_name = topic_name1
        self.blackboard = pt.blackboard.Blackboard()

        # become a behaviour
        super(align, self).__init__(name)


    def setup(self, **kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        
        self.logger.info("[align] setting up aligning behavior")
 
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        self.feedback_message = "setup"
        return True
    
    def terminate(self, new_status):
        '''
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[align] terminate: publishing velocity to align")

        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        
        return super().terminate(new_status)
        
    def update(self):
        '''
        Primary function of the behavior is implemented in this method

        Rotating the ROBILE at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        '''
        self.logger.info("[align] update: Aligning based on slope")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        self.rot=self.blackboard.get("rotation_direction")

        self.left_dist = self.blackboard.get("left_dist")
        self.front_dist = self.blackboard.get("front_dist")
        self.right_dist = self.blackboard.get("right_dist")

        slope_angle=self.blackboard.get("slope_angle")
        align_ang=self.blackboard.get("align")

        if align_ang <=ALIGN_THRES:
            self.logger.info("Aligned to wall")
            self.blackboard.set("aligned",True) # Triggering [follow_wall] behaviour
            return pt.common.Status.SUCCESS # continue
        '''
        Closed Corner
        '''
        if self.front_dist>10.:
            self.front_dist= 999.999 
        if self.left_dist>10.:
            self.left_dist= 999.999 
        if self.right_dist>10.:
            self.right_dist= 999.999

        print("This is front dist:", self.front_dist)
        print("This is left dist:", self.left_dist)
        print("This is right dist:", self.right_dist)

        # Left closed corner
        if (self.left_dist)<=1.4 and self.front_dist <= 0.9 and self.left_dist<self.right_dist:
            self.logger.info("[align]: CLOSED CORNER IDENTIFIED: Taking right turn, wall is on left")    
            print("Left Distance threshold: ",self.left_dist)
            twist_msg1 = Twist()
            twist_msg1.linear.x = 0.
            twist_msg1.linear.y = 0.
            twist_msg1.angular.z = -max_velocity
            self.cmd_vel_pub.publish(twist_msg1)
            return pt.common.Status.RUNNING 
        
        # Right closed corner
        if (self.right_dist)<=1.4 and self.front_dist<= 0.9 and self.left_dist>self.right_dist:
            self.logger.info("[align]: CLOSED CORNER IDENTIFIED: Taking left turn, wall is on right")
            print("Right Distance threshold: ",self.right_dist)
            twist_msg1 = Twist()
            twist_msg1.linear.x = 0.
            twist_msg1.linear.y = 0.
            twist_msg1.angular.z = max_velocity
            self.cmd_vel_pub.publish(twist_msg1)
            return pt.common.Status.RUNNING
         
        # Small space
        if (self.right_dist<=0.8 or self.left_dist <= 0.8) and self.front_dist<= 0.8:
            self.logger.info("[align]: CRAMPED CORNER IDENTIFIED: Moving Backwards")
            print("Left Distance threshold: ",self.left_dist)
            print("Right Distance threshold: ",self.right_dist)
            twist_msg1 = Twist()
            twist_msg1.linear.x = -max_velocity
            twist_msg1.linear.y = 0.
            twist_msg1.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg1)
            return pt.common.Status.RUNNING 
                
        else:
            rotation_direction=1
            self.logger.info("[align]: Rotating ROBILE to align to wall")
            if slope_angle > 0.0 and (self.left_dist > 0.99 or self.right_dist > 0.99) :
                rotation_direction = rotation_value # Clockwise rotation
            if slope_angle < 0.0 and (self.left_dist > 0.99 or self.right_dist > 0.99) :
                rotation_direction = -rotation_value # Counter-clockwise rotation
       
            angular_velocity = max_velocity
            twist_msg = Twist()
            twist_msg.linear.x = 0.
            twist_msg.linear.y = 0.
            twist_msg.angular.z = angular_velocity * rotation_direction
            self.blackboard.set("rotation_direction",rotation_direction)

            self.cmd_vel_pub.publish(twist_msg)

            return pt.common.Status.RUNNING 

class follow_wall(pt.behaviour.Behaviour):
    def __init__(self, name="Follow Wall", topic_name1="/cmd_vel"):
        # Set up topic name publish rotation commands
        self.topic_name = topic_name1
        self.aligned = False
        self.blackboard = pt.blackboard.Blackboard()
        self.rot=self.blackboard.get("rotation_direction")

        # become a behaviour
        super(follow_wall, self).__init__(name)

    def setup(self, **kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        
        self.logger.info("[follow] setting up following behavior")
 
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        self.feedback_message = "setup"
        return True
    
    def terminate(self, new_status):
        '''
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[follow] terminate: publishing velocity to terminate")
            
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z =0.
        self.cmd_vel_pub.publish(twist_msg)
        
        return super().terminate(new_status)
    

    def update(self):
        ''' 
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        '''
        self.logger.info('[follow] update: running follow update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(follow_wall, self).update()
        self.rot=self.blackboard.get("rotation_direction")
        self.slope_angle=self.blackboard.get("slope_angle")
        self.left_dist = self.blackboard.get("left_dist")
        self.front_dist = self.blackboard.get("front_dist")
        self.right_dist = self.blackboard.get("right_dist")

        if self.front_dist>15.:
            self.front_dist= 999.999 
        if self.left_dist>15.:
            self.left_dist= 999.999 
        if self.right_dist>15.:
            self.right_dist= 999.999

        print("This is front dist:", self.front_dist)
        print("This is left dist:", self.left_dist)
        print("This is right dist:", self.right_dist)
        print("Rotation Direction",self.rot)

        perp_dist=self.blackboard.get("dist")
        align_ang=self.blackboard.get("align")
        point_at_min_dist=self.blackboard.get("point_at_min_dist")
        print("perp_dist-",perp_dist)
        print("point at min dist: ",point_at_min_dist)


        if point_at_min_dist<0.4 and perp_dist<1.0:
            self.logger.info("[follow]: EXECUTION DONE")
            return pt.common.Status.SUCCESS 


        elif self.left_dist > 1.2 and self.right_dist>1.2 and self.front_dist>1.2:
            twist_msg = Twist()
            twist_msg.linear.x = -0.0
            twist_msg.linear.y = self.rot*0.20
            twist_msg.angular.z = -self.rot*1.1
            self.cmd_vel_pub.publish(twist_msg)
            # print("perp_dist-",perp_dist)

            self.logger.info("[follow]: OPEN CORNER FOUND: Now taking a Turn")
            return pt.common.Status.RUNNING
        
        elif point_at_min_dist>0.65:
            self.logger.info("[follow]: Moving closer to the wall")
            twist_msg = Twist()
            twist_msg.linear.x = max_velocity*0.45
            twist_msg.linear.y = 0.
            twist_msg.angular.z = -self.rot*ROT_THRES
            self.cmd_vel_pub.publish(twist_msg)
            return pt.common.Status.RUNNING 
        
        elif point_at_min_dist<0.55:
            self.logger.info("[follow]: Moving away from the wall")
            twist_msg = Twist()
            twist_msg.linear.x = max_velocity*0.45
            twist_msg.linear.y = 0.
            twist_msg.angular.z = self.rot*ROT_THRES
            self.cmd_vel_pub.publish(twist_msg)
            return pt.common.Status.RUNNING 
        
        else:
            self.logger.info("[follow]: Trying to be Parallel to wall")
            twist_msg = Twist()
            twist_msg.linear.x = max_velocity
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.
            self.cmd_vel_pub.publish(twist_msg)
            return pt.common.Status.RUNNING 

class no_wall(pt.behaviour.Behaviour):
    def __init__(self, name="No Wall", topic_name1="/cmd_vel"):
        # Set up topic name to publish commands
        self.topic_name = topic_name1
        self.aligned = False
        self.blackboard = pt.blackboard.Blackboard()

        # become a behaviour
        super(no_wall, self).__init__(name)

    def setup(self, **kwargs):
        '''
        Setting up things which generally might require time to prevent delay in tree initialisation
        '''
        
        self.logger.info("[no_wall]: setting up following behavior")
 
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        self.feedback_message = "setup"
        return True
    
    def terminate(self, new_status):
        '''
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        '''
        self.logger.info("[no_wall] terminate: publishing velocity to terminate")
            
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
 
        self.cmd_vel_pub.publish(twist_msg)
        
        return super().terminate(new_status)

    def update(self):
        ''' 
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        '''
        self.logger.info('[no_wall] update: running No Wall update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(no_wall, self).update()
        self.laser=self.blackboard.get("process_scan")
        self.rot=self.blackboard.get("rotation_direction")
        self.dist=self.blackboard.get("dist")

        print("rotation direction", self.rot)
        if run_online(self.laser,0.0005,5) != 0:
            self.blackboard.set("wall_found", False)
            return pt.common.Status.SUCCESS # continue
           
        else:
            if self.rot != 0:
                twist_msg = Twist()
                twist_msg.linear.x = 0.
                twist_msg.linear.y = 0.
                twist_msg.angular.z = self.rot*max_velocity
            else:
                twist_msg = Twist()
                twist_msg.linear.x = 0.
                twist_msg.linear.y = 0.
                twist_msg.angular.z = max_velocity
    
            self.cmd_vel_pub.publish(twist_msg)
            return pt.common.Status.RUNNING


def run_online(process_laser,sigma,k):

    '''
    Conversion of Polar to Cartesian coordinates
    '''
    angles = np.linspace(min_angle_rad, max_angle_rad, process_laser.shape[0]) * 180 / np.pi
    # process_laser
    polar_data = np.hstack([process_laser.reshape(-1, 1), angles.reshape(-1, 1)])
    polar_data = np.delete(polar_data, np.where(polar_data[:, 0] == 0), axis=0)
    points = np_polar2rect(reduction_filter(polar_data, sigma, 20, median_filter_size=1, mean_filter_size=2))

    ran_line = online_line_detection(points,e=0.01, incr=0.02, max_dist=2, k=35)

    # print("Lines found are:",(ran_line))
    blackboard = pt.blackboard.Blackboard()

    d=blackboard.get("dist")
    m=blackboard.get("slope_angle")
    '''
    Code to PLOT the walls found using Online line detection
    '''
    #plot_lines(ran_line)
 
    '''
    Parsing lines till range of 4.2 metres
    '''
    ran_line = [line for line in ran_line if abs(line[0]) < 4.2]

    if len(ran_line):
        near_line = sorted(ran_line, key=lambda x:x[0])[0]
        d, m,_,_= near_line
        return d, m
    
    # No lines found
    if len(ran_line)==0:
        return 0
