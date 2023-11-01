#!/usr/bin/env python3

import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
from sensor_msgs.msg import LaserScan
import rclpy
import sys
from rclpy.node import Node
import tf2_ros
from autonomous_map_navigate.behaviors import *

def create_root() -> pt.behaviour.Behaviour:
    """
    Method to structure the behavior tree to monitor battery status and start rotation if battery is low.
    Also, the robot will stop if it detects an obstacle in front of it.
    
    The "collison_avoidance" behavior tree extends the "battery_monitor" behavior tree by adding a new feature
    to avoid collison with obstacles. Whenever the robot is about to collide with an object, the robot will
    automatically stop, overriding the input commands. The robot can be controlled either by joystick,
    where the command is published on the '/joy' topic or by command that is published on '/cmd_vel' topic.
    The laser scan data will be stored in blackboard by reading '/scan' topic. When an obstacle
    is detected, the 'stop_motion' behavior will be executed. The stop_motion behavor is prioritized over
    the rotate behavior.
    """

    ## define nodes and behaviors

    # define root node  
    root = pt.composites.Parallel(
        name="root",
        policy=pt.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )    

    """
    Create a sequence node called "Topics2BB" and a selector node called "Priorities"    
    """

    topics2bb = pt.composites.Parallel("Topics2BB")
    priorities = pt.composites.Selector("Priorities",memory=True)

    """
    Using the battery_status2bb class, create a node called "Battery2BB" which subscribes to the topic "/battery_voltage"
    and the laser_scan_2bb class, create a node called "LaserScan2BB" which subscribes to the topic "/scan"
    """  

    battery2bb=battery_status2bb(
        name="Battery2BB",
        topic_name="/battery_voltage",
        threshold=30.0
    )

    laser2bb = laser_scan_2bb(
        name="LaserScan2BB",
        safe_range=0.25
    )
    """
    Using the rotate class, create a node called "rotate_platform", and using the stop_motion class, create a node called "stop_platform"
    """
    rotate_platform = rotate(
        name="Rotate Platform"
    )
    stop_platform = stop_motion(
        name="Stop Platform"
    )

    move_platform = move_2bb(
        name="Move Platform"
    )
    align_platform = align(
        name="Align Platform"
    )
    follow_platform = follow_wall(
        name= "Follow Wall")
    
    no_wall_check=no_wall(
        name="No Wall")
    
    """
    Read the 'battery_low_warning' and 'collision_warning' from the blackboard and set a decorator node called "Battery Low?" to check if the battery is low 
    and "Colliding?" to check if any obstacle is within minimum distance.
    Please refer to the pt documentation for more information on decorators.
    """
    def check_battery_low_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning
    def check_collision(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.collision_warning 
    def check_wall(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.wall_reached
    def check_align(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.position 
    def check_follow(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.follow 
    def check_no_wall(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.wall_found
    
    # Define Battery Low? decorator node
    battery_emergency = pt.decorators.EternalGuard(

                name="Battery Low?",
                condition=check_battery_low_on_blackboard,
                blackboard_keys={"battery_low_warning"},
                child=rotate_platform
            )

    # Define Colliding? decorator node
    collision_emergency = pt.decorators.EternalGuard(
            name="Colliding?",
            condition=check_collision,
            blackboard_keys={"collision_warning"},
            child=stop_platform
        )
    
    # Define Wall Behaviour? decorator node
    wall_behaviour=pt.composites.Sequence("Wall Behaviour?",memory=True)
    
    # Define Move to Wall Behaviour? decorator node
    move_to_wall = pt.decorators.EternalGuard(
            name="Move to Wall?",
            condition=check_wall,
            blackboard_keys={"wall_reached"},
            child=move_platform
        )

    # Define Align to Wall Behaviour? decorator node
    align_to_wall = pt.decorators.EternalGuard(
            name="Align to Wall?",
            condition=check_align,
            blackboard_keys={"position"},
            child=align_platform
        )
    
    # Define Follow Wall Behaviour? decorator node
    following_wall = pt.decorators.EternalGuard(
            name="Follow Wall?",
            condition=check_follow,
            blackboard_keys={"follow"},
            child=follow_platform
        )
    
    # Define No Wall Behaviour? decorator node
    no_wall_found=pt.decorators.EternalGuard(
            name="No Wall?",
            condition=check_no_wall,
            blackboard_keys={"wall_found"},
            child=no_wall_check
    )
    idle = pt.behaviours.Running(name="Idle")

    """
    Construct the behaviour tree structure using the nodes and 
    behaviors defined above
    """

    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    topics2bb.add_child(laser2bb)
    root.add_child(priorities)
    priorities.add_children([move_to_wall,collision_emergency, no_wall_found, wall_behaviour,idle])
    wall_behaviour.add_children([align_to_wall,following_wall])

    return root


def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
   
        )
    """
    construct the behvior tree structure using the nodes and behaviors defined above
    """
  
    try:
        tree.setup(timeout=70.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    tree.tick_tock(period_ms=200)    
    
    try:
        rclpy.spin(tree.node)
        

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:

        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()