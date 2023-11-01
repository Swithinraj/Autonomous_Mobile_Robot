#!/usr/bin/env python3

import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
import rclpy
import sys
from autonomous_map_navigate.behaviors import *

def create_root():
    """
    Method to structure the behavior tree to monitor battery status and start rotation if battery is low
    
    The "battery_monitor" behavior tree writes the voltage of the battery on 'blackboard' of behavior tree. 
    If the battery level is lower than a threshold, the robot will execute certain behavior. Here the robot 
    rotation is used to indicate low level of the battery
    """
    ## define nodes and behaviors

    # defining root node
    root = pt.composites.Parallel(
        name="root",
        policy=pt.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )    

    """
    Create a sequence node called "Topics2BB" and a selector node called "Priorities"    
    """
    ### YOUR CODE HERE ###
    topics2bb = pt.composites.Sequence("Topics2BB",memory=True)
    priorities = pt.composites.Selector("Priorities",memory=False)
    """
    Using the battery_status2bb class, create a node called "Battery2BB" which subscribes to the topic "/battery_voltage"
    """
    ### YOUR CODE HERE ###
    battery2bb=battery_status2bb(
        name="Battery2BB",
        topic_name="/battery_voltage",
        threshold=30.0
    )

    """
    Using the rotate class, create a node called "rotate_platform"
    """
    ### YOUR CODE HERE ###
    rotate_platform = rotate(
        name="Rotate Platform"
    )

    """
    Read the 'battery_low_warning' from the blackboard and set a decorator node called "Battery Low?" to check if the battery is low.
    Please refer to the py_trees documentation for more information on decorators.
    """
    ### YOUR CODE HERE ### 
    def check_battery_low_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning
    
    # Define Battery Low? decorator node
    battery_emergency = pt.decorators.EternalGuard(

                name="Battery Low?",
                condition=check_battery_low_on_blackboard,
                blackboard_keys={"battery_low_warning"},
                child=rotate_platform
            )



    idle = pt.behaviours.Running(name="Idle")
    

    """
    construct the behavior tree structure using the nodes and behaviors defined above
    """

    ### YOUR CODE HERE ###
    root.add_child(topics2bb)
    topics2bb.add_child(battery2bb)
    
    root.add_child(priorities)
    priorities.add_children([battery_emergency, idle])
    battery_emergency.add_child(rotate_platform)
  

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

    # setup the tree
    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=10)    
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()    