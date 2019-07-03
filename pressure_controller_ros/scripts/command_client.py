#! /usr/bin/env python
from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import pressure_controller_ros.msg

def command_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    print("Starting the client")
    client = actionlib.SimpleActionClient('pressure_control', pressure_controller_ros.msg.CommandAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waiting for server")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    print("creating the gaol")
    goal = pressure_controller_ros.msg.CommandGoal(command="set", args=[5,5], wait_for_ack = False)

    # Sends the goal to the action server.
    print("sending the goal")
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    print("waiting for result")
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('command_client')
        result = command_client()
        print("Result: %d"%(result.success))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)