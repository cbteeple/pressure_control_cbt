#! /usr/bin/env python
from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import pressure_controller_ros.msg
import validate_commands


import time
import sys
import os
import numbers
import numpy as np
import serial_coms



class trajSender:
    def __init__(self):
        self._client = actionlib.SimpleActionClient('pressure_control', pressure_controller_ros.msg.CommandAction)
        self._client.wait_for_server()

        
        # Get trajectory from the parameter server
        all_settings = rospy.get_param(rospy.get_name())
        self.data_back = all_settings.get("data_back")

        


    def start_traj(self):
        self.send_command("_flush",[])
        self.send_command("off",[])
        self.send_command("_flush",[])
        self.send_command("mode",2)
        self.send_command("trajstart",[])
        if self.data_back:
            self.send_command("on",[],wait_for_ack=False)

            while True:
                try:
                    self.send_command("_read",[])
                except KeyboardInterrupt:
                    break


            

    def send_command(self, command, args, wait_for_ack = True):
        command, args = validate_commands.go(command, args)
        # Send commands to the commader node and wait for things to be taken care of
        goal = pressure_controller_ros.msg.CommandGoal(command=command, args=args, wait_for_ack = wait_for_ack)
        self._client.send_goal(goal)
        self._client.wait_for_result()

        if not self._client.get_result():
            raise serial_coms.SerialIssue('Something went wrong and a setting was not validated')
            pass
        else:
            pass

      
    def shutdown(self):
        self.send_command("off",[],wait_for_ack=False)
        self.send_command("echo",True)
        self.send_command("trajstop",[])
        self.send_command("mode",1)
        self.send_command("set",0)
        self.send_command("echo",False)
        self._client.cancel_all_goals()




if __name__ == '__main__':
    try:
        rospy.init_node('run_traj_node', disable_signals=True)
        node = trajSender()
        node.start_traj()
        node.shutdown()
        


    except rospy.ROSInterruptException:
       print("program interrupted before completion", file=sys.stderr)