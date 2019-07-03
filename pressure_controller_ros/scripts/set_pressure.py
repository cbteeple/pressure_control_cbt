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
from pynput.keyboard import Key, Listener


restartFlag = False


class setpointSender:
    def __init__(self):
        self._client = actionlib.SimpleActionClient('set_setpoints', pressure_controller_ros.msg.SetpointAction)
        self._client.wait_for_server()

        
        # Get setpoint from the parameter server
        all_settings = rospy.get_param(rospy.get_name())
        self.setpoints = all_settings.get("setpoints",None)
        print(self.setpoints)
        self.transition_time = all_settings.get("transition_time",None)
        print(self.transition_time)

        if self.setpoints is not None:
            self.send_setpoint(self.setpoints, self.transition_time)

            

    def send_setpoint(self, command, transition_time=1.0):
        
        goal = pressure_controller_ros.msg.SetpointGoal(setpoints=command, transition_time=transition_time)
        self._client.send_goal(goal)
        self._client.wait_for_result()

        if not self._client.get_result():
            rospy.logErr('Something happened with the setpoint server')
            pass
        else:
            pass





if __name__ == '__main__':
    try:

        rospy.init_node('setpoint_setting_test', disable_signals=True)
        node = setpointSender()        


    except rospy.ROSInterruptException:
       print("program interrupted before completion", file=sys.stderr)