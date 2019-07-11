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
    _feedback = pressure_controller_ros.msg.RunFeedback()
    _result = pressure_controller_ros.msg.RunResult()

    def __init__(self, name):

        self.mode_set = False
        self.r = rospy.Rate(100)
        self.ack_buffer = []

        self._client = actionlib.SimpleActionClient('pressure_control', pressure_controller_ros.msg.CommandAction)
        self._client.wait_for_server()

        # start an actionlib server for converting setpoints
        self._action_name = name
        self._as = actionlib.SimpleActionServer('pre_built_traj', pressure_controller_ros.msg.RunAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.Subscriber('pressure_control/echo', pressure_controller_ros.msg.Echo, self.ack_waiter)

               
        # Get trajectory from the parameter server
        all_settings = rospy.get_param(rospy.get_name())
        self.data_back = all_settings.get("data_back",False)

        
        

    def execute_cb(self, goal):

        if not self.mode_set:
            self.send_command("_flush",[])
            self.send_command("off",[],wait_for_ack=False)
            self.send_command("_flush",[])
            self.send_command("mode",2,wait_for_ack=False)
            self.mode_set = True

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:

            self.ack_buffer = []

            if goal.data:
                self.start_data()

            if goal.traj:
                self.start_traj()
                
                if goal.wait_for_finish:
                    self.wait_for_match("trajstop")
                self._feedback.success = True
            
            elif not goal.traj:
                self.stop_traj()
                
                if goal.wait_for_finish:
                    self.wait_for_match("trajstop")
                self._feedback.success = True


            if not goal.data:
                self.stop_data()

        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)



    def restart_traj(self):
        self.send_command("trajstart",[],wait_for_ack=False)


    def start_traj(self):
        self.send_command("mode",2,wait_for_ack=False)
        self.send_command("trajstart",[],wait_for_ack=False)


    def stop_traj(self):
        self.send_command("trajstop",[],wait_for_ack=False)


    def start_data(self):
        if self.data_back:
            self.send_command("on",[],wait_for_ack=False)

    def stop_data(self):
        if self.data_back:
            self.send_command("off",[],wait_for_ack=False)


    def ack_waiter(self,data):
        self.ack_buffer.append(data)


    def wait_for_match(self,match_str="trajstop"):
        #Spin and wait for an acknowledgment from the controller
        ack_curr = ""
        cmd = str(match_str).lower()
        while ack_curr != cmd :
            if len(self.ack_buffer)>0:
                ack_curr = self.ack_buffer.pop().command
            r.sleep()

        self.ack_buffer = []

    
            

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
        print("_Stopping trajectory follower")
        self.send_command("off",[],wait_for_ack=False)
        self.send_command("echo",True)
        self.send_command("trajstop",[])
        self.send_command("mode",1)
        self.send_command("set",[0,0])
        self.send_command("echo",False)
        self._client.cancel_all_goals()




if __name__ == '__main__':
    try:
        rospy.init_node('run_traj_server', disable_signals=True)
        node = trajSender(rospy.get_name())
        rospy.spin()
        


    except rospy.ROSInterruptException:
       node.shutdown()
       print("program interrupted before completion", file=sys.stderr)