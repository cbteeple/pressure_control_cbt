#! /usr/bin/env python
from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from pressure_controller_ros.msg import *


import time
import sys
import os
import numbers
import numpy as np



class trajSender:
    def __init__(self, filename):
        self.command_client = actionlib.SimpleActionClient('pressure_control', CommandAction)
        self.traj_client = actionlib.SimpleActionClient('hand/pressure_trajectory', PressureTrajectoryAction)
        self.command_client.wait_for_server()
        self.traj_client.wait_for_server()

        self.speed_factor = rospy.get_param(rospy.get_name()+'/speed_factor')

        self.num_channels = rospy.get_param('/config_node/channels/num_channels')

        
        # Get trajectory from the parameter server
        all_settings = rospy.get_param(rospy.get_name())
        self.DEBUG = all_settings.get("DEBUG",False)
        self.traj = all_settings.get("setpoints")
        self.wrap = all_settings.get("wrap")  

        
        self.fix_traj()

        #IN LINUX: Fastest rate for individual data is ~100Hz. With ROS implementation, 30 HZ is stable
        self.send_rate =  50

        self.r = rospy.Rate(self.send_rate)



    def fix_traj(self):
        num_pad = self.num_channels - (len(self.traj[0])-1)
        traj_arr = np.asarray(self.traj)
        # If the trajectory is too small, pad with zeros
        if num_pad>0:
            pad = np.zeros(len(self.traj),num_pad)
            traj_arr = np.concatenate(traj_arr, pad, axis=1)

            rospy.loginfo('%s: Padding trajectory with %d columns of zeros.' % (rospy.get_name(), num_pad))

        # If the trajectory is too big, cut off the end
        elif num_pad<0:
            traj_arr = traj_arr[:,0:1+self.num_channels]
            

            rospy.loginfo('%s: Clipping trajectory after %d columns.' % (rospy.get_name(), self.num_channels))
        # Otherwise, do nothing. The trajectory is already the correct size!
        else:
            pass


        # Update the speed multiplier and add indices to the front
        traj_arr[:,0] = self.speed_factor*traj_arr[:,0]


        self.traj = traj_arr.tolist()


    # build the whole trajectory
    def build_traj(self):
        self.traj_goal = PressureTrajectoryGoal()
        self.traj_goal.trajectory = PressureTrajectory()

        for entry in self.traj:
            self.traj_goal.trajectory.points.append(PressureTrajectoryPoint(pressures=entry[1:], time_from_start=rospy.Duration(entry[0])))



    def send_traj(self):
        self.send_command("_flush",[])
        self.send_command("echo",False,wait_for_ack = False)
        self.send_command("on",[],wait_for_ack = False)
        
        self.send_command("mode" ,3,wait_for_ack = False)

        self.traj_client.send_goal(self.traj_goal)
        self.traj_client.wait_for_result()



            

    def send_command(self, command, args, wait_for_ack=True):
        # Validate inputs
        if not isinstance(command, str):
            raise ValueError('CONFIG: Command must be a string')

        if isinstance(args, list) or isinstance(args, tuple):
            pass
        elif isinstance(args, numbers.Number):
            args=[args]
        else:
            raise ValueError('CONFIG: Args must be a list, tuple, or number')

        # Send commands to the commader node and wait for things to be taken care of
        goal = CommandGoal(command=command, args=args, wait_for_ack = wait_for_ack)
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()

        if not self.command_client.get_result():
            raise serial_coms.Issue('Something went wrong and a setting was not validated')
        else:
            pass



    def shutdown(self):
        self.send_command("off",[],wait_for_ack=False)
        self.send_command("echo",True)
        self.send_command("mode",1)
        self.send_command("set",[0,0])
        self.send_command("echo",False)
        self.command_client.cancel_all_goals()
        




if __name__ == '__main__':
    try:
        rospy.init_node('send_traj_node')
        profile = rospy.get_param(rospy.get_name()+'/traj_profile')
        print("LIVE TRAJECTORY FOLLOWER: Uploading Trajectory '%s'"%(profile))  
        node = trajSender(profile)
        node.build_traj()
        node.send_traj()
        node.shutdown()


    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)