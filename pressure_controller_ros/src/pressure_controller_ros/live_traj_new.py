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
import copy
import yaml

filepath_default = os.path.join('..','trajectories')



class trajSender:
    def __init__(self, speed_factor = 1.0, name = "pressure_control"):
        self.filepath_default = filepath_default

        self._name = name


        self.command_client = actionlib.SimpleActionClient(self._name, CommandAction)
        self.traj_client = actionlib.SimpleActionClient(self._name+'/pressure_trajectory', PressureTrajectoryAction)
        self.command_client.wait_for_server()
        self.traj_client.wait_for_server()

        if speed_factor <=0.0:
            speed_factor = 1.0

        self.speed_multiplier = 1./speed_factor
        self.DEBUG = rospy.get_param(rospy.get_name()+"/debug",False)

        self.num_channels = np.sum(rospy.get_param(self._name+'/num_channels'))

        


    def load_trajectory(self, filename, filepath=None):

        if filename is None:
            # Get trajectory from the parameter server
            all_settings = rospy.get_param(rospy.get_name())
            
        else:
            # Load the trajectory from a file
            if filepath is None:
                # Use the default file path if we don't give it one explicitly
                filepath = self.filepath_default

            inFile=os.path.join(filepath,'hand',filename+".yaml")
            with open(inFile,'r') as f:
                # use safe_load instead of load
                all_settings = yaml.safe_load(f)
                f.close()

        # Extract settings
        traj = all_settings.get("setpoints",None)

        # Fix errors in the trajectory before building it
        traj_fixed = self.fix_traj(traj)

        return traj_fixed



    def fix_traj(self, traj):
        num_pad = self.num_channels - (len(traj[0])-1)
        traj_arr = np.asarray(traj)
        # If the trajectory is too small, pad with zeros
        if num_pad>0:
            pad = np.zeros((len(traj),num_pad))
            traj_arr = np.hstack((traj_arr, pad))

            rospy.loginfo('%s: Padding trajectory with %d columns of zeros.' % (rospy.get_name(), num_pad))

        # If the trajectory is too big, cut off the end
        elif num_pad<0:
            traj_arr = traj_arr[:,0:1+self.num_channels]
            

            rospy.loginfo('%s: Clipping trajectory after %d columns.' % (rospy.get_name(), self.num_channels))
        # Otherwise, do nothing. The trajectory is already the correct size!
        else:
            pass


        # Update the speed multiplier and add indices to the front
        #traj_arr[:,0] = self.speed_multiplier*traj_arr[:,0]

        return traj_arr.tolist()


    # build the whole trajectory
    def build_traj(self, traj):
        traj_goal = PressureTrajectoryGoal()
        traj_goal.trajectory = PressureTrajectory()

        for entry in traj:
            traj_goal.trajectory.points.append(PressureTrajectoryPoint(
                        pressures       = entry[1:],
                        time_from_start = self.speed_multiplier*rospy.Duration(entry[0])))

        return traj_goal



    def go_to_start(self, traj_goal, reset_time, blocking=True):
        self.start_pressures = self.fix_traj([traj_goal[0]])[0][1:]

        if self._name !='servo':
            self.send_command("_flush",[])
            self.send_command("mode" ,3,wait_for_ack = False)
            self.send_command("_flush",[])
            
        self.send_command("echo",False,wait_for_ack = False)      
        self.send_command("on",[],wait_for_ack = False)

        #print('Waiting for current state')
        current_states = rospy.wait_for_message("/"+self._name+"/pressure_data", DataIn)
        #print('Got current state')

        goal_tmp = PressureTrajectoryGoal()
        goal_tmp.trajectory = PressureTrajectory()

        #print(len(current_states.measured),current_states.measured)
        #print(len(self.start_pressures),self.start_pressures)
        goal_tmp.trajectory.points.append(PressureTrajectoryPoint(pressures=current_states.measured, time_from_start=rospy.Duration(0.0)))
        goal_tmp.trajectory.points.append(PressureTrajectoryPoint(pressures=self.start_pressures, time_from_start=rospy.Duration(traj_goal[0][0])))

        self.execute_traj(goal_tmp, blocking)





    def execute_traj(self, traj_goal, blocking=True):
        #try:
        if self.DEBUG:
            print(traj_goal)
        self.traj_client.send_goal(traj_goal)

        if self.DEBUG:
            print('Sent Trajectory')

        if blocking:
            self.traj_client.wait_for_result()
        else:
            return self.traj_client

        #except KeyboardInterrupt:
        #    self.traj_client.cancel_goal()
        #except:
        #    raise


            

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

        if self.DEBUG:
            print(command, args)

        # Send commands to the commader node and wait for things to be taken care of
        goal = CommandGoal(command=command, args=args, wait_for_ack = wait_for_ack)
        self.command_client.send_goal(goal)
        self.command_client.wait_for_result()

        if not self.command_client.get_result():
            print('Something went wrong and a setting was not validated')
            raise
        else:
            pass



    def shutdown(self, reset=None):
        print('HAND CONTROLLER: Cancel current goals')
        self.traj_client.cancel_all_goals()

        
        self.send_command("echo",True)
        self.send_command("off",[],wait_for_ack=True)

        if self._name !='servo':
            print('HAND CONTROLLER: Switching to direct pressure control mode')
            self.send_command("mode",3)
            
        if reset is not None:
            print('HAND CONTROLLER: Setting resting pressures')

            if reset == 'resting':
                out_press= copy.deepcopy(self.start_pressures)
                out_press.insert(0, 2)
                print(out_press)
                self.send_command("set",out_press)
            else:
                self.send_command("set",reset)

        self.send_command("echo",False, wait_for_ack=False)
        self.command_client.cancel_all_goals()
        




if __name__ == '__main__':
    try:
        rospy.init_node('send_traj_node')
        profile = rospy.get_param(rospy.get_name()+'/traj_profile')
        print("LIVE TRAJECTORY FOLLOWER: Uploading Trajectory '%s'"%(profile))  
        node = trajSender()
        traj_built = node.load_trajectory(profile)
        inp=raw_input("Go to starting point?")
        node.go_to_start(traj_built)
        inp=raw_input("Continue?")
        node.execute_traj(traj_built)
        node.shutdown()


    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)