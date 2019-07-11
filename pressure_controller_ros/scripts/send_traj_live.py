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



class trajSender:
    def __init__(self, filename):
        self._client = actionlib.SimpleActionClient('pressure_control', msg.CommandAction)
        self._client.wait_for_server()

        self.speed_factor = rospy.get_param(rospy.get_name()+'/speed_factor')

        self.num_channels = rospy.get_param('/config_node/channels/num_channels')

        
        # Get trajectory from the parameter server
        all_settings = rospy.get_param(rospy.get_name())
        self.DEBUG = all_settings.get("DEBUG",False)
        self.traj = all_settings.get("setpoints")
        self.wrap = all_settings.get("wrap")  

        
        self.fix_traj()

        #IN LINUX: Fastest rate for individual data is ~100Hz. With ROS implementation, 30 HZ is stable
        self.send_rate =  30

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
        inds = np.vstack(np.arange(0,len(self.traj)))
        traj_arr = np.concatenate((inds, traj_arr), axis=1)


        self.traj = traj_arr.tolist()



    def send_traj(self):
        self.send_command("_flush",[])
        self.send_command("echo",False,wait_for_ack = False)
        self.send_command("on",[],wait_for_ack = False)
        
        self.send_command("mode" ,3,wait_for_ack = False)

        # Send the whole trajectory
        start_time = time.time()
        len_traj = len(self.traj)
        for idx, entry in enumerate(self.traj):
            # Send each line to the action server and DONT WAIT
            self.send_command('set',[1./self.send_rate]+entry[2:], wait_for_ack = False)
            #if not idx%5:
            #    print('\r'+"LIVE TRAJECTORY FOLLOWER: Uploading Trajectory, %0.1f"%((idx+1)/float(len_traj)*100) +'%' +" complete", end='')
            #    sys.stdout.flush()

            self.r.sleep()

        end_time = time.time()

        print('\r',end='')
        sys.stdout.write("\033[K") #clear line
        print("LIVE TRAJECTORY FOLLOWER: Uploading Trajectory, %d points, Done in %0.2f sec"%(len_traj,end_time - start_time))


        self.send_command("echo",False)

            

    def send_command(self, command, args, wait_for_ack=True):
        command, args = validate_commands.go(command, args)
        # Send commands to the commader node and wait for things to be taken care of
        goal = msg.CommandGoal(command=command, args=args, wait_for_ack = wait_for_ack)
        self._client.send_goal(goal)
        self._client.wait_for_result()

        if not self._client.get_result():
            raise serial_coms.SerialIssue('Something went wrong and a setting was not validated')
        else:
            pass



    def shutdown(self):
        self.send_command("off",[],wait_for_ack=False)
        self.send_command("echo",True)
        self.send_command("mode",1)
        self.send_command("set",[0,0])
        self.send_command("echo",False)
        self._client.cancel_all_goals()
        




if __name__ == '__main__':
    try:
        rospy.init_node('send_traj_node')
        profile = rospy.get_param(rospy.get_name()+'/traj_profile')
        print("LIVE TRAJECTORY FOLLOWER: Uploading Trajectory '%s'"%(profile))  
        node = trajSender(profile)
        node.send_traj()
        node.shutdown()


    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)