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



class trajSender:
    def __init__(self):
        self.r = rospy.Rate(100)

        rospy.Subscriber('pressure_control/echo', pressure_controller_ros.msg.Echo, self.ack_waiter)


        self._client = actionlib.SimpleActionClient('pressure_control', pressure_controller_ros.msg.CommandAction)
        self._client.wait_for_server()
        self._running = False
        self._reset = False
        self._paused=False

        # Get trajectory from the parameter server
        all_settings = rospy.get_param(rospy.get_name())
        self.data_back = all_settings.get("data_back")

        self.send_command("echo",True)
        self.send_command("_flush",[])
        self.send_command("off",[],wait_for_ack=False)
        self.send_command("_flush",[])
        self.send_command("mode",2,wait_for_ack=False)

        if self. data_back:
            self.send_command("on",[],wait_for_ack=False)


        listener = Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()



    def start_traj(self):
        self.send_command("trajstart",[],wait_for_ack=False)
        #self._running = True
        self._reset = False


    def stop_traj(self):
        self.send_command("trajstop",[],wait_for_ack=False)
        #self._running = False
        self._reset = True


    def pause_traj(self):
        self.send_command("trajpause",[],wait_for_ack=False)
        self._paused = True
        #self._running = False


    def resume_traj(self):
        self.send_command("trajresume",[],wait_for_ack=False)
        self._paused = False
        #self._running = True

    
    def spin(self):
        print("Spinning")
        while not rospy.is_shutdown():
            self.r.sleep()

                    

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


    def ack_waiter(self,data):
        if data.command == "traj":
            if data.args[0] == "Start":
                self._running=True
                print('Started....')
            else:
                self._running=False
                print('Stopped....')






    def on_press(self,key):
        pass


    def on_release(self,key):
        if key == Key.space:
            if self._reset or not self._running:
                print('_START')
                self.start_traj()

            else:
                if self._paused:
                    print('_RESUME')
                    self.resume_traj()
                else:
                    print('_PAUSE')
                    self.pause_traj()
                

        elif key == Key.alt:
            print('_RESET')
            self.stop_traj()




if __name__ == '__main__':
    try:

        rospy.init_node('run_traj_node', disable_signals=True)
        

        node = trajSender()


        print("TRAJECTORY FOLLOWER")  

        print("\nControls:")
        print("\tSPACE  - Restart trajectory")
        print("\tALT    - Stop trajectory")
        print("\tCTRL-C - Stop running")
        print('')

        node.spin()


    except KeyboardInterrupt:
        node.shutdown()
        


    except rospy.ROSInterruptException:
       print("program interrupted before completion", file=sys.stderr)