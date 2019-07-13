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

import colorama.init 
import termcolor.colored as cterm



class PressureSender:
    def __init__(self):
        self.transition_time=0.5
        self.curr_ind=0
        self.curr_pressures = []
        self.num_channels=0


        self._client = actionlib.SimpleActionClient('set_setpoints', pressure_controller_ros.msg.SetpointAction)
        self._client.wait_for_server()

        rospy.Subscriber('pressure_control/echo', pressure_controller_ros.msg.Echo, self.ack_waiter)

        
        # Get trajectory from the parameter server
        all_settings = rospy.get_param(rospy.get_name())
        self.data_back = all_settings.get("data_back")

        self.send_command("_flush",[])
        self.send_command("off",[],wait_for_ack=False)
        self.send_command("_flush",[])
        self.send_command("echo",True)
        self.send_command("mode",3)

        #Get the curent setpoint
        self.send_command("set",[])
        if self.data_back:
            self.send_command("on",[])

        self.r = rospy.Rate(100)


        listener = Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()



        #later: get this automatically from listening for messages in the echo topic

    def ack_waiter(self,data):
        if data.command == "set"
            self.num_channels = len(data.args)
            self.curr_pressures = data.args




    def set_press(self, pressure):
        if self.curr_ind ==-1:
            self.send_command("set",[self.transition_time,pressure])
        else:
            new_pressure = curr_pressures[:]
            new_pressure[self.curr_ind] = pressure
            self.send_command("set",[self.transition_time].extend(new_pressure))



    def all_zero(self):
        self.send_command("set",[self.transition_time,0])


    def ind_plus(self):
        self.curr_ind=int(np.clip(self.curr_ind+1,-1,self.num_channels-1))


    def ind_minus(self):
        self.curr_ind=int(np.clip(self.curr_ind-1,-1,self.num_channels-1))

    
    def spin(self):
        while not rospy.is_shutdown():
            try:
                inp = raw_input("New Pressure: ")

                if len(inp)==1:
                    set_press(self, inp)

                if len(inp)==self.num_channels and self.curr_ind ==-1:
                    set_press(self, inp)

            except:
                pass


    def redraw(self):
        print('\r',end='')
        sys.stdout.write("\033[K") #clear line

        out_str = ""

        for idx, pres in enumerate(self.curr_pressures):
                if self.curr_ind == idx or self.curr_ind == -1:
                    out_str+= colored("\t%0.3f"%(pres), 'black','on_green')
                else:
                    out_str+= colored("\t%0.3f"%(pres), 'green')

        print(our_str+'\t')

                    

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
        self.send_command("set",[0,0])
        self.send_command("echo",False)
        self._client.cancel_all_goals()



    def on_press(self,key):
        pass


    def on_release(self,key):       
        if key == Key.esc:
            self.all_zero()
            self.redraw()

        elif key == Key.left:
            self.ind_minus()
            self.redraw()

        elif key == Key.right:
            self.ind_plus()
            self.redraw()




if __name__ == '__main__':
    try:
        colorama.init() 

        rospy.init_node('run_traj_node', disable_signals=True)
        

        node = PressureSender()


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