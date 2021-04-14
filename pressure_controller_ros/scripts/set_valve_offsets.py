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
from pynput.keyboard import Key, Listener

from colorama import init as cinit
from termcolor import colored as cterm
import re



class ValveOffsetSender:
    def __init__(self):
        self.transition_time=0.5
        self.curr_ind=0
        self.curr_ind_tmp = 0;
        self.curr_pressures = []
        self.num_channels=0
        self._outputs = False


        self._client = actionlib.SimpleActionClient('pressure_control', pressure_controller_ros.msg.CommandAction)
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


        self._listener = Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self._listener.start()

        self.redraw()



        #later: get this automatically from listening for messages in the echo topic

    def ack_waiter(self,data):
        if data.command == "set":
            self.num_channels = len(data.args)-1
            self.curr_pressures =[]
            for x in data.args[1:]:
                if x !='':
                    self.curr_pressures.append(float(x))
            print(self.curr_pressures)




    def set_press(self, pressure):
        if self.curr_ind ==-1:
            self.send_command("set",[self.transition_time,pressure])
        else:
            new_pressure = self.curr_pressures[:]
            new_pressure[self.curr_ind] = pressure
            new_pressure.insert(0,self.transition_time)

            self.send_command("set",new_pressure)



    def all_zero(self):
        self.send_command("set",[self.transition_time,0])

    def data_stream(self):
        self._outputs = not self._outputs
        if self._outputs:
            self.send_command("on",[])
        else:
            self.send_command("off",[])


    def ind_plus(self):
        self.curr_ind=int(np.clip(self.curr_ind+1,-1,self.num_channels-1))
        self.curr_ind_tmp = self.curr_ind


    def ind_minus(self):
        self.curr_ind=int(np.clip(self.curr_ind-1,-1,self.num_channels-1))
        self.curr_ind_tmp = self.curr_ind

    def ind_all(self):
        self.curr_ind=-1

    def ind_res(self):
        if self.curr_ind_tmp is not None:
            self.curr_ind = self.curr_ind_tmp



    
    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.r.sleep()
                inp = str(raw_input())

                inp = re.sub('[^0-9 .-]','', inp)

                if inp is not None:
                    self.set_press(float(inp))
                
                self.redraw(True)

            except:
                raise


    def redraw(self,extra=False):
        print('\r',end='')

        if extra:
            sys.stdout.write("\033[K") #clear line
            sys.stdout.write("\033[A") #Up one
            sys.stdout.write("\033[K") #clear line
            sys.stdout.write("\033[A") #Up one

        sys.stdout.write("\033[K") #clear line
        sys.stdout.write("\033[A") #Up one
        sys.stdout.write("\033[K") #clear line
        sys.stdout.write("\033[A") #Up one
        sys.stdout.write("\033[K") #clear line

        out_str = ""
        if self._outputs:
            out_str+=cterm("Data ON!", 'blue',attrs=['bold'])
        else:
            out_str+=cterm("Data OFF", 'blue')

        for idx, pres in enumerate(self.curr_pressures):
                if self.curr_ind == idx or self.curr_ind == -1:
                    out_str+= cterm("\t%0.3f"%(pres), 'green', attrs=['bold', 'underline'])
                else:
                    out_str+= cterm("\t%0.3f"%(pres), 'green')

        print(out_str+'\t')
        print("New Pressure: ")



                   

    def send_command(self, command, args, wait_for_ack = True):
        command, args = validate_commands.go(command, args)
        # Send commands to the commader node and wait for things to be taken care of
        goal = pressure_controller_ros.msg.CommandGoal(command=command, args=args, wait_for_ack = wait_for_ack)
        self._client.send_goal(goal)
        self._client.wait_for_result()

        if not self._client.get_result():
            raise ('Something went wrong and a setting was not validated')
            pass
        else:
            pass

      
    def shutdown(self):
        print("_Stopping setpoint follower")
        self.send_command("off",[],wait_for_ack=False)
        self.send_command("set",[0,0])
        self.send_command("echo",False)
        self._client.cancel_all_goals()
        self._listener.stop()



    def on_press(self,key):
        pass


    def on_release(self,key):
        if key == Key.esc:
            self.all_zero()
            #self.redraw()

        elif key == Key.left:
            self.ind_minus()
            self.redraw()

        elif key == Key.right:
            self.ind_plus()
            self.redraw()

        elif key == Key.up:
            self.ind_all()
            self.redraw()

        elif key == Key.down:
            self.ind_res()
            self.redraw()

        elif key == Key.shift:
            self.data_stream()
            self.redraw()




if __name__ == '__main__':
    try:
        cinit() 

        rospy.init_node('run_traj_node', disable_signals=True)
        

        node = PressureSender()

        node.spin()


    except KeyboardInterrupt:
        node.shutdown()
        


    except rospy.ROSInterruptException:
       print("program interrupted before completion", file=sys.stderr)