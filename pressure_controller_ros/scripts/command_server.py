#!/usr/bin/env python

import rospy
import actionlib


#Import the specific messages that we created in our tutorials folder.
import pressure_controller_ros.msg as msg

#Import custom serial coms, threading, and queueing
import serial_coms
import threading


class CommandAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.CommandFeedback()
    _result = msg.CommandResult()

    def __init__(self, name):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._action_name = name
        self.ack_buffer = []

        # Begin serial communication
        self.devname = rospy.get_param(rospy.get_name()+'/devname')
        self.baud = rospy.get_param(rospy.get_name()+'/baudrate')

        self.ser = serial_coms.SerialComs(self.devname,self.baud)

        if not self.ser.connected:
            raise Exception("Serial port was not connected")

        # Start a serial reader in a second thread.
        # The polling rate only affects how often data gets read. It can be read in large blocks and doesn't take much time at all
        self.ser.start_read_thread(poll_rate=500, reading_cb=self.process_serial_in)

        # Start some message publishers and subscribers
        self.data_pub = rospy.Publisher('pressure_control/pressure_data', msg.DataIn, queue_size=10)
        self.echo_pub = rospy.Publisher('pressure_control/echo', msg.Echo, queue_size=10)
        rospy.Subscriber('pressure_control/echo', msg.Echo, self.ack_waiter)

        # Start an actionlib server
        self._as = actionlib.SimpleActionServer('pressure_control', msg.CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def execute_cb(self, goal):

        # helper variables
        r = rospy.Rate(3000)
        success = False
        
        # Initiatilize the feedback
        self._feedback.sent = False
        self._feedback.success = False
        self._feedback.out_string = ""

        if goal.command.startswith("_"):


            if "flush"  in goal.command:
                rospy.loginfo('%s: Flushing serial coms' % (self._action_name))
                self.ser.flushAll()

            elif "read" in goal.command:
                print("NEED TO FIX READING FUNCTION IN COMMAND SERVER")
                #ack="_"
                #while ack is not None:
                #    ack = self.ser.readStuff()
                #    #self._as.publish_feedback(self._feedback)
                #    if ack is not None:
                #        print(ack)

            self._feedback.success = True
            self._as.publish_feedback(self._feedback)


        else:           
            # start executing the action
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
            else:

                self._feedback.out_string = self.ser.sendCommand(goal.command, goal.args)
                self._feedback.sent = True

                if goal.wait_for_ack:
                    #Spin and wait for an acknowledgment from the controller
                    ack_curr = ""
                    cmd = str(goal.command).lower()
                    while ack_curr != cmd :
                        if len(self.ack_buffer)>0:
                            ack_curr = self.ack_buffer.pop().command
                        r.sleep()

                    
                    self.ack_buffer = []
                    
                self._feedback.success = True
    
        
        
          
        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


    def process_serial_in(self, line_in):
        if not line_in:
            return

        if line_in.startswith('_'):
            #Look for an underscore - This is an echo response
            line_in=line_in.replace("_NEW ",'')
            line_in=line_in.strip('_')
            line_split = line_in.split(": ")

            cmd = line_split[0].strip(' ')

            if len(line_split) <= 1:
                args = ""
            else:
                args = line_split[1].split('\t')

            echo_in = msg.Echo()
            echo_in.command = str(cmd).lower() 
            echo_in.args = args

            if self.DEBUG:
                rospy.loginfo(echo_in)

            self.echo_pub.publish(echo_in)

        else:
            #All other incomming lines are tab-separated data
            line_split = line_in.split('\t')

            data_in = msg.DataIn();
            data_in.time = long(line_split[0])
            data_in.setpoints = [float(i) for i in line_split[2::2]] 
            data_in.measured  = [float(i) for i in line_split[3::2]]

            if self.DEBUG:
                rospy.loginfo(data_in)

            self.data_pub.publish(data_in)




    def ack_waiter(self,data):
        self.ack_buffer.append(data)




    def shutdown(self):
        self.ser.shutdown()


        
if __name__ == '__main__':
    try:
        rospy.init_node('pressure_control', disable_signals=True)
        print("COMMAND SERVER: Node Initiatilized (%s)"%(rospy.get_name()))
        server = CommandAction(rospy.get_name())
        print("COMMAND SERVER: Ready!")
        rospy.spin()

    except KeyboardInterrupt:
        print("COMMAND SERVER: Shutting Down")
        server.shutdown()

    except rospy.ROSInterruptException:
        print("COMMAND SERVER: Shutting Down")
        server.shutdown()

    