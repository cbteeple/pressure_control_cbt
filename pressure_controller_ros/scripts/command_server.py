#!/usr/bin/env python

import rospy

#Import the actionlib library to use
import actionlib


#Import the specific messages that we created in our tutorials folder.
import pressure_controller_ros.msg as msg

#Import custom serial coms commands
import serial_coms


class CommandAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.CommandFeedback()
    _result = msg.CommandResult()

    def __init__(self, name):
        self._action_name = name

        # begin serial communication
        self.devname = rospy.get_param(rospy.get_name()+'/devname')
        self.baud = rospy.get_param(rospy.get_name()+'/baudrate')

        self.ser = serial_coms.resume(self.devname,self.baud)
        print(self.ser)

        # start an actionlib server
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
                serial_coms.flushAll(self.ser)

            elif "read" in goal.command:
                ack="_"
                while ack is not None:
                    ack = serial_coms.readStuff(self.ser)
                    #self._as.publish_feedback(self._feedback)
                    if ack is not None:
                        print(ack)

            self._feedback.success = True
            self._as.publish_feedback(self._feedback)


        else:
            
            # publish info to the console for the user
            rospy.loginfo('%s: Executing, sending %s command.' % (self._action_name, goal.command))
            
            # start executing the action
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
            else:

                self._feedback.out_string = serial_coms.sendCommand(self.ser, goal.command, goal.args)
                self._feedback.sent = True

                if goal.wait_for_ack:
                    #Spin and wait for an acknowledgment from the controller
                    #   NOTE: Right now we are just waiting for serial data to stop comming.
                    #         Later we will use a second thread to interpret every incomming
                    #         line of serial data and separate acknowledgements from data

                    ack="_"
                    while ack is not None:
                        ack = serial_coms.readStuff(self.ser)
                        self._as.publish_feedback(self._feedback)
                        print(ack)
                        #r.sleep()
                    
                self._feedback.success = True
    
        
        
          
        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    print("Starting Server")
    rospy.init_node('pressure_control')
    print("Node Initiatilized")
    server = CommandAction(rospy.get_name())
    print("Server Spinning")
    rospy.spin()