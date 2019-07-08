#!/usr/bin/env python

import rospy

#Import the actionlib library to use
import actionlib


#Import the specific messages that we created in our tutorials folder.
import pressure_controller_ros.msg as msg

#Import custom serial coms commands
import serial_coms
import validate_commands


class SetpointAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.CommandFeedback()
    _result = msg.CommandResult()

    def __init__(self, name):
        self._action_name = name

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._client = actionlib.SimpleActionClient('pressure_control', msg.CommandAction)
        self._client.wait_for_server()


        # start an actionlib server for converting setpoints
        self._as = actionlib.SimpleActionServer('set_setpoints', msg.SetpointAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def execute_cb(self, goal):
        
        # Initiatilize the feedback
        self._feedback.sent = False
        self._feedback.success = False

        
        # start executing the action
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:
            #Send a setpoint command
            rospy.logdebug([goal.transition_time]+list(goal.setpoints))
            self.send_command("set", [goal.transition_time]+list(goal.setpoints), wait_for_ack = False)
            self._feedback.sent = True                
            self._feedback.success = True
    
        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)



    def send_command(self, command, args, wait_for_ack = True):
        command, args = validate_commands.go(command, args)
        # Send commands to the commader node and wait for things to be taken care of
        goal = msg.CommandGoal(command=command, args=args, wait_for_ack = wait_for_ack)
        self._client.send_goal(goal)
        self._client.wait_for_result()

        if not self._client.get_result():
            raise serial_coms.SerialIssue('Something went wrong and a setting was not validated')
            pass
        else:
            pass


        
if __name__ == '__main__':
    rospy.init_node('set_setpoints')
    print("SETPOINT SERVER: Node Initiatilized (%s)"%(rospy.get_name()))
    server = SetpointAction(rospy.get_name())
    print("SETPOINT SERVER: Ready!")
    rospy.spin()