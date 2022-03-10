#!/usr/bin/env python

import time
import rospy
import actionlib


#Import the specific messages that we created.
import pressure_controller_ros.msg as msg





class CommandAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.CommandFeedback()
    _result = msg.CommandResult()

    def __init__(self, name, comms_obj, command_handler, cmd_sleep_time=0.0):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._action_name = name
        self.ack_buffer = []
        self.cmd_sleep_time = cmd_sleep_time

        self.comms=comms_obj

        self.command_handler = command_handler

        # Start some message publishers and subscribers
        rospy.Subscriber(self._action_name+'/echo', msg.Echo, self.ack_waiter)


        # Start an actionlib server
        self._as = actionlib.SimpleActionServer(self._action_name, msg.CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


    def send_command(self, cmd, args):
        """
        Split up commands to various devices

        Parameters
        ----------
		cmd : str
            a command to use
		args : list
            arguments for the command

		OUTPUTS:
			out_str - the output string sent
        """

        # Split the command into two:
        commands = self.command_handler.split_command(cmd, args)

        if self.DEBUG:
            rospy.loginfo(commands)

        if len(commands) != len(self.comms):
            raise ValueError("COMM HANDLER: length of command list does not equal the number of devices")

        cmd_str = ""
        for idx, command in enumerate(commands):
            if command is not None:
                cmd_str+= self.comms[idx]['interface'].sendCommand(command['command'],command['values'], self.comms[idx]['cmd_format'])
        
        return cmd_str


    def execute_cb(self, goal):

        #rospy.loginfo("Start: %s"%(goal.command))

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
                for comm in self.comms:
                    comm['interface'].flushAll()

            self._feedback.success = True
            self._as.publish_feedback(self._feedback)


        else:           
            # start executing the action
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
            else:

                cmd_str = self.send_command(goal.command, goal.args)

                self._feedback.out_string = cmd_str
                #self._feedback.out_string = self.comms.sendCommand(goal.command, goal.args)
                self._feedback.sent = True

                if goal.wait_for_ack:
                    #Spin and wait for an acknowledgment from the controller
                    ack_curr = ""
                    cmd = str(goal.command).lower()
                    while ack_curr != cmd and not rospy.is_shutdown() and not self._as.is_preempt_requested():
                        if len(self.ack_buffer)>0:
                            ack_curr = self.ack_buffer.pop().command
                        r.sleep()

                    
                    self.ack_buffer = []
                    
                self._feedback.success = True
                
                if self.cmd_sleep_time >0:
                    time.sleep(self.cmd_sleep_time)
    
        
        
          
        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            #rospy.loginfo("End: %s"%(goal.command))




    def ack_waiter(self,data):
        self.ack_buffer.append(data)




    def shutdown(self):
        self.comms.shutdown()


        
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

    