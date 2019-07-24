#!/usr/bin/env python

import rospy
import actionlib


#Import the specific messages that we created.
import pressure_controller_ros.msg as msg





class CommandAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.CommandFeedback()
    _result = msg.CommandResult()

    def __init__(self, name, comms_obj):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._action_name = name
        self.ack_buffer = []

        self.comms=comms_obj

        # Start some message publishers and subscribers
        rospy.Subscriber('pressure_control/echo', msg.Echo, self.ack_waiter)


        # Start an actionlib server
        self._as = actionlib.SimpleActionServer('pressure_control', msg.CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


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
                self.comms.flushAll()

            self._feedback.success = True
            self._as.publish_feedback(self._feedback)


        else:           
            # start executing the action
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
            else:

                self._feedback.out_string = self.comms.sendCommand(goal.command, goal.args)
                self._feedback.sent = True

                if goal.wait_for_ack:
                    #Spin and wait for an acknowledgment from the controller
                    ack_curr = ""
                    cmd = str(goal.command).lower()
                    while ack_curr != cmd and not rospy.is_shutdown():
                        if len(self.ack_buffer)>0:
                            ack_curr = self.ack_buffer.pop().command
                        r.sleep()

                    
                    self.ack_buffer = []
                    
                self._feedback.success = True
    
        
        
          
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

    