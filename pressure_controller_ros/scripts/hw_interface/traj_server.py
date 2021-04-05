#!/usr/bin/env python

import rospy
import actionlib
from scipy.interpolate import interp1d


#Import the specific messages that we created in our tutorials folder.
import pressure_controller_ros.msg as msg




class TrajAction(object):
    # create messages that are used to publish feedback/result
    _feedback = msg.PressureTrajectoryFeedback()
    _result = msg.PressureTrajectoryResult()

    def __init__(self, name, comm_obj, command_handler, controller_rate=500):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._action_name = name
        self.controller_rate=controller_rate
        self.comms = comm_obj
        self.command_handler = command_handler

        # Start an actionlib server
        self._as = actionlib.SimpleActionServer('/'+self._action_name+'/pressure_trajectory', msg.PressureTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
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

        if len(commands) != len(self.comms):
            raise ValueError("COMM HANDLER: length of command list does not equal the number of devices")

        cmd_str = ""
        for idx, command in enumerate(commands):
            if command is not None:
                cmd_str+= self.comms[idx]['interface'].sendCommand(command['command'],command['values'], self.comms[idx]['cmd_format'])
        
        return cmd_str


    def execute_cb(self, goal):
        traj= goal.trajectory.points

        # Put all of the setpoints and durations into numpy arrays for later use
        self.traj_points=[]
        self.traj_times=[]
        for point in traj:
            self.traj_points.append(point.pressures)
            self.traj_times.append(point.time_from_start.to_sec())

        print(self.traj_points)
        print(self.traj_times)

        traj_interp = interp1d(self.traj_times,self.traj_points, axis=0)

        

        # helper variables
        r = rospy.Rate(self.controller_rate)
        success = False
        
        # Initiatilize the feedback
        self._feedback.current_time = 0
        self._feedback.success = True

        # Send the pressure trajectory in real time
        start_time=rospy.Time.now()
        curr_time = rospy.Duration(0.0)

        idx=0

        while curr_time.to_sec() < self.traj_times[-1] and not rospy.is_shutdown():
            # start executing the action
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self._feedback.success = False
                break
            else:
                # Interpolate point to send from the trajectory given
                curr_time = rospy.Time.now()-start_time

                if curr_time.to_sec() >= self.traj_times[-1] : 
                    break
                
                new_pressures = traj_interp(curr_time.to_sec()).tolist()

                # Send pressure setpoint to the controller
                self.send_command("set", [1/self.controller_rate] + new_pressures)


                # Update the server
                self._feedback.current_time  = curr_time
                self._as.publish_feedback(self._feedback)

                r.sleep()
                idx += 1

        self.send_command("set", [1/self.controller_rate] + list(self.traj_points[-1]))
        



        if self._feedback.success:
            self._result.success = self._feedback.success
            #rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            #rospy.loginfo("End: %s"%(goal.command))




    def shutdown(self):
        self.comms.shutdown()


        
if __name__ == '__main__':
    try:
        rospy.init_node('pressure_control', disable_signals=True)
        print("TRAJECTORY SERVER: Node Initiatilized (%s)"%(rospy.get_name()))
        server = TrajAction(rospy.get_name())
        print("TRAJECTORY SERVER: Ready!")
        rospy.spin()

    except KeyboardInterrupt:
        print("TRAJECTORY SERVER: Shutting Down")
        server.shutdown()

    except rospy.ROSInterruptException:
        print("TRAJECTORY SERVER: Shutting Down")
        server.shutdown()

    