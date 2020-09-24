#!/usr/bin/env python
import sys

import rospy
import threading

#Import the specific messages that we created in our tutorials folder.
import pressure_controller_ros.msg as msg

#Import custom serial coms, threading, etc
import pressure_controller_ros.hardware_interface.serial_coms as serial_coms
import pressure_controller_ros.hardware_interface.HID_coms as HID_coms

import command_server
import traj_server



class CommHandler(object):
    def __init__(self, name):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)

        self._action_name = name
        self.ack_buffer = []
        self.data_in = None

        # Begin communication with the pressure controller
        devname = rospy.get_param(rospy.get_name()+'/devname',None)
        baud = rospy.get_param(rospy.get_name()+'/baudrate',None)

        vendor_id = rospy.get_param(rospy.get_name()+'/vendor_id',None)
        product_id = rospy.get_param(rospy.get_name()+'/product_id',None)

        if devname is not None:
            self.comms = serial_coms.SerialComs(devname, baud)
        elif vendor_id is not None:
            self.comms = HID_coms.HIDComs(vendor_id, product_id)
        else:
            print("NO COMUNICATION INTERFACES PRESENT")

        if not self.comms.connected:
            raise Exception("Comms were not connected")


        # Start some message publishers and subscribers
        self.data_pub = rospy.Publisher('pressure_control/pressure_data', msg.DataIn, queue_size=10)
        self.echo_pub = rospy.Publisher('pressure_control/echo', msg.Echo, queue_size=10)


        self.comms.start_read_thread(poll_rate=5000, reading_cb=self.process_serial_in)



    def process_serial_in(self, line_in):
        if not line_in:
            return

        try:
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
                #All other incomming lines are tab-separated data, where the 
                line_split = line_in.split('\t')

                data_type  = int(line_split[1])

                if data_type == 0: # Handle incomming setpoint data
                    # Here marks a new data point. Send the previous one.
                    if self.data_in is not None:
                        if self.DEBUG:
                            rospy.loginfo(self.data_in)

                        self.data_pub.publish(self.data_in)

                    # Now begin the next one
                    self.data_in = msg.DataIn();
                    self.data_in.time = long(line_split[0])
                    self.data_in.setpoints = [float(i) for i in line_split[2:]]

                elif data_type == 1: # Handle incomming measured data
                    if self.data_in is None:
                        return

                    if self.data_in.time == long(line_split[0]):
                        self.data_in.measured  = [float(i) for i in line_split[2:]]

                    else:
                        if self.DEBUG:
                            print("COMM_HANDLER: Measured data message not recieved")

                elif data_type == 2: # Handle incomming master pressure data
                    if self.data_in is None:
                        return

                    if self.data_in.time == long(line_split[0]):
                        self.data_in.input_pressure  = [float(i) for i in line_split[2:]]


        except rospy.ROSException:
            return



    '''
    Function to start the cmd server thread
    '''

    def start_cmd_server_thread(self):
        thread = threading.Thread(target=self.cmd_server_thread)
        thread.start()

    '''
    The actual function that gets run in the command server thread
    '''
    def cmd_server_thread(self):
        try:
            server = command_server.CommandAction('command_server', self.comms)
            print("COMMAND SERVER: Ready!")
            rospy.spin()

        except KeyboardInterrupt:
            print("COMMAND SERVER: Shutting Down")
            server.shutdown()

        except rospy.ROSInterruptException:
            print("COMMAND SERVER: Shutting Down")
            server.shutdown()



    '''
    Function to start the trajectory server thread
    '''

    def start_traj_server_thread(self):
        self.traj_server_rate = rospy.get_param(rospy.get_name()+'/traj_server_rate',500)
        thread = threading.Thread(target=self.traj_server_thread)
        thread.start()

    '''
    The actual function that gets run in the trajectory server thread
    '''
    def traj_server_thread(self):
        try:
            server = traj_server.TrajAction('command_server', self.comms, self.traj_server_rate)
            print("TRAJECTORY SERVER: Ready!")
            rospy.spin()

        except KeyboardInterrupt:
            print("TRAJECTORY SERVER: Shutting Down")
            server.shutdown()

        except rospy.ROSInterruptException:
            print("TRAJECTORY SERVER: Shutting Down")
            server.shutdown()







    def shutdown(self):
        self.comms.shutdown()


        
if __name__ == '__main__':
    try:
        rospy.init_node('pressure_control', disable_signals=True)
        print("HW INTERFACE: Node Initiatilized (%s)"%(rospy.get_name()))
        server = CommHandler(rospy.get_name())
        server.start_cmd_server_thread()
        server.start_traj_server_thread()
        print("HW INTERFACE: Ready!")
        rospy.spin()

    except KeyboardInterrupt:
        print("HW INTERFACE: Shutting Down")
        server.shutdown()

    except rospy.ROSInterruptException:
        print("HW INTERFACE: Shutting Down")
        server.shutdown()

    
