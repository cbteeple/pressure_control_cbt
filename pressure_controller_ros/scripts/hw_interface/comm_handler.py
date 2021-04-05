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

import pressure_control_interface
import pressure_control_interface.utils
from pressure_control_interface.utils.comm_handler import CommandHandler


class CommHandler(object):
    def __init__(self, name):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self.data_channel = rospy.get_param(rospy.get_name()+"/data_channel","pressure_control")

        self._action_name = name
        self.ack_buffer = []

        # Begin communication with the pressure controller
        devices = rospy.get_param(rospy.get_name()+'/devices',None)

        self.comms = []
        for idx, device in enumerate(devices):
            # Try to get serial device information
            devname = device.get('devname', None)
            baud    = device.get('baudrate', None)

            # Try to get hid device information
            vendor_id     = device.get('vendor_id',None)
            product_id    = device.get('product_id',None)
            serial_number = device.get('serial_number',None)

            # Get other information about the controller
            num_channels = device.get('num_channels',None)
            cmd_spec = device.get('cmd_spec',None)
            cmd_format = device.get('cmd_format',None)

            if devname is not None:
                curr_dev = serial_coms.SerialComs(devname, baud, devnum=idx)
                curr_dev.DEBUG = self.DEBUG
            elif vendor_id is not None:
                curr_dev = HID_coms.HIDComs(vendor_id, product_id, serial_number, devnum=idx)
                curr_dev.DEBUG = self.DEBUG
            else:
                curr_dev = None
            
            if curr_dev is not None:
                dev_dict = {
                    'interface':curr_dev,
                    'num_channels':num_channels,
                    'cmd_spec':cmd_spec,
                    'cmd_format': cmd_format,
                    }

                self.comms.append(dev_dict)

                if not curr_dev.connected:
                    raise Exception("Comms were not connected for device #%d in the list"%(idx))
            else:
                raise Exception("Comms were not connected for device #%d in the list"%(idx))  

        self.data_in = [msg.DataIn()]*len(self.comms)

        # Start some message publishers and subscribers
        self.data_pub = rospy.Publisher(self.data_channel+'/pressure_data', msg.DataIn, queue_size=10)
        self.echo_pub = rospy.Publisher(self.data_channel+'/echo', msg.Echo, queue_size=10)

        self.command_handler = CommandHandler(self.comms)
        for comm in self.comms:
            comm['interface'].start_read_thread(poll_rate=5000, reading_cb=self.process_serial_in)




    def process_serial_in(self, data_in):
        if not data_in:
            return

        if data_in is None:
            return

        devnum = data_in.get('devnum', None)
        line_in = data_in.get('data', None)

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
                echo_in.devnum = devnum

                if self.DEBUG:
                    rospy.loginfo(echo_in)

                self.echo_pub.publish(echo_in)

            else:
                #All other incomming lines are tab-separated data, where the 
                line_split = line_in.split('\t')

                data_type  = int(line_split[1])

                if data_type == 0: # Handle incomming setpoint data
                    # Here marks a new data point. Send the previous one.
                    if self.data_in[devnum] is not None:
                        if self.DEBUG:
                            rospy.loginfo(self.data_in[devnum])

                        self.data_pub.publish(self.data_in[devnum])

                    # Now begin the next one
                    self.data_in[devnum] = msg.DataIn();
                    self.data_in[devnum].time = long(line_split[0])
                    self.data_in[devnum].setpoints = [float(i) for i in line_split[2:]]

                elif data_type == 1: # Handle incomming measured data
                    if self.data_in[devnum] is None:
                        return

                    if self.data_in[devnum].time == long(line_split[0]):
                        self.data_in[devnum].measured  = [float(i) for i in line_split[2:]]

                    else:
                        if self.DEBUG:
                            print("COMM_HANDLER: Measured data message not recieved")

                elif data_type == 2: # Handle incomming master pressure data
                    if self.data_in[devnum] is None:
                        return

                    if self.data_in[devnum].time == long(line_split[0]):
                        self.data_in[devnum].input_pressure  = [float(i) for i in line_split[2:]]


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
            server = command_server.CommandAction(self.data_channel, self.comms, self.command_handler)
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
            server = traj_server.TrajAction(self.data_channel, self.comms, self.command_handler, self.traj_server_rate)
            print("TRAJECTORY SERVER: Ready!")
            rospy.spin()

        except KeyboardInterrupt:
            print("TRAJECTORY SERVER: Shutting Down")
            server.shutdown()

        except rospy.ROSInterruptException:
            print("TRAJECTORY SERVER: Shutting Down")
            server.shutdown()







    def shutdown(self):
        for comm in self.comms:
            comm['interface'].shutdown()


        
if __name__ == '__main__':
    try:
        rospy.init_node('pressure_control', disable_signals=True)
        name = rospy.get_param(rospy.get_name()+"/data_channel","pressure_control")
        use_traj_server = rospy.get_param(rospy.get_name()+"/use_traj_server",True)
        
        print("HW INTERFACE: Node Initiatilized (%s)"%(rospy.get_name()+'/'+name))
        server = CommHandler(rospy.get_name())
        server.start_cmd_server_thread()
        if use_traj_server:
            print("HW INTERFACE: Starting traj server...")
            server.start_traj_server_thread()
        else:
            print("HW INTERFACE: Not using traj server.")

        print("HW INTERFACE: Ready!")
        rospy.spin()

    except KeyboardInterrupt:
        print("HW INTERFACE: Shutting Down")
        server.shutdown()

    except rospy.ROSInterruptException:
        print("HW INTERFACE: Shutting Down")
        server.shutdown()

    
