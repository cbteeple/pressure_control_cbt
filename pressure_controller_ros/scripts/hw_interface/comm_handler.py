#!/usr/bin/env python
import sys
import time

import rospy
import threading
import copy
import numpy as np

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

        self.DEBUG =  rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self.data_channel = rospy.get_param(rospy.get_name()+"/data_channel","pressure_control")
        self.use_separate_topics = rospy.get_param(rospy.get_name()+"/separate_topics" , False)
        self.cmd_sleep_time = rospy.get_param(rospy.get_name()+"/cmd_sleep_time" , 0.0)
        self.read_poll_rate = rospy.get_param(rospy.get_name()+"/read_poll_rate" , 5000)

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
                curr_dev = serial_coms.SerialComs(devname, baud, devnum=idx, debug=self.DEBUG)
            elif vendor_id is not None:
                curr_dev = HID_coms.HIDComs(vendor_id, product_id, serial_number, devnum=idx, debug=self.DEBUG)
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


        # Start some message publishers and subscribers
        self.echo_pub = rospy.Publisher(self.data_channel+'/echo', msg.Echo, queue_size=10)
        self.data_pub = rospy.Publisher(self.data_channel+'/pressure_data', msg.DataIn, queue_size=10)
        #self.data_pub = []
        #for comm in self.comms:
        #    self.data_pub.append()
        
        self.num_channels = [self.comms[idx]['num_channels'] for idx in range(len(self.comms))]
        rospy.set_param(self.data_channel+'/num_channels',self.num_channels)
        self.num_devs=len(self.comms)

        #self.data_in = [msg.DataIn()]*len(self.comms)
        self.data_in = self.initialize_data(self.num_channels)
        
        self.pub_data_separate = []
        self.command_handler = CommandHandler(self.comms)
        for idx,comm in enumerate(self.comms):
            if self.use_separate_topics:
                self.pub_data_separate.append(rospy.Publisher(self.data_channel+'/pressure_data_%d'%(idx), msg.DataInSep, queue_size=10))
            comm['interface'].start_read_thread(poll_rate=self.read_poll_rate, reading_cb=self.process_serial_in)
        
        #self.sub_list=self.start_data_msg_condenser()
            


    def initialize_data(self, num_channels):
        num_devs = len(num_channels)
        num_channels_tot = sum(num_channels)
        data_in = {}
        data_in['new']            = [None]*num_devs
        data_in['time']           = [None]*num_devs
        data_in['setpoints']      = [None]*num_channels_tot
        data_in['measured']       = [None]*num_channels_tot
        data_in['input_pressure'] = [None]*num_devs

        return data_in

    def reset_data(self, data_in):
        for key in  data_in:
            len_curr = len(data_in[key])
            data_in[key] = [None]*len_curr

        return data_in


    def process_serial_in(self, data_in):
        if not data_in:
            return

        if data_in is None:
            return

        devnum = data_in.get('devnum', None)
        line_in = data_in.get('data', None)


        validator=self.command_handler.validators[devnum]
        channel_nums = self.command_handler.num_chans

        if self.DEBUG:
            print("RAW SERIAL IN: %s"%(line_in))


        try:
            message = validator.process_line(line_in)

            if message is None:
                return

            message_data = message[0]
            message_type = message[1]


            if message_type == 'echo':
                echo_in = msg.Echo()
                echo_in.command =  message_data['_command']
                echo_in.args = message_data['_args']
                echo_in.devnum = devnum

                if self.DEBUG:
                    rospy.loginfo(echo_in)

                self.echo_pub.publish(echo_in)

            elif message_type == 'data':
                data_msg = msg.DataInSep(
                        devnum=devnum,
                        time=message_data['time'],
                        setpoints=message_data.get('setpoints',[]),
                        measured=message_data.get('measured',[]),
                        input_pressure=message_data.get('input_pressure',[]),
                        )

                if self.use_separate_topics:

                    data_pub_curr = self.pub_data_separate[devnum]

                    if self.DEBUG:
                        rospy.loginfo('PUBLISHING data from dev #%d in channel "%s"'%(devnum, data_pub_curr.name))
                    data_pub_curr.publish(data_msg)
                    
                    if self.DEBUG:
                        rospy.loginfo('PUBLISHED data from dev #%d in channel "%s"'%(devnum, data_pub_curr.name))

                else:
                    self.combine_data_streams(data_msg)

                return

            else:
                if self.DEBUG:
                    rospy.loginfo("Data message has unknown type")
                return
       

        except rospy.ROSException:
            return



    def combine_data_streams(self, data_in):
        """
        Combine the messages from separate 
        """

        if isinstance(data_in, dict):
            data_use = data_in
            devnum = data_in['devnum']
        
        else:
            data_use = {}
            devnum = data_in.devnum
            data_use['time'] = data_in.time
            data_use['setpoints'] = data_in.setpoints
            data_use['measured'] = data_in.measured
            data_use['input_pressure'] = data_in.input_pressure

        if self.DEBUG:
            rospy.loginfo('GOT DATA from device #%d'%(devnum))

        max_idx = sum(self.num_channels[0:devnum+1])-1
        min_idx = max_idx - self.num_channels[devnum] +1

       
        self.data_in['new'][devnum] = True
        self.data_in['time'][devnum]= int(data_use['time'])

        if len(data_use['setpoints']) >0:
            self.data_in['setpoints'][min_idx:max_idx+1]= list(data_use['setpoints'])

        if len(data_use['measured']) >0:  
            self.data_in['measured'][min_idx:max_idx+1]= list(data_use['measured'])
        
        if len(data_use['input_pressure']) >0:
            self.data_in['input_pressure'][devnum]= list(data_use['input_pressure'])[0]
        
        
        if self.data_in['input_pressure'][devnum] is None:
            self.data_in['input_pressure'][devnum] = 0.0


        # If all controllers are in (and it's the last controller), publish new data
        ready = np.all(self.data_in['new'])
        if ready:
            close = np.allclose(self.data_in['time'],self.data_in['time'][0], rtol=5)
            if close:
                self.data_in['new'] = [None]*self.num_devs

                data_msg_comb = msg.DataIn()
                data_msg_comb.time=np.mean(self.data_in['time']),
                data_msg_comb.setpoints=self.data_in['setpoints']
                data_msg_comb.measured=self.data_in['measured']
                data_msg_comb.input_pressure=self.data_in['input_pressure']


                data_msg_comb.time = data_msg_comb.time[0]


                if self.DEBUG:
                    rospy.loginfo('PUBLISHING combined data in channel "%s"'%(self.data_pub.name))
                self.data_pub.publish(data_msg_comb)

                if self.DEBUG:
                    rospy.loginfo('PUBLISHED combined data in channel "%s"'%(self.data_pub.name))

                #self.data_in = self.reset_data(self.data_in)
        
        return


    def start_data_msg_condenser(self):
        # Subscribe to all the data topics, then combine data into "pressure_control" topic topic
        sub_list = []
        for idx,_ in enumerate(self.comms):
            sub_list.append(rospy.Subscriber(
                self.data_channel+'/pressure_data_%d'%(idx),
                msg.DataInSep,
                self.combine_data_streams,
                ))
        return sub_list


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
            server = command_server.CommandAction(self.data_channel, self.comms, self.command_handler, self.cmd_sleep_time)
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

    
