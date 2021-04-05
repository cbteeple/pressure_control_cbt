#!/usr/bin/env python

import serial
import numbers
import threading
import rospy
import time
import hid



class Error(Exception):
   """Base class for other exceptions"""
   pass

class Issue(Error):
   """Raised when there's an issue"""
   pass





"""Start serial communication
INPUTS:
	devname - the short name of the device you want to use
	baud    - baud rate

OUTPUTS:
	s - the serial object created
"""	
class HIDComs:
	def __init__(self, vendor_id, product_id, serial_number=None, devnum=0):
		self.connected = False
		try:
			self.h = self.get_device(vendor_id, product_id, serial_number)
			self.devnum = devnum
			
			# enable non-blocking mode
			self.h.set_nonblocking(1)

			self.DEBUG = False


			
			self.connected = True
		except:
			self.connected=False
			print("HID ERROR: Maybe the device is unplugged?")
			pass


	def get_device(self, vendor_id, product_id, serial_number=None):
		h = hid.device()
		if serial_number is None:
			# If no serial number is geven, open the first device with the product and vendor info
			h.open(vendor_id, product_id) # TREZOR VendorID/ProductID
		else:
			# If a serial number is geven, find that device
			serial_number = str(serial_number)
			path = None
			for device_dict in hid.enumerate():
				vid = device_dict.get('vendor_id', None)
				pid = device_dict.get('product_id', None)
				snum = device_dict.get('serial_number', None)

				if (vid==vendor_id) & (pid==product_id) & (snum==serial_number):
					path = device_dict.get('path', None)
					break
			
			h.open_path(path) # Open HID device by path
		return h

	def initialize(self):
		pass  # Remove this function eventually


	def flushAll(self):
		pass  # Remove this function eventually
		#self.h.reset_input_buffer()
		#self.h.reset_output_buffer()


	def sendCommand(self, command, values):
		"""Send commands via serial
		INPUTS:
			command - a command to use
			args    - arguments for the command

		OUTPUTS:
			out_str - the output string sent
		"""
		command_toSend = command
		if isinstance(values, list) or isinstance(values, tuple):
			if values:
				for val in values:
					command_toSend+= ";%0.2f"%(val)
		elif isinstance(values, numbers.Number):
			command_toSend+=";%0.2f"%(values)
		else:
			raise ValueError('sendCommand expects either a list or a number')


		#Share the value with the main looping thread
		if not self.reader.new_command.is_set():
			self.reader.command_toSend = command_toSend
			self.reader.new_command.set()

		return command_toSend



	def start_read_thread(self, reading_cb=None, poll_rate=None):
		if not reading_cb and not poll_rate:
			self.reader = HIDReadWriteThreaded(self.h)
		elif reading_cb and not poll_rate:
			self.reader = HIDReadWriteThreaded(self.h, reading_cb=reading_cb)
		elif not reading_cb and poll_rate:
			self.reader = HIDReadWriteThreaded(self.h, poll_rate=poll_rate)
		else:
			self.reader = HIDReadWriteThreaded(self.h, reading_cb=reading_cb, poll_rate=poll_rate)

		self.reader.DEBUG= self.DEBUG
		self.reader.start_threaded()



	def shutdown(self):
		print("HID COMS: Shutting Down")
		self.reader.shutdown()











class HIDReadWriteThreaded:
	def __init__(self, hid_in, reading_cb = None, poll_rate = 2000, devnum=0):
		self.h = hid_in
		self.r = rospy.Rate(poll_rate)
		self.reading_cb = reading_cb
		self.devnum = devnum

		self.command_toSend = None
		self.curr_send_time = rospy.get_rostime().to_nsec()
		self.last_send_time = self.curr_send_time
		self.last_read_time = self.curr_send_time
		
		if not self.reading_cb:
			self.reading_cb = self.do_nothing


	def do_nothing(self,line):
		print(line)


	def start_threaded(self):
		self.read_now = threading.Event()
		self.read_now.set()
		self.new_command = threading.Event()
		self.new_command.clear()

		reading_thread = threading.Thread(target=self.thread_run)
		reading_thread.start()
		print('Communication thread started')

	

	def thread_run(self):
		while self.read_now.is_set() and not rospy.is_shutdown():
			if self.new_command.is_set():
				
				if self.DEBUG:
					self.curr_send_time = rospy.get_rostime().to_nsec()
					print("SEND: %0.4f ms"%((self.curr_send_time-self.last_send_time)/1000000.0))
					self.last_send_time = self.curr_send_time
				
				self.h.write( [ ord(char) for char in list(self.command_toSend)] + [0] * (64-len(self.command_toSend)))
				self.new_command.clear()

			raw_reading = self.readStuff()
			if raw_reading is not None:
				if self.DEBUG:
					self.curr_read_time = rospy.get_rostime().to_nsec()
					print("READ: %0.4f ms"%((self.curr_read_time-self.last_read_time)/1000000.0))
					self.last_read_time = self.curr_read_time

				sendout = {'dev_num':self.devnum, 'data':raw_reading}
				self.reading_cb(sendout);
			else:
				self.r.sleep()


	def readStuff(self):
		"""read one line from the incomming buffer
		INPUTS:
			n/a
		OUTPUTS:
			out_str - the output string sent
		"""
		d = self.h.read(64)
		if d:
			in_str = ""
			for char_int in d:
				if char_int !=0:
					in_str+=chr(char_int)
			return in_str
		else:
			return None

	def shutdown(self):
		print("HID READER: Shutting Down")
		self.read_now.clear()
		self.h.close()






class TrajThread:
	def __init__(self, comm_obj, reading_cb = None, traj_rate = 2000):
		self.comm_obj = comm_obj
		self.r = rospy.Rate(traj_rate)
		self.command_toSend = None
		

		if not reading_cb:
			reading_cb = self.do_nothing

		self.start_thread(reading_cb)


	def do_nothing(self,line):
		print(line)


	def start_thread(self, reading_cb):
		self.reading_cb = reading_cb
		self.running_now = threading.Event()
		self.running_now.set()
		self.new_command = self.comm_obj.new_command

		traj_thread = threading.Thread(target=self.thread_run)
		traj_thread.start()
		print('Communication thread started')

	

	def thread_run(self):
		while self.running_now.is_set() and not rospy.is_shutdown():


			# Interpolate to get the next setpoint:


			# Actually send the setpoint if there's nothing stacked up:
			if not self.reader.new_command.is_set():
				self.reader.command_toSend = command_toSend
				self.new_command.set()			
			self.r.sleep()


	def shutdown(self):
		print("HID READER: Shutting Down")
		self.running_now.clear()