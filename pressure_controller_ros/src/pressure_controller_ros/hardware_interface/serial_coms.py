#!/usr/bin/env python

import serial
import numbers
import threading
import rospy
import time



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
class SerialComs:
	def __init__(self, devname,baud):
		self.connected = False
		self.DEBUG = False
		try:
			if self.DEBUG:
				print(devname)
				print(baud)


			self.s = serial.Serial(devname, baud)
			self.connected = True

			

		except serial.SerialException:
			self.connected=False
			print("SERIAL ERROR: Maybe the device is unplugged?")
			pass


	def resume(self):
		if not self.s.isOpen():
			self.s.open()


	def initialize(self):
		self.s.flushInput()  # Flush startup text in serial input


	def flushAll(self):
		self.s.reset_input_buffer()
		self.s.reset_output_buffer()


	def sendCommand(self, command, values):
		"""Send commands via serial
		INPUTS:
			command - a command to use
			args    - arguments for the command

		OUTPUTS:
			out_str - the output string sent
		"""
		if self.DEBUG:
			print("SERIAL_COMS:",command,values)
		command_toSend = command
		if isinstance(values, list) or isinstance(values, tuple):
			if values:
				for val in values:
					command_toSend+= ";%0.5f"%(val)
		elif isinstance(values, numbers.Number):
			command_toSend+=";%0.5f"%(values)
		else:
			raise ValueError('sendCommand expects either a list or a number')

		#self.s.write(command_toSend +'\n')


		#Share the value with the main looping thread
		if not self.reader.new_command.is_set():
			self.reader.command_toSend = command_toSend
			self.reader.new_command.set()



		return command_toSend


	def start_read_thread(self, reading_cb=None, poll_rate=None):
		if not reading_cb and not poll_rate:
			self.reader = SerialReadWriteThreaded(self.s)
		elif reading_cb and not poll_rate:
			self.reader = SerialReadWriteThreaded(self.s, reading_cb=reading_cb)
		elif not reading_cb and poll_rate:
			self.reader = SerialReadWriteThreaded(self.s, poll_rate=poll_rate)
		else:
			self.reader = SerialReadWriteThreaded(self.s, reading_cb=reading_cb, poll_rate=poll_rate)

		self.reader.DEBUG= self.DEBUG


	def shutdown(self):
		print("SERIAL COMS: Shutting Down")
		self.reader.shutdown()









class SerialReadWriteThreaded:
	def __init__(self, serial_in, reading_cb = None, poll_rate = 100000):
		self.s = serial_in
		self.r = rospy.Rate(poll_rate);

		self.command_toSend = None
		self.curr_send_time = rospy.get_rostime().to_nsec()
		self.last_send_time = self.curr_send_time
		self.last_read_time = self.curr_send_time


		if not reading_cb:
			reading_cb = self.do_nothing

		self.start_threaded(reading_cb)


	def do_nothing(self,line):
		print(line)


	def start_threaded(self, reading_cb):
		self.reading_cb = reading_cb
		self.read_now = threading.Event()
		self.read_now.set()
		self.new_command = threading.Event()
		self.new_command.clear()

		reading_thread = threading.Thread(target=self.thread_run)
		reading_thread.start()
		print('Communication thread started')
	

	def thread_read(self):
		print(self.read_now)
		while self.read_now.is_set() and not rospy.is_shutdown():
			while self.s.in_waiting and not rospy.is_shutdown():
				raw_reading = self.readStuff()
				self.reading_cb(raw_reading);
			self.r.sleep()


	def thread_run(self):
		while self.read_now.is_set() and not rospy.is_shutdown():
			if self.new_command.is_set():
				
				if self.DEBUG:
					self.curr_send_time = rospy.get_rostime().to_nsec()
					print("SEND: %0.4f ms"%((self.curr_send_time-self.last_send_time)/1000000.0))
					self.last_send_time = self.curr_send_time
				
				self.s.write(self.command_toSend +'\n')
				self.new_command.clear()

			raw_reading = self.readStuff()
			if raw_reading is not None:
				if self.DEBUG:
					self.curr_read_time = rospy.get_rostime().to_nsec()
					print("READ: %0.4f ms"%((self.curr_read_time-self.last_read_time)/1000000.0))
					self.last_read_time = self.curr_read_time

				self.reading_cb(raw_reading);
			else:
				self.r.sleep()


	def readStuff(self):
		"""read one line from the incomming buffer
		INPUTS:
			n/a
		OUTPUTS:
			out_str - the output string sent
		"""
		if self.s.in_waiting:  # Or: if ser.inWaiting():
			return self.s.readline().strip()
		else:
			return None

	def shutdown(self):
		print("SERIAL READER: Shutting Down")
		self.read_now.clear()
		self.s.close()






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




'''
A more efficient serial readline function from:
https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522
'''
'''
class ReadLine:
	def __init__(self, s):
		self.buf = bytearray()
		self.s = s
	
	def readline(self):
		i = self.buf.find(b"\n")
		if i >= 0:
			r = self.buf[:i+1]
			self.buf = self.buf[i+1:]
			return r
		while True:
			i = max(1, min(2048, self.s.in_waiting))
			data = self.s.read(i)

			#if not data:
			#	raise SerialIssue("There was a serial timeout")
			#	pass

			i = data.find(b"\n")
			if i >= 0:
				r = self.buf + data[:i+1]
				self.buf[0:] = data[i+1:]
				return r
			else:
				self.buf.extend(data)

'''