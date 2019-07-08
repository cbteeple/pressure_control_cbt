#!/usr/bin/env python

import serial
import numbers
import threading
import rospy



class Error(Exception):
   """Base class for other exceptions"""
   pass

class SerialIssue(Error):
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
		try:
			self.s = serial.Serial(devname, baud)
			self.resume()
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
		command_toSend = command
		if isinstance(values, list) or isinstance(values, tuple):
			if values:
				for val in values:
					command_toSend+= ";%0.5f"%(val)
		elif isinstance(values, numbers.Number):
			command_toSend+=";%0.5f"%(values)
		else:
			raise ValueError('sendCommand expects either a list or a number')

		self.s.write(command_toSend +'\n')
		return command_toSend


	def start_read_thread(self, reading_cb=None, poll_rate=None):
		if not reading_cb and not poll_rate:
			self.reader = SerialReadThreaded(self.s)
		elif reading_cb and not poll_rate:
			self.reader = SerialReadThreaded(self.s, reading_cb=reading_cb)
		elif not reading_cb and poll_rate:
			self.reader = SerialReadThreaded(self.s, poll_rate=poll_rate)
		else:
			self.reader = SerialReadThreaded(self.s, reading_cb=reading_cb, poll_rate=poll_rate)


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
		print("SERIAL COMS: Shutting Down")
		self.reader.shutdown()









class SerialReadThreaded:
	def __init__(self, serial_in, reading_cb = None, poll_rate = 100000):
		self.s = serial_in
		self.r = rospy.Rate(poll_rate);


		if not reading_cb:
			reading_cb = self.do_nothing

		self.start_threaded(reading_cb)


	def do_nothing(self,line):
		print(line)


	def start_threaded(self, reading_cb):
		self.reading_cb = reading_cb
		self.read_now = threading.Event()
		self.read_now.set()

		reading_thread = threading.Thread(target=self.thread_reading)
		reading_thread.start()
	

	def thread_reading(self):
		print(self.read_now)
		while self.read_now.is_set() and not rospy.is_shutdown():
			while self.s.in_waiting and not rospy.is_shutdown():
				raw_reading = self.readStuff()
				self.reading_cb(raw_reading);
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