#!/usr/bin/env python

import serial
import numbers



class Error(Exception):
   """Base class for other exceptions"""
   pass

class SerialIssue(Error):
   """Raised when the input value is too small"""
   pass




def resume(devname,baud):
	"""Start serial communication
	INPUTS:
		devname - the short name of the device you want to use
		baud    - baud rate

	OUTPUTS:
		s - the serial object created
	"""	
	s = serial.Serial(devname, baud)
	if not s.isOpen():
		s.open()
		
	return s

def initialize(s):
	s.flushInput()  # Flush startup text in serial input


def flushAll(s):
	s.reset_input_buffer()
	s.reset_output_buffer()



def sendCommand(s, command, values):
	"""Send commands via serial
	INPUTS:
		s       - an open serial object
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

	s.write(command_toSend +'\n')
	return command_toSend





def readStuff(s):
	"""read one line from the incomming buffer
	INPUTS:
		s       - an open serial object
	OUTPUTS:
		out_str - the output string sent
	"""
	if s.in_waiting:  # Or: if ser.inWaiting():
		return s.readline().strip()
	else:
		return None

