#!/usr/bin/python3.4

import ev3dev.ev3 as ev3
import signal
import time
import threading
from enum import Enum

PRINT = True

LOG_FILE = "log.txt"
MAP_FILE = "map2017"
STARTING_ORIENTATION = 's'

LIMIT_BLACK = 40



mode = "NORMAL" #"NORMAL" #"FAST"
if mode == "SLOW":
	BASE_SPEED = 60
	BASE_SPEED_P = 70
	CORRECTION_SPEED_OUTSIDE=BASE_SPEED
	K=0.5
	CORRECTION_SPEED_INSIDE=K*CORRECTION_SPEED_OUTSIDE
	CORRECTION_SPEED_OUTSIDE_P = BASE_SPEED_P
	CORRECTION_SPEED_INSIDE_P = K* CORRECTION_SPEED_OUTSIDE_P
	TURN_SPEED_90=60
	TURN_SPEED_P = 60
	TURN_SPEED_180=50
else:
	if mode == "NORMAL":
		K=0.6
		BASE_SPEED = 70
		BASE_SPEED_P = 90
		CORRECTION_SPEED_OUTSIDE_P = BASE_SPEED_P
		CORRECTION_SPEED_OUTSIDE=BASE_SPEED
		CORRECTION_SPEED_INSIDE_P = K*CORRECTION_SPEED_OUTSIDE_P
		CORRECTION_SPEED_INSIDE=K*CORRECTION_SPEED_OUTSIDE
		TURN_SPEED_90=60
		TURN_SPEED_P = 90
		TURN_SPEED_180=50

def is_black(value):
	return value<LIMIT_BLACK

def is_light_black(value):
	return value<500

#{{{ Logger class
class Logger:
	def __init__(self):
		self.file = open(LOG_FILE, "at")
		self.log("New iteration")

	def __del__(self):
		self.file.close()

	def log(self, text, level=0):
		print(text)
		self.file.write(text+"\n")
LOGGER = Logger()
# }}}
# {{{ Robot class

class Robot:
	def __init__(self, map):
		self.leftSensor = ev3.ColorSensor('in1')
		self.rightSensor = ev3.ColorSensor('in2')
		self.armSensor = ev3.LightSensor('in3')
		self.leftMotor = ev3.LargeMotor('outA')
		self.rightMotor = ev3.LargeMotor('outB')
		self.orientation = STARTING_ORIENTATION

		assert self.leftMotor.connected, "Left Motor not connected"
		assert self.rightMotor.connected, "Right Motor not connected"
		assert self.leftSensor.connected, "Left Sensor not connected"
		assert self.rightSensor.connected, "Right Sensor not connected"
		assert self.armSensor.connected, "Arm Sensor not connected"

		signal.signal(signal.SIGINT, self.exit_gracefully)
		signal.signal(signal.SIGTERM, self.exit_gracefully)

		self.leftMotor.reset()
		self.rightMotor.reset()
		self.running = True
		self.leftMotor.run_direct()
		self.rightMotor.run_direct()


	def shutdown(self):
		self.rightMotor.duty_cycle_sp = 0
		self.leftMotor.duty_cycle_sp = 0
		self.running = False

	def exit_gracefully(self, signum, frame):
		self.rightMotor.duty_cycle_sp = 0
		self.leftMotor.duty_cycle_sp = 0
		self.running = False


	def run_forward(self, number=0):

		if number > 0 :
			self.rightMotor.duty_cycle_sp = BASE_SPEED_P
			self.leftMotor.duty_cycle_sp = BASE_SPEED_P
		else:
			self.rightMotor.duty_cycle_sp = BASE_SPEED
			self.leftMotor.duty_cycle_sp = BASE_SPEED

		while self.running:
			left = self.leftSensor.value()
			right = self.rightSensor.value()
			if PRINT : print("{} {}".format(left, right))
			if is_black(left) and is_black(right):
				if number==1:
					self.rightMotor.duty_cycle_sp = BASE_SPEED
					self.leftMotor.duty_cycle_sp = BASE_SPEED
				else:
					if number==0:
						self.stop()
						self.escape_from_black()
						return
				number -= 1
				while self.running and is_black(left) and is_black(right):
					if PRINT: print("On black")
					self.rightMotor.duty_cycle_sp = BASE_SPEED_P
					self.rightMotor.duty_cycle_sp = BASE_SPEED_P
					left = self.leftSensor.value()
					right = self.rightSensor.value()

			if is_black(left):
				if number == 0:
					self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE
					self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE
				else:
					self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE_P
					self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE_P
				if PRINT : print("Correction")
			else:
				if is_black(right):
					if number == 0:
						self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE
						self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE
					else:
						self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE_P
						self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE_P
					if PRINT : print("Correctoin")
				else:
					if number == 0:
						self.leftMotor.duty_cycle_sp = BASE_SPEED
						self.rightMotor.duty_cycle_sp = BASE_SPEED
					else:
						self.leftMotor.duty_cycle_sp = BASE_SPEED_P
						self.rightMotor.duty_cycle_sp = BASE_SPEED_P

	def run_backward(self):
		self.rightMotor.duty_cycle_sp = BASE_SPEED
		self.leftMotor.duty_cycle_sp = BASE_SPEED
		self.rightMotor.polarity = "inversed"
		self.leftMotor.polarity = "inversed"


		while self.running:
			left = self.leftSensor.value()
			right = self.rightSensor.value()
			if PRINT: print("{} {}".format(left, right))
			if is_black(left) and is_black(right):

				while self.running:
					left = self.leftSensor.value()
					right = self.rightSensor.value()
					if not is_black(left) or not is_black(right):
						self.stop()
						self.rightMotor.polarity = "normal"
						self.leftMotor.polarity = "normal"
						self.run_forward()
						return
			#if is_black(left):
			#	if PRINT: print("Correction 1")
			#	self.rightMotor.polarity = "normal"
			#	self.leftMotor.polarity = "normal"
			#	self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE
			#	self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE
			#else:
			#	if is_black(right):
			#		if PRINT: print("Correction 2")
			#		self.rightMotor.polarity = "normal"
			#		self.leftMotor.polarity = "normal"
			#		self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE
			#		self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE
			#	else:
			self.rightMotor.polarity = "inversed"
			self.leftMotor.polarity = "inversed"
			self.leftMotor.duty_cycle_sp = BASE_SPEED
			self.rightMotor.duty_cycle_sp = BASE_SPEED

	def run_forward_with_can(self):
		self.rightMotor.duty_cycle_sp = BASE_SPEED
		self.leftMotor.duty_cycle_sp = BASE_SPEED
		last_color = self.armSensor.value()
		while self.running:
			new_color = self.armSensor.value()
			left = self.leftSensor.value()
			right = self.rightSensor.value()
			if PRINT: print("{} {} {}".format(left,right,new_color))
			if is_light_black(new_color): #and not is_light_black(new_color):
				self.stop()
				return
			last_color = new_color
			if is_black(left):
				if PRINT: print("Correction")
				self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE
				self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE
			else:
				if is_black(right):
					if PRINT: print("Correction")
					self.leftMotor.duty_cycle_sp = CORRECTION_SPEED_OUTSIDE
					self.rightMotor.duty_cycle_sp = CORRECTION_SPEED_INSIDE
				else:
					self.leftMotor.duty_cycle_sp = BASE_SPEED-10
					self.rightMotor.duty_cycle_sp = BASE_SPEED-10
	def escape_from_black(self):
		self.leftMotor.duty_cycle_sp = TURN_SPEED_90
		self.rightMotor.duty_cycle_sp = TURN_SPEED_90
		#while self.running and (not is_black(right) or not is_black(left):
			#righ = self.rightSensor.value()
			#left = self.ri
		while self.running:
			right = self.rightSensor.value()
			left = self.leftSensor.value()
			if not is_black(right) or not is_black(left):
				self.stop()
				return

	def turn_right(self):
		last_color = self.rightSensor.value()
		self.leftMotor.duty_cycle_sp = TURN_SPEED_P
		self.rightMotor.duty_cycle_sp = 0
		#self.leftMotor.run_to_rel_pos(speed_sp=900, position_sp=self.rightMotor.count_per_rot)
		#self.leftMotor.run_direct()
		while self.running:
			new_color = self.rightSensor.value()
			if not is_black(new_color):
				break
		last_color = new_color
		
		while self.running:
			new_color = self.rightSensor.value()
			if is_black(new_color):
			#	self.leftMotor.duty_cycle_sp = TURN_SPEED_90
			#if is_black(last_color) and not is_black(new_color):
				self.stop()
				self.run_forward()
				return
			last_color = new_color

	def turn_left(self):
		last_color = self.leftSensor.value()
		self.leftMotor.duty_cycle_sp = 0
		self.rightMotor.duty_cycle_sp = TURN_SPEED_P
		while self.running:
			new_color = self.leftSensor.value()
			if not is_black(new_color):
				break
		last_color = new_color
		while self.running:
			new_color = self.leftSensor.value()
			if is_black(new_color):
			#	self.rightMotor.duty_cycle_sp = TURN_SPEED_90
			#if is_black(last_color) and not is_black(new_color):
				self.stop()
				self.run_forward()
				return
			last_color = new_color

	def turn_right_right(self, is_between_cross = False):
		self.rightMotor.polarity = "inversed"
		self.rightMotor.duty_cycle_sp = TURN_SPEED_180
		self.leftMotor.duty_cycle_sp = TURN_SPEED_180
		last_color = self.rightSensor.value()
		count = 0

		while self.running:
			new_color = self.rightSensor.value()
			if not is_black(new_color) and is_black(last_color):
				count = count + 1
				if is_between_cross:
					self.rightMotor.polarity = "normal"
					self.stop()
					self.run_forward()
					return
				else:
					if count == 2:
						self.rightMotor.polarity = "normal"
						self.stop()
						self.run_forward()
						return

			last_color = new_color

	def turn_left_left(self, is_between_cross = False):
		self.leftMotor.polarity = "inversed"
		self.rightMotor.duty_cycle_sp = TURN_SPEED_180
		self.leftMotor.duty_cycle_sp = TURN_SPEED_180
		last_color = self.leftSensor.value()
		count = 0
		while self.running:
			new_color = self.leftSensor.value()
			if not is_black(new_color):
				last_color = new_color
				break
		while self.running:
			new_color = self.leftSensor.value()
			if not is_black(new_color) and is_black(last_color):
				count = count + 1
				if is_between_cross:
					self.leftMotor.polarity = "normal"
					self.stop()
					self.run_forward()
					return
				else:
					if count == 2:
						self.leftMotor.polarity = "normal"
						self.stop()
						self.run_forward()
						return

			last_color = new_color

	def stop(self):
		if PRINT: print("STOP ----------------------------")
		self.rightMotor.duty_cycle_sp = 0
		self.leftMotor.duty_cycle_sp = 0

	def execution(self, instruction):
		dicte = {'f':self.run_forward,
			'r':self.turn_right,
			'l':self.turn_left,
			'b':self.run_backward,
			'c':self.run_forward_with_can,
			'a':self.turn_left_left,}
		while self.running and len(instruction) != 0:
			if PRINT: print("Nouvelle instruction : escape")
			self.escape_from_black()
			i = instruction.pop(0)
			j = 0
			while i=='f' and instruction[0]=='f':
				j = j+1
				i = instruction.pop(0)				
			if PRINT: print("Nouvelle instruction : {}".format(i))
			if i=='f':
				dicte[i](number=j)
			else :
				dicte[i]()

	def test_sensor(self):
		while self.running:
			if PRINT: print("Left {}, Right {}, Arm {}".format(self.leftSensor.value(), self.rightSensor.value(), self.armSensor.value()))
# }}}
# {{{ Instruction Converter
class InstructionConverter:
	def is_upper(i):
		if i == 'N'or i == 'S' or i == 'W' or i == 'E':
			return True
		else:
			return False

	def what_next(self, instruction):
		new = InstructionConverter.switch_direction(instruction)
		if InstructionConverter.is_upper(instruction) and not InstructionConverter.is_upper(self.previous):
			b = 'c'
		else:
			if InstructionConverter.is_upper(instruction) and InstructionConverter.is_upper(self.previous):
				return 'fc'
			if InstructionConverter.is_upper(instruction):
				self.orientation = new
				return 'c'
			else:
				b = ''
		if not InstructionConverter.is_upper(instruction) and InstructionConverter.is_upper(self.previous):
			a = 'b'
		else:
			a = ''
		if self.orientation == new:
			x= a+'f'+b
		if (self.orientation + 2) % 4 == new:
			x= a+'a'+b
		if (self.orientation + 1) % 4 == new:
			x= a+'r'+b
		if (self.orientation - 1) % 4 == new:
			x= a+'l'+b
		self.orientation = new
		return x

	def switch_direction(direction):
		if direction == 'n' or direction == 'N':
			return 0
		if direction == 'e' or direction == 'E':
			return 1
		if direction == 's' or direction == 'S':
			return 2
		if direction == 'w' or direction == 'W':
			return 3
		LOGGER.log("Error switch_direction")
		raise Exception()

	def translate(self,instruction):
		self.has_can = False
		self.previous = None
		self.orientation = InstructionConverter.switch_direction(STARTING_ORIENTATION)
		result = ''
		for i in instruction:
			v= self.what_next(i)
			result = result + v
			self.previous = i
		return list(result)

	def convert_letter(i):
		dict = {'u':'n', 'U':'N', 'd':'s', 'D':'S', 'l':'w', 'L':'W', 'r':'e', 'R':'E'}
		res = ''
		for char in i:
			res = res + dict[char]
		print(res)
		return res

# }}}
if __name__ == "__main__":
	robot = Robot(0)
	print("START")
	LOGGER.log("Test log 1")
	LOGGER.log("Test log 2")
	#robot.test_sensor()
	#instruction = ['f', 'r',  'c', 'b', 'a']
	#robot.execution(instruction)
	instruction = "UddlluuRRddlUrrruuuulldDDuuurrddddlLuuuuruulllddRRddddrruuuLUUUruLLLulDrrrddddrdddlluuUruuullddRdrUUUruLulDrddddrddLdlUUUruuuulldddRdrUUUddlluuurRurDlddddlddddlluRdrUUUUruuuulldddRdrUUUUddddldddlluRdrUUU"

	instruction = "eNsswwnnEEsW"
	instruction = "d"
	instruction = "dlllluuuuRRdrUUUruLLLulDrrrdddlllddrUluRRdrUUUruLdlUruLLrrddddlllddddrrrruLdllUUUluRRdrUUUluRurDlddddlldddrruLdlUUUluRRdrUUUluurrdLulD"
	instruction = InstructionConverter.convert_letter(instruction)
	instruction = list(instruction)
	converter = InstructionConverter()
	x=converter.translate(instruction)
	#x = ['r']
	LOGGER.log(str(x))
	robot.execution(x)
	#robot.test_sensor()
	print("STOP")
	robot.stop()
