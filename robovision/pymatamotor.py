import numpy
import serial
from threading import Thread
from time import sleep, time

from PyMata.pymata import PyMata
from math import cos, sin, radians, atan2, pi


class Motor(Thread):
	# Motor pins on Arduino
	MOTOR_1_PWM = 11
	MOTOR_2_PWM = 10
	MOTOR_3_PWM = 9

	MOTOR_1_A = 5
	MOTOR_2_A = 7
	MOTOR_3_A = 6

	MOTOR_1_B = 8
	MOTOR_2_B = 4
	MOTOR_3_B = 3

	KICKER = 12

	def __init__(self, *args, **kwargs):
		Thread.__init__(self)
		self.daemon = True
		self.running = False
		self.mock_mode = False
		self.board = None
		self.data = {}
		self.last_direction = None
		self.kicker_start = time() - 0.71  # no kick on start
		self.start()

	def get_kicker_status(self):
		REGEN_TIME = 4

		since = time() - self.kicker_start
		if self.make_kick() and since > REGEN_TIME:
			self.kicker_start = time()

		return (time() - self.kicker_start) < 0.7

	def load_data(self, data):
		self.data = data

	def setup_pymata(self):
		# Here we initialize the motor pins on Arduino
		try:
			for k, v in [
				("MOTOR_1_PWM", self.MOTOR_1_PWM),
				("MOTOR_2_PWM", self.MOTOR_2_PWM),
				("MOTOR_3_PWM", self.MOTOR_3_PWM),
				("MOTOR_1_A", self.MOTOR_1_A),
				("MOTOR_2_A", self.MOTOR_2_A),
				("MOTOR_3_A", self.MOTOR_3_A),
				("MOTOR_1_B", self.MOTOR_1_B),
				("MOTOR_2_B", self.MOTOR_2_B),
				("MOTOR_3_B", self.MOTOR_3_B),
			]:
				print(k, v)

			board = PyMata(bluetooth=False)

			board.set_pin_mode(self.MOTOR_1_PWM, board.PWM,    board.DIGITAL)
			board.set_pin_mode(self.MOTOR_1_A,   board.OUTPUT, board.DIGITAL)
			board.set_pin_mode(self.MOTOR_1_B,   board.OUTPUT, board.DIGITAL)
			board.set_pin_mode(self.MOTOR_2_PWM, board.PWM,    board.DIGITAL)
			board.set_pin_mode(self.MOTOR_2_A,   board.OUTPUT, board.DIGITAL)
			board.set_pin_mode(self.MOTOR_2_B,   board.OUTPUT, board.DIGITAL)
			board.set_pin_mode(self.MOTOR_3_PWM, board.PWM,    board.DIGITAL)
			board.set_pin_mode(self.MOTOR_3_A,   board.OUTPUT, board.DIGITAL)
			board.set_pin_mode(self.MOTOR_3_B,   board.OUTPUT, board.DIGITAL)

			board.digital_write(self.KICKER, False)

			self.board = board
			self.running = True

		except (serial.serialutil.SerialException, FileNotFoundError) as err:
			print("# Something wrong with the serial device:{}".format(err))
			print("# Running in MOCK mode")
			self.mock_mode = True

	def clean_up(self):
		if self.board:
			# self.board.reset() # does weird shit
			# so we do manual labor
			self.board.digital_write(self.MOTOR_1_B, False)
			self.board.digital_write(self.MOTOR_1_A, False)
			self.board.digital_write(self.MOTOR_2_B, False)
			self.board.digital_write(self.MOTOR_2_A, False)
			self.board.digital_write(self.MOTOR_3_B, False)
			self.board.digital_write(self.MOTOR_3_A, False)

			self.board.analog_write(self.MOTOR_1_PWM, 255)
			self.board.analog_write(self.MOTOR_2_PWM, 255)
			self.board.analog_write(self.MOTOR_3_PWM, 255)

		self.board.digital_write(self.KICKER, True)  # should close with discharge

		self.running = False
		self.mock_mode = False

	def __del__(self):
		print("# DELETING MOTOR")
		self.clean_up()

	def close(self):
		self.clean_up()
		print("# Motor closed")

	def get_xyw(self):
		Fx, Fy, Fw = -self.data.get('Fx', 0), self.data.get('Fy', 0), self.data.get('Fw', 0)
		Fx, Fy, Fw = Fx * 0.25, Fy * 0.25, Fw * 0.25
		return Fx, Fy, Fw

	def make_kick(self):
		return self.data.get('K')

	def translate(self):
		Fx, Fy, Fw = self.get_xyw()
		a,b = Fx, Fy
		f = radians(45)
		print(f)
		Fx = Fx * cos(f) - Fy * sin(f)
		Fy = Fy * cos(f) + Fx * sin(f)

		angle = lambda x,y: atan2(y,x)/pi*180
		print("Ang",angle(a,b),angle(Fx,Fy))

		print((round(a,4), round(b,4)),(round(Fx,4), round(Fy,4)), )

		matrix = [[0.58, -0.33, 0.33], [-0.58, -0.33, 0.33], [0, 0.67, 0.33]]

		Fa, Fb, Fc = numpy.dot(matrix, [Fx, Fy, Fw])

		print((int(float(Fa) * 255)), (int(float(Fb) * 255)), (int(float(Fc) * 255)))

		return Fa, Fb, Fc

	def run(self):
		print("# START MOTOR THREAD")
		self.setup_pymata()
		print("# LOADED PYMATA 2")

		last = ()
		while self.mock_mode:
			Fa, Fb, Fc = self.translate()
			if self.last_direction != (Fa, Fb, Fc):
				# print("INPUT:  {:.4f} {:.4f} {:.4f}".format(*self.get_xyw()))
				print("RESULT: {:.4f} {:.4f} {:.4f}".format(Fa, Fb, Fc))
			self.last_direction = (Fa, Fb, Fc)
			sleep(0.02)

		while self.running:
			sleep(0.1)

			#print(self.get_kicker_status(), time() - self.kicker_start)
			self.board.digital_write(self.KICKER, self.get_kicker_status())

			Fa, Fb, Fc = self.translate()
			if self.last_direction == (Fa, Fb, Fc):
			    continue
			self.last_direction = (Fa, Fb, Fc)
			#print("RESULT: {:.4f} {:.4f} {:.4f}".format(Fa, Fb, Fc))

			

			# reset things TODO: wai thou?
			self.board.analog_write(self.MOTOR_1_PWM, 255)
			self.board.analog_write(self.MOTOR_2_PWM, 255)
			self.board.analog_write(self.MOTOR_3_PWM, 255)

			self.board.digital_write(self.MOTOR_1_B, 0)
			self.board.digital_write(self.MOTOR_1_A, 0)
			self.board.digital_write(self.MOTOR_2_B, 0)
			self.board.digital_write(self.MOTOR_2_A, 0)
			self.board.digital_write(self.MOTOR_3_B, 0)
			self.board.digital_write(self.MOTOR_3_A, 0)

			# Set directions
			self.board.digital_write(self.MOTOR_1_A, Fa < 0)
			self.board.digital_write(self.MOTOR_1_B, Fa > 0)
			self.board.digital_write(self.MOTOR_2_A, Fb < 0)
			self.board.digital_write(self.MOTOR_2_B, Fb > 0)
			self.board.digital_write(self.MOTOR_3_A, Fc < 0)
			self.board.digital_write(self.MOTOR_3_B, Fc > 0)

			# Set duty cycle
			dFa, dFb, dFc = abs(int(float(Fa) * 255)), abs(int(float(Fb) * 255)), abs(int(float(Fc) * 255))

			print(	dFa,
					dFb,
					dFc)
			self.board.analog_write(self.MOTOR_1_PWM, 255 - dFa)
			self.board.analog_write(self.MOTOR_2_PWM, 255 - dFb)
			self.board.analog_write(self.MOTOR_3_PWM, 255 - dFc)
