import os
import numpy as np
import serial
from threading import Thread, Event
from time import sleep
from math import cos, sin, radians, atan2, pi
import logging
logger = logging.getLogger("arduino")

class Arduino(Thread):
    def __init__(self, factor=1):
        Thread.__init__(self)
        self.daemon = True
        from configparser import ConfigParser
        cp = ConfigParser()
        cp.readfp(open(os.path.expanduser("~/.robovision/pymata.conf")))

        self.wake = Event() # wake up signal for the /dev observer
        self.changed = Event() # motor values have changed
        self.factor = factor

        indexes = 1, 2, 3
        device =  cp.get("global", "path")
        self.path = os.path.join("/dev/serial/by-path", device)
        self.kicker = cp.getint("global", "kicker")
        self.pwm = [cp.getint("motor%d" % j, "pwm") for j in indexes]
        self.fwd = [cp.getint("motor%d" % j, "fwd") for j in indexes]
        self.rev = [cp.getint("motor%d" % j, "rev") for j in indexes]

        # X, Y, rotation to A, B, C transformation matrix
        self.matrix = np.linalg.inv([
            [-1/2,     -1/2,      1],
            [3**0.5/2, -3**0.5/2, 0],
            [1,        1,         1]])
        self.alive = False
        self.running = False
        self.speed = None
        self.set_abc(0,0,0)
        self.board = None

    def clean_up(self):
        if self.board:
            for pin in self.pwm:
                self.board.analog_write(pin, 255)
            for pin in self.fwd + self.rev:
                self.board.digital_write(pin, False)
            self.board.digital_write(self.kicker, True) # should close with discharge
        self.running = False

    def set_yw(self, y, w):
        """
        Set forward-backwards speed and rotation
        """
        self.set_abc(0.866*y, -0.866*y, w)

    def set_xyw(self, x, y, w):
        a = 0.5 * x + 0.866 * y - w
        b = 0.5 * x - 0.866 * y - w
        c = -x - w

        m = max([abs(a), abs(b), abs(c), 2.366])
        if m > 1.0:
            a = a / m
            b = b / m
            c = c / m
        self.set_abc(a,b,-c)

    def set_abc(self, *speed):
        for j in speed:
            assert j >= -1.0 and j <= 1.0
        speed = [j*self.factor for j in speed]
        if self.speed != speed:
            logger.debug("Setting motor speeds: %s %s %s", *speed)
            self.speed = speed
            self.changed.set()

    def stop(self):
        self.running = False
        self.wake.set()
        self.changed.set()

    def run(self):
        try:
            from PyMata.pymata import PyMata
        except ImportError:
            logger.error("Pymata not installed, Arduino disabled!")
            return
        else:
            self.running = True
            while self.running:
                if not self.board:
                    if not os.path.exists(self.path):
                        logger.info("No Arduino, waiting for %s to become available", self.path)
                        self.wake.wait()
                    if not self.running:
                        break
                    board = PyMata(self.path, bluetooth=False)
                    try:
                        for pin in self.pwm:
                            logger.debug("Setting up pin %s as PWM output"%pin)
                            board.set_pin_mode(pin, board.PWM, board.DIGITAL)
                        for pin in self.fwd + self.rev:
                            logger.debug("Setting up pin %s as digital output" %pin)
                            board.set_pin_mode(pin, board.OUTPUT, board.DIGITAL)
                        board.set_pin_mode(self.kicker,   board.OUTPUT, board.DIGITAL)
                        board.digital_write(self.kicker, False)
                    except serial.serialutil.SerialException:
                        logger.error("Failed to connect to Arduino")
                        continue # Try again
                    else:
                        self.board = board
                        self.alive = True

                self.changed.wait()
                if not self.running:
                    break
                self.changed.clear()
                self.write()
            self.set_abc(0,0,0)
            self.write()

    def write(self):
        try:
            for speed, pwm_pin, fwd_pin, rev_pin in zip(self.speed, self.pwm, self.fwd, self.rev):
                self.board.analog_write(pwm_pin, 255) # Disable H-bridge
                self.board.digital_write(fwd_pin, 0) # Reset forward
                self.board.digital_write(rev_pin, 0) # Reset reverse
                self.board.digital_write(fwd_pin, speed <= 0) # Set forward
                self.board.digital_write(rev_pin, speed >= 0) # Set reverse
                if speed == 0:
                    self.board.analog_write(pwm_pin, 0) # Set max duty cycle to brake
                else:
                    self.board.analog_write(pwm_pin, 255-abs(int(float(speed) * 255))) # Set duty cycle
        except serial.serialutil.SerialException:
            logger.info("Arduino disconnected at %s", self.path)
            self.alive = False
            self.board = None

    def kick(self):
        self.board.digital_write(self.kicker, True)
        sleep(0.1)
        self.board.digital_write(self.kicker, False)
        sleep(0.1)

if __name__ == "__main__":
    arduino = Arduino()
    arduino.set_xyw(0,0,1)
    sleep(6)
    arduino.stop()
    arduino.join()
