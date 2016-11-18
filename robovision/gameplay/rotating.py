
import logging
logger = logging.getLogger("gameplay")
from managed_threading import ManagedThread
from image_recognition import ImageRecognition
from arduino import Arduino
import math

class Gameplay(ManagedThread):
    """
    Naive rotating gameplay
    """

    def step(self, r, *args):
        """
        r.goal_yellow - Yellow goal relative to kicker, None if not detected
        r.goal_blue - Blue goal relative to kicker, None if not detected
        r.goal_blue.x
        r.goal_blue.y
        r.goal_blue.dist
        r.goal_blue.angle_deg
        r.robot - Robot's position on the field, None if not positioned
        r.orientation - Kicker's rotation along field grid, None if not positioned
        r.balls - List of recognized ball tuples, first element relative and second one absolute if robot is positioned
        """

        if r.ball_grabbed:
            if r.goal_blue:
                if r.goal_blue.angle_deg > 5:
                    delta = -0.3
                elif r.goal_blue.angle_deg < -5:
                    delta = 0.3
                else:
                    self.arduino.set_abc(0,0,0)
                    self.arduino.kick()
                    delta = 0

                r = delta
                if r < 0:
                    self.arduino.set_abc(-abs(r) * 0.05, -abs(r) * 0.1, r*0.8)
                else:
                    self.arduino.set_abc(abs(r) * 0.1, abs(r) * 0.05, r*0.8)
                state = "Got the ball, found goal turning towards goal"

            else:
                state = "Got the ball, looking for goal"
        else:
            for relative, absolute, _, _, _ in r.balls:
                if abs(relative.angle_deg) < 10:
                    j = -relative.angle_deg/180
                elif -relative.angle_deg < 0:
                    j = -0.3
                elif -relative.angle_deg > 0:
                    j = 0.3
                else:
                    j = 0

                if j > 0 and j < 0.2:
                    j = 0.2
                if j < 0 and j > -0.2:
                    j = -0.2

                self.arduino.set_xyw(
                    relative.y/relative.dist*min(relative.dist, 0.99),
                    relative.x/relative.dist*min(relative.dist, 0.99),
                    j  )
                state = "Turning and going towards the ball"
                break
            else:
                if r.robot:
                    state = "Driving towards the center of the field"
                    if r.orientation > 0 and r.orientation < math.pi:
                        self.arduino.set_xyw((r.robot.x-2.3)*0.5, r.robot.y*0.5, 0.25)
                    else:
                        self.arduino.set_xyw((r.robot.x-2.3)*0.5, r.robot.y*0.5, -0.25)
                else:
                    state = "Could not find any balls and I don't know where I am"
                    self.arduino.set_xyw(0,0,0)

        if state != self.state:
            logger.info(state)
            self.state = state

    def __init__(self, *args):
        ManagedThread.__init__(self, *args)
        self.arduino = Arduino() # config read from ~/.robovision/pymata.conf
        self.state = None

    def start(self):
        self.arduino.start()
        ManagedThread.start(self)
