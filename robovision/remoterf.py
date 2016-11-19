from threading import Thread
from enum import Enum
import serial
from time import sleep

class State(Enum):
    start = 1
    field_id = 2
    robot_id = 3
    command = 4
    command_all = 5
    end = 6


class RemoteRF(Thread):
    def __init__(self, dev, field_id='A', robot_id='A'):
        Thread.__init__(self)
        self.ser = serial.Serial(dev, timeout=1)
        self.field_id = field_id
        self.robot_id = robot_id
        self.daemon = True
        self._stop = False
        self._fight = False


    @property
    def fight(self):
        """Return True or False based on judge remotecontrol"""
        return self._fight

    def parse_packet(self, packet):
        pass

    def stop(self):
        self._stop = True

    def run(self):
        state = State.start
        while not self._stop:
            ack_packet = "a{}{}ACK------".format(self.field_id, self.robot_id)
            ack_packet = ack_packet.encode()
            c = self.ser.read(1)
            try:
                c = c.decode()
            except:
                continue
            if not c:
                continue
            if state == State.start and c == 'a':
                state = State.field_id
                print("packet start")
                continue
            if state == State.field_id:
                print("got field id", c, end=" ")
                if c == self.field_id:
                    state = State.robot_id
                    print("mine")
                    continue
                else:
                    state = State.start
                    print("restart")
                    continue
            if state == State.robot_id:
                print("got robot id", c, end=" ")
                if c == self.robot_id:
                    state = State.command
                    print("mine")
                    continue
                elif c == 'X':
                    state = State.command_all
                    print("all")
                    continue
                else:
                    state = State.start
                    print("restart")
                    continue
            if state == State.command or state == State.command_all:
                cmd = self.ser.read(4).decode()
                cmd = c + cmd
                resp = state == State.command
                print("got command", cmd, end=" ")
                if cmd == "START":
                    self._fight = True
                    print("start", end=" ")
                    if resp:
                        self.ser.write(ack_packet)
                        print("ack", end=" ")
                elif cmd == "STOP-":
                    self._fight = False
                    print("stop", end=" ")
                    if resp:
                        self.ser.write(ack_packet)
                        print("ack", end=" ")
                elif cmd == "PING-":
                    print("ping", end=" ")
                    self.ser.write(ack_packet)
                    print("ack")
                state = State.start
                self.ser.read(4) # read end of the packet
                print()
                continue

if __name__ == "__main__":
    rf = RemoteRF("/dev/ttyACM0", field_id='B', robot_id='A')
    rf.start()
    while True:
        #print(rf.fight)
        sleep(0.1)
    rf.stop()
