import struct
import serial
from kbhit import KBHit
from time import sleep

ser = serial.Serial("/dev/ttyACM0", 19200, timeout=0)
kb = KBHit()

while True:
    for x in range(0, 0xff, 10):
        packet = struct.pack("5B", 0xaa, 0xaa, x, x, x)
        print(x, end=' ')
        ser.write(packet)
        sleep(0.05)
        if ser.inWaiting() > 0:
            data_str = ser.read(ser.inWaiting())
            print(data_str.decode())
            #print(struct.unpack('5B', data_str), end='')
        print()

while True:
    if kb.kbhit():
        c = kb.getch()
        print(c)
    if ser.inWaiting() > 0:
        data_str = ser.read(ser.inWaiting()).decode('ascii')
        print(data_str, end='')
