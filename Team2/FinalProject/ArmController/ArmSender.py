#!/usr/bin/env python
import signal
import sys
import numpy as np
import time
import serial
import io
import glob


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


class ArmSender:
    def __init__(self, comport='COM3') -> None:
        self.com = serial.Serial(comport, 115200)
        #self.sio = io.TextIOWrapper(io.BufferedRWPair(self.com, self.com))
        assert (self.com.is_open), "Failed to open comport"

    def send(self, yaw, pitch, height):
        if (self.com.is_open):
            self.com.write(('{0}, {1}, {2}\n'.format(
                yaw, pitch, height)).encode('ascii'))
            # self.sio.flush()

    def close(self):
        self.com.close()


if __name__ == '__main__':
    print(serial_ports())

    sender = ArmSender('/dev/ttyUSB0')

    def signal_handler(sig, frame):
        print('Terminating comport...')
        sender.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    t = 0
    dt = 0.1
    while t < 50:
        pitch = np.sin(2*t) * np.pi / 8 + np.pi / 2
        yaw = np.cos(t) * np.pi / 4
        height = np.cos(t/5) * 2 + 10
        sender.send(yaw, pitch, height)
        time.sleep(dt)
        t += dt
    sender.close()
