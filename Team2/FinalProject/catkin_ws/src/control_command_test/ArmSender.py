#!/usr/bin/env python
import signal
import sys
import numpy as np
import time
import serial
import glob
import threading


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


arm_param = {
    'h0': 530,
    'alpha': 508
}


def get_command_from_mRTP(mRTP):
    phi = np.arctan2(mRTP[1, 1], -mRTP[0, 1])
    theta = np.arctan2(mRTP[2, 0], -mRTP[2, 2])
    h = mRTP[2, 3] - arm_param['h0'] - arm_param['alpha'] * np.cos(theta)
    return np.array([phi, theta, h])

def gen_command_from_mRTP(P_r, h=arm_param['h0']):
    x, y, z = P_r
    #x, y, z = mRTP[:3, 3]
    d = np.sqrt(x**2 + y**2)
    theta = np.pi/2 - np.arctan2((z-(h-200)/1000.0), d)
    phi = np.arctan2(y, x)
    return np.array([phi, theta, 0.1])

class ArmSender():
    """This module allow you to communicate with the Arm controller with 
    send() and read() function. It will send the command through USB com
    port, so please remember to check the available com ports which will 
    be printed in the begining of the program.

    Usage: 
        1. Construct the object.
        2. Start the port listener by start().
        3. You can send or read commands.
        4. Remember to call close() to terminate the daemon.
    """

    def __init__(self, comport='/dev/ttyUSB0', timeout=5) -> None:
        self.com = serial.Serial(comport, 115200, timeout=timeout)
        #self.sio = io.TextIOWrapper(io.BufferedRWPair(self.com, self.com))
        assert (self.com.is_open), "Failed to open comport"
        self.thread = threading.Thread(target=self.__daemon)
        self.thread_alive = True
        self.data = np.zeros(3)
        self.pitch = np.pi / 2

        # Setup signal handler
        def signal_handler(sig, frame):
            self.close()
            sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)

    def __daemon(self):
        while self.thread_alive:
            str = self.com.readline()
            try:
                token = str.decode('ascii').split((','))
                self.data = np.array(token, dtype=float)
            except:
                print(str)
        print("Daemon for serial port listener ended")

    def start(self):
        self.thread.start()
        
    def send(self, P_r, h=arm_param['h0']):
        """
        P_r: np array [x, y, z]
        """
        cmds = gen_command_from_mRTP(P_r, h=arm_param['h0'])
        self.__send(cmds[0], cmds[1], cmds[2])

    def __send(self, yaw, pitch, height):
        if (self.com.is_open):
            self.com.write(('{0}, {1}, {2}\n'.format(
                yaw, pitch, height)).encode('ascii'))

    def send_raw(self, yaw, pitch, height):
        self.pitch = pitch
        self.__send(yaw, pitch, height)

    def read(self):
        return self.data

    def get_mRTA(self):
        p, h = self.data
        t = self.pitch
        # print("angles:", p, t, h)
        c, s = np.cos, np.sin
        cp, sp = c(p), s(p)
        ct, st = c(t), s(t)
        return np.array([
            [cp*ct, -sp, -cp*st, 508*cp*st / 1000],
            [ct*sp,  cp, -sp*st, 508*sp*st / 1000],
            [st   ,   0,     ct, (h+arm_param['h0']+508*ct) / 1000],
            [0, 0, 0, 1]])

    def get_end_effector_pos(self):
        return self.get_mRTA() @ (np.array([[0,0,0,1]]).T)

    def close(self):
        print('Terminating comport...')
        self.thread_alive = False
        self.com.close()
        self.thread.join()


if __name__ == '__main__':
    print(serial_ports())

    sender = ArmSender('/dev/ttyUSB0')
    sender.start()

    t = 0
    dt = 0.1
    while t < 50:
        pitch = np.sin(2*t) * np.pi / 2 + np.pi / 2
        yaw = np.cos(4*t) * np.pi / 4
        if (t > 30):
            yaw = np.pi/4
        height = np.cos(t/5) * 2 + 10
        sender.send_raw(yaw, pitch, height)
        print(sender.read())

        time.sleep(dt)
        t += dt
    sender.close()
