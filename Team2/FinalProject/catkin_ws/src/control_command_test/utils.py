import numpy as np


def Rx(t):
    return np.array([[1, 0, 0],
                     [0, np.cos(t), -np.sin(t)],
                     [0, np.sin(t),  np.cos(t)]])


def Ry(t):
    return np.array([[np.cos(t), 0, np.sin(t)],
                     [0, 1,         0],
                     [-np.sin(t), 0, np.cos(t)]])


def Rz(t):
    return np.array([[np.cos(t), -np.sin(t), 0],
                     [np.sin(t),  np.cos(t), 0],
                     [0,          0, 1]])

def get_rotation_matrix_from_quaternian(Q):
    q = np.array((Q.w, Q.x, Q.y, Q.z))
    return 2 * np.array([[q[0]**2+q[1]**2-0.5, q[1]*q[2]-q[0]*q[3], q[0]*q[2]+q[1]*q[3]],
                        [q[0]*q[3]+q[1]*q[2], q[0]**2+q[2]
                            ** 2-0.5, q[2]*q[3]-q[0]*q[1]],
                        [q[1]*q[3]-q[0]*q[2], q[0]*q[1]+q[2]*q[3], q[0]**2+q[3]**2-0.5]])

class Filter:
    # init
    def __init__(self):
        self.sample_count  = 1
        self.x_accumulate = 0  
        self.y_accumulate = 0
        self.z_accumulate = 0  

    # method
    def accumulate(self, new_x, new_y, new_z):
        self.x_accumulate += new_x
        self.y_accumulate += new_y
        self.z_accumulate += new_z
        self.sample_count += 1

    def reset(self):
        self.x_accumulate = 0
        self.y_accumulate = 0
        self.z_accumulate = 0
        self.sample_count = 0
    
    def filter(self):
        x_filter = self.x_accumulate / self.sample_count
        y_filter = self.y_accumulate / self.sample_count
        z_filter = self.z_accumulate / self.sample_count
        return x_filter, y_filter, z_filter