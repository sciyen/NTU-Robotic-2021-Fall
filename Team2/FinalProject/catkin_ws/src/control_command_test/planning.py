from time import time
import rospy
from tf.transformations import *
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
from queue import Queue

from utils import Rx, Ry, Rz, get_rotation_matrix_from_quaternian, Filter
from ArmSender import ArmSender, serial_ports

from matplotlib import pyplot as plt


# =================================
# Global variable for planning
# =================================
ideal_distance = 2   # (meters)
waypoint_number = 4

spine_pose = PoseStamped().pose  # Posestamped data type
head_pose = PointStamped().point  # Pointstamped data type
angle_offset = 0  # degree
kinect_yaw = 0  # radian


###DEBUG ONLY
from std_msgs.msg import Float32MultiArray
fig = plt.figure(figsize=(10,10))
plt.ion()
plt.grid()
#####


# =================================
# Subscriber Callback
# =================================
# Head
def callback_Head(data): 
    global head_pose
    head_pose = data.point
    # head_pose.accumulate(data.point.x, data.point.y, data.point.z)        

# Spine
def callback_Spine(data):    
    global spine_pose
    spine_pose = data.pose
    
# CMD
def callback_CMD(data):
    # right: 1, left: 0
    global angle_offset

    # only cares about the left and right command
    if data.point.x == 0 :
        angle_offset += 10
    elif data.point.x == 1:
        angle_offset -= 10

# =================================
# Listener (listen from Kinect)
# =================================
def listener():
    rospy.Subscriber("/Head", PointStamped, callback_Head)
    rospy.Subscriber("/Spine", PoseStamped, callback_Spine)
    rospy.Subscriber("/Kinect_CMD",PointStamped,callback = callback_CMD)

    rate = rospy.Rate(20)

    i = 0
    while not rospy.is_shutdown():
        
        # #If you need Knee Distance
        # if(q_knee_left.full() and q_knee_right.full()):
        #     dist = 0
        #     k_l = q_knee_left.get()
        #     k_r = q_knee_right.get()
        #     dist = math.sqrt(((k_l.x+k_r.x)/2)**2 + ((k_l.y+k_r.y)/2)**2 + ((k_l.z+k_r.z)/2)**2)
        #     print(dist)

        rate.sleep()

# =================================
# Publisher
# =================================
class Planning_publisher:

    def __init__(self):
        global fig
        self.planning_publisher = rospy.Publisher('/Planning',PointStamped,queue_size=1)
        self.waypoint = PointStamped() # ideal pioneer pose in Robot frame
        ###DEBUG ONLY
        self.fig = fig
        self.pub_arr = rospy.Publisher('/debug_waypoints',Float32MultiArray,queue_size=1)
        

    def planning(self, event=None):
        p = spine_pose.position
        print("here")

        if (p.x == 0 and p.y ==0 and p.z ==0):
            print("in if")
            return

        print("out if")
        # mKTP
        mKTP = np.eye(4)
        mKTP[:3, :3] = get_rotation_matrix_from_quaternian(spine_pose.orientation)

        mKTP[:3, 3] = np.array((p.x, p.y, p.z), ndmin=2)

        # robot and performancer pose in p
        R_p = np.array([0, 0, ideal_distance, 1], ndmin=2).T    #ideal distance
        P_p = np.array([0, 0, 0, 1], ndmin=2).T

        rotationAngle = np.eye(4)
        #rotationAngle[:3, :3] = Ry(angle_offset*np.pi/180)
        print("[offset]: ", angle_offset)
        
        #R_p = rotationAngle @ R_p

        # robot and performancer pose in k
        R_k = (mKTP@R_p)
        P_k = (mKTP@P_p)

        # print("R_k: ",R_k)
    

        # robot and performancer pose in k
        mRTK = np.eye(4)
        mRTK[:3, :3] = Ry(90*np.pi/180)@Rz(90*np.pi/180)@Ry(kinect_yaw)@Rx(-10*np.pi/180)
        R_r = mRTK@R_k
        P_r = mRTK@P_k
        # print("R_r: ",R_r)
        # print("P_r: ",P_r)

        # waypoint
        vector_Origin_to_P = np.array([0, 0], ndmin=2) - P_r[:2,0]
        vector_R_to_P = R_r[:2, 0] - P_r[:2, 0]
        # print("o to p: ", vector_Origin_to_P)
        # print("r to p: ", vector_R_to_P)

        unit_vector_1 = vector_Origin_to_P / np.linalg.norm(vector_Origin_to_P)
        unit_vector_2 = vector_R_to_P / np.linalg.norm(vector_R_to_P)

        dot_product = np.dot(unit_vector_1, unit_vector_2)
        vector_product = np.cross(unit_vector_2, unit_vector_1)

        theta = np.arccos(dot_product)*np.sign(vector_product)[0]
        theta = (waypoint_number-1)*theta/waypoint_number

        # print("theta:", theta)

        rotationAngle[:3, :3] = Rz(theta[0])
        waypoint = rotationAngle@(R_r-P_r) + P_r

        # print("=========waypoint=========: ", waypoint)

        ### DEBUG ONLY
        floatarr = Float32MultiArray()
        floatarr.data.append(waypoint[0,0])
        floatarr.data.append(waypoint[1,0])
        floatarr.data.append(P_r[0,0])
        floatarr.data.append(P_r[1,0])
        floatarr.data.append(R_r[0,0])
        floatarr.data.append(R_r[1,0])

        self.pub_arr.publish(floatarr)
        ####################
        
        # plt.clf()
        # plt.scatter(waypoint[0,0],waypoint[1,0],color = 'r')
        # plt.scatter(P_r[0,0],P_r[1,0],color = 'b')
        # plt.scatter(R_r[0,0],R_r[1,0])
        # plt.show()
        

        # publish
        self.waypoint = PointStamped() # ideal pioneer pose in Robot frame
        self.waypoint.point.x = waypoint[0,0]
        self.waypoint.point.y = waypoint[1,0]
        # self.waypoint.point.z = waypoint[2,0]

        self.waypoint.header.stamp = rospy.Time.now()
        self.waypoint.header.frame_id = "Plan"

        # print((mKTP@R_p)[:3, 0])
        # print((mRTK@mKTP@R_p)[:3, 0])

        self.publish()


    def publish(self, event=None):
        self.planning_publisher.publish(self.waypoint)

class Arm_Interface:
    def __init__(self):
        import time
        print(serial_ports())
        self.publisher = rospy.Publisher('/end_effector',PointStamped,queue_size=1)
        self.end_effector = PointStamped()
        self.arm_sender = ArmSender('/dev/ttyUSB2')
        self.arm_sender.start()

        data = self.arm_sender.read()

        while np.all(data == 0):
            data = self.arm_sender.read()
            print("Wait for Arm to reset, current input: ", data)
            time.sleep(0.5)


    def read(self, event=None):
        #print("========================")
        global kinect_yaw
        # print("===", self.arm_sender.read())
        kinect_yaw = self.arm_sender.read()[0]
        # print(f"kinect yaw: {kinect_yaw}")

        End_r = self.arm_sender.get_end_effector_pos()
        #print("End_r:", End_r)

        self.end_effector.point.x = End_r[0,0]
        self.end_effector.point.y = End_r[1,0]
        self.end_effector.point.z = End_r[2,0]

        self.end_effector.header.stamp = rospy.Time.now()
        self.end_effector.header.frame_id = "Plan"
        self.publisher.publish(self.end_effector)

    
    def send(self, event=None):
        # send Arm command
        head_array = np.array([head_pose.x, head_pose.y, head_pose.z, 1]).T

        # print(((mRTK@head_array).T)[0:3])
        mRTK = np.eye(4)
        mRTK[:3, :3] = Ry(90*np.pi/180)@Rz(90*np.pi/180)@Ry(kinect_yaw)@Rx(-10*np.pi/180)
        
        #print("command: ", ((mRTK@head_array).T)[0:3])
        self.arm_sender.send(((mRTK@head_array).T)[0:3])
    

# =================================
# Main
# =================================
if __name__ == '__main__':

    rospy.init_node('planning', anonymous=True)

    ai = Arm_Interface()
    rospy.Timer(rospy.Duration(1.0/10.0), ai.read)
    rospy.Timer(rospy.Duration(1.0/10.0), ai.send)
    

    pp = Planning_publisher()
    rospy.Timer(rospy.Duration(1.0), pp.planning)


    listener()
