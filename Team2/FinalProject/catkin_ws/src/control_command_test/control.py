import rospy
from nav_msgs.msg import Odometry
import numpy as np
from utils import get_rotation_matrix_from_quaternian
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Twist

# =================================
# Global variable for control
# =================================
robot_raw_pose = Odometry()  # odometry data type
goal_o = np.array([0,0])
end_effector_list = []
# =================================
# Subscriber Callback
# =================================

theta_status = False
position_ststus = False


def feedback_loop(data):
    global robot_raw_pose
    robot_raw_pose = data.pose.pose
    robot_x_o = data.pose.pose.position.x
    robot_y_o = data.pose.pose.position.y

    quaternion_w = data.pose.pose.orientation.w
    quaternion_x = data.pose.pose.orientation.x
    quaternion_y = data.pose.pose.orientation.y
    quaternion_z = data.pose.pose.orientation.z

    (row, pitch, yaw) = euler_from_quaternion([quaternion_x, quaternion_y, quaternion_z, quaternion_w])
    robot_theta_o = yaw*180/math.pi

    pioneer_point = PointStamped()
    pioneer_point.point.x = robot_x_o
    pioneer_point.point.y = robot_y_o
    pioneer_point.point.z = robot_theta_o
    pioneer_point.header.stamp = rospy.Time.now()

    pioneer_pub.publish(pioneer_point)
    


    global goal_o
    
    theta = math.atan2((goal_o[1] - robot_y_o),(goal_o[0] - robot_x_o))*180/math.pi # - robot_theta_o
    twist_cmd = Twist()
    position_stop_thresh = 0.1
    if(abs(theta)+abs(robot_theta_o)>180):
        if (theta>0):
            theta -= 180
        else:
            theta += 180
        if (robot_theta_o>0):
            robot_theta_o -= 180
        else:
            robot_theta_o += 180
    
    # theta = 45
    if(theta-robot_theta_o >= 0):
        rotate_sign = 1
    else:
        rotate_sign = -1


    dist = math.sqrt((goal_o[0] - robot_x_o)**2 + (goal_o[1] - robot_y_o)**2)
    print("[Dist:]",dist,"\t T,Tr:",theta," ", robot_theta_o)
    if (dist > 0.2 or abs(theta-robot_theta_o)>15):
        if(abs(theta - robot_theta_o) > 15):
            if(abs(theta - robot_theta_o) > 5):
                twist_cmd.angular.z = 0.2 * rotate_sign
                pass
            elif(abs(theta - robot_theta_o) <5):
                twist_cmd.angular.z = 0.1 * rotate_sign
                pass
            # elif(abs(theta - robot_theta_o) <2):
            #     twist_cmd.angular.z = 0.1 * rotate_sign
            #     pass
        else:
            twist_cmd.angular.z = 0
            print("dist: ", dist)
            if(dist > position_stop_thresh):
                if(dist>1.0):
                    twist_cmd.linear.x = 0.3
                else:
                    twist_cmd.linear.x = 0.1
            else:
                twist_cmd.linear.x = 0
    #print("Goal", goal_o ,"Curr Loc:",(robot_x_o,robot_y_o),"Target:", theta , "Current: " ,robot_theta_o)
    print("[Goal]",goal_o)
    
    pub_vel.publish(twist_cmd)

    
    
def callback_endeff(data):
    # print(data)
    end_effector_list.clear()
    end_effector_list.append(data)
    pass


def callback_planning(data):
    goal_r = np.array([data.point.x, data.point.y, 0, 1], ndmin=2).T 

    # print('goal_r:', goal_r)

    mOTR = np.eye(4)
    mOTR[:3,:3] = get_rotation_matrix_from_quaternian(robot_raw_pose.orientation)
    mOTR[0,3] = robot_raw_pose.position.x
    mOTR[1,3] = robot_raw_pose.position.y
    mOTR[2,3] = robot_raw_pose.position.z

    if(len(end_effector_list)==1):
        data = end_effector_list.pop()
        eff_r = data.point
        eff_o = mOTR @ (np.array([[eff_r.x, eff_r.y, eff_r.z, 1]]).T)
        print("eff_o: ", eff_o)
        
        aux = PointStamped() # ideal pioneer pose in Robot frame
        aux.point.x = eff_o[0,0]
        aux.point.y = eff_o[1,0]
        aux.point.z = eff_o[2,0]

        aux.header.stamp = rospy.Time.now()
        aux.header.frame_id = "eff"

        # print((mKTP@R_p)[:3, 0])
        # print((mRTK@mKTP@R_p)[:3, 0])

        eff_o_pub.publish(aux)
    global goal_o
    goal_o = (mOTR@goal_r)[:2, 0].T

    # print(goal_o)

    # print("Planning data:")
    # print(data.point)
    # print("Goal in frame O:")
    # print(goal_o)
    

# =================================
# Listener
# =================================
def listener():
    rospy.Subscriber("/Planning",PointStamped,callback = callback_planning)
    rospy.Subscriber("/RosAria/pose",Odometry,callback = feedback_loop)
    rospy.Subscriber('/end_effector',PointStamped,callback=callback_endeff)
    rate = rospy.Rate(20)

    i = 0
    while not rospy.is_shutdown():
        rate.sleep()


# =================================
# Main
# =================================
if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    pub_vel = rospy.Publisher('/RosAria/cmd_vel',Twist,queue_size=1)
    pioneer_pub = rospy.Publisher('/pioneer_pose',PointStamped,queue_size=1)
    eff_o_pub = rospy.Publisher('/eff_o',PointStamped,queue_size=1)
    listener()
