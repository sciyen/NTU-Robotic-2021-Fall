import queue
from matplotlib import pyplot as plt
import numpy as np
import math
import rospy
#input pioneer_data[x,y,theta], arm_data[height, armpoint:[x,y,z]]
from mpl_toolkits.mplot3d import Axes3D
from queue import Queue
from geometry_msgs.msg import PointStamped

pioneer_height = 0.25
pioneer_width = 0.4
pioneer_length = 0.45
pioneer_angle = math.atan(0.225/0.2)



print(pioneer_angle)
r = math.sqrt(0.225**2 + 0.2**2)
fig = plt.figure(figsize = (10,10))


def draw_3D(pioneer_data , arm_data ):
    arm_height = 0.5
    ###DH Table 
    arm_endpoint = arm_data

    ###
    pioneer_x = pioneer_data [0]
    pioneer_y = pioneer_data [1]
    pioneer_orentation = pioneer_data[2]
    print(math.cos(pioneer_orentation + pioneer_angle))
    pioneer_point_rightfront_x  = pioneer_x + r * math.cos(pioneer_orentation + pioneer_angle)
    pioneer_point_rightfront_y  = pioneer_y + r * math.sin(pioneer_orentation + pioneer_angle)
    pioneer_point_leftfront_x = pioneer_x - r * math.cos(pioneer_angle - pioneer_orentation)
    pioneer_point_leftfront_y = pioneer_y + r * math.sin(pioneer_angle - pioneer_orentation)
    pioneer_point_rightback_x = pioneer_x + r * math.cos(-pioneer_angle + pioneer_orentation)
    pioneer_point_rightback_y = pioneer_y + r * math.sin(-pioneer_angle + pioneer_orentation)
    pioneer_point_leftback_x = pioneer_x - r * math.cos(-pioneer_angle - pioneer_orentation)
    pioneer_point_leftback_y = pioneer_y + r * math.sin(-pioneer_angle - pioneer_orentation)
    print(pioneer_point_leftfront_x,pioneer_point_rightfront_x)

    x1 = [[pioneer_point_rightfront_x,pioneer_point_leftfront_x,pioneer_point_leftback_x,pioneer_point_rightback_x,pioneer_point_rightfront_x]]
    y1= [[pioneer_point_rightfront_y,pioneer_point_leftfront_y,pioneer_point_leftback_y,pioneer_point_rightback_y,pioneer_point_rightfront_y]]

    x2 = [[pioneer_point_rightfront_x,pioneer_point_leftfront_x,pioneer_point_leftback_x,pioneer_point_rightback_x,pioneer_point_rightfront_x]]
    y2 = [[pioneer_point_rightfront_y,pioneer_point_leftfront_y,pioneer_point_leftback_y,pioneer_point_rightback_y,pioneer_point_rightfront_y]]
    z2 = [[pioneer_height,pioneer_height,pioneer_height,pioneer_height,pioneer_height]]

    x3 = [[pioneer_point_rightfront_x,pioneer_point_rightfront_x],[pioneer_point_leftfront_x,pioneer_point_leftfront_x],
          [pioneer_point_rightback_x,pioneer_point_rightback_x],[pioneer_point_leftback_x,pioneer_point_leftback_x]]
    y3 = [[pioneer_point_rightfront_y,pioneer_point_rightfront_y],[pioneer_point_leftfront_y,pioneer_point_leftfront_y],
          [pioneer_point_rightback_y,pioneer_point_rightback_y],[pioneer_point_leftback_y,pioneer_point_leftback_y]]
    z3 = [[0,pioneer_height],[0,pioneer_height],[0,pioneer_height],[0,pioneer_height]]
    z = [0,0]

    arm_bottom_x = pioneer_x
    arm_bottom_y = pioneer_y
    arm_bottom_z = pioneer_height

    arm_top_z = pioneer_height + arm_height

    arm_end_point_x = arm_endpoint[0]
    arm_end_point_y = arm_endpoint[1]
    arm_end_point_z = arm_endpoint[2]

    plot_x = [[arm_bottom_x ,arm_bottom_x],[arm_bottom_x,arm_end_point_x]]
    plot_y = [[arm_bottom_y, arm_bottom_y],[arm_bottom_y,arm_end_point_y]]
    plot_z = [[pioneer_height , arm_top_z],[arm_top_z,arm_end_point_z]]





    ax = fig.add_subplot(111,projection = '3d')
    ax.plot(x1[0], y1[0],color = 'b')
    ax.plot(x2[0],y2[0],z2[0],color = 'b')
    for i in range(len(x3)):
        ax.plot(x3[i],y3[i],z3[i],color = 'b')
    for i in range(len(plot_x)):
        ax.plot(plot_x[i],plot_y[i],plot_z[i])
    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    ax.set_zlim(0,2)
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_zlabel("z(m)")

    ax.scatter(pioneer_x,pioneer_y,0)

    #plt.show()
plt.ion()

pioneer_q = []
endpoint_q = []

def callback_pioneer(data):
    # print (data.point)
    global pioneer_q
    pioneer_q.clear()
    pioneer_q.append(data)

def callback_eff(data):

    global endpoint_q
    endpoint_q.clear()
    endpoint_q.append(data)

rospy.init_node('visulization_node',anonymous=True)
rospy.Subscriber('/pioneer_pose',PointStamped,callback=callback_pioneer)
rospy.Subscriber('/eff_o',PointStamped,callback = callback_eff)
rate = rospy.Rate(20)


draw_3D([0,0,0], [0,0,0])
plt.pause(0.01)

while not rospy.is_shutdown():

    ##check data and plot
    if(len(pioneer_q) ==1 and len(endpoint_q)==1):
        p_data = pioneer_q.pop()
        eff_data = endpoint_q.pop()
        p = PointStamped()
        
        plt.clf()
        draw_3D([p_data.point.x,p_data.point.y,p_data.point.z],[eff_data.point.x,eff_data.point.y,eff_data.point.z+0.30])
        plt.pause(0.01)
    pass
###

# for i in height:
#     plt.clf()
#     draw_3D([1-i*0.1,1-i*0.1,math.pi/3+i*0.1],[1,[1.5-i*0.1,1.2+i*0.1,i*0.1]])
#     plt.pause(0.01)
plt.ioff()
plt.show()




