import rospy
from  std_msgs.msg import Float32MultiArray 
rospy.init_node("debug_node",anonymous=True)

from matplotlib import pyplot as plt 

from queue import Queue
import numpy as np

point_q = Queue(maxsize = 1)





def callback_waypoints(data):
    # print(data)

    # plt.show()
    # if(point_q.full()):
    # point_q.get()
    # point_q.put(data)

    point_q.put(data)




rate = rospy.Rate(1)

# plt.show()


rospy.Subscriber("/debug_waypoints",Float32MultiArray,callback = callback_waypoints)

fig = plt.figure(figsize=(10,10))
plt.ion()

while not rospy.is_shutdown():
    plt.clf()
    if(point_q.full()):
        
        data = point_q.get()
        print(data.data)
        plt.xlim((-2,2))
        plt.ylim((-2,2))
        plt.xticks(np.linspace(-4,4,21))

        plt.yticks(np.linspace(-4,4,21))
        plt.scatter(0,0,color = 'black')
        plt.scatter(data.data[0],data.data[1],color='r')  #waypoint
        plt.scatter(data.data[2],data.data[3],color ='g') #Person 
        plt.scatter(data.data[4],data.data[5],color ='b') #Robot
        plt.grid()
        plt.pause(0.05)
    rate.sleep()
plt.ioff()
plt.show()
    