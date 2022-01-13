import rospy
from geometry_msgs.msg import Twist
from queue import Queue

waypoints = 3


node = rospy.init_node('pioneer_controller',anonymous = True)
pub_twist = rospy.Publisher('/RosAria/cmd_vel',Twist,queue_size=1)




twist = Twist()

twist.linear.x = 0.01
twist.linear.y = 0
twist.linear.z = 0

twist.angular.z = 0.1
# twist.angular = [0,0,0]

rate = rospy.Rate(2)





while not rospy.is_shutdown():
    # twist.header.stamp = rospy.Time().now()
    pub_twist.publish(twist)
    print(twist)
    rate.sleep()



    