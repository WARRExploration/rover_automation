#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
import matplotlib.pyplot as plt



def check_model(msg):
    i = msg.name.index("rover_model")
    global model
    model = msg.pose[i]


def check_odom(msg):
    global odom
    odom = msg.pose.pose

def compare_poses():
    global model
    global odom
    
    x = float(odom.position.x) - float(model.position.x)
    y = float(odom.position.y) - float(model.position.y)
    z = float(odom.position.z) - float(model.position.z)

    return np.linalg.norm([x,y,z])

if __name__ == '__main__':
    global model
    global odom

    model = Pose()
    odom = Pose()
  
    rospy.init_node("odometry_comparison")
    subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, check_model)
    subscriber = rospy.Subscriber("/odometry/filtered", Odometry, check_odom)

    r = rospy.Rate(10)

    l = []
    while not rospy.is_shutdown():    
        publisher = rospy.Publisher("odometry_distance", Float64, queue_size=1)
        d = compare_poses()
        publisher.publish(float(d))
        r.sleep()
        l.append(d)
        #if len(l)>1000:
        #    plt.plot(l)
        #    plt.show()
        #print (len(l))

    rospy.spin()
