#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Header



def generate_fake_localization(msg):
	global model_odom
	i = msg.name.index("rover_model")
	model_odom.pose.pose = msg.pose[i]
	model_odom.pose.covariance = [0] * 36
#	model_odom.twist = msg.twist[i]

def fake_loc():
	global model_odom
	model_odom = Odometry()
	model_odom.header = Header()
	model_odom.header.frame_id = "fake_loc"
	model_odom.child_frame_id = "base_link"	

	rospy.init_node("fake_localization_generation")
	
	subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, generate_fake_localization)

	r = rospy.Rate(1)
	pub = rospy.Publisher('base_pose_ground_truth', Odometry, queue_size=1)
	while not rospy.is_shutdown():    
		pub.publish(model_odom)
		r.sleep()

	rospy.spin()

if __name__ == '__main__':
	try:
		fake_loc()
	except rospy.ROSInterruptException:
		pass
