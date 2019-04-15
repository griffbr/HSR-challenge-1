import rospy
import geometry_msgs.msg
import hsrb_interface
from math import pi
import tf_conversions
import numpy as np
from std_srvs.srv import Empty, EmptyRequest

def init_amcl(base):
	base.go_rel(0, 0, -pi / 4)
	base.go_rel(0, 0, pi / 2)
	base.go_rel(0, 0, -pi / 4)
def init_amcl_fast(base, frac):
	base.go_rel(0, 0, frac*pi)
	base.go_rel(0, 0, -frac*pi)
def get_amcl_pose():
	stop_client = rospy.ServiceProxy('/request_nomotion_update', Empty)
	stop_client.call(EmptyRequest())
	msg = rospy.wait_for_message("/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped)
	pose = msg.pose.pose
	x = pose.position.x
	y = pose.position.y
	quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
	theta = tf_conversions.transformations.euler_from_quaternion(quaternion)
	return (x,y,theta[2])
def get_desired_abs_pose(base, start_loc, goal_loc):
	rel = np.subtract(goal_loc, start_loc)
	absolute = np.add(tuple(base.pose), tuple(rel))
	return(tuple(absolute))
def go_to_amcl_pose(base, end_amcl_pose):
	current_amcl_pose = get_amcl_pose()
	des_abs = get_desired_abs_pose(base, current_amcl_pose, end_amcl_pose)
	base.go_abs(des_abs[0], des_abs[1], des_abs[2])