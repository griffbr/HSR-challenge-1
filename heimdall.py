#!/usr/bin/python
# -*- coding: utf-8 -*-

import cPickle as pickle; import IPython; import sys; import time; from copy import deepcopy; import time
import cv2; import numpy as np; import glob; import os; import math; import scipy
from skimage.measure import label
import matplotlib.pyplot as plt
import sys
from osvos import *
from data_subscriber import image_subscriber, state_subscriber, odometry_subscriber
from cyclops import *
import hsrb_interface;
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState, PointCloud2
from nav_msgs.msg import Odometry

## Functions. #########################################################################

def get_mask(img_sub, OSVOS):
	vs_img = deepcopy(img_sub._input_image)
	mask = OSVOS.segment_image(vs_img)
	mask, n_mask_pxls = largest_region_only(mask)
	return mask, n_mask_pxls
def find_mask_centroid(mask):
	centroid_idx = np.array(scipy.ndimage.measurements.center_of_mass(mask))
	return centroid_idx
def largest_region_only(init_mask):
	labels = label(init_mask)
	bin_count = np.bincount(labels.flat)
	if len(bin_count)>1:
		mask_bin = np.argmax(bin_count[1:]) + 1
		n_mask_pixels = bin_count[mask_bin]
		single_mask = labels == mask_bin
	else: single_mask = init_mask; n_mask_pixels = 0
	return single_mask, n_mask_pixels
def mask_to_PCA(single_mask):
	mask_idx = np.where(single_mask)
	X = np.array([mask_idx[0],mask_idx[1]]).T
	M = np.mean(X, axis=0)
	cov_matrix = np.cov((X - M).T)
	eig_values, eig_vectors = np.linalg.eig(cov_matrix)
	eig_vectors[[0,1],[1,0]] *= -1
	PCA_idx = np.argmin(eig_values)
	PCA_angle = math.atan(eig_vectors[PCA_idx,1]/eig_vectors[PCA_idx,0])
	return X, M, eig_vectors, eig_values, PCA_angle

def load_pose(whole_body, pickle_file, filter_list = []):
	pose = pickle.load(open(pickle_file, 'rb'))
	for i, pose_name in enumerate(filter_list):
		del pose[pose_name]
	whole_body.move_to_joint_positions(pose)
def filter_pose(pose):
	del pose['base_roll_joint']
	del pose['hand_l_spring_proximal_joint']
	del pose['hand_r_spring_proximal_joint']
	del pose['base_l_drive_wheel_joint']
	del pose['base_r_drive_wheel_joint']
	return pose
def move_joint_amount(whole_body, joint, amount):
	pose = whole_body.joint_positions
	pose[joint] += amount
	pose = filter_pose(pose)
	whole_body.move_to_joint_positions(pose)

def grab_target(whole_body, base, gripper, OSVOS, img_sub, name, mask_center_init):
	w_max_iv = 350; w_min_wr = 235
	s_des_iv = [240, 330]; s_des_ps3_x_grasp = [240, 220];
	s_des_ps3_tri = [240, 180]
	Je_pinv = np.array([[0, -0.001],[0.001, 0]])
	Je_pinv_tri = np.array([[0, -0.001],[0.0002, 0]])
	Je_pinv_lat = np.array([[0, 0],[0.001, 0]])
	object_grasped = False
	if mask_center_init[1] < w_max_iv:
		center_target_on_pose(whole_body, base, gripper, OSVOS, img_sub, './initial_view.pk', s_des_iv, Je_pinv)
		while not object_grasped:
			center_target_on_pose(whole_body, base, gripper, OSVOS, img_sub, './ps3_x_grasp.pk', s_des_ps3_x_grasp, Je_pinv*0.3)
			if mask_center_init[1] > w_min_wr:
				rotate_wrist(whole_body, OSVOS, img_sub)
			object_grasped = grab_and_check(whole_body, gripper, OSVOS, img_sub)
	else:
		center_target_on_pose(whole_body, base, gripper, OSVOS, img_sub, './initial_view.pk', s_des_iv, Je_pinv_lat)
		while not object_grasped:
			center_target_on_pose(whole_body, base, gripper, OSVOS, img_sub, './ps3_tri_tilt_low.pk', s_des_ps3_tri, Je_pinv_tri)
			object_grasped = grab_and_check(whole_body, gripper, OSVOS, img_sub)
def center_target_on_pose(whole_body, base, gripper, OSVOS, img_sub, pose_file, s_des, Je_pinv, pose_filter = [], error_max = 30):
	load_pose(whole_body, pose_file, pose_filter)
	error_sum = 100
	error_logic = abs(Je_pinv).sum(axis=0) > 0
	while error_sum > error_max:
		mask, n_mask_pxls = get_mask(img_sub, OSVOS)
		if n_mask_pxls < 200:
			move_joint_amount(whole_body, 'arm_lift_joint', 0.01)
			base.go_rel(-0.02,0,0)
			mask = []
		else:
			mask_center = find_mask_centroid(mask)
			error = s_des - mask_center
			delta_state = np.matmul(Je_pinv, error)
			base.go_rel(delta_state[0], delta_state[1], 0)
			error_sum = np.sum(abs(error*error_logic)) 
			error_max *= 1.05
def grab_and_check(whole_body, gripper, OSVOS, img_sub):
	gripper.apply_force(1.0)
	try:
		move_joint_amount(whole_body, 'arm_lift_joint', 0.3)
		mask, n_mask_pxls = get_mask(img_sub, OSVOS)
		if n_mask_pxls > 3000:
			object_grasped = True
		else: object_grasped = False
	except:
		gripper.command(1.0)
		object_grasped = False
	return object_grasped
def rotate_wrist(whole_body, OSVOS, img_sub):
	mask, n_mask_pxls = get_mask(img_sub, OSVOS)
	_, _, _, _, object_angle = mask_to_PCA(mask)
	wrist_angle = whole_body.joint_positions['wrist_roll_joint']
	new_wrist_angle = (wrist_angle - object_angle) % 3.1416
	if abs(new_wrist_angle) > 1.5708:
		new_wrist_angle = -new_wrist_angle % 1.5708
	whole_body.move_to_joint_positions({'wrist_roll_joint':new_wrist_angle})
	
def place_target(whole_body, base, gripper, img_sub, cab_pos, home_pose):
	placed = False
	go_to_amcl_pose(base, home_pose)
	go_to_amcl_pose(base, home_pose)
	if cab_pos == 'tl': 
		load_pose(whole_body, './tl_b.pk',['hand_motor_joint'])
		action_sequence = [[0,0.52,-0.7854], [0.22,0.22,0]]
		back_sequence = [[-0.22,-0.22,0]]
	elif cab_pos == 'tr':
		load_pose(whole_body, './tr.pk',['hand_motor_joint'])
		action_sequence = [[0,-0.58,0.7854], [0.30,-0.30,0]]
		back_sequence = [[-0.30,0.30,0]]
	elif cab_pos == 'tll':
		load_pose(whole_body, './tl_b.pk',['hand_motor_joint'])
		action_sequence = [[0,0.77,-0.7854],[0.22,0.22,0]]
		back_sequence = [[-0.22,-0.22,0]]
	while not placed:
		placed = dead_reckon_and_back(base, gripper, action_sequence, back_sequence)
		go_to_amcl_pose(base, home_pose)
def dead_reckon_and_back(base, gripper, sequence, back_sequence):
	try:	
		for i, action in enumerate(sequence):
			print (action)
			base.go_rel(action[0],action[1],action[2])
		gripper.command(1.0)
		placed = True
	except:
		placed = False
	for i, action in enumerate(back_sequence):
		print (action)
		base.go_rel(action[0],action[1],action[2])
	return placed

## Main. ##############################################################################
def main(OSVOS, grasp_img_sub, robot_state_sub, base_odometry_sub, base, whole_body):
	gripper = robot.get('gripper')
	names = ['r','y','b']
	r_dir = './r.ckpt-10001'
	y_dir = './y.ckpt-10001'
	b_dir = './b.ckpt-10001'
	dirs = [r_dir, y_dir, b_dir]
	cab_pos = ['tll','tr','tl']
	n_targets = len(names)

	print ('\n\nRobot moves next, make sure you are ready!\n\n')
	IPython.embed()
	init_amcl_fast(base, 0.5)
	home_amcl_pose = get_amcl_pose()

	loop_count = 0
	while True:
		load_pose(whole_body, './initial_view.pk')
		grasp_img = deepcopy(grasp_img_sub._input_image)
		masks = [[] for i in range(n_targets)]
		n_mask_pxls = np.zeros(n_targets)
		mask_centers = np.zeros(shape=(n_targets,2))
		for i, name in enumerate(names):
			OSVOS.change_model(dirs[i])
			masks[i] = OSVOS.segment_image(grasp_img)
			masks[i], n_mask_pxls[i] = largest_region_only(masks[i])
			mask_centers[i] = find_mask_centroid(masks[i])
		lat_place = [a or b for a,b in zip(mask_centers[:,0]<190,  mask_centers[:,0]>430)]
		target_order = np.argsort(mask_centers[:,1] + 2*mask_centers[:,1]*lat_place)
		for i, idx in enumerate(target_order):
			if n_mask_pxls[idx]>200:
				OSVOS.change_model(dirs[idx])
				grab_target(whole_body, base, gripper, OSVOS, grasp_img_sub, names[idx], mask_centers[idx])
				place_target(whole_body, base, gripper, grasp_img_sub, cab_pos[idx], home_amcl_pose)
		print ('Finished grabbing all blocks!')
		loop_count += 1
		if loop_count > 10:
			base.go_rel(0,0.05,0)
	IPython.embed()

if __name__ == '__main__':
	with hsrb_interface.Robot() as robot:
		base = robot.try_get('omni_base')
		whole_body = robot.get('whole_body')
		grasp_img_sub = image_subscriber('/hsrb/hand_camera/image_raw', True)
		robot_state_sub = state_subscriber('/hsrb/joint_states')
		base_odometry_sub = odometry_subscriber('/hsrb/odom')
		OSVOS = osvos_seg('./r.ckpt-10001')	
		main(OSVOS, grasp_img_sub, robot_state_sub, base_odometry_sub, base, whole_body)
