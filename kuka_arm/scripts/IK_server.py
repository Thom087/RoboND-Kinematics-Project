#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

            # Joint angle symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	    # Conversion Factors
	    rtd = 180./mpmath.pi # radians to degrees


            # Modified DH params
            d01 = 0.75	# meters
            a12 = 0.35	# meters
            a23 = 1.25	# meters
            a34 = -0.054 # meters
            d34 = 1.5	# meters
            d67 = 0.303	# meters


            # Define Modified DH Transformation matrix
            s = {alpha0: 0, 	a0: 0,	d1: d01, 
                 alpha1: -np.pi/2, a1: a12,d2: 0,	q2:q2-np.pi/2,
                 alpha2: 0, 	a2: a23,d3: 0,	
                 alpha3: -np.pi/2, a3: a34,d4: d34,
                 alpha4: np.pi/2, 	a4: 0,	d5: 0,
                 alpha5: -np.pi/2, a5: 0,	d6: 0,
                 alpha6: 0, 	a6: a23,d7: d67, q7:0}


            # Create individual transformation matrices
            T0_1 = Matrix([[		cos(q1),	   -sin(q1),	       0,	      a0],
                           [sin(q1)*cos(alpha0),cos(q1)*cos(alpha0),-sin(alpha0),-sin(alpha0)*d1],
                           [sin(q1)*sin(alpha0),cos(q1)*sin(alpha0), cos(alpha0), cos(alpha0)*d1],
                           [		      0,		  0,	       0,	       1]])
            T0_1 = T0_1.subs(s)

            T1_2 = Matrix([[		cos(q2),	   -sin(q2),	       0,	      a1],
                           [sin(q2)*cos(alpha1),cos(q2)*cos(alpha1),-sin(alpha1),-sin(alpha1)*d2],
                           [sin(q2)*sin(alpha1),cos(q2)*sin(alpha1), cos(alpha1), cos(alpha1)*d2],
                           [		      0,		  0,	       0,	       1]])
            T1_2 = T1_2.subs(s)

            T2_3 = Matrix([[		cos(q3),	   -sin(q3),	       0,	      a2],
                           [sin(q3)*cos(alpha2),cos(q3)*cos(alpha2),-sin(alpha2),-sin(alpha2)*d3],
                           [sin(q3)*sin(alpha2),cos(q3)*sin(alpha2), cos(alpha2), cos(alpha2)*d3],
                           [		      0,		  0,	       0,	       1]])
            T2_3 = T2_3.subs(s)


            T3_4 = Matrix([[		cos(q4),	   -sin(q4),	       0,	      a3],
                           [sin(q4)*cos(alpha3),cos(q4)*cos(alpha3),-sin(alpha3),-sin(alpha3)*d4],
                           [sin(q4)*sin(alpha3),cos(q4)*sin(alpha3), cos(alpha3), cos(alpha3)*d4],
                           [		      0,		  0,	       0,	       1]])
            T3_4 = T3_4.subs(s)


            T4_5 = Matrix([[		cos(q5),	   -sin(q5),	       0,	      a4],
                           [sin(q5)*cos(alpha4),cos(q5)*cos(alpha4),-sin(alpha4),-sin(alpha4)*d5],
                           [sin(q5)*sin(alpha4),cos(q5)*sin(alpha4), cos(alpha4), cos(alpha4)*d5],
                           [		      0,		  0,	       0,	       1]])
            T4_5 = T4_5.subs(s)


            T5_6 = Matrix([[		cos(q6),	   -sin(q6),	       0,	      a5],
                           [sin(q6)*cos(alpha5),cos(q6)*cos(alpha5),-sin(alpha5),-sin(alpha5)*d6],
                           [sin(q6)*sin(alpha5),cos(q6)*sin(alpha5), cos(alpha5), cos(alpha5)*d6],
                           [		      0,		  0,	       0,	       1]])
            T5_6 = T5_6.subs(s)


            T6_G = Matrix([[		cos(q7),	   -sin(q7),	       0,	      a6],
                           [sin(q7)*cos(alpha6),cos(q7)*cos(alpha6),-sin(alpha6),-sin(alpha6)*d7],
                           [sin(q7)*sin(alpha6),cos(q7)*sin(alpha6), cos(alpha6), cos(alpha6)*d7],
                           [		      0,		  0,	       0,	       1]])

            #T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

	    T0_3 = T0_1 * T1_2 * T2_3

	    # Get rotation matrix R0_3
            R0_3 = T0_3[0:3, 0:3]

            #correction matrix
            #rotate about -90 degrees about y-axis
            R_y = Matrix([[ cos(-np.pi/2),        0, sin(-np.pi/2)],
                          [          0,	       1,	   0],
                          [-sin(-np.pi/2),        0, cos(-np.pi/2)]])
            #rotate about 180 degrees about z-axis
            R_z = Matrix([[ cos(np.pi), 	-sin(np.pi),          0],
                          [ sin(np.pi),     cos(np.pi),	   0],
                          [       0, 	       0, 	   1]])
            R_corr = R_z*R_y

	    #ta = Matrix([[0], [0], [0]])
	    #th = Matrix([[0, 0, 0, 1]])

	    #T_corr = (R_corr.row_join(ta)).col_join(th)

            #T_total = T0_G * T_corr


            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            print("Requested poses")
            print("x", px, "y", py, "z", pz)
            print("roll", roll, "pitch", pitch, "yaw", yaw)

            # Calculate joint angles using Geometric IK method

            R_roll = Matrix([[ 1,              0,         0],
                             [ 0,        cos(roll), -sin(roll)],
                             [ 0,        sin(roll),  cos(roll)]])

            R_pitch = Matrix([[ cos(pitch), 0,  sin(pitch)],
                              [       0,        1,           0],
                              [-sin(pitch),     0,  cos(pitch)]])

            R_yaw = Matrix([[ cos(yaw), -sin(yaw),       0],
                            [ sin(yaw),  cos(yaw),         0],
                            [ 0,              0,           1]])

            #Rrpy -> R0_6
	    Rrpy = R_roll*R_pitch*R_yaw

            # Calculate wrist positions
            px_wc = px - d67*Rrpy[0,0]
            py_wc = py - d67*Rrpy[1,0]
            pz_wc = pz - d67*Rrpy[2,0]
            p_wc  = Matrix([[px_wc], [py_wc], [pz_wc]])
	    print("p_wc: ", p_wc)   

            # Calculate joint theta 1
            theta1 = mpmath.atan2(py_wc,px_wc)
	    theta1_big   = 2*pi-theta1
	    #print("theta1_big: ", theta1_big, ", theta1_big_deg: ", theta1_big*rtd)

            # Calculate joint theta 3
            p_02 = Matrix([[a12*cos(theta1)], [a12*sin(theta1)], [d01]])
            p_25 = p_wc - p_02
            #								1.25     -0.054   1.5
            D = (a23**2 + a34**2 + d34**2 -p_25[0]**2 - p_25[2]**2)/(2*a23*sqrt(a34**2 + d34**2))
	    #atan2(GK->y, AK->x)
	    gamma = atan2(sqrt(1-D**2),D)
	    delta = atan2(0.054, 0.96)-atan2(0.054, 1.5)
	    theta3 = -np.pi/2 + gamma - delta
	    theta3 = -theta3 	#because z-axis is in the other direction..
	    theta3_big = np.pi - theta3 	#check which one is wright
	    #print("theta3_big: ", theta3_big, ", theta3_big_deg: ", theta3_big*rtd)

            # Calculate joint theta 2
	    delta2=mpmath.atan2(p_25[2],p_25[0])
	    E = (a23**2 +p_25[0]**2 + p_25[2]**2 - a34**2 - d34**2)/(2*a23*sqrt(p_25[0]**2 + p_25[2]**2))  
	    delta3 = mpmath.atan2(sqrt(1-E**2),E)
	    theta2 = mpmath.pi/2 - (delta2 + delta3) 
	    #print("theta2: ", theta2*rtd)

            # Calculate numerical R0_3_num
            R0_3_num = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

            # Calculate R_3_6
            # R3_6 = (R0_3_num**(-1)) * Rrpy 
            #R3_6 = R0_3_num.transpose() * Rrpy * R_corr
	    R3_6 = R0_3_num.transpose() * Rrpy

            # Get Euler angles theta 4-6
            # Rotation about x -> theta 4
            theta4 = atan2(R3_6[2,1],R3_6[2,2])+pi/2

            # Rotation about z -> theta 5
            theta5 = atan2(R3_6[1,0],R3_6[0,0])
	    #theta5 = -theta5 #coordinate system in opposite direction

            # Rotation about y -> theta 6
            theta6 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]**2+R3_6[1,0]**2))

	    print("theta1: ", theta1*rtd)
	    print("theta2: ", theta2*rtd)
	    print("theta3: ", theta3*rtd)
	    print("theta4: ", theta4*rtd)
	    print("theta5: ", theta5*rtd)
	    print("theta6: ", theta6*rtd)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            #T0_G_num = T0_G.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
