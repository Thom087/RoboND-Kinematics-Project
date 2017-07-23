## Project: Kinematics Pick & Place
---


**Steps to complete for this project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode] (set demo flag to true)(https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/FW_Kinematics.png
[image2]: ./misc_images/FW_Kinematics_1.png
[image3]: ./misc_images/Theta1.png
[image4]: ./misc_images/Theta2.png
[image5]: ./misc_images/Theta3.png
[image6]: ./misc_images/Theta4_6.png
[image7]: ./misc_images/P25.png
[image8]: ./misc_images/IK_ROS1.png
[image9]: ./misc_images/IK_ROS2.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
To run the forward_kinemtaics start roscore:
```sh
~$ roscore
```
In a new terminal launch the forward kinematics demo:
```sh
~$ roslaunch kuka_arm forward_kinematics.launch
```

![alt text][image2]

![alt text][image1]

DH-Parameter Table:

$i$ | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$
--- | --- | --- | --- | ---
T0_1 | 0 | 0 | 0.75 | $\theta_1$
T1_2 | -90° | 0.35 | 0 | $\theta_2$-90°
T2_3 | 0 | 1.25 | 0 | $\theta_3$
T3_4 | -90° | -0.054 | 1.5 | $\theta_4$
T4_5 | 90° | 0 | 0 | $\theta_5$
T5_6 | -90° | 0 | 0 | $\theta_6$
T6_G | 0 | 0 | 0.303 | 0

    # Modified DH params
    d01 = 0.75	# meters
    a12 = 0.35	# meters
    a23 = 1.25	# meters
    a34 = -0.054 # meters
    d34 = 1.5	# meters
    d67 = 0.303	# meters

    s = {alpha0: 0, 	a0: 0,	d1: d01, 
         alpha1: -pi/2, a1: a12,d2: 0,	q2:q2-pi/2,
         alpha2: 0, 	a2: a23,d3: 0,	
         alpha3: -pi/2, a3: a34,d4: d34,
         alpha4: pi/2, 	a4: 0,	d5: 0,
         alpha5: -pi/2, a5: 0,	d6: 0,
         alpha6: 0, 	a6: a23,d7: d67, q7:0}

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

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

##### Generalized homogeneous transform between base_link and gripper_link:
    T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

##### Generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose:
First I had to calculate the correction matrix:

    #correction matrix
    #rotate about -90 degrees about y-axis
    R_y = Matrix([[ cos(-pi/2),        0, sin(-pi/2)],
                  [          0,	       1,	   0],
                  [-sin(-pi/2),        0, cos(-pi/2)]])
    #rotate about 180 degrees about z-axis
    R_z = Matrix([[ cos(pi), 	-sin(pi),          0],
                  [ sin(pi),     cos(pi),	   0],
                  [       0, 	       0, 	   1]])
    
    R_corr = R_z*R_y

Then calculate the Roll / Pitch / Yaw matrixes:

    # Calculate joint angles using Geometric IK method
    # R_x
    R_roll = Matrix([[ 1,           0,          0],
                     [ 0,   cos(roll), -sin(roll)],
                     [ 0,   sin(roll),  cos(roll)]])
    # R_y
    R_pitch = Matrix([[ cos(pitch), 0,  sin(pitch)],
                      [       0,    1,           0],
                      [-sin(pitch), 0,  cos(pitch)]])
    # R_z
    R_yaw = Matrix([[ cos(yaw), -sin(yaw),       0],
                    [ sin(yaw),  cos(yaw),       0],
                    [ 0,              0,        1]])

    #Rrpy -> R0_G
    Rrpy = R_yaw*R_pitch*R_roll*R_corr

##### Calculate the wrist position
    # Calculate wrist positions
    px_wc = px - d67*Rrpy[0,2]
    py_wc = py - d67*Rrpy[1,2]
    pz_wc = pz - d67*Rrpy[2,2]

    p_wc  = Matrix([[px_wc], [py_wc], [pz_wc]])

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Theta 1
Its the projection of of p_wc to the xy-plane of world coordinate system.
![alt text][image3] 

##### Get P2_5
To calcuate $\theta_2$ and $\theta_3$ the distance from $O_2$ to $O_3$ needs to be known. Its derived as follows:
![alt text][image7] 

##### Theta 3
The trick here is to know the cosinus law and the magnitude of the vector P2_5  
![alt text][image5]

##### Theta 2
The trick here is again to know the cosinus law and the magnitude of the vector P2_5  
![alt text][image4]

##### Theta 4-6
To determine the angles the rotation matrixes of wrist to Gripper have to be taken into account:
The R3_6 was calculated with the homogeneous transformations where the angles $\theta_4$ to $\theta_6$ are not known:
    R3_6 = (T3_4*T4_5*T5_6)[0:3, 0:3]
$$
R^3_6 = \begin{pmatrix} -sin(\theta_4) \cdot sin(\theta_6) + cos(\theta_4) \cdot cos(\theta_5) \cdot cos(\theta_6) & -sin(\theta_4) \cdot cos(\theta_6) - sin(\theta_6) \cdot cos(\theta_4) \cdot cos(\theta_5) & -sin(\theta_5) \cdot cos(\theta_4) \\ sin(\theta_5) \cdot cos(\theta_6) & -sin(\theta_5) \cdot sin(\theta_6) & cos(\theta_5) \\
                       -sin(\theta_4) \cdot cos(\theta_5) \cdot cos(\theta_6) - sin(\theta_6) \cdot cos(\theta_4) & sin(\theta_4) \cdot sin(\theta_6) \cdot cos(\theta_5) - cos(\theta_4) \cdot cos(\theta_6) & sin(\theta_4) \cdot sin(\theta_5) \end{pmatrix} = \begin{pmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{pmatrix}
$$
Whereas the R3_6 calculated with the end effector position is like follows:
    R3_6 = R0_3.transpose() * Rrpy

With a some trigonometrical laws and math the thetas are obtained:
![alt text][image6] 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The solution is described on top, the programming was not very special no special technics needed. 

To run the project I extended the safe_spawner.sh files as mentioned on slack channel:
    sleep 5 &&
    x-terminal-emulator -e rosrun kuka_arm IK_server.py

The kuka robot can sucessfully pick up all pieces and drop them in the bin.

Sometimes the motion path generated was kind of weird. 
![alt text][image8]
It took very long to move from A to B. Overall the project was a lot of effort and sometimes not at all easy to implement, there are a lot of math and trigonometrical skills needed to sucessfully fullfill this task. There were some frustating moments.. what helped a lot was the IK_debug.py file which came for me a bit late.. therefore at the beginning it was kind of exhausting to test the code, the lecture material could be better.. 
