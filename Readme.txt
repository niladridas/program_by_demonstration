All these codes are for my Master's Thesis at IIT Kanpur. My thesis is about PbD (Robot Programming by Demonstration). My test-bed is a Barrett Technology's 7 DoF WAM arm and 3 fingured WAM hand. I am using a single Microsoft Kinect for it's vision.
MODULE_HEISENBERG has two parts. All the codes except those in Inside_WAM are meant to run on a Personal Computer. A short description of these codes are provided below:

1. test_data_write_mod.cpp
This code contains a client socket which connects with the move_hand2.cpp in the Inside_WAM folder. The code is used for hand camera calibration.

2. best_circle.cpp
This code fits a circle in 3D, thus finding out the centre of the tool position from camera measurements. We need four (4) such measurements.

3. create_input.cpp
This code simply creates an augmented text file with co-ordinates of four points (tool centre) in robot frame and camera frame respectively.

4. gen_abs_orient.cpp
This code calculates the transformation parameters: R (Rotation), t (Translation), c (scale).
Our code has a way of checking if our calibration was right or not by seeing the value of "c" which in our case should always be "1" (one).

5. saving_private_cloud.cpp
This code uses Point Cloud Library to segment the objects lying on the table and cluster them. Then it would try to fit models (cylinder, shpere) on those clusters. It also calculates the desired position and orientation of the robot arm for grasping. (For now the z axis of grasping is hard coded into it).

6. Inverse_Kinematics.cpp
This code calculates the search region of PHI, which gives us all the solutions of Inverse Kinematics.

7. multiple_solutions.cpp 
This code gives us all the redundant Inverse Kinematics solution for a specific position and orientation along with user defined resolution.



