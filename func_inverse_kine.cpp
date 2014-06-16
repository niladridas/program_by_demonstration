/*
 * Inverse_Kinematics.cpp
 *
 *  Created on: 17-Apr-2014
 *      Author: Niladri Das & Ankit Pensia
 */
/* This function gives us the range of phi to search for the inverse kinematics solution for a given
 * end effector position and orientation. The position values are given as simple {x,y,z} and the orientation
 * is given as k_x,k_y and k_theta

 This function takes 2 input paramters and then updates phi_search array with the calculated values.

 This function was made from the programme Inverse_Kinematics.cpp
 */


#include "func_inverse_kine.h"



void func_inverse_kine(float *output, float *phi_search)
{


	//cout << "This program can be run in two steps.\n In the first step the program calculates the search space of \n the parameter phi depending upon the constraint of theta_2, theta_4 and theta_6." << endl;
	//cout << "The second part of the program calculates a look up table containing all the values of phi \n which satisfies the other constraints i.e. theta_1, theta_3, theta_5 and theta_7 at any desired resolution." << endl;
	// Taking the input from the text file end_effector.txt
// Format of the text file is fairly simple
// X Y Z TR_z TR_x
//

/*
 *  READING FROM THE TEXT FILE
 */
//	 ifstream myReadFile;
//	 myReadFile.open("/home/niladri-64/module_heisenberg/data/end_effector_cartesian.txt");
//	 float output[9];
//	 for(int i =0 ; i<9; i++)
//	 {
//		 myReadFile >> output[i];
//	 }

//	myReadFile.close();

	/*
	 * SETTING UP IMPORTANT VARIABLES
	 */
	// Desired end tool position
	Vector3f DT_pos;
	DT_pos(0)= output[0];
	DT_pos(1)= output[1];
	DT_pos(2)= output[2];

	// Desired tool Z axis
	Vector3f TR_z;
	TR_z(0) = output[3];
	TR_z(1) = output[4];
	TR_z(2) = output[5];

	// Desired x axis should also be provided
	Vector3f TR_x;
	Vector4f TR_x_mod;
	TR_x(0) = output[6];
	TR_x(1) = output[7];
	TR_x(2) = output[8];

	TR_x_mod(0) = output[6];
	TR_x_mod(1) = output[7];
	TR_x_mod(2) = output[8];
	TR_x_mod(3) = 1;


	// Desired wrist position
	Vector3f DW_pos, DW_pos_unit;
	DW_pos = DT_pos - 0.06*TR_z;
	DW_pos_unit = DW_pos.normalized();

	// Distance between the wrist and the base
	float d = DW_pos.norm();

	Vector3f Z_axis;
	Z_axis << 0, 0, 1;

	Vector3f temp_cross_product;
	temp_cross_product = DW_pos_unit.cross(Z_axis);

	Vector3f temp_cross_product_unit;
	temp_cross_product_unit = temp_cross_product.normalized();

	float K_x  = temp_cross_product_unit(0);
	float K_y  = temp_cross_product_unit(1);
	float K_z  = temp_cross_product_unit(2);

	float C_theta = DW_pos_unit.dot(Z_axis);
	float V_theta = 1 - C_theta;
	float angle =  acos(C_theta);
	float S_theta = -sin(angle);



	Matrix3f R;
	R <<              K_x*K_x*V_theta + C_theta,      K_x*K_y*V_theta + K_z*S_theta,  K_x*K_z*V_theta - K_y*S_theta,
			          K_x*K_y*V_theta - K_z*S_theta,  K_y*K_y*V_theta + C_theta,      K_y*K_z*V_theta + K_x*S_theta,
			          K_x*K_z*V_theta + K_y*S_theta,  K_y*K_z*V_theta - K_x*S_theta,  K_z*K_z*V_theta + C_theta;

	// Calculating the parameters of the 3 circles
	// First calculating them in normalized form

	// Lower and Upper Link Lenghts
	float L1 = sqrt(0.55*0.55 + 0.045*0.045);
	float L2 = sqrt(0.3*0.3 + 0.045*0.045);

	// Upper and Lower Link Angles
	if( (d*d + L2*L2 - L1*L1)/(2*d*L2) > 1 || (d*d + L2*L2 - L1*L1)/(2*d*L2) < -1)
	{
		//cout << "No solutions are possible" << endl;
	}

	float alpha_2 = acos((d*d + L2*L2 - L1*L1)/(2*d*L2));
	float alpha_1 = asin((L2*sin(alpha_2))/L1);

	// Middle Circle position and radius
	float d_C = L1*cos(alpha_1);
	float R_C = L1*sin(alpha_1);

	// Lower and Upper Circle positiona and radius

	// For out-elbow position
	const double pi = boost::math::constants::pi<double>();

	float theta_U_out = pi - (pi/2 - alpha_2) - pi/2 + atan(0.045/0.3);
	float theta_L_out = pi - (pi/2 - alpha_1) - pi/2 + atan(0.045/0.55);

	// Upper Circle position and radius
	float d_UJ_out = d_C + 0.045*sin(theta_U_out);
	float R_UJ_out = R_C + 0.045*cos(theta_U_out);

	// Lower Circle position and radius
	float d_LJ_out = d_C - 0.045*sin(theta_L_out);
	float R_LJ_out = R_C + 0.045*cos(theta_L_out);

	////////////////////
	//
	// MOST IMPORTANT
	//
	////////////////////

	Vector3f sample_phi;
	sample_phi << pi/2, 0, pi/4;
	Vector3f temp_1;
	Vector3f temp_2;
	float theta_1_out, theta_2_out, theta_4_out, theta_3_out, theta_5_out, theta_6_out, theta_7_out;




for(int k = 0; k < 3; k++)
{
	// All the above calculations were done in normalized form
	float phi = sample_phi(k); // This is the parameter which determines the pose

	// when phi varies from 0 to 360 degrees the points on the circle are traced out
	Matrix3f C_norm_out;
	C_norm_out <<  R_C*cos(phi),      R_C*sin(phi),      d_C,
	               R_UJ_out*cos(phi), R_UJ_out*sin(phi), d_UJ_out,
	               R_LJ_out*cos(phi), R_LJ_out*sin(phi), d_LJ_out;

	Matrix3f C_rot_out;
	C_rot_out = C_norm_out*R;

	// Finding anlge theta_1 and theta_2 as a function of this parameter phi
	// For out elbow pose
	theta_1_out = atan2(C_rot_out(2,1),C_rot_out(2,0));
	theta_2_out = acos(C_rot_out(2,2)/0.55);

	// Finding angles theta_4
	// For out elbow pose
	theta_4_out = theta_U_out + theta_L_out;

	// Finding angle theta_3 which is the twist angle
	// Vector from lower link joint to A
	// For out position

	Vector3f temp_1_C_rot_out;
	Vector3f temp_2_C_rot_out;

	temp_1_C_rot_out(0) = C_rot_out(0,0);
	temp_1_C_rot_out(1) = C_rot_out(0,1);
	temp_1_C_rot_out(2) = C_rot_out(0,2);

	temp_2_C_rot_out(0) = C_rot_out(2,0);
	temp_2_C_rot_out(1) = C_rot_out(2,1);
	temp_2_C_rot_out(2) = C_rot_out(2,2);

	Vector3f V_out;
	V_out = (temp_1_C_rot_out - temp_2_C_rot_out)/0.045;

	Vector2f V_out_p;
	Vector2f temp_V_out_p;

	temp_V_out_p(0) = V_out(0);
	temp_V_out_p(1) = V_out(1);

	V_out_p = temp_V_out_p.normalized();

	theta_3_out = - theta_1_out + atan2(V_out_p(1),V_out_p(0));
	// Put a check
	if(theta_3_out < -pi)
	    {
		theta_3_out = 2*pi +theta_3_out;
	    }
	if(theta_3_out > pi)
	    {
		theta_3_out = -2*pi +theta_3_out;
	    }


	// Link Parameters as input:
	VectorXf A_link_vector(7);
	A_link_vector(0) = 0;
	A_link_vector(1) = 0;
	A_link_vector(2) = 0.045;
	A_link_vector(3) = - 0.045;
	A_link_vector(4) = 0;
	A_link_vector(5) = 0;
	A_link_vector(6) = 0;

	VectorXf Alpha_link_vector(7);
	Alpha_link_vector(0) = -pi/2 ;
	Alpha_link_vector(1) = pi/2 ;
	Alpha_link_vector(2) = -pi/2 ;
	Alpha_link_vector(3) = pi/2 ;
	Alpha_link_vector(4) = -pi/2 ;
	Alpha_link_vector(5) = pi/2;
	Alpha_link_vector(6) = 0;

	VectorXf D_link_vector(7);
	D_link_vector(0) = 0;
	D_link_vector(1) = 0 ;
	D_link_vector(2) = 0.55 ;
	D_link_vector(3) = 0 ;
	D_link_vector(4) = 0.3 ;
	D_link_vector(5) = 0;
	D_link_vector(6) = 0.06;


	Vector4f theta_link_vector_out;
	theta_link_vector_out << theta_1_out, theta_2_out, theta_3_out, theta_4_out;

// CORRECT UPTO THIS

	// For elbow out pose
	Matrix4f temp_frame_trans_out;
	temp_frame_trans_out = Matrix4f::Identity();

	for( int i = 0; i < 4; i++)
	{
	    temp_frame_trans_out = temp_frame_trans_out*frame_transformation(A_link_vector(i),Alpha_link_vector(i),D_link_vector(i),theta_link_vector_out(i));
	}

	Vector4f DT_pos_mod;
	DT_pos_mod << DT_pos(0), DT_pos(1), DT_pos(2), 1;

	Vector4f DW_pos_mod;
	DW_pos_mod << DW_pos(0), DW_pos(1), DW_pos(2), 1;

	Vector4f tmp_wrist_pos_out;
	Vector4f temp_tmp_wrist_pos_out;

	temp_tmp_wrist_pos_out << 0, 0, 0.3, 1;

	tmp_wrist_pos_out = temp_frame_trans_out*temp_tmp_wrist_pos_out;

	Vector3f wrist_pos_out;
	wrist_pos_out << tmp_wrist_pos_out(0) , tmp_wrist_pos_out(1), tmp_wrist_pos_out(2);

	Vector4f pos_tool_local_out;
	pos_tool_local_out = (temp_frame_trans_out.transpose())*(DT_pos_mod - DW_pos_mod);

	float R_out = sqrt(pos_tool_local_out(0)*pos_tool_local_out(0) + pos_tool_local_out(1)*pos_tool_local_out(1) + pos_tool_local_out(2)*pos_tool_local_out(2));

	float TH_out = atan2(pos_tool_local_out(1),pos_tool_local_out(0));

	float PHI_out = acos(pos_tool_local_out(2)/R_out);

	theta_5_out = TH_out;
	theta_6_out = PHI_out;

	// Now we just need to adjust the theta_7 joint to finalize the orientation

	Matrix4f trans_frame_5_out;
	Matrix4f trans_frame_6_out;

	trans_frame_5_out = temp_frame_trans_out*frame_transformation(A_link_vector(4),Alpha_link_vector(4),D_link_vector(4),theta_5_out);
	trans_frame_6_out = trans_frame_5_out * frame_transformation(A_link_vector(5),Alpha_link_vector(5),D_link_vector(5),theta_6_out);

	Vector4f pos_tool_local_out_frame6;
	pos_tool_local_out_frame6 = (trans_frame_6_out.transpose())*(TR_x_mod);

	theta_7_out = atan2(pos_tool_local_out_frame6(1),pos_tool_local_out_frame6(0));

	// Put a check
	if(theta_7_out < -pi)
	{
	    theta_7_out = 2*pi +theta_7_out;
	}

	if(theta_7_out > pi)
	{
	    theta_7_out = -2*pi +theta_7_out;
	}

	Matrix4f trans_frame_7_out;

	trans_frame_7_out = trans_frame_6_out * frame_transformation(A_link_vector(6), Alpha_link_vector(6), D_link_vector(6), theta_7_out);

//	//cout << "The calculated end-effector position X, Y, Z: " << " " << trans_frame_7_out(0,3) << " " << trans_frame_7_out(1,3) << " " << trans_frame_7_out(2,3) << endl;
//	//cout << "The given end-effector position X, Y, Z: " << " " << DT_pos << endl;



	//////////////////////////////////
	//
	// IMPOSING JOINT ANGLE CONSTRAINTS
	//
	/////////////////////////////////

	// theta_4_out >= 0 i.e the elbow in position is neglected.

	// using the constraint on theta_2
	temp_1(k) = (C_rot_out(2,2)/0.55) - cos(2);
	// Hence x >= 0
	// Simplifying the above equation by keeping phi variable we get a equation of the form
	// A_1*sin(phi) + B_1*cos(phi) + C_1

	// using the constraint on theta_6
	Vector3f temp_temp_C_rot_out;
	temp_temp_C_rot_out << C_rot_out(1,0), C_rot_out(1,1), C_rot_out(1,2);
	temp_2(k) = ((temp_temp_C_rot_out -  DW_pos)/0.3).dot(TR_z);
	// Hence x >= 0
	// Simplifying the above equation by keeping phi variable we get a equation of the form
	// A_2*sin(phi) + B_2*cos(phi) + C_2

}


	float x1 = temp_1(0) + temp_1(1);
	float x2 = temp_1(2);
	float C_1 = (x1 - sqrt(2)*x2)/(2 - sqrt(2));
	float A_1 = x1 - C_1;
	float B_1 = x2 - C_1;

	float x3 = temp_2(0) + temp_2(1);
	float x4 = temp_2(2);
	float C_2 = (x3 - sqrt(2)*x4)/(2 - sqrt(2));
	float A_2 = x3 - C_2;
	float B_2 = x4 - C_2;

	//cout << "A_1 " << A_1 << " " << "B_1 " << B_1 << " " << "C_1 "  << C_1 <<  endl;


	float eta_1 = atan(B_1/A_1);
	float phi1_UB = pi - asin(-1/sqrt((A_1/C_1)*(A_1/C_1) + (B_1/C_1)*(B_1/C_1))) - eta_1;

	float phi1_LB = eta_1 -  asin(-1/sqrt((A_1/C_1)*(A_1/C_1) + (B_1/C_1)*(B_1/C_1)));

	float eta_2 = atan(B_2/A_2);
	float phi2_UB = pi - asin(-1/sqrt((A_2/C_2)*(A_2/C_2) + (B_2/C_2)*(B_2/C_2))) - eta_2;

	float phi2_LB = eta_2 -  asin(-1/sqrt((A_2/C_2)*(A_2/C_2) + (B_2/C_2)*(B_2/C_2)));

	////cout << "phi1_UB " << phi1_UB*180/pi << " " << "phi1_LB " << phi1_LB*180/pi << " " << "phi2_UB "  << phi2_UB*180/pi << "phi2_LB "  << phi2_LB*180/pi <<  endl;



	if(isnan(phi1_LB))
				phi1_LB = phi2_LB;
	if(isnan(phi2_LB))
				phi2_LB = phi1_LB;
	if(isnan(phi1_UB))
				phi1_UB = phi2_UB;
	if(isnan(phi2_UB))
				phi2_UB = phi1_UB;


	// Hence the search zone for phi is
	float FLB;
	float FUB;

	if(phi1_LB >= phi2_LB)
	{
		FLB = phi1_LB;
	}
	else
		FLB = phi2_LB;

	if(phi1_UB >= phi2_UB)
	{
		FUB = phi2_UB;
	}
	else
		FUB = phi1_UB;



	//cout << "The search space for phi is : " << FLB*180/pi << " to " << FUB*180/pi << endl;

	double phi_max = FUB*180/pi ;
	double phi_min = FLB*180/pi;

//	if(phi_max == NAN)
//	{
//		phi_max =
//	}
//	  std::ofstream file_cartesian("/home/niladri-64/module_heisenberg/data/phi_search.txt");
//	  if (file_cartesian.is_open())
//	    {
//	      file_cartesian << phi_max << " " << phi_min;   ;
//	    }
//
//	  file_cartesian.close();

phi_search[0]= phi_max;
phi_search[1] = phi_min;

//	//cout << "theta_2_out " << theta_2_out*180/pi << " " << "theta_4_out " << theta_4_out*180/pi << " " << "theta_6_out "  << theta_6_out*180/pi << endl;

//	//cout << "exmaple " << int(10.3) << endl;

cout << "Returned from func inverse" << endl ;
	//return 0;
}


