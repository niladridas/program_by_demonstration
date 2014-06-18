/*
 * hand_control_complete.cpp
 *
 *  Created on: May 27, 2014
 *      Author: niladri-64
 */

/* ex03_simple_move.cpp
 *
 * This example expands on the previous one by allowing users to enter target
 * positions that the WAM will move to. Starting at whatever location it happens
 * to be in, the WAM will accelerate smoothly, move at a constant velocity, and
 * then come to a stop at the target location. Once there, it will hold position
 * until the next command.
 *
 * Target positions are specified using "barrett::units". These types are used
 * by libbarrett to distinguish between different kinds of data. For example, a
 * vector containing joint positions is different from a vector containing joint
 * torques because they have different units. barrett::units provide an easy
 * mechanism to increase code clarity by labeling one vector as containing joint
 * positions (barrett::units::JointPositions<DOF>::type) and another vector as
 * containing joint torques (barrett::units::JointTorques<DOF>::type).
 *
 * All of the built-in barrett::units types are actually just specializations of
 * barrett::math::Matrix. math::Matrix is the class template that libbarrett
 * uses to represent vectors and matrices. math::Matrix inherits from
 * Eigen::Matrix, so you can use barrett::units in the same ways you would use
 * an Eigen::Matrix. (There's no runtime penalty for using barrett::units,
 * though there is some additional memory allocated by math::Matrix in order to
 * allow interoperation with the GSL math library.) Check out barrett/units.h to
 * see what units are predefined, and feel free to create your own
 * barrett::units as you see fit!
 */

#include <iostream>
#include <string>
#include <cstdlib>  // For strtod()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>

#include <iostream>
#include <string>
#include <cstdlib>  // For strtod()

// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>

#include <math.h> /* For sqrt() */
#include <signal.h>
#include <syslog.h>
#include <unistd.h>
#include <curses.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <boost/tuple/tuple.hpp>
#include <barrett/exception.h>
#include <barrett/os.h>
#include <barrett/cdlbt/gsl.h>
#include <barrett/cdlbt/kinematics.h>
#include <barrett/cdlbt/calgrav.h>
#include<fstream>

using namespace barrett;
//float angle_rad;
using detail::waitForEnter;
// This function template will accept a math::Matrix with any number of rows,
// any number of columns, and any units. In other words: it will accept any
// barrett::units type.
//----------------------------------------------------------------------------------------------------
template<int R, int C, typename Units>
bool parseDoubles(math::Matrix<R,C, Units>* dest, const std::string& str) {
	const char* cur = str.c_str();
	const char* next = cur;

	for (int i = 0; i < dest->size(); ++i) {
		(*dest)[i] = strtod(cur, (char**) &next);
		if (cur == next) {
			return false;
		} else {
			cur = next;
		}
	}

	// Make sure there are no extra numbers in the string.
	double ignore = strtod(cur, (char**) &next);
	(void)ignore;  // Prevent unused variable warnings

	if (cur != next) {
		return false;
	}

	return true;
}
//---------------------------------------------------------------------------------------------------

template<size_t DOF, int R, int C, typename Units>
void moveToStr(systems::Wam<DOF>& wam, math::Matrix<R,C, Units>* dest,
		const std::string& description, const std::string& str)
{
	if (parseDoubles(dest, str)) {
		std::cout << "Moving to " << description << ": " << *dest << std::endl;
		wam.moveTo(*dest);
	} else {
		printf("ERROR: Please enter exactly %d numbers separated by "
				"whitespace.\n", dest->size());
	}
}
//----------------------------------------------------------------------------------------------------

void printMenu() {
	printf("Commands:\n");
	printf("  p  Enter a tool position destination\n");
	printf("  h  Move to the home position\n");
	printf("  o  Only Initialize\n");
	printf("  a  Only close spread\n");
	printf("  c  Only close grasp\n");
	printf("  b  Trapezoidal Move\n");
	printf("  d  Only open spread\n");
	printf("  e  Only open grasp\n");
	printf("  j  Enter a joint position destination\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
	printf("  t  trapezoidal movement grasp 30 degrees\n");
	printf("  m  Custom trapezoidal move\n");

	//printf("  f  Calibrate using standard joint positions\n");
	printf("  g  Fingures spread by 60 degrees 1\n");

}
float angle_radian;
float angle_degree;
std::istringstream iss2;
std::string temp_str;
std::ifstream infile_jp("/home/robot/sam/cal_joint_pos.txt");
std::string line_jp;
//---------------------------------------------------------------------------------------------------
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// These vectors are fixed sized, stack allocated, and zero-initialized.
	jp_type jp;  // jp is a DOFx1 column vector of joint positions
	cp_type cp;  // cp is a 3x1 vector representing a Cartesian position

	wam.gravityCompensate();
	printMenu();
	Hand* hand = pm.getHand();

	std::string line;
	bool going = true;
	while (going) {
		printf(">>> ");
		std::getline(std::cin, line);

		switch (line[0]) {

		case 'p':
			moveToStr(wam, &cp, "tool position", line.substr(1));
			break;

		case 'j':
					moveToStr(wam, &jp, "joint positions", line.substr(1));
					break;

		case 'h':
			std::cout << "Moving to home position: "
					<< wam.getHomePosition() << std::endl;
			wam.moveHome();
			break;

		case 'i':
			printf("WAM idled.\n");
			wam.idle();
			break;

		case 'g':
			hand->trapezoidalMove(Hand::jp_type(M_PI/3.0), Hand::SPREAD);
			break;


		case 'o':
			hand->initialize();
//			hand->close(Hand::GRASP);
//			hand->open(Hand::GRASP);
//		   	hand->close(Hand::SPREAD);
//		   	hand->trapezoidalMove(Hand::jp_type(M_PI/2.0),1, Hand::GRASP);
//		   	hand->trapezoidalMove(Hand::jp_type(M_PI/2.0), Hand::GRASP);
		   	break;

		case 'a':
//			hand->initialize();
			hand->close(Hand::SPREAD);
//			hand->close(Hand::GRASP);
			break;

		case 'c':
//			hand->initialize();
			hand->close(Hand::GRASP);
			break;

//		case 'f':
//			while (std::getline(file_jp,line_jp))
//			{
//			moveToStr(wam, &jp, "joint positions", line_jp);
//			waitForEnter();
//			}
//			break;

		case 'd':
//					hand->initialize();
					hand->open(Hand::SPREAD);
					break;

		case 'e':
//							hand->initialize();
							hand->open(Hand::GRASP);
							break;

		case 'b':
			hand->trapezoidalMove(Hand::jp_type(M_PI/2.0), Hand::GRASP);
			break;

		case 't':
			hand->trapezoidalMove(Hand::jp_type((M_PI)/6.0), Hand::GRASP);
//			std::cout << "sending back a message..." << std::endl;
			break;

		case 'm':
//			temp_str = line.substr(2);
			iss2.clear();
			iss2.str("");
			iss2.str(line.substr(2)) ;
		    iss2 >> angle_degree;
//			std::cin >> angle_degree;
			angle_radian = angle_degree*(M_PI/180);
			hand->trapezoidalMove(Hand::jp_type(angle_radian), Hand::GRASP);
			std::cout << angle_degree << std::endl;
			break;


		case 'q':
		case 'x':
			printf("Quitting.\n");
			going = false;
			break;

		default:
			if (line.size() != 0) {
				printf("Unrecognized option.\n");
				printMenu();
			}
			break;
		}
	}


	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}



