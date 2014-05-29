/*
 * ex08_teach_and_play.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;



template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	typedef boost::tuple<double, jp_type> jp_sample_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	const double T_s = pm.getExecutionManager()->getPeriod();


	wam.gravityCompensate();

	systems::Ramp time(pm.getExecutionManager());

	systems::TupleGrouper<double, jp_type> jpLogTg;

	// Record at 1/10th of the loop rate
	systems::PeriodicDataLogger<jp_sample_type> jpLogger(pm.getExecutionManager(),
			new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, 10*T_s), 10);


	printf("Press [Enter] to start teaching.\n");
	waitForEnter();
	{
		// Make sure the Systems are connected on the same execution cycle
		// that the time is started. Otherwise we might record a bunch of
		// samples all having t=0; this is bad because the Spline requires time
		// to be monotonic.
		BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());

		connect(time.output, jpLogTg.template getInput<0>());
		connect(wam.jpOutput, jpLogTg.template getInput<1>());
		connect(jpLogTg.output, jpLogger.input);
		time.start();

	}
	int flag = 1 ;
	char key;


	std::ofstream myfile;
	myfile.open ("/home/robot/sam/raw_cal_joint_pos.txt");


	std::ofstream myfile_cp;
	myfile_cp.open ("/home/robot/sam/raw_cal_cp.txt");






while(flag)
	{
	std::cin >> key;
	if(key == 's')
	{
		std::cout << wam.getJointPositions() << std::endl;
		myfile << wam.getJointPositions()<< std::endl;
		myfile_cp << wam.getToolPosition() << std::endl;
//
	}
	if(key == 'q')
	{
		flag = 0;
	}
	}

	myfile.close();

	printf("Press [Enter] to stop teaching.\n");
	waitForEnter();
	jpLogger.closeLog();
	disconnect(jpLogger.input);





	printf("Press [Enter] to idle the WAM.\n");
	waitForEnter();
	wam.idle();

	std::ofstream final_file;
		final_file.open ("/home/robot/sam/cal_joint_pos.txt");

	  std::ifstream infile("/home/robot/sam/raw_cal_joint_pos.txt");

	  std::string s1;

	  while (std::getline(infile, s1))
	  {
		  			s1.erase(std::remove(s1.begin(),s1.end(),','),s1.end());
		  			s1.erase(std::remove(s1.begin(),s1.end(),']'),s1.end());
		  			s1.erase(std::remove(s1.begin(),s1.end(),'['),s1.end());
		  			std::cout << s1 << std::endl;
		  			final_file << s1 << std::endl;

	  }

	  std::ofstream final_file_cp;
	  		final_file_cp.open ("/home/robot/sam/robot_cordinates.txt");

	  	  std::ifstream infile_cp("/home/robot/sam/raw_cal_cp.txt");

	  	  std::string s2;

	  	  while (std::getline(infile_cp, s2))
	  	  {
	  		  			s2.erase(std::remove(s2.begin(),s2.end(),','),s2.end());
	  		  			s2.erase(std::remove(s2.begin(),s2.end(),']'),s2.end());
	  		  			s2.erase(std::remove(s2.begin(),s2.end(),'['),s2.end());
	  		  			std::cout << s2 << std::endl;
	  		  		final_file_cp << s2 << std::endl;

	  	  }

	  	final_file_cp.close();
	  final_file.close();

	std::remove(tmpFile);
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
