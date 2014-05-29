/*
 * ex08_teach_and_play.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <string>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

//#include <barrett/standard_main_function.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;

const size_t DOF = 7;

int main(){


 	typedef barrett::units::JointPositions<7>::type jp_type;
//	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	typedef boost::tuple<double, jp_type> jp_sample_type;




//	wam.gravityCompensate();

//	systems::Ramp time(pm.getExecutionManager());

//	systems::TupleGrouper<double, jp_type> jpLogTg;

	// Record at 1/10th of the loop rate

	char tmpFile[] = "/home/robot/sam/btK9mftQ";

	// Build spline between recorded points
	//log::Reader<jp_sample_type> lr1(tmpFile);
	//lr1.exportCSV(outputFileName);
	//printf("Output written to %s.\n", outputFileName);

	log::Reader<jp_sample_type> lr(tmpFile);
	std::vector<jp_sample_type> vec;
	for (size_t i = 0; i < lr.numRecords(); ++i) {
		vec.push_back(lr.getRecord());
		std::cout << boost::get<0>(vec.at(i)) << std::endl;
		std::cout << boost::get<1>(vec.at(i)) << std::endl;
	}

//	std::cout<<vec[0];
	//for( std::vector<float>::const_iterator i = vec.begin(); i != vec.end(); ++i)
	//	    std::cout << *i << ' ';

	math::Spline<jp_type> spline(vec);

	// First, move to the starting position
//	wam.moveTo(spline.eval(spline.initialS()));
	std::cout<<spline.eval(spline.initialS());
	// Then play back the recorded motion
	//time.stop();
	//
	std::cout<< spline.initialS();




//	while (trajectory.input.getValue() < spline.finalS()) {
//		usleep(100000);
//	}



	return 0;
}
