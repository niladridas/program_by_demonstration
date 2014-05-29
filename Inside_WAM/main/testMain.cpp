#include <iostream>
#include <string>
#include <cstdlib>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>



using namespace barrett;

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	const char* path = pm.getWamDefaultConfigPath();
	std::cout<<"got configuration "<<std::endl;
	size_t i=0;
	while(path[i]!='\0'){
	  std::cout<<path[i];
	  i++;
	}
	std::cout<<"configuration should be printed"<<std::endl;
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    return 0;
}
