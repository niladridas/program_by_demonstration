#include "ServerSocket.h"
#include "SocketException.h"

#include <iostream>
#include <string>
#include <cstdlib>  // For strtod()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <fstream>


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

using namespace barrett;
using detail::waitForEnter;

std::ifstream file_jp("/home/robot/sam/cal_joint_pos.txt");

std::string line_jp;
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


void printMenu() {
	printf("Commands:\n");
	printf("  j  Enter a joint position destination\n");
	printf("  h  Move to the home position\n");
	printf("  p  Enter a tool position destination\n");
	printf("  o  Open and Close the hand\n");
	printf (" M Sending messages\n");
	printf("  i  Idle (release position/orientation constraints)\n");
	printf("  q  Quit\n");
	printf("  f  Calibrate using standard joint positions\n");

}



template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// These vectors are fixed sized, stack allocated, and zero-initialized.
	jp_type jp;  // jp is a DOFx1 column vector of joint positions
	cp_type cp;  // cp is a 3x1 vector representing a Cartesian position

	wam.gravityCompensate();
	printMenu();
	Hand* hand = pm.getHand();


// Write a socket program here. It is a server socket that would close only when the program closes
// This client socket would connect to a server socket on a external machine a recieve data from it
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
    int status;
    struct addrinfo host_info;       
    struct addrinfo *host_info_list; 
    memset(&host_info, 0, sizeof host_info);
    host_info.ai_family = AF_UNSPEC;     
    host_info.ai_socktype = SOCK_STREAM; 
    host_info.ai_flags = AI_PASSIVE;    
    status = getaddrinfo("192.168.0.110", "5555", &host_info, &host_info_list); //192.168.0.110 this is the wam(server) address
    int socketfd ;
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
    int yes = 1;
    status = setsockopt(socketfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
    status = bind(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    status =  listen(socketfd, 5);
    int new_sd;
    struct sockaddr_storage their_addr;
    socklen_t addr_size = sizeof(their_addr);
    new_sd = accept(socketfd, (struct sockaddr *)&their_addr, &addr_size);
	std::string line;

	bool going = true;


	while (going) {
//-----------------------------------------------------------------------------------
 ssize_t bytes_recieved;
    char incomming_data_buffer[1000];
    bytes_recieved = recv(new_sd, incomming_data_buffer,1000, 0);
//    if (bytes_recieved == 0) std::cout << "host shut down." << std::endl ;
//    if (bytes_recieved == -1)std::cout << "recieve error!" << std::endl ;
    incomming_data_buffer[bytes_recieved] = '\0';

    const char *msg = "Niladri Das I am the Server";
    const char *msg1 = "Press f exactly four times";
    int len1;
    	int len;
    	ssize_t bytes_sent;
    	ssize_t bytes_sent1;
    	len = strlen(msg);
    	len1 = strlen(msg1);
//-----------------------------------------------------------------------------------
//		printf(">>> ");

//		std::cin.getline(incomming_data_buffer, line);
    	line = std::string(incomming_data_buffer);
	std::cout << incomming_data_buffer << std::endl;
		switch (line[0]) {
		case 'j':
			moveToStr(wam, &jp, "joint positions", line.substr(1));
			std::cout << "sending back a message..." << std::endl;
			bytes_sent = send(new_sd, msg, len, 0);
			break;
		case 'f':
					std::getline(file_jp,line_jp);
					{
					moveToStr(wam, &jp, "joint positions", line_jp);

					}
					std::cout << "sending back a message..." << std::endl;
								bytes_sent1 = send(new_sd, msg1, len1, 0);
					break;

		case 'p':
			moveToStr(wam, &cp, "tool position", line.substr(1));
			std::cout << "sending back a message..." << std::endl;
			bytes_sent = send(new_sd, msg, len, 0);
			break;

		case 'M':
			std::cout << "send()ing back a message..."  << std::endl;
			bytes_sent = send(new_sd, msg, len, 0);
			break;

		case 'h':
			std::cout << "Moving to home position: "
					<< wam.getHomePosition() << std::endl;
			wam.moveHome();
			std::cout << "sending back a message" << std::endl;
			bytes_sent = send(new_sd, msg, len,0);
			break;

		case 'i':
			printf("WAM idled.\n");
			wam.idle();
			std::cout << "sending back a message" << std::endl;
			bytes_sent = send(new_sd, msg, len,0);
			break;
               case 'o':
			hand->initialize();
			hand->close(Hand::GRASP);
			hand->open(Hand::GRASP);
          		hand->close(Hand::SPREAD);
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

//-----------------------------------------------------------------------------
	freeaddrinfo(host_info_list);
    close(new_sd);
    close(socketfd);

//-----------------------------------------------------------------------------
	return 0;
}
