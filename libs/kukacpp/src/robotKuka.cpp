/*
	Author: Guilherme R. Moreira
	Local: University of São Paulo
	Creation: 2018, June
*/

#include <iostream> 			// cout, endl,
#include <cstdlib>
#include <unistd.h>  			// sleep,
#include <boost/thread.hpp>
#include <stdio.h>      		// Default System Calls
#include <stdlib.h>     		// Needed for OS X
#include <string.h>     		// Needed for Strlen
#include <sys/socket.h> 		// Needed for socket creating and binding
#include <netinet/in.h> 		// Needed to use struct sockaddr_in
#include <time.h>       		// To control the timeout mechanism
#include <chrono>
#include <queue> 				// queue

#include "robotKuka.hpp"
#include "models.hpp"
#include "xml_handler.hpp"

using namespace kuka;

/*
RobotKuka::RobotKuka(float CYCLE_TIME)
 - This is the RobotKuka class constructor that receives cycle_time value. Cycle_time definition is important
 in order to configure movementation functions, since each part of movement will be executed at this time in-
 terval.
 ~ Arguments: (float) CYCLE_TIME
 ~ Returns: none
*/
RobotKuka::RobotKuka(float CYCLE_TIME)
{
	this->CYCLE_TIME = CYCLE_TIME;
}

/*
RobotKuka::RobotKuka()
 - This is the default RobotKuka class constructor that receives cycle_time = 0.012 as default value. 
 ~ Arguments: none
 ~ Returns: none
*/
RobotKuka::RobotKuka()
{
	this->CYCLE_TIME = 0.012;
}

/*
void RobotKuka::set_communicator_running_flag(bool value)
 - This function sets the boolean value to communicator_running_flag, flag responsible to states if communi-
 cation is running or not.
 ~ Arguments: (bool) value
 ~ Returns: void
*/
void RobotKuka::set_communicator_running_flag(bool value)
{
	this->communicator_running_flag = &value;
}

/*
bool* RobotKuka::get_communicator_running_flag()
 - This function returns the pointer to the boolean value of communicator_running_flag, flag responsible to 
 states if communication is running or not.
 ~ Arguments: none
 ~ Returns: (bool*) communicator_runnung_flag
*/
bool* RobotKuka::get_communicator_running_flag()
{
	return this->communicator_running_flag;
}

/*
void connection(std::string ip, unsigned int port, bool* running_flag)
 - This is the main function of communication, called in a new thread. It creates a UDP socket with startCommunicator
 server ip and port. While running_flag is true it receives a XML Send string from robot, getting all data, and
 generates a XML Receive string that sends to the robot all movement and digital outputs information.
 ~ Arguments: (std::string) ip, (unsigned int) port, (bool*) running_flag
 ~ Returns: void
*/
void connection(std::string ip, unsigned int port, bool* running_flag, RobotKuka* robotKuka)
{	
	#define EXPR_SIZE   2048
	#define BUFLEN      2048
	#define TRUE        1
	#define SERVERLEN   2048

	int fd;

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror( "socket failed" );
    }

    struct sockaddr_in serveraddr;
    memset( &serveraddr, 0, sizeof(serveraddr) );
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons( port );
    serveraddr.sin_addr.s_addr = htonl( INADDR_ANY );

	struct sockaddr_in clientaddr;
    socklen_t sendsize = sizeof(clientaddr);
	bzero(&clientaddr, sizeof(clientaddr));


    if (bind(fd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0) {
        perror( "bind failed" );
    }


	while (*running_flag)
	{
		char* xml_send = (char*)malloc(sizeof(char) * 2048);

		int length = recvfrom(fd, xml_send, sizeof(char) * 2048 , 0, (struct sockaddr *) &clientaddr, &sendsize );
        if ( length < 0 ) {
            perror( "recvfrom failed" );
            break;
        }

        /* Starts chrono to account CYCLE TIME */
		//auto started = std::chrono::high_resolution_clock::now();

		//std::cout << "Msg len: " << length << std::endl;

		xml_send[length] = '\0';  // This is necessary to avoid xml_parse errors;

		/* Gets all data read from received XML Send and puts it into a instance of Data. */
		Data data;
		data = xml_handler::get_data_from_xml_send(const_cast<char*>(xml_send), length);

		//data.get_r_ist().print();
		//std::cout << data.get_delay() << std::endl;

		//data.print();

		std::string xml_receive;  // creates string for XML Receive

		/* Creates void instances of pose, axis and digital outputs to send thru XML Receive. */
		kuka::KukaPose pose = kuka::KukaPose();
		kuka::Axes axis = kuka::Axes();
		kuka::DigitalOutputs digout = kuka::DigitalOutputs();

		/* Push one element from the queue if the movement queue is not empty. If the queue is empty,
		the void instances are placed. */
		if (!robotKuka->move_queue.empty())
		{
			pose = robotKuka->move_queue.front();
			robotKuka->move_queue.pop();
		}

		/* Generates the XML Receive string using the pose, axis, and digital outputs instances and
		the previous XML Send IPOC value. */
    	xml_receive = xml_handler::generate_xml_receive(pose, axis, digout, data.get_ipoc());

    	/* Sends the XML Receive string thru the socket created previously. */
		int send_status = sendto(fd, xml_receive.c_str(), xml_receive.length()*sizeof(char), 0, (struct sockaddr *) &clientaddr, 
								 sendsize);

		/* Free the allocated space for XML Send string. */
		free(xml_send);  // might be a bottleneck <!>

		/* Stops chrono to account CYCLE TIME. */
		//auto done = std::chrono::high_resolution_clock::now();
		//std::cout << "Time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(done-started).count();
	}


	/* Closes the previously openned communication socket. */
    close(fd);

    /* Exits the previously created communication thread. */
	pthread_exit(NULL);
	exit(0);
}

/*
void RobotKuka::startCommunicator(char *const ip, unsigned int port)
 - This function only sets communicator_running_flag to true and starts a new thread with function connection as argument.
 ~ Arguments: (char* const) ip, (unsigned int) port
 ~ Returns: void
*/
void RobotKuka::startCommunicator(char *const ip, unsigned int port)
{
	std::cout << "Comunicação iniciada..." << std::endl;

	bool flag = true;

	RobotKuka::set_communicator_running_flag(flag);

	boost::thread communication_thread(connection, ip, port, communicator_running_flag, this);
}

/*
void RobotKuka::stopCommunicator()
 - This function sets communicator_running_flag to false, killing the connection while loop and exiting the communication.
 ~ Arguments: none
 ~ Returns: void
*/
using namespace kuka;
void RobotKuka::stopCommunicator()
{
	bool flag = false;
	RobotKuka::set_communicator_running_flag(flag);
}


void split_pose(KukaPose pose, double time, RobotKuka* robotKuka)
{
	int packages_ammount = (int) time/robotKuka->CYCLE_TIME;

	KukaPose package_move(	pose.get_x()/packages_ammount, 
						pose.get_y()/packages_ammount, 
						pose.get_z()/packages_ammount, 
						pose.get_a()/packages_ammount, 
						pose.get_b()/packages_ammount, 
						pose.get_c()/packages_ammount);

	KukaPose package_speed(	package_move.get_x()/robotKuka->CYCLE_TIME, 
						package_move.get_y()/robotKuka->CYCLE_TIME, 
						package_move.get_z()/robotKuka->CYCLE_TIME, 
						package_move.get_a()/robotKuka->CYCLE_TIME, 
						package_move.get_b()/robotKuka->CYCLE_TIME, 
						package_move.get_c()/robotKuka->CYCLE_TIME);

	package_speed.print();

	robotKuka->security_limit_speed.print();

	if(package_speed.get_x() > robotKuka->security_limit_speed.get_x() || 
	   package_speed.get_y() > robotKuka->security_limit_speed.get_y() || 
	   package_speed.get_z() > robotKuka->security_limit_speed.get_z() || 
	   package_speed.get_a() > robotKuka->security_limit_speed.get_a() || 
	   package_speed.get_b() > robotKuka->security_limit_speed.get_b() || 
	   package_speed.get_c() > robotKuka->security_limit_speed.get_c())
	{
		std::cout << "ERROR | Security: security_limit_speed exceeded while planning path. Check the log." << std::endl;
		robotKuka->stopCommunicator();
	} else if(package_speed.get_x() > robotKuka->security_limit_speed.get_x()/2 || 
	   		  package_speed.get_y() > robotKuka->security_limit_speed.get_y()/2 || 
	   		  package_speed.get_z() > robotKuka->security_limit_speed.get_z()/2 || 
	   		  package_speed.get_a() > robotKuka->security_limit_speed.get_a()/2 || 
	   		  package_speed.get_b() > robotKuka->security_limit_speed.get_b()/2 || 
	   		  package_speed.get_c() > robotKuka->security_limit_speed.get_c()/2)
	{
		std::cout << "WARNING | Security: half of security_limit_speed exceeded while planning path. Check the log." << std::endl;
	}

	for(int i = 0; i < packages_ammount; i++)
	{
		try
		{
			robotKuka->move_queue.push(package_move);
		}
		catch(int e)
		{
			std::cout << "ERROR | Exception: problem while pushing element to move_queue (" << e << ") . Check the log." << std::endl;
		}
	}

	std::cout << "LOG | " << packages_ammount << " elements pushed to move_queue." << std::endl;

	pthread_exit(NULL);
}


void RobotKuka::move(KukaPose pose, double time)
{
	boost::thread split_pose_thread(split_pose, pose, time, this);
}