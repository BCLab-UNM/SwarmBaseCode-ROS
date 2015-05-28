/*
 * Author: Kurt Leucht
 * Maintainer: Kurt Leucht
 * Email: kurt.leucht@nasa.gov
 * NASA Center: Kennedy Space Center
 * Mail Stop: NE-C1
 * 
 * Project Name: Swarmie Robotics NASA Center Innovation Fund
 * Principal Investigator: Cheryle Mako
 * Email: cheryle.l.mako@nasa.gov 
 * 
 * Date Created: October 2, 2014
 * Safety Critical: NO
 * NASA Software Classification: D
 * 
 * This software is copyright the National Aeronautics and Space Administration (NASA)
 * and is distributed under the GNU LGPL license.  All rights reserved.
 * Permission to use, copy, modify and distribute this software is granted under
 * the LGPL and there is no implied warranty for this software.  This software is provided
 * "as is" and NASA or the authors are not responsible for indirect or direct damage
 * to any user of the software.  The authors and NASA are under no obligation to provide
 * maintenence, updates, support or modifications to the software.
 * 
 * Revision Log:
 * -cpu utilization COMPLETE, TBD-date (KWL)
 */

// ROS headers
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
// C and Linux headers
#include <cstdlib>

using namespace std;

#define MODULE_VERSION	"1.0"
#define STRING_LENGTH	255

// optional parameter server stored variables
int HEALTH_PUB_BUF_SIZE =	1;	
int HEALTH_PUB_RATE =		1;

// TODO: move these globals to a shared header file or go OO with getters
#define SUCCESS			0
#define ERROR			-1

/*
 * This subroutine loads all parameters from the ROS Parameter Server
 */
int initParams() {

	// if the parameters exist in the parameter server they will get overwritten, 
	// otherwise hard coded values from above will remain in place
	ros::param::get("/HEALTH_PUB_BUF_SIZE", HEALTH_PUB_BUF_SIZE);	
	ros::param::get("/HEALTH_PUB_RATE", HEALTH_PUB_RATE);

	return(SUCCESS);
}

/*
 * Main just initializes publishers and handlers and then spins forever.
 */
int main(int argc, char** argv) {

	ROS_INFO("health_status: module is initializing (v%s)", MODULE_VERSION);
	
	char host[STRING_LENGTH];
	gethostname(host, sizeof (host));
	string hostname(host);
	string publishedName;

	if (argc >= 2) {
		publishedName = argv[1];
		cout << "Welcome to the world of tomorrow " << publishedName << "!  Health & Status module started." << endl;
	} else {
		publishedName = hostname;
		cout << "No Name Selected. Default is: " << publishedName << endl;
	}

	ros::init(argc, argv, (publishedName + "_HEALTH_STATUS"));
    	ros::NodeHandle n;

	// initialize variables from parameter server
	initParams();

	// set up publication of busy CPU per second
	ros::Publisher busy_cpu_sec_pub = n.advertise<std_msgs::Float32>((publishedName + "/health/busy_cpu_sec"), 
											HEALTH_PUB_BUF_SIZE);
	ros::Rate health_loop_rate(HEALTH_PUB_RATE); 

	// set up publication of busy CPU past minute average
	ros::Publisher busy_cpu_min_pub = n.advertise<std_msgs::Float32>((publishedName + "/health/busy_cpu_min"), 
											HEALTH_PUB_BUF_SIZE);

	// set up publication of used memory per second
	ros::Publisher used_memory_sec_pub = n.advertise<std_msgs::Float32>((publishedName + "/health/used_memory_sec"), 
											HEALTH_PUB_BUF_SIZE);

	// set up publication of used memory past minute average
	ros::Publisher used_memory_min_pub = n.advertise<std_msgs::Float32>((publishedName + "/health/used_memory_min"), 
											HEALTH_PUB_BUF_SIZE);

	// set up publication of uptime (in seconds)
	ros::Publisher uptime_pub = n.advertise<std_msgs::Int32>((publishedName + "/health/uptime_sec"), 
											HEALTH_PUB_BUF_SIZE);

	// set up publication of total threads
	ros::Publisher threads_pub = n.advertise<std_msgs::Int32>((publishedName + "/health/threads"), 
											HEALTH_PUB_BUF_SIZE);

	// variables used to calculate CPU utilization
	unsigned int prev_idle = 0;
	unsigned int prev_non_idle = 0;
	unsigned int prev_io_wait = 0;
	unsigned int prev_user = 0;
	unsigned int prev_nice = 0;
	unsigned int prev_system = 0;
	unsigned int prev_irq = 0;
	unsigned int prev_soft_irq = 0;
	unsigned int prev_steal = 0;
	unsigned int prev_total = 0;
	unsigned int curr_idle = 0;
	unsigned int curr_non_idle = 0;
	unsigned int curr_io_wait = 0;
	unsigned int curr_user = 0;
	unsigned int curr_nice = 0;
	unsigned int curr_system = 0;
	unsigned int curr_irq = 0;
	unsigned int curr_soft_irq = 0;
	unsigned int curr_steal = 0;
	unsigned int curr_total = 0;
	float cpu_min_calc = 0;
	float cpu_sec_calc[60];

	// preset array of floats to zero 
	for (int i = 0; i < 60; i++) {
		cpu_sec_calc[i] = 0;
	}

	// variables used to calculate memory utilization
	unsigned int total_memory = 0;
	unsigned int free_memory = 0;
	float memory_min_calc = 0;
	float memory_sec_calc[60];

	// preset array of floats to zero 
	for (int i = 0; i < 60; i++) {
		memory_sec_calc[i] = 0;
	}

	ROS_INFO("health_status: module is running");

	while (ros::ok()) {

		std_msgs::Float32	busy_cpu_sec_msg;
		std_msgs::Float32	busy_cpu_min_msg;
		std_msgs::Float32	used_memory_sec_msg;
		std_msgs::Float32	used_memory_min_msg;
		std_msgs::Int32		uptime_msg;
		std_msgs::Int32		threads_msg;

		// open the stat file that holds CPU utilization data and read one line
		FILE *f = fopen("/proc/stat", "r");
		size_t sz = 0;
		char * lin = 0;
		ssize_t lsz = getline (&lin, &sz, f);
		ROS_DEBUG("health_status: read line: %s", lin);		
		fclose (f);

		// parse tokens to get data into variables
		char seps[] = " \t\n/";
		char *token;
		char *prevToken;
		int token_ctr = 0;		
		token = strtok(lin, seps);
		token_ctr++;	
		while (token != NULL) {
			prevToken = token;
			token = strtok(NULL, seps);

			if (token_ctr == 1) {
				curr_user = atoi(token);
			}
			else if (token_ctr == 2) {
				curr_nice = atoi(token);
			}
			else if (token_ctr == 3) {
				curr_system = atoi(token);
			}
			else if (token_ctr == 4) {
				curr_idle = atoi(token);
			}
			else if (token_ctr == 5) {
				curr_io_wait = atoi(token);
			}
			else if (token_ctr == 6) {
				curr_irq = atoi(token);
			}
			else if (token_ctr == 7) {
				curr_soft_irq = atoi(token);
			}
			else if (token_ctr == 8) {
				curr_steal = atoi(token);
			}
	
			token_ctr++;	
		}

		// push all values down the array one index
		for (int i = 60 - 2; i >= 0; i--) {
			cpu_sec_calc[i + 1] = cpu_sec_calc[i];
		}

		// perform the cpu calculation for the current second in time
		curr_idle = curr_idle + curr_io_wait;
		curr_non_idle = curr_user + curr_nice + curr_system + curr_irq + curr_soft_irq + curr_steal;
		curr_total = curr_idle + curr_non_idle;
		cpu_sec_calc[0] = 100 * (float)((curr_total-prev_total)-(curr_idle-prev_idle))/(curr_total-prev_total);
		ROS_DEBUG("health_status: calculated: %0.1f", cpu_sec_calc[0]);

		// populate all previous variables for next cycle
		prev_idle = curr_idle;
		prev_non_idle = curr_non_idle;
		prev_io_wait = curr_io_wait;
		prev_user = curr_user;
		prev_nice = curr_nice;
		prev_system = curr_system;
		prev_irq = curr_irq;
		prev_soft_irq = curr_soft_irq;
		prev_steal = curr_steal;
		prev_total = curr_total;

		// write value to ROS message
		busy_cpu_sec_msg.data = cpu_sec_calc[0];
		busy_cpu_sec_pub.publish(busy_cpu_sec_msg);

		// calculate the past minute's average
		float grand_total = 0;
		for (int i = 0; i < 60; i++) {
			grand_total = grand_total + cpu_sec_calc[i];
		}
		cpu_min_calc = grand_total / 60;

		// write value to ROS message
		busy_cpu_min_msg.data = cpu_min_calc;
		busy_cpu_min_pub.publish(busy_cpu_min_msg);

		// open the stat file that holds memory utilization data and read one line
		f = fopen("/proc/meminfo", "r");
		lsz = getline (&lin, &sz, f);
		ROS_DEBUG("health_status: read line: %s", lin);		

		// parse tokens to get data into a variable
		token = strtok(lin, seps);
		token = strtok(NULL, seps);
		total_memory = atoi(token);
		
		// read the second line
		lsz = getline (&lin, &sz, f);
		ROS_DEBUG("health_status: read line: %s", lin);		
		fclose (f);

		// parse tokens to get data into a variable
		token = strtok(lin, seps);
		token = strtok(NULL, seps);
		free_memory = atoi(token);		

		// push all values down the array one index
		for (int j = 60 - 2; j >= 0; j--) {
			memory_sec_calc[j + 1] = memory_sec_calc[j];
		}

		// perform the memory calculation for the current second in time
		memory_sec_calc[0] = 100 * (float)(total_memory - free_memory) / total_memory;
		ROS_DEBUG("health_status: calculated: %0.1f", memory_sec_calc[0]);

		// write value to ROS message
		used_memory_sec_msg.data = memory_sec_calc[0];
		used_memory_sec_pub.publish(used_memory_sec_msg);

		// calculate the past minute's average
		float memory_grand_total = 0;
		for (int j = 0; j < 60; j++) {
			memory_grand_total = memory_grand_total + memory_sec_calc[j];
		}
		memory_min_calc = memory_grand_total / 60;

		// write value to ROS message
		used_memory_min_msg.data = memory_min_calc;
		used_memory_min_pub.publish(used_memory_min_msg);

		// open the stat file that holds uptime data and read one line
		f = fopen("/proc/uptime", "r");
		sz = 0;
		lsz = getline (&lin, &sz, f);
		ROS_DEBUG("health_status: read line: %s", lin);		
		fclose (f);

		// parse tokens to get data into a variable
		token = strtok(lin, seps);
		float uptime_float = atof(token);

		// write value to ROS message
		uptime_msg.data = (int)uptime_float;
		uptime_pub.publish(uptime_msg);

		// open the stat file that holds threads data and read one line
		f = fopen("/proc/loadavg", "r");
		sz = 0;
		lsz = getline (&lin, &sz, f);
		ROS_DEBUG("health_status: read line: %s", lin);		
		fclose (f);

		// parse tokens to get data into a variable
		token = strtok(lin, seps);
		token = strtok(NULL, seps);
		token = strtok(NULL, seps);
		token = strtok(NULL, seps);
		token = strtok(NULL, seps);
		int threads = atoi(token);

		// write value to ROS message
		threads_msg.data = threads;
		threads_pub.publish(threads_msg);

		// spin and sleep for next cycle
		ros::spinOnce();
		health_loop_rate.sleep();
	}

	return(SUCCESS);
}


