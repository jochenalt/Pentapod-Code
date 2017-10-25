#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

// Pentapod/Common
#include "core.h"
#include "basics/logger.h"
#include "Util.h"

// Pentapod/Kinematics
#include "setup.h"
#include "Engine.h"

// messages and services
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

// SLAM topics
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

// Pentapod messages
#include "pentapod_engine/engine_command_mode.h"
#include "OdomPublisher.h"

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

using namespace std;

int main(int argc, char * argv[]) {

	ROS_INFO_STREAM("starting pentapod_engine node");

	ros::init(argc, argv, "pentapod_engine_node");

	ros::NodeHandle rosNode;
	string cortexSerialPort;
	string cortexI2CPort;
	int cortexI2CAdr;
	int cortexBaudRate;

	rosNode.param<string>("pentapod_engine_node/cortex_serial_port", cortexSerialPort, CORTEX_CLI_SERIAL_PORT);
	rosNode.param<int>("pentapod_engine_node/cortex_serial_baudrate", cortexBaudRate, CORTEX_CLI_SERIAL_BAUDRATE);
	rosNode.param<string>("pentapod_engine_node/cortex_i2c_port", cortexI2CPort, CORTEX_I2C_PORT);
	rosNode.param<int>("pentapod_engine_node/cortex_i2c_address", cortexI2CAdr, CORTEX_I2C_ADDRESS);

	Engine engine;
	OdomPublisher odomPublisher;

	ROS_DEBUG_STREAM("cortex setup(" << cortexI2CPort << "(" << cortexI2CAdr << ") uart=" << cortexSerialPort << " @" << cortexBaudRate);
	bool cortexOk = engine.setupProduction(cortexI2CPort, cortexI2CAdr, cortexSerialPort, cortexBaudRate);
	if (!cortexOk) {
		ROS_ERROR_STREAM("cortex setup failed, I will continue with a simulation");
		cortexOk = engine.setupSimulation();
	}

	ROS_DEBUG_STREAM("cortex setup done " << cortexOk);

	// initialize all topics
       ROS_DEBUG_STREAM("initialization odom publisher");
	odomPublisher.setup(rosNode, engine);

	// main loop spins at the rate the cortex wants calls (around 45Hz)
	ros::WallRate mainLoopRate(1000.0/CORTEX_SAMPLE_RATE);

	// main loop that takes care of the engine
	// by sending move commands with approx. 45Hz
	TimeSamplerStatic lowPrioLoopTimer;
    ROS_DEBUG_STREAM("entering pentapod engine's main loop with " << 1000.0/CORTEX_SAMPLE_RATE << "Hz");
	while (rosNode.ok()) {
		// ensure that engine loop is timingwise correct since cortex
		// can tolerate only CORTEX_SAMPLE_RATE/2 ms = 10ms difference only.
		// so call engine.loop() right after sleep
		mainLoopRate.sleep();
		engine.loop(); // send move commands to cortex

		// after this call do anything which is not that timing relevant, e.g.
		// broadcasting all transformations. Do this at approx 10Hz.
		if (lowPrioLoopTimer.isDue(100)) {
			odomPublisher.broadcastState();
			odomPublisher.broadcastTransformation();
			odomPublisher.broadcastOdom();
		}

		// pump callbacks and topics
		// - listen to speed, bodypose and move commands coming from pentapod_server
		// - broadcast odom2base_link to be consumed by navigation
		// - broadcast base_link2laser to be consumed by hector slam
		// - broadcast pentapod_state to be consumed by pentapod_server
		ros::spinOnce();
	}

	return 0;
}
	




