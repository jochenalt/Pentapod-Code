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

// Pentapod/Server
#include "CmdDispatcher.h"

// basic ROS
#include <ros/ros.h>

// messages and services
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

// SLAM topics
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/ros.h"
#include "mongoose.h" 

// used for publishing map->odom
#include <tf/transform_broadcaster.h>

using namespace std;

static struct mg_serve_http_opts s_http_server_opts;

CommandDispatcher* dispatcher_ptr = NULL;
// define an mongoose event handler function that is called whenever a request comes in
static void ev_handler(struct mg_connection *nc, int ev, void *ev_data)
{
    switch (ev)
    {
    	case MG_EV_HTTP_REQUEST: {
    			struct http_message *hm = (struct http_message *) ev_data;
    			string uri(hm->uri.p, hm->uri.len);
    			string query(hm->query_string.p, hm->query_string.len);
    	        string body(hm->body.p, hm->body.len);

    	        bool ok;
    			string response;
    			// if our dispatcher knows the command, it creates a response and returns true.
    			// Otherwise assume that we deliver static content.
    			bool processed = dispatcher_ptr->dispatch(uri, query, body, response, ok);
    			if (processed) {
    				if (ok) {
    					mg_printf(nc, "HTTP/1.1 200 OK\r\n"
    						"Content-Type: text/plain\r\n"
    						"Content-Length: %d\r\n\r\n%s",
    							(int) response.length(), response.c_str());
    				} else {
    					mg_printf(nc, "HTTP/1.1 500 Server Error\r\n"
    							"Content-Length: %d\r\n\r\n%s",
    							(int) response.length(), response.c_str());
    				}
    			} else {
    				// no API call, serve static content
    				mg_serve_http(nc, (http_message*) ev_data, s_http_server_opts);
    			}
    		break;
    	}
    default:
        break;
    }
}

int main(int argc, char * argv[]) {

	ROS_INFO_STREAM("starting pentapod_server node");

	ros::init(argc, argv, "pentapod_server_node");
	CommandDispatcher cmdDispatcher;
	dispatcher_ptr = &cmdDispatcher;

	ros::NodeHandle rosNode;
	string cortexSerialPort;
	int cortexBaudRate;
	int webserverPort;
	rosNode.param<int>("pentapod_server_node/webserver_port", webserverPort, 8000);
	ROS_INFO_STREAM("pentapod server runs on port" << webserverPort);

	// initialize mongoose webserver
	struct mg_mgr mgr;
	mg_mgr_init(&mgr, NULL);

	// bind the webserver to the port
	string serverport_s = intToString(webserverPort);
	struct mg_connection *nc = mg_bind(&mgr, serverport_s.c_str(), ev_handler);
	if (nc == NULL) {
		ROS_ERROR_STREAM("Cannot bind to %i" << webserverPort << ". Maybe the server is already running?");
		exit(1);
	}

	// Set up HTTP server parameters
	mg_set_protocol_http_websocket(nc);
	s_http_server_opts.document_root = "web_root"; // Set up web root directory
	cs_stat_t st;
	char pwdBuffer[1024];
	char* pwd = getcwd(pwdBuffer, 1024);
	if (mg_stat(s_http_server_opts.document_root, &st) != 0) {
		ROS_ERROR("Cannot find web_root directory in %s", pwd);
		exit(1);
	}

	// setup log
	ROS_INFO_STREAM("pentapod webserver running on port " << webserverPort << " in " << pwd);

	// timer to limit frequency of subsequent setup calls in case they are failling
	TimeSamplerStatic setupTimer;

	// setup all publishers and subscribers
	cmdDispatcher.setup(rosNode);

	// main loop that takes care of the webserver as well as ROS
	TimeSamplerStatic odomTimer;
	while (rosNode.ok()) {
		// pump callbacks of topics
		ros::spinOnce();


		// broadcast map -> odom transformation at 10 Hz
		if (odomTimer.isDue(1000/10)) {
			cmdDispatcher.broadcastTransformationMapToOdom();
			cmdDispatcher.initNavigation(rosNode);
		}


		// check and dispatch incoming http requests (dispatched by CommandDispatcher) and wait for 10ms max.
		mg_mgr_poll(&mgr, 10);
	}
	mg_mgr_free(&mgr);

	return 0;
}
	




