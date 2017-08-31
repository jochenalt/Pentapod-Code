/*
 * main.cpp
 *
 * Webserver (Mongoose)
 *
 * Author: JochenAlt
 */

#include "mongoose.h" // mongoose MUST be first include, do not know why

#include "core.h"
#include "basics/logger.h"

#include "CmdDispatcher.h"
#include "Util.h"
#include "setup.h"
#include "Engine.h"
#include <Map.h>
#include <basics/zip.h>
INITIALIZE_EASYLOGGINGPP

static struct mg_serve_http_opts s_http_server_opts;

#include <stdlib.h>
#include <ctype.h>

Engine engine;

// called when ^C is pressed
void signalHandler(int s){
	cout << "Signal " << s << ". Exiting";
	cout.flush();
	exit(1);
}

void setupLogger() {
	// setup logger
	el::Configurations defaultConf;
    defaultConf.setToDefault();
    defaultConf.set(el::Level::Error,el::ConfigurationType::Format, "%datetime %level [%func] [%loc] %msg");
    defaultConf.set(el::Level::Error, el::ConfigurationType::Filename, "logs/manfred.log");

    defaultConf.set(el::Level::Info,el::ConfigurationType::Format, "%datetime %level %msg");
    defaultConf.set(el::Level::Info, el::ConfigurationType::Filename, "logs/manfred.log");

    defaultConf.set(el::Level::Debug, el::ConfigurationType::ToStandardOutput,std::string("false"));
    // defaultConf.set(el::Level::Debug, el::ConfigurationType::Enabled,std::string("false"));

    defaultConf.set(el::Level::Debug, el::ConfigurationType::Format, std::string("%datetime %level [%func] [%loc] %msg"));
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Filename, "logs/manfred.log");

    // logging from uC is on level Trace
    defaultConf.set(el::Level::Trace, el::ConfigurationType::ToStandardOutput,std::string("false"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Format, std::string("%datetime %level [uC] %msg"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Filename, "logs/manfred.log");

    el::Loggers::reconfigureLogger("default", defaultConf);

	LOG(INFO) << "Pentapod Setup";
}


// Define an event handler function
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
    			bool processed = CommandDispatcher::getInstance().dispatch(uri, query, body, response, ok);
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


int main(void) {
	struct mg_mgr mgr;
	struct mg_connection *nc = NULL;
	cs_stat_t st;

	mg_mgr_init(&mgr, NULL);


	// if we run on jochens notebook, use a different port than on Linux
	// (where the machine is occupied completely and we can use 8000)
#ifdef _WIN32
	#define SERVER_PORT WIN_WEB_SERVER_PORT
#else
	#define SERVER_PORT LINUX_WEB_SERVER_PORT
#endif


	string serverport_s = intToString(SERVER_PORT);
	nc = mg_bind(&mgr, serverport_s.c_str(), ev_handler);
	if (nc == NULL) {
		LOG(ERROR) << "Cannot bind to " << SERVER_PORT;
		exit(1);
	}

	// Set up HTTP server parameters
	mg_set_protocol_http_websocket(nc);
	s_http_server_opts.document_root = "web_root"; // Set up web root directory
	if (mg_stat(s_http_server_opts.document_root, &st) != 0) {
		LOG(ERROR) << "Cannot find web_root directory, exiting.";
		exit(1);
	}

	// catch SIGINT (ctrl-C)
    signal (SIGINT,signalHandler);

	// log 
	setupLogger();


	LOG(INFO) << "Pentapod webserver on port " << SERVER_PORT;

	TimeSamplerStatic setupTimer;

	engine.setup(LEG_CONTROLLER_COMMAND_SERIAL_PORT, LEG_CONTROLLER_COMMAND_BAUD_RATE);

	while (true) {
		engine.loop();
		mg_mgr_poll(&mgr, 5); // check every 10ms for incoming requests
	}
	mg_mgr_free(&mgr);

	return 0;
}
