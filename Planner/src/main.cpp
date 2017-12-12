//============================================================================
// Name        : main.cpp
// Author      : Jochen Alt
//============================================================================

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "basics/logger.h"

#include "Util.h"
#include "LegKinematics.h"
#include "WindowController.h"
#include "core.h"
#include "EngineProxy.h"
#include "uiconfig.h"
#include "util.h"

INITIALIZE_EASYLOGGINGPP

using namespace std;

bool exitMode = false;


void signalHandler(int s){
	exitMode = true;
	cout << "Signal " << s << ". Exiting";
	cout.flush();
	exit(1);
}

void setupLogging(int argc, char *argv[]) {
	// catch SIGINT (ctrl-C)
    signal (SIGINT,signalHandler);

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

    LOG(INFO) << "Manfred Setup";
}

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

void printUsage(string prg) {
	cout << "usage: " << prg << " [-h] [-d \"<command>\"] [-i]" << endl
	 	 << "  [-host <ip>]        webserver ip address of engine" << endl
	 	 << "  [-port <port>]      webserver's port " << endl
	 	 << "  [-standalone]       include the engine" << endl
	 	 << "  [-virtualbox]       -host 192.168.56.1 -port 3000" << endl
	 	 << "  [-odroid]           -host 192.168.178.73 -port 8000" << endl
	 	 << "  [-h]                help" << endl
	 	 << "  [-t (test)]         componten test" << endl
		 << "  <without par>       start engine and ui" << endl;
}

int main(int argc, char *argv[]) {
	// initialize Logging
	setupLogging(argc, argv);

	// print help
	string webserver_host = WIFI_WEB_SERVER_HOST;
	int webserver_port = LINUX_WEB_SERVER_PORT;
    char * arg= getCmdOption(argv, argv + argc, "-host");
	bool remoteEngine= false;
    if(arg != NULL) {
		webserver_host = arg;
		remoteEngine = true;
    }
	arg= getCmdOption(argv, argv + argc, "-port");
	if(arg != NULL) {
		webserver_port = atoi(arg);
	} else {
		if (remoteEngine)
			webserver_port = 8000;
	}

	bool uselocalEngine  = cmdOptionExists(argv, argv+argc, "-standalone");
	bool useVirtualBox   = cmdOptionExists(argv, argv+argc, "-virtualbox");
	bool useODroid = cmdOptionExists(argv, argv+argc, "-odroid");

	if (useVirtualBox) {
		webserver_host = "192.168.56.1";
		webserver_port = 3000;
	}

	if (useODroid) {
		webserver_host = "192.168.178.73";
		webserver_port = 8000;
	}


	remoteEngine = !uselocalEngine && (remoteEngine || useVirtualBox || useODroid);

	std::set_terminate([](){
		std::cout << "Unhandled exception\n"; std::abort();
	});

	// print help
	if(cmdOptionExists(argv, argv+argc, "-h")) {
		printUsage(argv[0]);
		exit(0);
    }

	// component test
	arg= getCmdOption(argv, argv + argc, "-t");
	if(arg != NULL) {
		string sarg = arg;
		if (sarg.compare(0,3,"leg") == 0) {
			Engine main;
			LegKinematics legCtrl;
			legCtrl.setup(main);
			legCtrl.selftest();
		}
		exit(0);
	}


	if (remoteEngine) {
		cout << "connecting to engine running at " << webserver_host << ":" << webserver_port << endl;
		EngineProxy::getInstance().setupRemoteEngine(webserver_host,webserver_port);
	}
	else {
		cout << "using local engine. Call with -h to change that" << endl;
		EngineProxy::getInstance().setupSimulatedEngine();
	}


	// run one loop to get all initial data necessary to initialize UI
	EngineProxy::getInstance().loop();

	// initialize ui
	bool UISetupOk= WindowController::getInstance().setup(argc, argv);
	if (!UISetupOk) {
		cerr << "UI initialization failed" << endl;
		exit(1);
	}

	while (true) {
		EngineProxy::getInstance().loop();
		delay_ms(10);
	}

    cout << "no dwim running. Try -h" << endl;
	return 0;
}
