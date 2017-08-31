/*
 * KinematicsDispatcher.cpp
 *
 *  Created on: 21.06.2017
 *      Author: JochenAlt
 */

#include "Poco/Net/HTTPClientSession.h"
#include "Poco/Net/HTTPRequest.h"
#include "Poco/Net/HTTPResponse.h"
#include <Poco/Net/HTTPCredentials.h>
#include "Poco/StreamCopier.h"
#include "Poco/NullStream.h"
#include "Poco/Path.h"
#include "Poco/URI.h"
#include "Poco/Exception.h"

#include "setup.h"

#include "EngineProxy.h"
#include "WindowController.h"

using namespace Poco;
using namespace Net;
using namespace std;


bool EngineCaller::httpGET(string path, string &responsestr, int timeout_ms) {
	std::ostringstream address;
	address << "http://" << host << ":" << port;
	if (path.find("/") != 0)
		address << "/";
	address << path;

	LOG(DEBUG) << "calling " << address.str();

	URI uri(address.str());
    std::string pathandquery(uri.getPathAndQuery());
    if (pathandquery.empty())
    	pathandquery = "/";

    HTTPClientSession session(uri.getHost(), uri.getPort());
    HTTPRequest request(HTTPRequest::HTTP_GET, pathandquery, HTTPMessage::HTTP_1_1);
    HTTPResponse response;

    session.setTimeout(Timespan(timeout_ms/1000,(timeout_ms%1000)*1000));
    try {
    	session.sendRequest(request);
    	std::istream& rs = session.receiveResponse(response);
        string line;
        responsestr = "";
        while(std::getline(rs, line))
        	responsestr += line + "\r\n";

    	return (response.getStatus() == HTTPResponse::HTTP_OK);
    }
    catch (Poco::TimeoutException ex) {
    	setError(WEBSERVER_TIMEOUT);
    	LOG(DEBUG) << "request timeout " << ex.name();
    	std::ostringstream s;
    	s << "NOK(" << WEBSERVER_TIMEOUT << ") " << getLastErrorMessage();
    	responsestr = s.str();
    	return false;
    }
}


bool EngineCaller::httpPOST(string path, string body, string &responsestr, int timeout_ms) {
	std::ostringstream address;
	address << "http://" << host << ":" << port;
	if (path.find("/") != 0)
		address << "/";
	address << path;

	URI uri(address.str());
    std::string pathandquery(uri.getPathAndQuery());
    if (pathandquery.empty())
    	pathandquery = "/";

    HTTPClientSession session(uri.getHost(), uri.getPort());
    HTTPRequest request(HTTPRequest::HTTP_POST, pathandquery, HTTPMessage::HTTP_1_1);
    request.setContentType("application/x-www-form-urlencoded");
    request.setKeepAlive(true); // notice setKeepAlive is also called on session (above)
    request.add("Content-Length", intToString((int)body.size()));

    HTTPResponse response;

    session.setTimeout(Timespan(timeout_ms/1000,(timeout_ms%1000)*1000));
    try {
    	std::ostream& bodyOStream = session.sendRequest(request);
    	bodyOStream << body;  // sends the body
   	    std::istream& rs = session.receiveResponse(response);

    	string line;
    	responsestr = "";
    	while(std::getline(rs, line))
    		responsestr += line + "\r\n";

    	return (response.getStatus() == HTTPResponse::HTTP_OK);
    }
    catch (Poco::TimeoutException ex) {
      	setError(WEBSERVER_TIMEOUT);
      	LOG(DEBUG) << "request timeout " << ex.name();
      	std::ostringstream s;
      	s << "NOK(" << WEBSERVER_TIMEOUT << ") " << getLastErrorMessage();
      	responsestr = s.str();
      	return false;
     }
}


void EngineCaller::setup(string pHost, int pPort) {
	host = pHost;
	port = pPort;

	Poco::Net::initializeNetwork();
}

EngineProxy::EngineProxy() {
	map.setGridDimension(2048,2048,50);
}

EngineProxy::~EngineProxy() {
}

EngineProxy& EngineProxy::getInstance() {
	static EngineProxy me;
	return me;
}

void EngineProxy::setupSimulatedEngine() {
	callRemoteEngine = false;

	engine.setupSimulation();

	// run one loop to get initial state
	loop();
}

void EngineProxy::setupRemoteEngine(string host, int port) {
	callRemoteEngine = true;

	// initialize local and remote engine
	// (during runtime, this can be switched)
	remoteEngine.setup(host, port);
	engine.setupSimulation();

	// run one loop to get initial state
	loop();
}

void EngineProxy::loop() {
	if (callRemoteEngine) {
		if (remoteEngineCallTimer.isDue(BotTrajectorySampleRate)) {
			updateEngineState(data);
			newBotDataAvailable  = (data != lastData);
			newMapPoseDataAvailable =  (data.currentFusedPose != lastData.currentFusedPose);
		}

		if (UpdateMapSampleRate > 0) {
			if (fetchMapTimer.isDue(UpdateMapSampleRate)) {
				updateMap();
			}
		}
		if (UpdateLaserScanSampleRate > 0) {
			if (fetchLaserScanTimer.isDue(UpdateLaserScanSampleRate)) {
				updateLaserScan();
			}
		}
		if (UpdateTrajectorySampleRate > 0) {
			if (fetchTrajectoryTimer.isDue(UpdateTrajectorySampleRate)) {
				updateTrajectory();
			}
		}

	} else {
		engine.ratedloop();

		// fetch state data from server and put it in the client
		engine.getState(data);

		newBotDataAvailable  = (data != lastData);
		newMapPoseDataAvailable =  (data.currentFusedPose != lastData.currentFusedPose);
		lastData = data;
	}
}

void EngineProxy::turnOn() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/engine/turnon";
		remoteEngine.httpGET(url.str(), responseStr, 5000);
	} else {

		engine.turnOn();
	}
};

void EngineProxy::turnOff() {
	if (callRemoteEngine) {
		string responseStr;
		remoteEngine.httpGET("/engine/turnoff", responseStr, 5000);
	} else {
		engine.turnOff();
	}
}


// start wakeup and fallasleep procedure
void EngineProxy::wakeUp() {
	if (callRemoteEngine) {
		string responseStr;
		remoteEngine.httpGET("/engine/wakeup", responseStr, 5000);
	} else {
		engine.wakeUp();
	}
};

void EngineProxy::fallAsleep() {
	if (callRemoteEngine) {
		string responseStr;
		remoteEngine.httpGET("/engine/fallasleep", responseStr, 5000);
	} else {
		engine.fallAsleep();
	}
};

void EngineProxy::terrainMode(bool terrainOn) {
	if (callRemoteEngine) {
		string responseStr;
		remoteEngine.httpGET("/engine/terrain?on=" + boolToJSonString(terrainOn), responseStr, 5000);
	} else {
		engine.terrainMode(terrainOn);
	}
}

void EngineProxy::setTargetBodyPose ( const Pose& bodyPose) {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream bodyposeIn;
		bodyPose.serialize(bodyposeIn);

		std::ostringstream url;
		url << "/engine/setpose?bodypose="  << stringToJSonString(bodyposeIn.str());
		remoteEngine.httpGET(url.str(), responseStr, 5000);
	} else {
		engine.setTargetBodyPose(bodyPose);
	}
}

void EngineProxy::setTargetMovement ( mmPerSecond newSpeed,  realnum newRotateZ /* radians/s */,realnum absWalkingDirection ) {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/engine/setspeed?"
			<< "&movementspeed=" << floatToString(newSpeed,1)
			<< "&movementrotatez=" << floatToString(newRotateZ,3)
			<< "&movementdirection=" << floatToString(absWalkingDirection,3);
		remoteEngine.httpGET(url.str(), responseStr, 5000);
	} else {
		engine.setTargetSpeed(newSpeed);
		engine.setTargetAngularSpeed(newRotateZ);
		engine.setTargetWalkingDirection(absWalkingDirection);
	}
}


void EngineProxy::setGaitMode(GaitModeType gaitType) {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/engine/setgaitmode"
			<< "?gaitmode="  << (int)gaitType;
		remoteEngine.httpGET(url.str(), responseStr, 5000);
	} else {
		engine.setTargetGaitMode(gaitType);
	}
}

void EngineProxy::setTargetFrontLegPoseWorld(const Point& frontLeg) {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream frontLegIn;
		frontLeg.serialize(frontLegIn);

		remoteEngine.httpGET("/engine/setfrontleg?frontleg=" + stringToJSonString(frontLegIn.str()), responseStr, 5000);
	} else {
		engine.setTargetFrontLegPose(frontLeg);
	}
}

void EngineProxy::updateEngineState(EngineState & state) {
	if (callRemoteEngine) {
		string responseStr;
		remoteEngine.httpGET("/engine/get", responseStr, 5000);
		std::istringstream in(responseStr);
		data.deserialize(in);
	} else {
		EngineState tmp;
		engine.getState(tmp);
		state = tmp;
	}
}


void EngineProxy::imposeDistanceSensors(realnum distance[NumberOfLegs]) {
	engine.imposeDistanceSensors(distance);
}

bool EngineProxy::isTurnedOn() {
	return data.isTurnedOn;
};

void EngineProxy::getCurrentMovement ( angle_rad& noseOrientation, mmPerSecond& newSpeed , realnum& rotateZ /* radians/s */, realnum& walkingDirection) {
	noseOrientation = data.currentNoseOrientation;
	newSpeed = data.currentSpeed;
	rotateZ = data.currentAngularSpeed;
	walkingDirection = data.currentWalkingDirection;
}

angle_rad EngineProxy::getNoseOrientation() {
	return data.currentNoseOrientation;
};

mmPerSecond EngineProxy::getCurrenSpeed() {
	return data.currentSpeed;
};

angle_rad EngineProxy::getCurrenWalkingDirection() {
	return data.currentWalkingDirection;
};

angle_rad EngineProxy::getCurrenAngularSpeed() {
	return data.currentAngularSpeed;
};


GaitModeType EngineProxy::getGaitMode() {
	return data.currentGaitMode;
}

// get all relevant data representing the current pose of the body and all legs
LegAnglesType EngineProxy::getLegAngles() {
	return data.legAngles;
}

Pose EngineProxy::getBodyPose() {
	return data.currentBodyPose;
}

const Pose& EngineProxy::getFusedPose() {
	return data.currentFusedPose;
}

const Pose& EngineProxy::getMapPose() {
	return data.currentMapPose;
}

GeneralEngineModeType EngineProxy::getGeneralMode() {
	return data.engineMode;
}

LegGaitPhase EngineProxy::getLegsGaitPhase(int legNo) {
	return data.legPhase[legNo];
};

LegPose EngineProxy::getFrontLegPoseWorld() {
	LegPose pose = data.frontLegPose;
	pose.position.rotateAroundZ(data.currentNoseOrientation);
	return pose;
}

angle_deg EngineProxy::getFootAngle(int legNo)  {
	return data.footAngle[legNo];
}


const FootOnGroundFlagType& EngineProxy::getFootOnGround() {
	return data.footOnGroundFlag;
}

Point EngineProxy::getToePointsWorld(int legNo)  {
	return data.toePointsWorld[legNo];
}


PentaPointType EngineProxy::getGaitRefPoints() {
	return data.currentGaitRefPoints;
}

const PentaPointType& EngineProxy::getGroundPoints() {
	return data.groundPoints;
}

// get the pose of all hips in world coordinates (relative to the belly buttons origin)
const PentaPoseType&  EngineProxy::getHipPoseWorld() {
	return data.hipPoseWorld;
}

Map& EngineProxy::getMap() {
	return map;
}

LaserScan& EngineProxy::getLaserScan() {
	return laserScan;
}

Trajectory& EngineProxy::getTrajectory() {
	return trajectory;
}

void EngineProxy::updateMap() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/map/get?no=" << map.getGenerationNumber();
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		Map tmp;
		tmp.deserialize(in, ok);

		if (ok) {
			newMapDataAvailable = true;
			map = tmp;
		}
	}
}


void EngineProxy::updateLaserScan() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/scan/get";
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		LaserScan tmp;
		tmp.deserialize(in, ok);

		if (ok) {
			newMapDataAvailable = true;
			laserScan = tmp;
		}
	}
}

void EngineProxy::updateTrajectory() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/trajectory/get";
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		Trajectory tmp;
		tmp.deserialize(in, ok);

		if (ok) {
			newTrajectoryDataAvailable = true;
			trajectory = tmp;
		}
	}
}

bool EngineProxy::isBotDataAvailable() {
	if (newBotDataAvailable) {
		newBotDataAvailable = false;
		return true;
	}
	return false;
}

bool EngineProxy::isMapDataAvailable() {
	if (newMapDataAvailable) {
		newMapDataAvailable = false;
		return true;
	}
	return false;
}

bool EngineProxy::isTrajectoryDataAvailable() {
	if (newTrajectoryDataAvailable) {
		newTrajectoryDataAvailable = false;
		return true;
	}
	return false;
}


bool EngineProxy::isEstimatedPoseAvailable() {
	if (newMapPoseDataAvailable) {
		newMapPoseDataAvailable = false;
		return true;
	}
	return false;
}


bool EngineProxy::isLaserScanAvailable() {
	if (newLaserScanAvailable) {
		newLaserScanAvailable = false;
		return true;
	}
	return false;
}

// return the occupancy grid
const Pose& EngineProxy::getOdomPose() {
	return data.currentOdomPose;
}
