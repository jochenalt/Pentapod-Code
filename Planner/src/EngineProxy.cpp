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
			newMapPoseDataAvailable =  (data.baseLinkInMapFrame != lastData.baseLinkInMapFrame);
		}

		if (UpdateMapSampleRate > 0) {
			if (fetchMapTimer.isDue(UpdateMapSampleRate)) {
				updateGlobalMaps();
			}
		}

		if (UpdateLocalCostmapSampleRate > 0) {
			if (fetchLocalCostmapTimer.isDue(UpdateLocalCostmapSampleRate)) {
				updateLocalCostmap();
			}
		}

		if (UpdateLaserScanSampleRate > 0) {
			if (fetchLaserScanTimer.isDue(UpdateLaserScanSampleRate)) {
				updateLaserScan();
			}
		}
		if (UpdateTrajectorySampleRate > 0) {
			if (fetchTrajectoryTimer.isDue(UpdateTrajectorySampleRate)) {
				// updateTrajectory();
				updatePlan();
				updateNavigation();
			}
		}

	} else {
		//
		bool actuallyLooped = engine.ratedloop();

		// fetch state data from server and put it in the client
		if (actuallyLooped) {
			engine.getState(data);

			newBotDataAvailable  = (data != lastData);
			newMapPoseDataAvailable =  (data.baseLinkInMapFrame != lastData.baseLinkInMapFrame);
			lastData = data;
		}
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
		engine.setGaitMode(gaitType);
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

radPerSecond EngineProxy::getCurrenAngularSpeed() {
	return data.currentAngularSpeed;
};


GaitModeType EngineProxy::getGaitMode() {
	return data.currentGaitMode;
}

// get all relevant data representing the current pose of the body and all legs
PentaLegAngleType EngineProxy::getLegAngles() {
	return data.legAngles;
}

Pose EngineProxy::getBodyPose() {
	return data.currentBodyPose;
}

const Pose& EngineProxy::getBaseLinkInMapFrame() {
	return data.baseLinkInMapFrame;
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
	pose.position = data.frontLegPose.position.getRotatedAroundZ(data.currentNoseOrientation);
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


Map& EngineProxy::getLocalCostmap() {
	return localCostmap;
}

Map& EngineProxy::getGlobalCostmap() {
	return globalCostmap;
}

LaserScan& EngineProxy::getLaserScan() {
	return laserScan;
}

Trajectory& EngineProxy::getTrajectory(TrajectoryType type) {
	switch (type) {
	case TRAJECTORY: return trajectory;
	case GLOBAL_PLAN: return globalPlan;
	case LOCAL_PLAN: return localPlan;
	}
	return trajectory;
}


// get last navigation goal(gets nulled once it has been reached)
Pose EngineProxy::getCurrentNavigationGoal() {
	return navigationGoal;
}

// get last navigation goal(gets nulled once it has been reached)
NavigationStatusType EngineProxy::getCurrentNavigationStatus() {
	return navigationStatus;
}

void EngineProxy::setNavigationGoal(const Pose& navigationGoal, bool latchOrientation) {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;

		std::ostringstream bodyposeIn;
		navigationGoal.serialize(bodyposeIn);

		url << "/navigation/goal/set"
			<< "?bodypose="  << stringToJSonString(bodyposeIn.str())
		    << "&latchorientation="  << boolToJSonString(latchOrientation);
		remoteEngine.httpGET(url.str(), responseStr, 5000);
	};
}

void EngineProxy::updateGlobalMaps() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/map/get?no=" << map.getGenerationNumber();
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		Map tmpSlamMap;
		tmpSlamMap.deserialize(in, ok);
		parseCharacter(in, ',', ok);
		Map tmpCostmap;
		tmpCostmap.deserialize(in, ok);
		parseCharacter(in, ',', ok);
		vector<Point> tmpHoles;
		deserializeVectorOfSerializable(in, tmpHoles,ok);

		if (ok) {
			newMapDataAvailable = true;
			map = tmpSlamMap;
			globalCostmap = tmpCostmap;
			newGlobalCostmapAvailable = true;
			darkScaryHoles = tmpHoles;

		}
	}
}

void EngineProxy::updateLocalCostmap() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/costmap/local/get?no=" << localCostmap.getGenerationNumber();
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		Map tmp;
		tmp.deserialize(in, ok);

		if (ok) {
			newLocalCostmapAvailable = true;
			localCostmap = tmp;
		}
	}
}


void EngineProxy::updateGlobalCostmap() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/costmap/global/get?no=" << globalCostmap.getGenerationNumber();
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		Map tmp;
		tmp.deserialize(in, ok);

		if (ok) {
			newGlobalCostmapAvailable = true;
			globalCostmap = tmp;
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
		url << "/trajectory/get?no=" << trajectory.getGenerationNumber();
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


void EngineProxy::updatePlan() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/plan/get?no=" << globalPlan.getGenerationNumber();
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		Trajectory tmpGlobal;
		tmpGlobal.deserialize(in, ok);
		parseCharacter(in,',', ok);
		Trajectory tmpLocal;
		tmpLocal.deserialize(in, ok);

		if (ok) {
			newTrajectoryDataAvailable = true;
			globalPlan = tmpGlobal;
			localPlan = tmpLocal;
		}
	}
}
void EngineProxy::updateNavigation() {
	if (callRemoteEngine) {
		string responseStr;
		std::ostringstream url;
		url << "/navigation/goal/get";
		remoteEngine.httpGET(url.str(), responseStr, 20000);
		std::istringstream in(responseStr);

		bool ok = true;
		// use intermediate variable since the UI thread is using the variable map, unless we have a semaphore do it quick at least
		navigationGoal.deserialize(in, ok);
		parseCharacter(in, ',', ok); // parse ","
		parseString(in, ok); // parse "status"
		parseCharacter(in, ':', ok); // parse ":"
		int statusInt = parseInt(in, ok);
		navigationStatus = (NavigationStatusType)statusInt;

		if (ok) {
			newNavigationStatusIsAvailable = true;
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

bool EngineProxy::isCostmapDataAvailable() {
	if (newLocalCostmapAvailable) {
		newLocalCostmapAvailable = false;
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


bool EngineProxy::isNavigationStatusAvailable() {
	if (newNavigationStatusIsAvailable) {
		newNavigationStatusIsAvailable = false;
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

vector<Point>& EngineProxy::getDarkScaryHoles() {
	return darkScaryHoles;
}
