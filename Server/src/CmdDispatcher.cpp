
#include <vector>
#include "setup.h"

#include "basics/logger.h"

#include "CommDef.h"
#include "core.h"

#include "Util.h"
#include "CmdDispatcher.h"
#include "Engine.h"



CommandDispatcher commandDispatcher;

using namespace std;


bool getURLParameter(vector<string> names, vector<string> values, string key, string &value) {
	for (int i = 0;i<(int)names.size();i++) {
		if (names[i].compare(key) == 0) {
			value = values[i];
			return true;
		}
	}
	return false;
}


void compileURLParameter(string uri, vector<string> &names, vector<string> &values) {
	names.clear();
	values.clear();

	std::istringstream iss(uri);
	std::string token;
	while (std::getline(iss, token, '&'))
	{
		// extract name and value of parameter
		int equalsIdx = token.find("=");
		if (equalsIdx > 0) {
			string name = token.substr(0,equalsIdx);
			string value = token.substr(equalsIdx+1);
			names.insert(names.end(), name);
			values.insert(values.end(), urlDecode(value));
		};
	}
}

CommandDispatcher::CommandDispatcher() {
}

CommandDispatcher& CommandDispatcher::getInstance() {
	return commandDispatcher;
}

string getResponse(bool ok) {
	std::ostringstream s;
	if (ok) {
		s << "\"ok\"=true";
	} else {
		s << "\"ok\"=false, \"error\":" << getLastError() << ", \"errormessage\":" << stringToJSonString(getErrorMessage(getLastError()));
	}
	string response = s.str();
	return response;
}

// central dispatcher of all url requests arriving at the webserver
// returns true, if request has been dispatched within dispatch. Otherwise the caller
// should assume that static content is to be displayed.
bool  CommandDispatcher::dispatch(string uri, string query, string body, string &response, bool &okOrNOk) {

	response = "";
	string urlPath = getPath(uri);

	vector<string> urlParamName;
	vector<string> urlParamValue;

	compileURLParameter(query,urlParamName,urlParamValue);

	// check if direct cortex command defined via URL parameter
	// example: /cortex/LED?blink
	if (hasPrefix(uri, "/cortex")) {
		LOG(DEBUG) << uri << " " << query;

		string cmd = uri.substr(string("/cortex/").length());
		for (int i = 0;i<CortexCommandDefinitionType::NumberOfCommands;i++) {
			string cortexCmdStr = string(commDef[i].name);
			if (hasPrefix(cmd, cortexCmdStr)) {
				string command = string(commDef[i].name);
				// are there any parameters ?
				if (query.length() > 0) {
					std::istringstream iss(query);
					std::string token;
					while (std::getline(iss, token, '&'))
					{
						// extract name and value of parameter
						int equalsIdx = token.find("=");
						if (equalsIdx > 0) {
							string name = token.substr(0,equalsIdx);
							string value = token.substr(equalsIdx+1);
							command += " " + name + "=" + value;
						} else {
							command += " " + token;
						}
					}
				};
				LOG(DEBUG) << "calling cortex with \"" << command << "\"";
				string cmdReply;
				engine.cortexCommand(command, cmdReply, okOrNOk);

				if (cmdReply.length() > 0)
					response += cmdReply + "";
				if (okOrNOk)
					response += "ok";
				else
					response += "failed";
				return true;
			}
		}
	}

	// check, if cortex is called via a command string
	// example /cortex/LED+blink
	if (hasPrefix(uri, "/cortex")) {
		string cortexCommand = uri.substr(string("/cortex/").length());
		LOG(DEBUG) << "calling cortex with \"" << cortexCommand << "\"";
		string cmdReply;
		engine.cortexCommand(cortexCommand, cmdReply, okOrNOk);
		response = getResponse(okOrNOk);
		return true;
	}

	// check, if TransactionExecutor is called with orchestrated calls
	if (hasPrefix(uri, "/engine/")) {
		string engineCommand = uri.substr(string("/engine/").length());

		// /engine/start?force=true or /engine/start
		if (hasPrefix(engineCommand, "turnon")) {
			LOG(DEBUG) << uri << " " << query;
			string forceParam = urlDecode(query.substr(string("force=").length()));
			bool forceParamGiven = true;
			bool force = jsonStringToBool(forceParam, forceParamGiven);
			if (forceParamGiven)
				engine.turnOn(force);
			else
				engine.turnOn();

			okOrNOk = engine.isTurnedOn();
			response = getResponse(okOrNOk);
			return true;
		}
		// /engine/stop
		else if (hasPrefix(engineCommand, "turnoff")) {
			LOG(DEBUG) << uri << " " << query;
			engine.turnOff();
			response = getResponse(!engine.isTurnedOn());
			return true;
		}
		// /engine/wakeup?bodypose={...}
		else if (hasPrefix(engineCommand, "wakeup")) {
			LOG(DEBUG) << uri << " " << query;
			string bodyPoseStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "bodypose", bodyPoseStr);

			std::stringstream paramIn(bodyPoseStr);
			string paramstr = parseString(paramIn, ok);
			Pose bodyPose;
			std::stringstream in(paramstr);
			bodyPose.deserialize(in,ok);
			if (ok)
				engine.wakeUp(bodyPose);
			response = getResponse(ok);
			return true;
		}
		// /engine/fallasleep
		else if (hasPrefix(engineCommand, "fallasleep")) {
			LOG(DEBUG) << uri << " " << query;
			engine.fallAsleep();
			response = getResponse(true);
			return true;
		}
		// /engine/alarmstart?bodypose={...}
		else if (hasPrefix(engineCommand, "terrain")) {
			LOG(DEBUG) << uri << " " << query;
			string terrainModeOnStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "on", terrainModeOnStr);
			string bodyPoseStr;
			bool terrainMode = jsonStringToBool(terrainModeOnStr, ok);
			ok = getURLParameter(urlParamName, urlParamValue, "bodypose", bodyPoseStr);
			std::stringstream paramIn (bodyPoseStr);
			string paramstr = parseString(paramIn, ok);
			std::stringstream in (paramstr);

			Pose bodyPose;
			bodyPose.deserialize(in,ok);
			if (ok)
				engine.terrainMode(terrainMode, bodyPose);
			response = getResponse(ok);
			return true;
		}
		// /engine/move?bodypose={...}&noseorientation=10&movementspeed=10&movementrotatez=10&movementdirection=10
		else if (hasPrefix(engineCommand, "move")) {
			LOG(DEBUG) << uri << " " << query;

			string bodyPoseStr, noseOrientationStr, movementSpeedStr, movementRotateZStr, movementDirectionStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "bodypose", bodyPoseStr);
			ok = ok && getURLParameter(urlParamName, urlParamValue, "noseorientation", noseOrientationStr);
			ok = ok && getURLParameter(urlParamName, urlParamValue, "movementspeed", movementSpeedStr);
			ok = ok && getURLParameter(urlParamName, urlParamValue, "movementrotatez", movementRotateZStr);
			ok = ok && getURLParameter(urlParamName, urlParamValue, "movementdirection", movementDirectionStr);

			std::stringstream paramIn(bodyPoseStr);
			string paramstr = parseString(paramIn, ok);
			std::stringstream in(paramstr);
			Pose bodyPose;
			bodyPose.deserialize(in,ok);

			realnum noseOrientation = stringToFloat(noseOrientationStr, ok);
			realnum movementSpeed = stringToFloat(movementSpeedStr, ok);
			realnum movementRotateZ = stringToFloat(movementRotateZStr, ok);
			realnum movementDirection = stringToFloat(movementDirectionStr, ok);
			if (ok)
				engine.setTargetMovement(bodyPose, noseOrientation, movementSpeed, movementRotateZ, movementDirection);
			response = getResponse(ok);
			return true;
		}
		// /engine/setgaitmode?gaitmode=1
		else if (hasPrefix(engineCommand, "setgaitmode")) {
			LOG(DEBUG) << uri << " " << query;
			string gaitModeStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "gaitmode", gaitModeStr);
			GaitModeType gaitMode = (GaitModeType)stringToInt(gaitModeStr, ok);
			if (ok)
				engine.setTargetGaitMode(gaitMode);
			response = getResponse(ok);
			return true;
		}
		// engine/setfrontleg?bodypose={...}
		else if (hasPrefix(engineCommand, "setfrontleg")) {
			LOG(DEBUG) << uri << " " << query;
			string frontLegPoseStr;
			bool ok = getURLParameter(urlParamName, urlParamValue, "frontleg", frontLegPoseStr);
			std::stringstream paramIn (frontLegPoseStr);
			string paramstr = parseString(paramIn, ok);
			std::stringstream in (paramstr);
			Point frontLeg;
			frontLeg.deserialize(in,ok);
			if (ok)
				engine.setTargetFrontLegPose(frontLeg);
			response = getResponse(ok);
			return true;
		}
		// engine/getstate
		else if (hasPrefix(engineCommand, "getstate")) {
			LOG(DEBUG) << uri << " " << query;
			EngineState state;
			engine.getState(state);
			std::stringstream out;
			state.serialize(out);
			response = out.str() + "," + getResponse(true);
			return true;
		}
	}

	okOrNOk = false;
	return false;
}





