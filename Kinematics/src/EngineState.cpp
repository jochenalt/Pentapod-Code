#include <cstdarg>
#include "EngineState.h"
#include "Engine.h"

std::ostream& EngineState::serialize(std::ostream &out) const {
	out << "{ \"bp\":";
	currentBodyPose.serialize(out);
	out << ",\"la\":";
	legAngles.serialize(out);
	out << ",\"flp\":";
	frontLegPose.serialize(out);
	out << ",\"odom\":";
	currentOdomPose.serialize(out);
	out << ",\"map\":";
	currentMapPose.serialize(out);
	out << ",\"baselink\":";
	baseLinkInMapFrame.serialize(out);
	out << ",\"hp\":";
	hipPoseWorld.serialize(out);
	out << ",\"gp\":";
	groundPoints.serialize(out);
	out << ",\"tp\":";
	toePointsWorld.serialize(out);
	out << ",\"grp\":";
	currentGaitRefPoints.serialize(out);

	out << ",\"ito\":" << boolToJSonString(isTurnedOn);
	out << ",\"ms\":" << floatToString(currentSpeed);
	out << ",\"mrz\":" << floatToString(currentAngularSpeed,3);
	out << ",\"md\":" << floatToString(currentWalkingDirection,3);
	out << ",\"no\":" << floatToString(currentNoseOrientation,3);
	out << ",\"gm\":" << intToString((int)currentGaitMode);
	out << ",\"em\":" << intToString((int)engineMode);
	out << ",\"cs\":" << floatToString(currentScaryness,3);

	out << ",\"fog\":";
	serializeArrayOfPrimitives(footOnGroundFlag, NumberOfLegs, out);
	out << ",\"lp\":";
	serializeArrayOfPrimitives(legPhase, NumberOfLegs, out);
	out << ",\"fa\":";
	serializeArrayOfPrimitives(footAngle, NumberOfLegs, out);

	out << "}";
	return out;
}

std::istream& EngineState::deserialize(std::istream &in) {
    if (in) {
    	bool ok = true;
    	parseCharacter(in, '{', ok);

    	parseString(in, ok); // "bodypose"
    	parseCharacter(in, ':', ok);
    	currentBodyPose.deserialize(in, ok);
    	parseCharacter(in, ',', ok);

    	parseString(in, ok); // "legangles"
    	parseCharacter(in, ':', ok);
    	legAngles.deserialize(in, ok);
    	parseCharacter(in, ',', ok);

    	parseString(in, ok); // "frontlegpose"
    	parseCharacter(in, ':', ok);
    	frontLegPose.deserialize(in, ok);
    	parseCharacter(in, ',', ok);

    	parseString(in, ok); // "odom"
    	parseCharacter(in, ':', ok);
    	currentOdomPose.deserialize(in, ok);
    	parseCharacter(in, ',', ok);

    	parseString(in, ok); // "map"
    	parseCharacter(in, ':', ok);
    	currentMapPose.deserialize(in, ok);
    	parseCharacter(in, ',', ok);

    	parseString(in, ok); // "baselink"
    	parseCharacter(in, ':', ok);
    	baseLinkInMapFrame.deserialize(in, ok);
    	parseCharacter(in, ',', ok);


       	parseString(in, ok); // "hippose"
       	parseCharacter(in, ':', ok);
       	hipPoseWorld.deserialize(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "groundpoints"
       	parseCharacter(in, ':', ok);
       	groundPoints.deserialize(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "toepoints"
       	parseCharacter(in, ':', ok);
       	toePointsWorld.deserialize(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "gaitrefpoints"
       	parseCharacter(in, ':', ok);
       	currentGaitRefPoints.deserialize(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "isturnedon"
       	parseCharacter(in, ':', ok);
       	isTurnedOn = parseBool(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "movementspeed"
       	parseCharacter(in, ':', ok);
       	currentSpeed = parseFloat(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "movementrotatez"
       	parseCharacter(in, ':', ok);
       	currentAngularSpeed = parseFloat(in,ok );
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "movementdirection"
       	parseCharacter(in, ':', ok);
       	currentWalkingDirection = parseFloat(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "noseorientation"
       	parseCharacter(in, ':', ok);
       	currentNoseOrientation = parseFloat(in, ok );
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "gaitmode"
       	parseCharacter(in, ':', ok);
       	currentGaitMode = (GaitModeType)parseInt(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "engineMode"
       	parseCharacter(in, ':', ok);
       	engineMode = (GeneralEngineModeType)parseInt(in, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "currentScariness"
       	parseCharacter(in, ':', ok);
       	currentScaryness = parseFloat(in, ok );
       	parseCharacter(in, ',', ok);


       	parseString(in, ok); // "footonground"
       	parseCharacter(in, ':', ok);
       	int len;
       	deserializeArrayOfPrimitives(in, footOnGroundFlag, len, ok);
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "legphase"
       	parseCharacter(in, ':', ok);
       	int dummy[NumberOfLegs];
       	deserializeArrayOfPrimitives(in, dummy, len, ok);
       	for (int i = 0;i<NumberOfLegs;i++)
       		legPhase[i] = (LegGaitPhase)dummy[i];
       	parseCharacter(in, ',', ok);

       	parseString(in, ok); // "footangle"
       	parseCharacter(in, ':', ok);
       	deserializeArrayOfPrimitives(in, footAngle, len, ok);
       	parseCharacter(in, '}', ok);

    }
    return in;
}

