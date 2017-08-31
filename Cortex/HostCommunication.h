/* 
* HostCommunication.h
*
* Created: 26.06.2016 22:16:52
* Author: JochenAlt
*/


#ifndef __HOSTCOMMUNICATION_H__
#define __HOSTCOMMUNICATION_H__

class HostCommunication;
extern HostCommunication hostComm;

#include "SerialCommand.h"
class HostCommunication
{
public:
	HostCommunication();

	void setup();
	void loop();


	SerialCommand sCmd;
	uint32_t setupTime;
}; //HostCommunication

#endif //__HOSTCOMMUNICATION_H__
