/*
 * Navigator.h
 *
 *  Created on: Dec 15, 2017
 *      Author: jochenalt
 */

#ifndef PENTAPOD_SERVER_SRC_NAVIGATOR_H_
#define PENTAPOD_SERVER_SRC_NAVIGATOR_H_

// pentapod engine basics
#include "basics/spatial.h"
#include "setup.h"

// move base action client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigator {
public:
	Navigator();
	virtual ~Navigator();

	static Navigator& getInstance() { static Navigator singleton; return singleton; };

};

#endif /* PENTAPOD_SERVER_SRC_NAVIGATOR_H_ */
