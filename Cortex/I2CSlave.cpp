/*
 * BinaryCommandInterface.cpp
 *
 *  Created on: 14.08.2017
 *      Author: JochenAlt
 */

#include <I2CSlave.h>
#include "CortexComPackage.h"
#include "Controller.h"
#include "core.h"
#include "OrientationSensor.h"
#include "PowerVoltage.h"
#include "BotMemory.h"

I2CSlave i2cSlave;

extern HardwareSerial* cmdSerial;


I2CSlave& I2CSlave::getInstance() {
	return i2cSlave;
}


void I2CSlave::onBlockReceive(size_t howManyBytes) {
	// first byte is register, which is used as block indicator:
	// first 4 bits indicates the total number of blocks, second 4 bits indicate the blockno
	// therefore, we can send a max array of 16*32 = 512 bytes
	int adr = wire->read();
	if (adr != 255) {
		int blockNo = adr & 0xF;
		int numberOfBlocks = adr >> 4;

		int posInBuffer = blockNo*32;
		// cmdSerial->print("onReceiveBlock:(");
		// cmdSerial->print(adr);
		// cmdSerial->print(",");
		// cmdSerial->print(posInBuffer);
		// cmdSerial->print(")");

		while (wire->available() > 0) {
			int c = wire->read();
			receiveBuffer[posInBuffer] = (uint8_t)c;
			// cmdSerial->print(c);
			// cmdSerial->print(':');
			// cmdSerial->print(posInBuffer);
			// cmdSerial->print(' ');

			posInBuffer++;
		}
		// cmdSerial->println("#");
		if (blockNo == numberOfBlocks-1) {
			receiveBufferLen = blockNo*32 + howManyBytes-1; // howManyBytes always contains the register-byte

			// wire->write(10);

			// set semaphor to indicate that main loop can execute this request.
			requestPending = true;
			/*
			cmdSerial->print("final:#");
			for (unsigned int i = 0;i<receiveBufferLen;i++) {
				cmdSerial->print((int)receiveBuffer[i]);
				cmdSerial->print(' ');
			}
			cmdSerial->println("#");
			*/
		}
	}
}

extern void headlessSetup();


void I2CSlave::executeRequest() {

	/*
	cmdSerial->print("onReceive:(");
	cmdSerial->print(receiveBufferLen);
	cmdSerial->print(")#");
	for (unsigned int i = 0;i<receiveBufferLen;i++) {
		int c = receiveBuffer[i];
		cmdSerial->print(c);
		cmdSerial->print(' ');
	}
	cmdSerial->println("#");
	*/

	// timeout is twice the time required by transfer
	int bytesRead = receiveBufferLen;

	bool ok = (bytesRead == Cortex::RequestPackageData::Size);

	if (ok) {
		for (unsigned int i = 0;i<receiveBufferLen;i++)
			request.data[i] = receiveBuffer[i];

		Cortex::Command cmd;
		float angles[NumberOfLimbs*NumberOfLegs];
		int duration_ms;
		uint32_t now = millis();

		ok = Cortex::ComPackage::readRequest(request, cmd, angles, duration_ms);
		if (ok) {
			switch (cmd) {
			case Cortex::SETUP:
				headlessSetup();
				break;
			case Cortex::ENABLE:
				controller.enable();
				break;
			case Cortex::DISABLE:
				controller.disable();
				break;
			case Cortex::MOVE: {
				LimbAnglesType legAngles;
				if (memory.logServo()) {
					cmdSerial->print("MOVE(");
					cmdSerial->print(millis()-now);
					cmdSerial->print("ms)");
				}

				for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
					Leg& leg = controller.getLeg(legNo);
					if (memory.logServo())
						cmdSerial->print('(');
					for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
						float angle = angles[limbNo + legNo*NumberOfLimbs];
						legAngles[limbNo] = angle;
						if (memory.logServo()) {
							if (limbNo > 0)
								cmdSerial->print(' ');
							cmdSerial->print(angle,1);
						}
					}
					if (memory.logServo()) {
						cmdSerial->print(' ');
						cmdSerial->print(leg.getDistance());
						cmdSerial->print("mm");
						cmdSerial->print(")");
					}
					leg.setAngles(legAngles, duration_ms);
				}

				static TimePassedBy timer(2000);
				if (timer.isDue() && memory.logServo()) {
					cmdSerial->print("STATUS(");
					for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
						cmdSerial->print('(');
						Leg& leg = controller.getLeg(legNo);
						float highVoltage = leg.servos[THIGH].getVoltage();
						float lowVoltage = leg.servos[FOOT].getVoltage();
						cmdSerial->print(highVoltage,1);
						cmdSerial->print("/");
						cmdSerial->print(lowVoltage,1);
						cmdSerial->print("V ");

						cmdSerial->print((int)leg.servos[0].stat());
						cmdSerial->print((int)leg.servos[1].stat());
						cmdSerial->print((int)leg.servos[2].stat());
						cmdSerial->print((int)leg.servos[3].stat());
						cmdSerial->print(')');
					}
					cmdSerial->print(')');
				}


				break;
			}
			case Cortex::GET:
				// do nothing, just return current state
				if (memory.logServo()) {
					cmdSerial->print("GET(");
					cmdSerial->print(millis()-now);
					cmdSerial->print("ms)");
				}
				break;
			}

			Cortex::Status status;
			if (controller.isEnabled())
				status = Cortex::ENABLED;
			else
				status = Cortex::DISABLED;

			uint8_t servoStatus[NumberOfLegs*NumberOfLimbs];
			uint8_t distance[NumberOfLegs];
			for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
				Leg& leg = controller.getLeg(legNo);
				distance[legNo] = leg.getDistance();
				for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
					float angle = leg.servos[limbNo].getCurrentAngle();
					angles[limbNo + legNo*NumberOfLimbs] = angle;
					ServoStatusType stat = leg.getStatus(limbNo);
					servoStatus[limbNo + legNo*NumberOfLimbs] = stat;
					if (stat != SERVO_STAT_OK) {
						cmdSerial->print("servo ");
						cmdSerial->print(limbNo);
						cmdSerial->print(" of leg ");
						cmdSerial->print(legNo);
						cmdSerial->print(" failed with status ");
						cmdSerial->println(stat);
					}
				}
			}

			// fetch IMU now in order to return most recent data for tilt compensation
			float imuX,imuY;
			uint8_t newSystem, newGyro, newAcc;
			orientationSensor.fetchData();
			orientationSensor.getData(imuX, imuY, newSystem, newGyro, newAcc);
			int imuStatus = newSystem*100 + newGyro*10 + newAcc;

			if (memory.logServo()) {
				cmdSerial->print("IMU((");
				cmdSerial->print(millis()-now);
				cmdSerial->print("ms)");
				cmdSerial->print(imuX,1);
				cmdSerial->print(',');
				cmdSerial->print(imuY,1);
				cmdSerial->print(',');
				cmdSerial->print(imuStatus);
				cmdSerial->print(')');
			}

			// create response
			ok = Cortex::ComPackage::createResponse(
					status, angles, distance, servoStatus,
					imuX, imuY, imuStatus,
					voltage.getHighVoltage(),
					controller.loopDuration_ms(),
					response);

			if (cmd == Cortex::MOVE) {
				controller.getCommunicationDuration_us() = ((millis()-now)*1000 + controller.getCommunicationDuration_us())/2;
				controller.adaptSynchronisation(now);
			}

			if (memory.logServo()) {
				cmdSerial->print("END( ");
				cmdSerial->print(millis()-now);
				cmdSerial->println("ms)");
			}

			// set semaphore to indicate that response can be sent within main loop
			responsePending = true;
		} else {
			cmdSerial->print("invalid I2C request(");
			cmdSerial->print(getLastError());
			cmdSerial->print(")");

		}
	}

}

// this listener is called when a I2C request comes in
void I2CSlave::onRequest() {
	// if a response has been prepared, send it to the master
	if (responsePending) {
		wire->write(response.data, Cortex::ResponsePackageData::Size);
		responsePending = false;
	}
	else {
		// we havent prepared a response yet, tell the master
		wire->write(Cortex::NotYetReadyMagicNumber); // not yet a response there
	}
}

void I2CSlave::setup(i2c_t3* myWire) {
	wire = myWire;
	wire ->onReceive([](size_t howMany){ i2cSlave.onBlockReceive(howMany);}); 	// register receive event
	wire ->onRequest([](){ i2cSlave.onRequest();}); 							// register request event
	requestPending = false;
	responsePending = false;
}

bool I2CSlave::loop() {
	if (requestPending == true) {
		requestPending = false;
		executeRequest();
		return true;
	}
	return false;
}
