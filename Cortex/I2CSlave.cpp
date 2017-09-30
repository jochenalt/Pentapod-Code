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

		// collect IMU data
		float imuX,imuY,zAccel;
		uint8_t newSystem, newGyro, newAcc;
		orientationSensor.getData(imuX, imuY, zAccel, newSystem, newGyro, newAcc);
		int imuStatus = newSystem*100 + newGyro*10 + newAcc;

		Cortex::Command cmd;
		float angles[NumberOfLimbs*NumberOfLegs];
		int duration_ms;
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
				controller.adaptSynchronisation();
				LimbAnglesType legAngles;
				cmdSerial->print("MOVE ");
				for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
					Leg& leg = controller.getLeg(legNo);
					cmdSerial->print('(');
					for (int limbNo = 0;limbNo<NumberOfLimbs;limbNo++) {
						float angle = angles[limbNo + legNo*NumberOfLimbs];
						legAngles[limbNo] = angle;
						if (limbNo > 0)
							cmdSerial->print(' ');
						cmdSerial->print((int)angle);
					}
					cmdSerial->print(' ');
					cmdSerial->print(leg.getDistance());
					cmdSerial->print("mm");
					cmdSerial->print(")");

					leg.setAngles(legAngles, duration_ms);
				}
				cmdSerial->print("IMU(");
				cmdSerial->print(imuX,1);
				cmdSerial->print(',');
				cmdSerial->print(imuY,1);
				cmdSerial->print(',');
				cmdSerial->print(imuStatus);
				cmdSerial->println(')');

				static TimePassedBy timer(1000);
				if (timer.isDue()) {
					cmdSerial->print("STATUS(");
					for (int legNo = 0;legNo<NumberOfLegs;legNo++) {
						cmdSerial->print('(');
						Leg& leg = controller.getLeg(legNo);
						float voltage = leg.servos[FOOT].getVoltage();
						cmdSerial->print(voltage,1);
						cmdSerial->print("V ");

						cmdSerial->print((int)leg.servos[0].stat());
						cmdSerial->print((int)leg.servos[1].stat());
						cmdSerial->print((int)leg.servos[2].stat());
						cmdSerial->print((int)leg.servos[3].stat());
						cmdSerial->print(')');
					}
					cmdSerial->println(')');
				}

				break;
			}
			case Cortex::GET:
				// do nothing, just return current state
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
					servoStatus[limbNo + legNo*NumberOfLimbs] = leg.getStatus(limbNo);
				}
			}



			// create response
			ok = Cortex::ComPackage::createResponse(
					status, angles, distance, servoStatus,
					imuX, imuY, imuStatus,
					voltage.get14Voltage(),
					controller.looptime(),
					response);

			// set semaphore to indicate that response can be sent within main loop
			responsePending = true;
		} else {
			cmdSerial->print("invalid I2C request(");
			cmdSerial->print(getLastError());
			cmdSerial->print(")");

		}
	}
}

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

void I2CSlave::loop() {
	if (requestPending == true) {
		requestPending = false;
		executeRequest();
	}
}
