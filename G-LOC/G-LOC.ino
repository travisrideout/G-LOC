/*
 Name:		G_LOC.ino
 Created:	9/30/2018 9:30:30 AM
 Author:	Travi
*/
#include <MultiStepper.h>
#include <AccelStepper.h>
#include "G-LOC.h"

/*
Assumptions:
Roll is master axis

*/

/*
NOTES:
How do angles roll over? large jump or count back down



*/

/* TODO
handle buttons for e-stop, resets, moves, etc
track connection states

*/

void setup() {
	Serial.begin(115200);
	pinMode(ESTOP_PIN, INPUT_PULLUP);
	pinMode(RESET_PIN, INPUT_PULLUP);
	pinMode(AXIS_SELECT_PIN, INPUT);
	pinMode(AXIS_SPEED_PIN, INPUT);
	pinMode(AXIS_FORWARD_PIN, INPUT_PULLUP);
	pinMode(AXIS_REVERSE_PIN, INPUT_PULLUP);

#if AXIS == 0
	WiFi.config(pitch_local_IP, gateway, subnet, primaryDNS);
	initWiFi();
	udp.begin(pitchPort);
#elif AXIS == 1
	WiFi.config(roll_local_IP, gateway, subnet, primaryDNS);
	initWiFi();
	udp.begin(rollPort);
#elif AXIS == 2
	WiFi.config(yaw_local_IP, gateway, subnet, primaryDNS);
	initWiFi();
	udp.begin(yawPort);
#else
	Serial.println("Incorrect axis assignment");
	while (1) {
	}
#endif			

	prev_time = millis();

	Serial.println("Setup complete");
	Serial.print("Waiting for data..");
}

void loop() {

#if AXIS == 0
	packetSize = udp.parsePacket();
	if (packetSize == sizeof(pitchMsg.msg_bytes)) {
		// read the packet into packetBufffer
		udp.read(pitchMsg.msg_bytes, sizeof(pitchMsg.msg_bytes));
		motorDriver.sendCommand(&pitchMsg);
	}

#elif AXIS == 1
	if (inputTimer.check()) {
		if (!digitalRead(ESTOP_PIN)) {
			isEStopped = true;
			Serial.println("Estop Active!");
			pitchMsg.driveMsg_struct.command = MotorDriver::DISABLE;
			rollMsg.driveMsg_struct.command = MotorDriver::DISABLE;
			yawMsg.driveMsg_struct.command = MotorDriver::DISABLE;
		} else if(isEStopped){
			isEStopped = false;
			pitchMsg.driveMsg_struct.command = MotorDriver::ENABLE;
			rollMsg.driveMsg_struct.command = MotorDriver::ENABLE;
			yawMsg.driveMsg_struct.command = MotorDriver::ENABLE;
		}

		analogAxisSelect = analogRead(AXIS_SELECT_PIN);
		if (analogAxisSelect > 1000) {
			selectedAxis = 3;
			isCalibrating = false;
		} else if (analogAxisSelect > 750) {
			selectedAxis = 2;
		} else if (analogAxisSelect > 500) {
			selectedAxis = 1;
		} else {
			selectedAxis = 0;
		}
		if (selectedAxis < 3 && !isEStopped 
			&& pitchMsg.driveMsg_struct.command != MotorDriver::ENABLE) {

			Serial.println("Calibrating Drives!");
			isCalibrating = true;
			axisSpeed = analogRead(AXIS_SPEED_PIN);

			pitchMsg.driveMsg_struct.command = MotorDriver::MOVE;
			pitchMsg.driveMsg_struct.speed = 0;

			rollMsg.driveMsg_struct.command = MotorDriver::MOVE;
			rollMsg.driveMsg_struct.speed = 0;

			yawMsg.driveMsg_struct.command = MotorDriver::MOVE;
			yawMsg.driveMsg_struct.speed = 0;

			axisSpeed = 0;
			if (!digitalRead(AXIS_FORWARD_PIN)) {
				axisSpeed = analogRead(AXIS_SPEED_PIN);				
			} else if (!digitalRead(AXIS_REVERSE_PIN)) {
				axisSpeed = -analogRead(AXIS_SPEED_PIN);				
			}

			if (selectedAxis == 0) {
				pitchMsg.driveMsg_struct.speed = axisSpeed;
			} else if (selectedAxis == 1) {
				rollMsg.driveMsg_struct.speed = axisSpeed;
			} else if (selectedAxis == 2) {
				yawMsg.driveMsg_struct.speed = axisSpeed;
			}

			if (!digitalRead(RESET_PIN)) {
				if (selectedAxis == 0) {
					pitchMsg.driveMsg_struct.command = MotorDriver::RESET;
				} else if (selectedAxis == 1) {
					rollMsg.driveMsg_struct.command = MotorDriver::RESET;
				} else if (selectedAxis == 2) {
					yawMsg.driveMsg_struct.command = MotorDriver::RESET;
				}
			}
		}

		if (isEStopped || isCalibrating) {
			Serial.println("Sending User Input Message");
			// send a message 
			udp.beginPacket(pitch_local_IP, pitchPort);
			udp.write(pitchMsg.msg_bytes, sizeof(pitchMsg.msg_bytes));
			udp.endPacket();

			//send local message
			motorDriver.sendCommand(&rollMsg);

			// send a message 
			udp.beginPacket(yaw_local_IP, yawPort);
			udp.write(yawMsg.msg_bytes, sizeof(yawMsg.msg_bytes));
			udp.endPacket();
		}
	}

	if (!isEStopped && !isCalibrating) {
		packetSize = udp.parsePacket();
		if (packetSize == 50) {
			/*Serial.print("Received packet of size ");
			Serial.println(packetSize);
			Serial.print("From ");
			remoteIp = udp.remoteIP();
			Serial.print(remoteIp);
			Serial.print(", port ");
			Serial.println(udp.remotePort());*/

			// read the packet into packetBufffer
			packetLength = udp.read(dcsBuffer, 255);
			if (packetLength > 0) {
				dcsBuffer[packetLength] = 0;
			}
			/*Serial.println("Contents:");
			Serial.println(packetBuffer);*/

			if (dcsBuffer[0] == '$') {
				//Serial.print("New data: ");

				stringBuffer = dcsBuffer;
				number = stringBuffer.substring(1 + 0 * offset);
				m._dcsTelemetry.pitch = number.toFloat();
				number = stringBuffer.substring(1 + 1 * offset);
				m._dcsTelemetry.roll = number.toFloat();
				number = stringBuffer.substring(1 + 2 * offset);
				m._dcsTelemetry.yaw = number.toFloat();
				number = stringBuffer.substring(1 + 3 * offset);
				m._dcsTelemetry.xAccel = number.toFloat();
				number = stringBuffer.substring(1 + 4 * offset);
				m._dcsTelemetry.yAccel = number.toFloat();
				number = stringBuffer.substring(1 + 5 * offset);
				m._dcsTelemetry.zAccel = number.toFloat();

				printParsedDCS();

				m.calculatePositions(&m._dcsTelemetry);

				pitchMsg.driveMsg_struct.command = MotorDriver::POSITION;
				pitchMsg.driveMsg_struct.position = m.getTargetPitchPosition();

				rollMsg.driveMsg_struct.command = MotorDriver::POSITION;
				rollMsg.driveMsg_struct.position = m.getTargetRollPosition();

				yawMsg.driveMsg_struct.command = MotorDriver::POSITION;
				yawMsg.driveMsg_struct.position = m.getTargetYawPosition();

				// send a message 
				udp.beginPacket(pitch_local_IP, pitchPort);
				udp.write(pitchMsg.msg_bytes, sizeof(pitchMsg.msg_bytes));
				udp.endPacket();

				//send local message
				motorDriver.sendCommand(&rollMsg);

				// send a message 
				udp.beginPacket(yaw_local_IP, yawPort);
				udp.write(yawMsg.msg_bytes, sizeof(yawMsg.msg_bytes));
				udp.endPacket();
			}
		}		
	} 

#elif AXIS == 2
	packetSize = udp.parsePacket();
	if (packetSize == sizeof(pitchMsg.msg_bytes)) {
		// read the packet into packetBufffer
		udp.read(pitchMsg.msg_bytes, sizeof(pitchMsg.msg_bytes));
		motorDriver.sendCommand(&pitchMsg);
	}
#endif	

	motorDriver.run();	
	
	//TODO: wrap in !connected if statment. track connection status
	if (millis() - prev_time > 1000) {
		prev_time = millis();
		Serial.print(".");
		dotCounter++;
		if (dotCounter > 10) {
			dotCounter = 0;
			Serial.println("!");
			Serial.print("Waiting for client..");
		}
	}	
}

void initWiFi() {
	WiFi.begin(WLAN_SSID, WLAN_PASS);

	Serial.print("Connecting to WiFi..");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	printWifiStatus();
}

void printWifiStatus() {
	Serial.println("");
	Serial.println("Connected to the WiFi network");

	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print WiFi IP address:
	Serial.print("IP Address: ");
	Serial.println(WiFi.localIP());

	// print WiFi Gateway
	Serial.print("IP Gateway: ");
	Serial.println(WiFi.gatewayIP());

	// print WiFi Subnet
	Serial.print("IP Subnet: ");
	Serial.println(WiFi.subnetMask());

	// print the received signal strength:
	Serial.print("signal strength (RSSI):");
	Serial.print(WiFi.RSSI());
	Serial.println(" dBm");

	Serial.println("");	
}

void printParsedDCS() {
	Serial.print(m._dcsTelemetry.pitch);
	Serial.print(", ");
	Serial.print(m._dcsTelemetry.roll);
	Serial.print(", ");
	Serial.print(m._dcsTelemetry.yaw);
	Serial.print(", ");
	Serial.print(m._dcsTelemetry.xAccel);
	Serial.print(", ");
	Serial.print(m._dcsTelemetry.yAccel);
	Serial.print(", ");
	Serial.print(m._dcsTelemetry.zAccel);
	Serial.print(", ");

	Serial.print("FPS = ");
	Serial.print(calcFPS());

	Serial.println("");
}

int calcFPS() {
	if (micros() != lastFrameTime) {
		fps = 1000000 / (micros() - lastFrameTime);
	}	
	lastFrameTime = micros();
	return fps;
}

