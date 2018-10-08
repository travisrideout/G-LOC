#pragma once


#include <WiFi.h>
#include <WiFiUdp.h>
#include <Rideout.h>
#include "MotionComp.h"
#include "MotorDriver.h"
#include <Metro.h>

#define AXIS 1	// select axis here: pitch = 0, roll = 1, yaw = 2. Roll is master

//prototypes

//pin definitions
const int ESTOP_PIN = 26;
const int RESET_PIN = 25;
const int AXIS_SELECT_PIN = 33;
const int AXIS_SPEED_PIN = 32;
const int AXIS_FORWARD_PIN = 35;
const int AXIS_REVERSE_PIN = 34;

//constants
const unsigned int rollPort = 5010;
const unsigned int pitchPort = 5011;
const unsigned int yawPort = 5012;

IPAddress pitch_local_IP(192, 168, 200, 201);
IPAddress roll_local_IP(192, 168, 200, 200);
IPAddress yaw_local_IP(192, 168, 200, 202);
IPAddress gateway(192, 168, 200, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 8, 8); //optional

char dcsBuffer[255];
MotorDriver::driveMsg_union pitchMsg;
MotorDriver::driveMsg_union rollMsg;
MotorDriver::driveMsg_union yawMsg;

//objects
WiFiUDP udp;
MotionComp m;
MotorDriver motorDriver(AXIS);

//Timers
Metro inputTimer = Metro(100);

//working variables
int packetSize = 0;
int packetLength = 0;
long prev_time = 0;
int dotCounter = 0;
int offset = 8;
String stringBuffer;
String number;
long lastFrameTime = 0;
int fps = 0;

byte selectedAxis = 3;	//0=pitch, 1=roll, 2=yaw, 3=off
uint16_t analogAxisSelect = 0;
uint16_t axisSpeed = 0;

//Activators
bool isEStopped = false;
bool isCalibrating = false;