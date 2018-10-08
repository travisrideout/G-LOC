#include "MotorDriver.h"

MotorDriver::MotorDriver(int axis) : motorDriver(AccelStepper::DRIVER, STEP_PIN, DIR_PIN){
	
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);
	pinMode(EN_PIN, OUTPUT);

	finalDriveRatio = numPulsesPerRotation * gearboxRatio * (numTeethDrivenSprocket / numTeethDriverSprocket);

	motorDriver.setPinsInverted(false, false, true);
	motorDriver.setEnablePin(EN_PIN);

	switch (axis) {
	case axisNames::pitch:
		motorDriver.setMaxSpeed(maxPitchRate * finalDriveRatio);
		motorDriver.setAcceleration(maxPitchAccel * finalDriveRatio);
		break;
	case axisNames::roll:
		motorDriver.setMaxSpeed(maxRollRate * finalDriveRatio);
		motorDriver.setAcceleration(maxRollAccel * finalDriveRatio);
		break;
	case axisNames::yaw:
		motorDriver.setMaxSpeed(maxYawRate * finalDriveRatio);
		motorDriver.setAcceleration(maxYawAccel * finalDriveRatio);
		break;
	default:
		break;
	}	
}

void MotorDriver::sendCommand(driveMsg_union * Msg) {
	switch (Msg->driveMsg_struct.command) {
	case ENABLE:		
		enableMotor();
		break;
	case DISABLE:		
		disableMotor();
		break;
	case RESET:
		resetZero();
		break;
	case POSITION:
		isCalibrating = false;
		if (isEnabled) {
			moveTo(Msg->driveMsg_struct.position);
		}
		break;
	case MOVE:
		motorDriver.setSpeed(Msg->driveMsg_struct.speed);
		isCalibrating = true;
		break;
	default:
		break;
	}
}

void MotorDriver::enableMotor() {
	isEnabled = true;
	motorDriver.enableOutputs();
}

void MotorDriver::disableMotor() {
	isEnabled = false;
	motorDriver.disableOutputs();
}

void MotorDriver::resetZero() {
	motorDriver.setCurrentPosition(0);
}

void MotorDriver::moveTo(float target) {
	targetStepPosition = target * finalDriveRatio;

	prevStepPosition = targetStepPosition;

	motorDriver.moveTo(targetStepPosition);
}

bool MotorDriver::run() {
	if (isCalibrating) {
		return motorDriver.runSpeed();
	} else {
		return motorDriver.run();
	}	
}

MotorDriver::~MotorDriver() {
}
