#include "MotionComp.h"

/*
initialize motion comp object
disable e-stop
enable axis'
pass a dcs telemetry frame
calculate desired output angles
	set goal as attitude first modified by applied G force bounded by limits
	think in terms of applied G force with negative z world vector of -1 as max
	
calculate desired motor outputs
	subtract previous position from desired position to get delta
	angle wrap around should be handled
	set direction and send pulses 

Motor command controller
	direction
	pulses togo
	pulse rate
	lastTime

*/

MotionComp::MotionComp() {

}

void MotionComp::calculatePositions(dcsTelemetry *data) {
	//TODO: need to deal with pitch angle issue: crossing nose down or nose up results in roll

	// Pitch ----------------------------------------------
	
	if (targetPitchPosition > maxPitchAngle) {
		targetPitchPosition = minPitchAngle + (targetPitchPosition - maxPitchAngle);
	} else if (targetPitchPosition < minPitchAngle) {
		targetPitchPosition = maxPitchAngle - (minPitchAngle - targetPitchPosition);
	}	

	targetPitchPosition = data->pitch / 2 + data->xAccel * 9;

	// Roll ------------------------------------------------
	if (prev_dcsTelemetry.roll > maxRollAngle - bufferAngle
		&& data->roll < minRollAngle + bufferAngle) {
		rollAbsoluteOffset += 360;
	} else if (prev_dcsTelemetry.roll < minRollAngle + bufferAngle
		&& data->roll > maxRollAngle - bufferAngle) {
		rollAbsoluteOffset -= 360;
	}

	targetRollPosition = data->roll + rollAbsoluteOffset;


	// Yaw -------------------------------------------------
	if (prev_dcsTelemetry.yaw > maxYawAngle - bufferAngle 
		&& data->yaw < minYawAngle + bufferAngle) {
		yawAbsoluteOffset += maxYawAngle;
	} else if (prev_dcsTelemetry.yaw < minYawAngle + bufferAngle
		&& data->yaw > maxYawAngle - bufferAngle) {
		yawAbsoluteOffset -= maxYawAngle;
	}

	targetYawPosition = data->yaw + yawAbsoluteOffset;

	//save this dcs data frame
	memcpy(&prev_dcsTelemetry, data, sizeof(prev_dcsTelemetry));
}

float MotionComp::getTargetPitchPosition() {
	return targetPitchPosition;
}

float MotionComp::getTargetRollPosition() {
	return targetRollPosition;
}

float MotionComp::getTargetYawPosition() {
	return targetYawPosition;
}

MotionComp::~MotionComp() {
}
