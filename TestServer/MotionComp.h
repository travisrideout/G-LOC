#pragma once
#include <string.h>

class MotionComp {
public:
	struct dcsTelemetry {
		float pitch;	//-90 to +90, does not jump
		float roll;		//-180 to +180, Jumps from 180 to -180
		float yaw;		//0 to 360
		float xAccel;	//-10 to 10
		float yAccel;	//-10 to 10, Pitch force
		float zAccel;	//-10 to 10
	};	

	dcsTelemetry _dcsTelemetry;	

	MotionComp();
	void calculatePositions(dcsTelemetry *data);
	float getTargetPitchPosition();
	float getTargetRollPosition();
	float getTargetYawPosition();
	~MotionComp();

private:
	int minPitchAngle = -90;		// degrees
	int maxPitchAngle = 90;		// degrees
	int minRollAngle = -180;		// degrees
	int maxRollAngle = 180;		// degrees
	int minYawAngle = 0;			// degrees
	int maxYawAngle = 360;		// degrees

	float pitchAbsoluteOffset = 0;
	float rollAbsoluteOffset = 0;
	float yawAbsoluteOffset = 0;
	
	//axis positions	
	float targetPitchPosition = 0;
	float targetRollPosition = 0;
	float targetYawPosition = 0;

	float prev_targetPitchPosition = 0;
	float prev_targetRollPosition = 0;
	float prev_targetYawPosition = 0;

	int bufferAngle = 5;

	dcsTelemetry prev_dcsTelemetry;
};

