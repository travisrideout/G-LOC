#pragma once
#include <AccelStepper.h>

class MotorDriver {
public:
	//commands
	static const char ENABLE = 'E';		//Enable Drive
	static const char DISABLE = 'D';	//Disable Drive
	static const char RESET = 'R';		//Reset Zero Point
	static const char POSITION = 'P';	//Move to position
	static const char MOVE = 'M';		//Move at rate in direction
	
	typedef struct _driveMsg {
		char command;
		float position;
		float speed;
	}driveMsg;

	typedef union _driveMsg_union {
		driveMsg driveMsg_struct;
		uint8_t msg_bytes[9];
	}driveMsg_union;

	MotorDriver(int axis); 	
	void sendCommand(driveMsg_union *Msg);
	bool run();
	~MotorDriver();
private:

	//prototypes
	void enableMotor();
	void disableMotor();
	void resetZero();
	void moveTo(float target);	

	enum axisNames {
		pitch,
		roll,
		yaw
	};

	//constants
	const int STEP_PIN = 12;
	const int DIR_PIN = 14;
	const int EN_PIN = 27;
	const int MIN_PULSE_WIDTH = 20;	//microseconds		

	//Drive Train variables
	const int numPulsesPerRotation = 800;
	const int gearboxRatio = 10;
	const int numTeethDriverSprocket = 18;
	const int numTeethDrivenSprocket = 36;
	float finalDriveRatio = 0;

	//limits
	int maxPitchRate = 180;		// degree/sec
	int maxRollRate = 270;		// degree/sec
	int maxYawRate = 90;		// degree/sec

	int maxPitchAccel = 180;	// degree/sec^2
	int maxRollAccel = 270;		// degree/sec^2
	int maxYawAccel = 90;		// degree/sec^2	

	//Working Variables
	long targetStepPosition = 0;
	long prevStepPosition = 0;
	boolean isEnabled = false;
	bool isCalibrating = false;
	
	AccelStepper motorDriver;
};

