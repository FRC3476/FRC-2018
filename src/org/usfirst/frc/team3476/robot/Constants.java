package org.usfirst.frc.team3476.robot;

public final class Constants {

	public static final boolean LOGGING = true;

	// Other
	public static final double SensorTicksPerMotorRotation = 4096;
	public static final double ExpectedCurrentTolerance = 0;
	public static final double ExpectedRPMTolerance = 0;
	public static final double ExpectedPositionTolerance = 0;
	// Driving
	public static final double HighDriveSpeed = 185;
	public static final double LowDriveSpeed = 95;
	public static final double kRightHighP = 0.03;
	public static final double kRightHighD = 0;
	public static final double kRightHighF = 0.035;
	public static final double kRightHighFIntercept = 0;
	public static final double kRightHighA = 0;
	public static final double kRightLowP = 0.1;
	public static final double kRightLowD = 0.1;
	public static final double kRightLowF = 0.05763730970902943999708309631717;
	public static final double kRightLowFIntercept = 0;
	public static final double kRightLowA = 0;

	public static final double kLeftHighP = 0.03;
	public static final double kLeftHighD = 0;
	public static final double kLeftHighF = 0.035;
	public static final double kLeftHighFIntercept = 0;
	public static final double kLeftHighA = 0;
	public static final double kLeftLowP = 0.1;
	public static final double kLeftLowD = 0;
	public static final double kLeftLowF = 0.05763730970902943999708309631717;
	public static final double kLeftLowFIntercept = 0;
	public static final double kLeftLowA = 0;

	public static final double kRightAutoP = 0.12;
	public static final double kRightAutoD = 0.7;
	public static final double kRightAutoF = 0.035;
	public static final double kLeftAutoP = 0.12;
	public static final double kLeftAutoD = 0.7;
	public static final double kLeftAutoF = 0.035;
	public static final double TeleopAccLimit = 120;
	public static final double TeleopJerkLimit = 2000;
	public static final double ExpectedDriveCurrent = 1.5;
	public static final double ExpectedDriveRPM = 0;
	public static final double ExpectedDrivePosition = 0;

	// Controller
	public static final double MinimumControllerInput = 0.15;
	public static final double MaximumControllerInput = 1;
	public static final double MinimumControllerOutput = 0;
	public static final double MaximumControllerOutput = 1;
	public static final double MaxAcceleration = 1000;

	// Autonomous Driving
	public static final double TrackRadius = 14;
	public static final double WheelDiameter = 6;
	public static final double MinimumTurningRadius = 40;
	public static final double MinPathSpeed = 20;
	public static final double MaxPathSpeed = 120;
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 30;

	// CAN IDs
	public static final int DriveShifterId = 0;
	public static final int IntakeSolenoid30PsiId = 4;
	public static final int IntakeSolenoid60PsiId = 1;
	public static final int ElevatorGearboxShifterId = 3;
	public static final int ForkId = 2;
	public static final int LeftMasterDriveId = 16;
	public static final int LeftSlaveDriveId = 15;
	public static final int LeftSlave2DriveId = 14;
	public static final int RightMasterDriveId = 11;
	public static final int RightSlaveDriveId = 12;
	public static final int RightSlave2DriveId = 13;

	public static final int Intake1Id = 22;
	public static final int Intake2Id = 23;

	public static final int ElevatorMotorId = 24;
	public static final int ElevatorSlaveMotorId = 25;
	public static final int ArmId = 30;

	public static final int Climber1TalonId = 21;
	public static final int Climber2TalonId = 26;

	// Elevator
	public static final double ElevatorMinHeight = -0.5;
	public static final double ElevatorMaxHeight = 70; //
	public static final double ElevatorInchesPerMotorRotation = 8;
	public static final double ElevatorSpeed = 120; // Inches Per Second - need to double check value
	public static final double ElevatorStallCurrent = 3; //
	public static final double ExpectedElevatorCurrent = 0; //
	public static final double ExpectedElevatorRPM = 0; //
	public static final double ExpectedElevatorPosition = 0;
	public static final double ElevatorDownHeight = 2.5; // Should be the proper position for getting cubes
	public static final double ElevatorUpHeight = 65; // Should be the standard position for placing cubes on the scale
	public static final double ElevatorVelocityLimit = 1000;
	public static final double ElevatorAccelerationLimit = 250;

	// Arm
	public static final double ArmLowerAngleLimit = -38;
	public static final double ArmUpperAngleLimit = 100;
	public static final double ArmRotationsPerMotorRotation = 1 / 1.5; 
	public static final double ArmSpeed = 0; // Degrees Per Second
	public static final double ArmLength = 18.6; // Inches
	public static final double ExpectedArmCurrent = 0; //
	public static final double ExpectedArmRPM = 0;
	public static final double ExpectedArmPosition = 0;
	public static final double ArmHorizontalDegrees = 0;
	public static final double ArmDownDegrees = -38;
	public static final double ArmIntakeDegrees = -20;
	public static final double ArmVelocityLimit = 200;
	public static final double ArmAccelerationLimit = 300;
	
	public static final int PracticeBotArmTicksOffset = 1855;

	public static final int PracticeBotArmAngleOffsetInTicks = (int)(ArmDownDegrees * (1d / 360) /** (1 / ArmRotationsPerMotorRotation)*/ * (SensorTicksPerMotorRotation));
 
	public static final double JoystickDeadzone = .15;

	public static final int CubeSwitchId = 0;
	public static final double ExpectedClimberCurrent = 0;

	public static final double ElevatorMaxCurrent = 25;
	public static final double ArmMaxCurrent = 15;

	private Constants() {
	}
}