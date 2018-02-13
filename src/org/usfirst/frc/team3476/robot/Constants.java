package org.usfirst.frc.team3476.robot;

public final class Constants {


	public static double MinimumControllerInput = 0.15;

	// Driving
	public static double MaxDriveSpeed = 170;
	public static double TeleopAccLimit = 90;
	public static double TeleopJerkLimit = 2000;
	
	//Controller
	public static double MaximumControllerInput = 1;
	public static double MinimumControllerOutput = 0;
	public static double MaximumControllerOutput = 1;
	public static double MaxAcceleration = 1000;

	//Autonomous Driving
	public static double LookAheadDistance = 20;
	public static double WheelScrub = 0.8;
	public static double WheelDiameter = 4;
	public static double ScrubFactor = 0.65;
	public static double MinimumTurningRadius = 40;
	public static double MinPathSpeed = 10;
	public static double MaxPathSpeed = 100;
	public static double MinLookAheadDistance = 14;
	public static double MaxLookAheadDistance = 30;
	
	public static double ElevatorTicksPerInc = 100000000000000000000000000000.0; //change to actual value
	public static double ElevatorStallCurrent = 100000000000.0;
	
	//CAN IDs
	public static int LeftMasterDriveId = 11;
	public static int LeftSlaveDriveId = 12;
	public static int LeftSlave2DriveId = 13;
	public static int RightMasterDriveId = 14;
	public static int RightSlaveDriveId = 15;
	public static int RightSlave2DriveId = 16;
	public static int ElevatorMotorId = 1000; //change to actual value
	public static int ElevatorSlaveMotorId = 1000;
	public static int ArmId = 1001; //change to actual value

	public static double ArmLength = 1000000;
	public static double ElevatorHeightToMotorRotations = 100.0;
	public static final double ElevatorHeight = 100;
	public static int SensorTicksPerRev = 1024;

	public static double ArmLowerAngleLimit;
	public static double ArmUpperAngleLimit;
	public static double ArmSpeed;
	public static double ElevatorSpeed;
	public static double ElevatorMinPosition;
	public static double ElevatorMaxPosition;
	public static final double ArmAngleToMotorRotations = 124203942035d;

	public static final double ExpectedElevatorCurrent = 0;

	public static final double ExpectedArmCurrent = 0;

	public static final double ExpectedElevatorRPM = 0;

	public static final double ExpectedElevator = 0;
	
	private Constants() {
	}
}