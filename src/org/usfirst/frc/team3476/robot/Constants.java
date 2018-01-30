package org.usfirst.frc.team3476.robot;

public final class Constants {

	public static double MinimumControllerInput = 0.15;

	// Driving
	public static double MaxDriveSpeed = 216;
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
	public static int LeftMasterDriveId = 2;
	public static int LeftSlaveDriveId = 3;
	public static int RightMasterDriveId = 4;
	public static int RightSlaveDriveId = 5;
	public static int ElevatorId = 1000; //change to actual value
	public static int ArmId = 1001; //change to actual value

	public static int TurretTicksPerRotations = 10;

	public static double ElevatorHeight = 1000000;
	public static double ArmLength = 1000000;
	public static double ElevatorHeightToEncoderTicks = 10000000000.0;
	
	private Constants() {
	}
}