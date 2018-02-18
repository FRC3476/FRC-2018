package org.usfirst.frc.team3476.robot;

public final class Constants {

	public static final double MinimumControllerInput = 0.15;

	// Driving
	public static final double MaxDriveSpeed = 170;
	public static final double TeleopAccLimit = 90;
	public static final double TeleopJerkLimit = 2000;

	// Controller
	public static final double MaximumControllerInput = 1;
	public static final double MinimumControllerOutput = 0;
	public static final double MaximumControllerOutput = 1;
	public static final double MaxAcceleration = 1000;

	// Autonomous Driving
	public static final double LookAheadDistance = 20;
	public static final double WheelScrub = 0.8;
	public static final double WheelDiameter = 4;
	public static final double ScrubFactor = 0.65;
	public static final double MinimumTurningRadius = 40;
	public static final double MinPathSpeed = 10;
	public static final double MaxPathSpeed = 100;
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 30;

	// CAN IDs
	public static final int LeftMasterDriveId = 11;
	public static final int LeftSlaveDriveId = 12;
	public static final int LeftSlave2DriveId = 13;
	public static final int RightMasterDriveId = 14;
	public static final int RightSlaveDriveId = 15;
	public static final int RightSlave2DriveId = 16;
	public static final int ElevatorMotorId = 21;
	public static final int ElevatorSlaveMotorId = 22;
	public static final int ArmId = 31;

	//Elevator
	public static final double ElevatorMinHeight = 0;
	public static final double ElevatorMaxHeight = 90; //
	public static final double ElevatorInchesPerMotorRotation = 100000000; //
	public static final double ElevatorSpeed = 0; //
	public static final double ElevatorStallCurrent = 3; //
	public static final double ExpectedElevatorCurrent = 0; //
	public static final double ExpectedElevatorRPM = 0; //
	
	//Arm
	public static final double ArmLowerAngleLimit = 55;
	public static final double ArmUpperAngleLimit = 180;
	public static final double ArmRotationsPerMotorRotation = 1 / 1.5;
	public static final double ArmSpeed = 0; //
	public static final double ArmLength = 20; //
	public static final double ExpectedArmCurrent = 0; //
	
	
	//Other
	public static final int SensorTicksPerMotorRotation = 4096;

	private Constants() {
	}
}