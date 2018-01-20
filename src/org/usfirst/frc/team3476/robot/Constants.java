package org.usfirst.frc.team3476.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public final class Constants {

	public static double MinimumControllerInput = 0.15;

	// Constants
	
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
	public static double MaxTurningSpeed = 60;
	public static double MinimumTurningRadius = 40;
	
	//CAN IDs
	public static int LeftMasterDriveId = 2;
	public static int LeftSlaveDriveId = 3;
	public static int RightMasterDriveId = 4;
	public static int RightSlaveDriveId = 5;

	public static int TurretTicksPerRotations = 10;
	private Constants() {
	}
}