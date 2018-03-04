package org.usfirst.frc.team3476.robot;

public final class Constants {

	public static final double MinimumControllerInput = 0.15;

	// Driving
	public static final double HighDriveSpeed = 170;
	public static final double LowDriveSpeed = 0;
	public static final double kHighP = 0.2;
	public static final double kHighF = 0.2942366898763145583753006757594;
	public static final double kLowP = 0;
	public static final double kLowF = 0;
	public static final double TeleopAccLimit = 10;
	public static final double TeleopJerkLimit = 2000;
	public static final double ExpectedDriveCurrent = 0;
	public static final double ExpectedDriveRPM = 0;
	public static final double ExpectedDrivePosition = 0;

	// Controller
	public static final double MaximumControllerInput = 1;
	public static final double MinimumControllerOutput = 0;
	public static final double MaximumControllerOutput = 1;
	public static final double MaxAcceleration = 1000;

	// Autonomous Driving
	public static final double LookAheadDistance = 20;
	public static final double WheelScrub = 0.8;
	public static final double WheelDiameter = 6;
	public static final double ScrubFactor = 0.65;
	public static final double MinimumTurningRadius = 40;
	public static final double MinPathSpeed = 10;
	public static final double MaxPathSpeed = 100;
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 30;

	// CAN IDs
	public static final int DriveShifterId = 0;
	public static final int IntakeSolenoid30PsiId = 4;
	public static final int IntakeSolenoid60PsiId = 1;
	public static final int ElevatorGearboxShifterId = 3; //
	public static final int LeftMasterDriveId = 11;
	public static final int LeftSlaveDriveId = 12;
	public static final int LeftSlave2DriveId = 13;
	public static final int RightMasterDriveId = 16;
	public static final int RightSlaveDriveId = 15;
	public static final int RightSlave2DriveId = 14;

	public static final int Intake1Id = 22;
	public static final int Intake2Id = 23;

	public static final int ElevatorMotorId = 24;
	public static final int ElevatorSlaveMotorId = 26;
	public static final int ArmId = 30;

	// Elevator
	public static final double ElevatorMinHeight = 0;
	public static final double ElevatorMaxHeight = 67; //
	public static final double ElevatorInchesPerMotorRotation = 8;
	public static final double ElevatorSpeed = 120; // Inches Per Second - need to double check value
	public static final double ElevatorStallCurrent = 1.25; //
	public static final double ExpectedElevatorCurrent = 0; //
	public static final double ExpectedElevatorRPM = 0; //
	public static final double ExpectedElevatorPosition = 0;
	public static final double ElevatorDownHeight = 0; // Should be the proper position for getting cubes
	public static final double ElevatorUpHeight = 65; // Should be the standard position for placing cubes on the scale

	// Arm
	public static final double ArmLowerAngleLimit = -55;
	public static final double ArmUpperAngleLimit = 85;
	public static final double ArmRotationsPerMotorRotation = 1 / 1.5;
	public static final double ArmSpeed = 0; // Degrees Per Second
	public static final double ArmLength = 18.6; // Inches
	public static final double ExpectedArmCurrent = 0; //
	public static final double ExpectedArmRPM = 0;
	public static final double ExpectedArmPosition = 0;
	public static final double ArmHorizontalDegrees = 0;
	public static final double ArmDownDegrees = -55;

	// Other
	public static final int SensorTicksPerMotorRotation = 4096;
	public static final double ExpectedCurrentTolerance = 0;
	public static final double ExpectedRPMTolerance = 0;
	public static final double ExpectedPositionTolerance = 0;

	public static final int PracticeBotArmTicksOffset = 184;
	public static final int PracticeBotArmAngleOffsetInTicks = (int)(ArmDownDegrees * (1d / 360) * (1 / ArmRotationsPerMotorRotation) * (SensorTicksPerMotorRotation));


	private Constants() {
	}
}