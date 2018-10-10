package org.usfirst.frc.team3476.utility.auto;

import java.util.ArrayList;
import java.util.Set;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.subsystem.Intake.SolenoidState;
import org.usfirst.frc.team3476.utility.control.motion.BezierCurve;
import org.usfirst.frc.team3476.utility.control.motion.BezierCurve.BezierPoint;
import org.usfirst.frc.team3476.utility.control.motion.Path;
import org.usfirst.frc.team3476.utility.math.Rotation;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class AutoRoutineGenerator {

	private static Translation2d robotRightStartPosition = new Translation2d(20, -115);
	private static Translation2d robotCenterStartPosition = new Translation2d(20, -5);
	private static Translation2d robotLeftStartPosition = new Translation2d(20, 115);

	private static Translation2d midFieldRightPosition = new Translation2d(240, -108);
	private static Translation2d midFieldLeftPosition = new Translation2d(240, 108);

	private static Translation2d midFieldRightBackUpPosition = new Translation2d(260, -128);
	private static Translation2d midFieldLeftBackUpPosition = new Translation2d(260, 128);

	private static Translation2d midFieldRightLeadUp = new Translation2d(120, -108);
	private static Translation2d midFieldLeftLeadUp = new Translation2d(120, 108);

	private static Translation2d rightScalePosition = new Translation2d(276, -90);
	private static Translation2d leftScalePosition = new Translation2d(270, 90);
	private static Translation2d leftScalePositionSecondCube = new Translation2d(275, 86);
	private static Translation2d rightScalePositionSecondCube = new Translation2d(275, -86);

	private static Translation2d rightSwitchCubePositionFar = new Translation2d(230, -76);
	private static Translation2d leftSwitchCubePositionFar = new Translation2d(230, 76);

	private static Translation2d rightSwitchOuttakePositionNear = new Translation2d(124, -50);
	private static Translation2d leftSwitchOuttakePositionNear = new Translation2d(124, 50);

	private static Translation2d rightSwitchOuttakeLeadUpNear = new Translation2d(100, -50);
	private static Translation2d leftSwitchOuttakeLeadUpNear = new Translation2d(100, 50);

	private static Translation2d rightSwitchOuttakePositionFar = new Translation2d(226, -46);
	private static Translation2d leftSwitchOuttakePositionFar = new Translation2d(226, 46);

	private static Translation2d rightSwitchLeadUpFar = new Translation2d(260, -72);
	private static Translation2d leftSwitchLeadUpFar = new Translation2d(260, 72);

	private static double switchSpeed = 60;
	private static double scaleSpeed = 70;
	private static double longDistanceSpeed = 140;
	private static double shortDistanceSpeed = 70;
	private static double midFieldSpeed = 80;
	private static double reverseSpeed = 70;

	private static AutoRoutine toMidFieldRightReverse;
	private static AutoRoutine toMidFieldLeftReverse;
	private static AutoRoutine placeCubeOnScale;
	private static AutoRoutine getRightSwitchCube;
	private static AutoRoutine getLeftSwitchCube;
	private static AutoRoutine placeCubeInSwitch;
	private static AutoRoutine scaleOuttakePosition;
	private static AutoRoutine highScaleOuttakePosition;
	private static AutoRoutine highScaleOuttakePositionBlock;
	private static AutoRoutine intakePosition;
	private static AutoRoutine secondIntakePosition;
	private static AutoRoutine switchOuttakePosition;
	private static AutoRoutine stowPosition;
	private static AutoRoutine initialDrive;

	private static Position switchPos;
	private static Position scalePos;

	public enum Position {
		LEFT, RIGHT
	}

	public enum PathOption {
		SCALE, SWITCH, BOTH, FORWARD, NONE
	}

	public enum StartPosition {
		LEFT, CENTER, RIGHT
	}

	static {
		toMidFieldRightReverse = new AutoRoutine(); // Drives to Mid Field Right
													// Position from Current
													// Location
		toMidFieldRightReverse.addCommands(new DriveToPoints(reverseSpeed, true, midFieldRightPosition));

		toMidFieldLeftReverse = new AutoRoutine();
		toMidFieldLeftReverse.addCommands(new DriveToPoints(reverseSpeed, true, midFieldLeftPosition));

		placeCubeOnScale = new AutoRoutine(); // Puts Cube on Right Scale from
												// Mid Field Right Position,
												// then backs up to Mid Field
												// Right Position
		placeCubeOnScale.addCommands(new SetArmAngle(80, true), new SetElevatorHeight(60, true),
				new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING), new Delay(.75),
				new SetElevatorHeight(10), new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP));

		getRightSwitchCube = new AutoRoutine(); // Grab Switch Cube, then back
												// up to Mid Field Right
												// Position
		getRightSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE, SolenoidState.AUTO),
				new SetElevatorHeight(Constants.ElevatorDownHeight), new SetArmAngle(Constants.ArmIntakeDegrees),
				new DriveToPoints(switchSpeed, false, rightSwitchCubePositionFar), new Delay(.5),
				new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), new Delay(.5));

		getLeftSwitchCube = new AutoRoutine();
		getLeftSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE, SolenoidState.AUTO),
				new SetElevatorHeight(Constants.ElevatorDownHeight), new SetArmAngle(Constants.ArmIntakeDegrees),
				new DriveToPoints(switchSpeed, false, leftSwitchCubePositionFar), new Delay(.5),
				new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), new Delay(.5));

		scaleOuttakePosition = new AutoRoutine();
		scaleOuttakePosition.addCommands(new SetElevatorHeight(50), new SetArmAngle(80));

		highScaleOuttakePosition = new AutoRoutine();
		highScaleOuttakePosition.addCommands(new SetElevatorHeight(69), new SetArmAngle(80));
		
		highScaleOuttakePositionBlock = new AutoRoutine();
		highScaleOuttakePositionBlock.addCommands(new SetArmAngle(80), new SetElevatorHeight(69, true));

		intakePosition = new AutoRoutine();
		intakePosition.addCommands(new SetElevatorHeight(Constants.ElevatorDownHeight),
				new SetArmAngle(Constants.ArmIntakeDegrees));

		secondIntakePosition = new AutoRoutine();
		secondIntakePosition.addCommands(new SetElevatorHeight(Constants.ElevatorDownHeight + 11),
				new SetArmAngle(Constants.ArmIntakeDegrees));

		switchOuttakePosition = new AutoRoutine();
		switchOuttakePosition.addCommands(new SetElevatorHeight(25), new SetArmAngle(0));

		stowPosition = new AutoRoutine();
		stowPosition.addCommands(new SetElevatorHeight(10), new SetArmAngle(80));

		placeCubeInSwitch = new AutoRoutine();
		placeCubeInSwitch.addRoutines(switchOuttakePosition);
		placeCubeInSwitch.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING),
				new Delay(.6), new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP));
		placeCubeInSwitch.addRoutines(stowPosition);
	}

	public static AutoRoutine generate(String gameMsg, PathOption option, StartPosition position, boolean mindBusiness,
			boolean good) {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
		initialDrive.addCommands(new HomeElevator(), new SetElevatorHeight(0), new SetArmAngle(80, false));
		System.out.println(gameMsg);
		if (gameMsg.charAt(0) == 'l') {
			switchPos = Position.LEFT;
		} else {
			switchPos = Position.RIGHT;
		}
		if (gameMsg.charAt(1) == 'l') {
			scalePos = Position.LEFT;
		} else {
			scalePos = Position.RIGHT;
		}
		if (mindBusiness) {
			if (position == StartPosition.LEFT) {
				if (scalePos != Position.LEFT) {
					if (true || switchPos != Position.LEFT) {
						option = PathOption.FORWARD;
						System.out.println("FORWARD");
					} else {
						option = PathOption.SWITCH;
						System.out.println("SWITCH");
					}
				} else {
					option = PathOption.SCALE;
					System.out.println("SCALE");
				}
			} else {
				if (scalePos != Position.RIGHT) {
					if (true || switchPos != Position.RIGHT) {
						option = PathOption.FORWARD;
						System.out.println("FORWARD");
					} else {
						option = PathOption.SWITCH;
						System.out.println("SWITCH");
					}
				} else {
					option = PathOption.SCALE;
					System.out.println("SCALE");
				}
			}

		}
		switch (position) {
			case LEFT:
				System.out.println("start left");
				RobotTracker.getInstance().setInitialTranslation(robotLeftStartPosition);
				initialPath = new Path(robotLeftStartPosition);
				switch (option) {
					case SWITCH:
						if (switchPos == Position.LEFT) {
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(leftSwitchOuttakePositionNear, switchSpeed);
						} else {
							// initialPath.addPoint(rightSwitchOuttakeLeadUpNear,
							// shortDistanceSpeed);
							// initialPath.addPoint(rightSwitchOuttakePositionNear,
							// switchSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
						break;
					case SCALE:
						/*
						
						*/
						if (scalePos == Position.LEFT) { // NEAR SCALE
							/*
							 * initialPath.addPoint(230, 135, 120);
							 * initialPath.addRoutine(scaleOuttakePosition,
							 * 0.8); initialPath.addPoint(260, 135, 60);
							 * initialPath.addPoint(290, 105, 60);
							 * initialPath.addCommand(new
							 * SetIntakeState(IntakeState.OUTTAKE_FAST,
							 * SolenoidState.INTAKING), 0.4);
							 * initialPath.addPoint(260, 85, 60);
							 * initialPath.addRoutine(intakePosition, 0.3);
							 * initialPath.addPoint(240, 85, 60);
							 * initialPath.addCommand(new
							 * SetIntakeState(IntakeState.INTAKE,
							 * SolenoidState.AUTO), 0.3);
							 * initialPath.addPoint(204, 72, 80);
							 * initialDrive.addCommands(new
							 * SetDrivePath(initialPath, false));
							 * 
							 * Path secondPath = new Path(new Translation2d(204,
							 * 72)); secondPath.addPoint(268, 105, 50);
							 * secondPath.addCommand(new
							 * SetIntakeState(IntakeState.NEUTRAL,
							 * SolenoidState.CLAMP), 0.1);
							 * secondPath.addRoutine(scaleOuttakePosition, 0.2);
							 * secondPath.addPoint(264, 120, 40);
							 * secondPath.addCommand(new
							 * SetIntakeState(IntakeState.OUTTAKE_FAST,
							 * SolenoidState.INTAKING), 0.4);
							 * secondPath.addRoutine(stowPosition, 0.6);
							 * initialDrive.addCommands(new
							 * SetDrivePath(secondPath, true));
							 * 
							 * Path thirdPath = new Path(new Translation2d(264,
							 * 120)); thirdPath.addPoint(264, 105, 60);
							 * thirdPath.addPoint(230, 90, 60);
							 * thirdPath.addRoutine(intakePosition, 0.2);
							 * thirdPath.addPoint(190, 58, 60);
							 * thirdPath.addCommand(new
							 * SetIntakeState(IntakeState.INTAKE,
							 * SolenoidState.AUTO), 0.2);
							 * initialDrive.addCommands(new
							 * SetDrivePath(thirdPath, false));
							 * 
							 * Path fourthPath = new Path(new Translation2d(190,
							 * 58)); fourthPath.addPoint(230, 90, 60);
							 * fourthPath.addCommand(new
							 * SetIntakeState(IntakeState.NEUTRAL,
							 * SolenoidState.CLAMP), 0.5);
							 * fourthPath.addRoutine(scaleOuttakePosition, 0.6);
							 * fourthPath.addPoint(268, 105, 60);
							 * fourthPath.addPoint(264, 120, 40);
							 * fourthPath.addCommand(new
							 * SetIntakeState(IntakeState.OUTTAKE_FAST,
							 * SolenoidState.INTAKING), 0.4);
							 * fourthPath.addRoutine(stowPosition, 0.6);
							 * initialDrive.addCommands(new
							 * SetDrivePath(fourthPath, true));
							 */
							/*
							initialPath.addPoint(180, 115, 100);
							initialPath.addPoint(275, 80, 60);
							initialPath.addRoutine(scaleOuttakePosition, 0.7);
							initialPath.addCommand(
									new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING), 0.95);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(216, 80)));

							Path secondPath = new Path(new Translation2d(275, 80));
							secondPath.addPoint(216, 74, 80);
							secondPath.addRoutine(intakePosition, 0.0);
							secondPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.INTAKING), 0);
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.8);
							initialDrive.addCommands(new SetDrivePath(secondPath, false));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(230, 80)));
							Path thirdPath = new Path(new Translation2d(216, 74));
							thirdPath.addPoint(230, 80, 60);
							thirdPath.addPoint(275, 80, 60);
							thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0);
							thirdPath.addRoutine(highScaleOuttakePosition, 0.5);
							thirdPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING),
									0.9);
							initialDrive.addCommands(new SetDrivePath(thirdPath, false));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(216, 80)));
							/*
							 * Path fourthPath = new Path(new Translation2d(200,
							 * 90)); fourthPath.addPoint(204, 58, 80);
							 * fourthPath.addRoutine(intakePosition, 0);
							 * fourthPath.addCommand(new
							 * SetIntakeState(IntakeState.NEUTRAL,
							 * SolenoidState.INTAKING), 0);
							 * fourthPath.addCommand(new
							 * SetIntakeState(IntakeState.INTAKE,
							 * SolenoidState.INTAKING), 0.9);
							 * initialDrive.addCommands(new
							 * SetDrivePath(fourthPath, false));
							 * initialDrive.addCommands(new SetDriveAngle(new
							 * Translation2d(204, -74))); Path fifthPath = new
							 * Path(new Translation2d(204, 58));
							 * fifthPath.addPoint(204, 74, 80);
							 * fifthPath.addPoint(200, 90, 60);
							 * fifthPath.addCommand(new
							 * SetIntakeState(IntakeState.NEUTRAL,
							 * SolenoidState.CLAMP), 0);
							 * fifthPath.addRoutine(highScaleOuttakePosition,
							 * 0.5); fifthPath.addCommand(new
							 * SetIntakeState(IntakeState.OUTTAKE_MIDDLE,
							 * SolenoidState.INTAKING), 0.8);
							 * initialDrive.addCommands(new
							 * SetDrivePath(fifthPath, false));
							 
							overallRoutine.addRoutines(initialDrive);

							overallRoutine.addRoutines(initialDrive);
							*/
							initialPath.addPoint(180, 115, 120); //80
							initialPath.addPoint(270, 90, 120); //60
							initialPath.addRoutine(highScaleOuttakePosition, 0.7);
							initialPath.addCommand(
									new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING), 0.9);
							
							//initialPath.addCommand(
							//		new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.OPEN), 1.0);
							initialPath.addRoutine(stowPosition, 1.0);;
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(231, 80)));
							Path secondPath = new Path(new Translation2d(270, 90));
							secondPath.addPoint(231, 80, 50); //35 
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.OPEN), 0.0);
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.CLAMP), 0.95);
							secondPath.addRoutine(intakePosition, 0.0);
							//secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.85);
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.CLAMP), 0.9); //0.9

							
							Path thirdPath = new Path(new Translation2d(228, 78));
							thirdPath.addPoint(271, 97, 110); //change speed to 60
							thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
							thirdPath.addRoutine(highScaleOuttakePosition, 0.4); //0.2
							
							/* optional ending after 2 cube
							Path fourthPath = new Path(new Translation2d(270, 90));
							fourthPath.addPoint(250, 110, 60);
							fourthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
							fourthPath.addRoutine(stowPosition, 0.2);;
							*/
							
							//expiermental 'third cube' from here down
							Path fourthPath = new Path(new Translation2d(271, 97));
							fourthPath.addPoint(228, 87, 100); //240 87
							fourthPath.addRoutine(intakePosition, 0.0);
							fourthPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.OPEN), 0.0);
							fourthPath.addPoint(213, 58, 45); //CHANGE THIS SPEED TO 50, POS 225, 56 | 219 56
							fourthPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.8);
							
							
							Path fifthPath = new Path(new Translation2d(218, 56)); //225 62
							fifthPath.addPoint(271, 97, 100);  //CHANGE THIS SPEED TO 50
							fifthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.15);
							fifthPath.addRoutine(highScaleOuttakePosition, 0.5);
							
							//end of 3 cube
							
							overallRoutine.addRoutines(initialDrive);
							overallRoutine.addCommands(new SetDrivePath(secondPath, false));
							overallRoutine.addCommands(new SetDrivePath(thirdPath, true));
							//overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-60)));
							//overallRoutine.addCommands(new Delay(0.1));
							//overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-35)));
							//overallRoutine.addCommands(new Delay(0.1));
							overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-55)));
							
							overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FASTEST, SolenoidState.INTAKING));
							overallRoutine.addRoutines(stowPosition); //x

							overallRoutine.addCommands(new Delay(0.5));
							
							/* 2 cube ending 
							overallRoutine.addCommands(new SetDrivePath(fourthPath, true));
							overallRoutine.addCommands(new SetDriveAngle(new Translation2d(202, 39)));
							*/
							/* 3 cube expiermental */
							overallRoutine.addRoutines(intakePosition);
							overallRoutine.addCommands(new SetDriveAngle(new Translation2d(220, 58))); //226, 66
							overallRoutine.addCommands(new SetDrivePath(fourthPath, false));
							overallRoutine.addCommands(new SetDrivePath(fifthPath, true));
							overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-55))); //-55
							overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FASTEST, SolenoidState.INTAKING));
							overallRoutine.addCommands(new Delay(3.0));
						} else { // FAR SCALE
							/*
							initialPath.addPoint(180, 115, 120);
							initialPath.addPoint(220, 115, 120);
							initialPath.addPoint(220, 75, 120);
							initialPath.addPoint(220, 20, 50);
							initialPath.addPoint(220, -20, 50);
							initialPath.addPoint(220, -120, 120);

							initialPath.addPoint(230, -135, 60);
							initialPath.addRoutine(highScaleOuttakePosition, 0.8);
							initialPath.addPoint(260, -135, 60);
							initialPath.addPoint(290, -105, 60);
							initialPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING),
									0.4);
							initialPath.addPoint(260, -85, 60);
							initialPath.addRoutine(intakePosition, 0.3);
							initialPath.addPoint(240, -85, 60);
							initialPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.AUTO), 0.3);
							initialPath.addPoint(202, -72, 80);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));

							Path secondPath = new Path(new Translation2d(200, -72));
							secondPath.addPoint(268, -105, 50);
							secondPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.1);
							secondPath.addRoutine(scaleOuttakePosition, 0.2);
							secondPath.addPoint(264, -120, 40);
							secondPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING),
									0.4);
							secondPath.addRoutine(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(secondPath, true));

							overallRoutine.addRoutines(initialDrive);
							*/
							/*
							initialPath.addPoint(180, 115, 80);
							initialPath.addPoint(250, -55, 80);
							initialPath.addPoint(270, -90, 80);
							initialPath.addRoutine(highScaleOuttakePosition, 0.7);
							
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							//initialDrive.addCommands(new SetDriveAngle(new Translation2d(232, -80)));
						
							Path secondPath = new Path(new Translation2d(250, -90));
							secondPath.addPoint(270, -90, 60);
							secondPath.addCommand(
									new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING), 0.5);
							*/
							/*
							Path thirdPath = new Path(new Translation2d(270, -90));
							thirdPath.addPoint(250, 80, 40);
							thirdPath.addRoutine(intakePosition, 0.3);
							
							
							Path fourthPath = new Path(new Translation2d(250, 80));
							fourthPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.0);
							fourthPath.addPoint(230,  90, 30);
							*/
							
						
							
							
							
							//overallRoutine.addRoutines(initialDrive);
							//overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-35)));
							//overallRoutine.addCommands(new SetDrivePath(secondPath, false));
							//overallRoutine.addCommands(new SetDrivePath(thirdPath, true));
							//overallRoutine.addCommands(new SetDriveAngle(new Translation2d(230, 80)));
							
							
							//overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-35)));
							//overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING));
							initialPath.addPoint(200, 115, 120);
							initialPath.addPoint(235, 90, 120);
							initialPath.addPoint(235, 40, 60);
							initialPath.addPoint(235, -40, 120);
							initialPath.addPoint(235, -70, 120);
							initialPath.addPoint(271, -97, 120); //-100 TO -92
							initialPath.addRoutine(highScaleOuttakePosition, 0.8);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(35)));
							overallRoutine.addRoutines(initialDrive);
							overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING));
							overallRoutine.addCommands(new Delay(0.25));
							overallRoutine.addRoutines(stowPosition);
							overallRoutine.addCommands(new SetDriveAngle(new Translation2d(231, -85)));
							
							
							Path secondPath = new Path(new Translation2d(271, -97));
							secondPath.addPoint(228, -95, 35); 
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.0);
							secondPath.addRoutine(intakePosition, 0.0);
							//secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.85);
							overallRoutine.addCommands(new SetDrivePath(secondPath, false));
							
							Path thirdPath = new Path(new Translation2d(228, -95));
							thirdPath.addPoint(271, -100, 60); //97
							thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
							thirdPath.addRoutine(stowPosition, 0.2);
							thirdPath.addRoutine(highScaleOuttakePosition, 1.0);
							
							overallRoutine.addCommands(new SetDrivePath(thirdPath, true));
							overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(45)));
							//overallRoutine.addRoutines(highScaleOuttakePositionBlock);
							overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING));
							overallRoutine.addCommands(new Delay(0.5));

							
							Path fourthPath = new Path(new Translation2d(271, -97));
							fourthPath.addPoint(250, -110, 60);
							fourthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
							fourthPath.addRoutine(stowPosition, 0.2);;
							overallRoutine.addCommands(new SetDrivePath(fourthPath, true));
							overallRoutine.addCommands(new SetDriveAngle(new Translation2d(202, -44)));
							
							
							//initialDrive.addCommands(new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING));
							//initialDrive.addCommands(new Delay(1.0));
							
							
							
							
							//overallRoutine.addCommands(new SetDrivePath(fourthPath, true));
							//overallRoutine.addCommands(new SetDriveAngle(new Translation2d(202, 39)));
							
							/*
							initialPath.addPoint(260, -135, 60);
							initialPath.addPoint(290, -105, 60);
							initialPath.addCommand(
									new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING), 0.4);
							
							
							initialPath.addPoint(260, -85, 60);
							initialPath.addRoutine(intakePosition, 0.3);
							initialPath.addPoint(240, -85, 60);
							initialPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.AUTO), 0.3);
							initialPath.addPoint(202, -72, 80);
							*/
							
							
								/* <===
							Path secondPath = new Path(new Translation2d(200, -72));
							secondPath.addPoint(268, -105, 50);
							secondPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP),
									0.1);
							secondPath.addRoutine(scaleOuttakePosition, 0.2);
							secondPath.addPoint(264, -120, 40);
							secondPath.addCommand(
									new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING), 0.4);
							secondPath.addRoutine(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							*/
						}
						break;
					case BOTH:
						/* OLD one
						initialPath.addPoint(midFieldLeftPosition, longDistanceSpeed);
						if (scalePos == Position.RIGHT) {

							initialPath.addPoint(midFieldRightPosition, midFieldSpeed);
							initialPath.addPoint(rightScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						} else {
							initialPath.addPoint(leftScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						}

						if (switchPos == Position.RIGHT) {
							overallRoutine.addRoutines(toMidFieldRightReverse, getRightSwitchCube);
							overallRoutine
									.addCommands(new DriveToPoints(switchSpeed, false, rightSwitchOuttakePositionFar));
						} else {
							overallRoutine.addRoutines(toMidFieldLeftReverse, getLeftSwitchCube);
							overallRoutine
									.addCommands(new DriveToPoints(switchSpeed, false, leftSwitchOuttakePositionFar));
						}
						overallRoutine.addRoutines(placeCubeInSwitch);
						*/
						if(switchPos == Position.LEFT) { //Near switch
							
							//Preload cube in switch
							initialPath.addPoint(new Translation2d(168, 110), 80);
							initialPath.addRoutine(switchOuttakePosition, 0.5);
							initialPath.addPoint(new Translation2d(168, 95), 80);
							initialPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING), 0.95);
							
							
							//align with next cube 
							Path secondPath = new Path(new Translation2d(168, 90));
							secondPath.addPoint(new Translation2d(238, 90), 80);
							secondPath.addRoutine(intakePosition, 0.1);
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.1);
							//secondPath.addPoint(new Translation2d(238, 78), 80);
							secondPath.addPoint(new Translation2d(242, 78), 80); 
							
							//intake next cube
							Path thirdPath = new Path(new Translation2d(242, 78));
							thirdPath.addPoint(new Translation2d(228, 73), 35);
							thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.9);
							
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(226, 73))); //228 75
							initialDrive.addCommands(new SetDrivePath(thirdPath, false));
							
							if(scalePos == Position.LEFT) {
								//backtrack, static turn, release
								Path fourthPath = new Path(new Translation2d(228, 73));
								fourthPath.addPoint(270, 90, 60);
								fourthPath.addRoutine(highScaleOuttakePosition, 0.7);
								
								Path fifthPath = new Path(new Translation2d(228, 73));
								fifthPath.addPoint(220, 78, 50);
								fifthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.INTAKING), 0.0);
								fifthPath.addRoutine(stowPosition, 0.1);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(-60)));
								initialDrive.addCommands(new Delay(0.1));
								initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(-35)));
								initialDrive.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN));
								initialDrive.addCommands(new Delay(1.0));
								
								initialDrive.addCommands(new SetDrivePath(fifthPath, true));
								
							} else if(scalePos == Position.RIGHT) {
								//drive backward to the right scale, then drive forward and release
								/*
								Path fourthPath = new Path(new Translation2d(228, 78));
								fourthPath.addPoint(new Translation2d(242, -88), 80);
								fourthPath.addPoint(new Translation2d(238, -88), 80);
								
								Path fifthPath = new Path(new Translation2d(228, -88));
								fifthPath.addPoint(new Translation2d(248, -88), 35);
								fifthPath.addRoutine(scaleOuttakePosition, 0.0);
								fifthPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN), 0.9);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDrivePath(fifthPath, false));
								*/
							}
							
							
						} else { //far switch
							//Drive to the other side of the switch and release preload
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							//initialPath.addPoint(new Translation2d(168, 100), 80); may or may not be needed
							initialPath.addPoint(new Translation2d(237, 110), 80);
							initialPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
							initialPath.addRoutine(switchOuttakePosition, 0.9);
							initialPath.addPoint(new Translation2d(237, -56), 80);
							
							
							//initialDrive.addCommands(new SetDrivePath(initialPath, false));
							
							//Grab cube right in front of robot and score
							Path secondPath = new Path(new Translation2d(237, -56));
							secondPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING), 0.0);
							secondPath.addPoint(new Translation2d(242, -56), 20);
							secondPath.addRoutine(intakePosition, 0.9);
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.9);
							/*
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.0);
							secondPath.addRoutine(switchOuttakePosition, 0.9);
							secondPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN), 0.9);
							*/
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(-180)));
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(237,-60)));
							
							/*
							if(scalePos == Position.LEFT) {
								Path fourthPath = new Path(new Translation2d(228, -78));
								fourthPath.addPoint(new Translation2d(242, 88), 80);
								fourthPath.addPoint(new Translation2d(238, 88), 80);
								
								Path fifthPath = new Path(new Translation2d(228, 88));
								fifthPath.addPoint(new Translation2d(248, 88), 35);
								fifthPath.addRoutine(scaleOuttakePosition, 0.0);
								fifthPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN), 0.9);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDrivePath(fifthPath, false));
							} else if(scalePos == Position.RIGHT) {
								Path fourthPath = new Path(new Translation2d(228, -78));
								fourthPath.addPoint(270, -90, 60);
								fourthPath.addRoutine(highScaleOuttakePosition, 0.7);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(15)));
								initialDrive.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN));
								
							} */
						}
						overallRoutine.addRoutines(initialDrive);
						break;
					case FORWARD:
						initialPath.addPoint(midFieldLeftLeadUp, longDistanceSpeed);
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive);
						break;
					case NONE:
						break;
				}
				break;
			case CENTER:
				System.out.println("start centers");
				RobotTracker.getInstance().setInitialTranslation(robotCenterStartPosition);
				initialPath = new Path(robotCenterStartPosition);
				switch (option) {
					case SWITCH:
						if (good) {
							System.out.println("switch");
							if (switchPos == Position.LEFT) {
								System.out.println("left");
								initialPath.addPoint(leftSwitchOuttakeLeadUpNear, 100);
								initialPath.addPoint(leftSwitchOuttakePositionNear, 100);
							} else {
								System.out.println("right");
								initialPath.addPoint(rightSwitchOuttakeLeadUpNear, 100);
								initialPath.addPoint(rightSwitchOuttakePositionNear, 100);
							}
							initialPath.addRoutine(switchOuttakePosition, 0.5);
							initialPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE, SolenoidState.OPEN),
									0.95);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							Path secondPath;
							if (switchPos == Position.LEFT) {
								secondPath = new Path(leftSwitchOuttakePositionNear);
							} else {
								secondPath = new Path(rightSwitchOuttakePositionNear);
							}
							secondPath.addPoint(65, 0, 80);
							secondPath.addPoint(63, 0, 80);
							secondPath.addRoutine(intakePosition, 0.5);
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							Path thirdPath = new Path(new Translation2d(63, 0));
							thirdPath.addPoint(83, 0, 80);
							thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.OPEN), 0);
							thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.95);
							initialDrive.addCommands(new SetDrivePath(thirdPath, false));
							Path fourthPath = new Path(new Translation2d(83, 0));
							fourthPath.addPoint(65, 0, 80);
							fourthPath.addPoint(63, 0, 80);
							fourthPath.addRoutine(switchOuttakePosition, 0.5);
							initialDrive.addCommands(new SetDrivePath(fourthPath, true));
							Path fifthPath = new Path(new Translation2d(60, 0));
							if (switchPos == Position.LEFT) {
								fifthPath.addPoint(100, 42, 80);
								fifthPath.addPoint(124, 42, 80);
							} else {
								fifthPath.addPoint(100, -42, 80);
								fifthPath.addPoint(124, -42, 80);
							}
							fifthPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE, SolenoidState.OPEN), 0.95);
							initialDrive.addCommands(new SetDrivePath(fifthPath, false));

							Path sixthPath;
							if (switchPos == Position.LEFT) {
								sixthPath = new Path(leftSwitchOuttakePositionNear);
								sixthPath.addPoint(81, -3, 80);
							} else {
								sixthPath = new Path(rightSwitchOuttakePositionNear);
								sixthPath.addPoint(81, 3, 80);
							}
							sixthPath.addPoint(72, 0, 80);
							sixthPath.addPoint(70, 0, 80);
							sixthPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.OPEN), 0.7);
							sixthPath.addRoutine(secondIntakePosition, 0.8);
							initialDrive.addCommands(new SetDrivePath(sixthPath, true));
							Path seventhPath = new Path(new Translation2d(67, 0));
							seventhPath.addPoint(96, 0, 80);
							seventhPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.95);
							initialDrive.addCommands(new SetDrivePath(seventhPath, false));
							Path eightPath = new Path(new Translation2d(96, 0));
							eightPath.addPoint(75, 0, 70);
							eightPath.addRoutine(switchOuttakePosition, 0.8);
							initialDrive.addCommands(new SetDrivePath(eightPath, true));
							Path ninthPath = new Path(new Translation2d(75, 0));
							if (switchPos == Position.LEFT) {
								ninthPath.addPoint(100, 42, 80);
								ninthPath.addPoint(124, 42, 80);
							} else {
								ninthPath.addPoint(100, -42, 80);
								ninthPath.addPoint(124, -42, 80);
							}
							ninthPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE, SolenoidState.OPEN), 0.95);
							initialDrive.addCommands(new SetDrivePath(ninthPath, false));
							Path tenthPath;
							;
							if (switchPos == Position.LEFT) {
								tenthPath = new Path(new Translation2d(124, 42));
								tenthPath.addPoint(94, 42, 80);
								tenthPath.addPoint(94, 50, 80);
							} else {
								tenthPath = new Path(new Translation2d(124, -42));
								tenthPath.addPoint(94, -42, 80);
								tenthPath.addPoint(94, -50, 80);
							}
							tenthPath.addRoutine(intakePosition, 0.5);
							initialDrive.addCommands(new SetDrivePath(tenthPath, true));
							overallRoutine.addRoutines(initialDrive);
						} else {
							if (switchPos == Position.LEFT) {
								initialPath.addPoint(leftSwitchOuttakeLeadUpNear, 80);
								initialPath.addPoint(leftSwitchOuttakePositionNear, 80);
							} else {
								initialPath.addPoint(rightSwitchOuttakeLeadUpNear, 80);
								initialPath.addPoint(rightSwitchOuttakePositionNear, 80);
							}
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
						}
						break;
					case SCALE:
						break;
					case BOTH:
						break;
					case FORWARD:
						break;
					case NONE:
						break;
				}
				break;
			case RIGHT:
				System.out.println("start right");
				RobotTracker.getInstance().setInitialTranslation(robotRightStartPosition);
				initialPath = new Path(robotRightStartPosition);
				switch (option) {
					case SWITCH:
						if (switchPos == Position.LEFT) {
							// initialPath.addPoint(leftSwitchOuttakeLeadUpNear,
							// shortDistanceSpeed);
							// initialPath.addPoint(leftSwitchOuttakePositionNear,
							// switchSpeed);
							//initialPath.addPoint(new Translation2d(100, -100) , shortDistanceSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
						} else {
							//initialPath.addPoint(rightSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(new Translation2d(100, -100) , shortDistanceSpeed);
							//initialPath.addPoint(rightSwitchOuttakePositionNear, switchSpeed);
							initialPath.addPoint(new Translation2d(150, -75), shortDistanceSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
						}
						
						break;
					case SCALE:
						/*
						
						*/
						if (good) {
							if (scalePos == Position.RIGHT) { // NEAR SCALE
								/*
								initialPath.addPoint(180, -115, 80);
								initialPath.addPoint(270, -90, 60);
								initialPath.addRoutine(highScaleOuttakePosition, 0.7);
								initialPath.addCommand(
										new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING), 0.9);
								
								//initialPath.addCommand(
								//		new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.OPEN), 1.0);
								initialPath.addRoutine(stowPosition, 1.0);;
								initialDrive.addCommands(new SetDrivePath(initialPath, false));
								initialDrive.addCommands(new SetDriveAngle(new Translation2d(232, -82)));
								Path secondPath = new Path(new Translation2d(270, -90));
								secondPath.addPoint(230, -82, 50); 
								secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.0);
								secondPath.addRoutine(intakePosition, 0.0);
								//secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.85);
								
								Path thirdPath = new Path(new Translation2d(230, -82));
								thirdPath.addPoint(270, -90, 60);
								thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
								thirdPath.addRoutine(highScaleOuttakePosition, 0.2);
								
								
								Path fourthPath = new Path(new Translation2d(270, -90));
								fourthPath.addPoint(250, -110, 60);
								fourthPath.addRoutine(stowPosition, 0.2);;
								
								
								
								overallRoutine.addRoutines(initialDrive);
								overallRoutine.addCommands(new SetDrivePath(secondPath, false));
								overallRoutine.addCommands(new SetDrivePath(thirdPath, true));
								overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(35)));
								overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING));
								overallRoutine.addCommands(new Delay(1.5));
								overallRoutine.addCommands(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.INTAKING));
								//overallRoutine.addCommands(new SetDrivePath(fourthPath, true));
								*/
								initialPath.addPoint(180, -115, 120); //80
								initialPath.addPoint(270, -90, 120); //60
								initialPath.addRoutine(highScaleOuttakePosition, 0.7);
								initialPath.addCommand(
										new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING), 0.9);
								
								//initialPath.addCommand(
								//		new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.OPEN), 1.0);
								initialPath.addRoutine(stowPosition, 1.0);;
								initialDrive.addCommands(new SetDrivePath(initialPath, false));
								initialDrive.addCommands(new SetDriveAngle(new Translation2d(231, -80)));
								Path secondPath = new Path(new Translation2d(270, -90));
								secondPath.addPoint(231, -80, 50); //35 
								secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.OPEN), 0.0);
								secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.CLAMP), 0.95);
								secondPath.addRoutine(intakePosition, 0.0);
								//secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.85);
								secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.CLAMP), 0.9); //0.9

								
								Path thirdPath = new Path(new Translation2d(228, -78));
								thirdPath.addPoint(271, -97, 110); //change speed to 60
								thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
								thirdPath.addRoutine(highScaleOuttakePosition, 0.4); //0.2
								
								/* optional ending after 2 cube
								Path fourthPath = new Path(new Translation2d(270, 90));
								fourthPath.addPoint(250, 110, 60);
								fourthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
								fourthPath.addRoutine(stowPosition, 0.2);;
								*/
								
								//expiermental 'third cube' from here down
								Path fourthPath = new Path(new Translation2d(271, -97));
								fourthPath.addPoint(228, -87, 100); //240 87
								fourthPath.addRoutine(intakePosition, 0.0);
								fourthPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.OPEN), 0.0);
								fourthPath.addPoint(213, -58, 45); //CHANGE THIS SPEED TO 50, POS 225, 56 | 219 56
								fourthPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.8);

								
								Path fifthPath = new Path(new Translation2d(218, -56)); //225 62
								fifthPath.addPoint(271, -97, 100);  //CHANGE THIS SPEED TO 50
								fifthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.15);
								fifthPath.addRoutine(highScaleOuttakePosition, 0.5);
								
								//end of 3 cube
								
								overallRoutine.addRoutines(initialDrive);
								overallRoutine.addCommands(new SetDrivePath(secondPath, false));
								overallRoutine.addCommands(new SetDrivePath(thirdPath, true));
								//overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-60)));
								//overallRoutine.addCommands(new Delay(0.1));
								//overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-35)));
								//overallRoutine.addCommands(new Delay(0.1));
								overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(55)));
								
								overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FASTEST, SolenoidState.INTAKING));
								overallRoutine.addRoutines(stowPosition); //x

								overallRoutine.addCommands(new Delay(0.5));
								
								/* 2 cube ending 
								overallRoutine.addCommands(new SetDrivePath(fourthPath, true));
								overallRoutine.addCommands(new SetDriveAngle(new Translation2d(202, 39)));
								*/
								/* 3 cube expiermental */
								overallRoutine.addRoutines(intakePosition);
								overallRoutine.addCommands(new SetDriveAngle(new Translation2d(220, -58))); //226, 66
								overallRoutine.addCommands(new SetDrivePath(fourthPath, false));
								overallRoutine.addCommands(new SetDrivePath(fifthPath, true));
								overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(55))); //-55
								overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FASTEST, SolenoidState.INTAKING));
								overallRoutine.addCommands(new Delay(3.0));
								
							} else { // FAR SCALE
								/*
								initialPath.addPoint(180, -115, 120);
								initialPath.addPoint(220, -115, 120);
								initialPath.addPoint(220, -75, 120);
								initialPath.addPoint(220, -20, 50);
								initialPath.addPoint(220, 20, 50);
								initialPath.addPoint(220, 120, 120);

								initialPath.addPoint(230, 135, 60);
								initialPath.addRoutine(highScaleOuttakePosition, 0.8);
								initialPath.addPoint(260, 135, 60);
								initialPath.addPoint(290, 105, 60);
								initialPath.addCommand(
										new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING), 0.4);
								initialPath.addPoint(260, 85, 60);
								initialPath.addRoutine(intakePosition, 0.3);
								initialPath.addPoint(240, 85, 60);
								initialPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.AUTO), 0.3);
								initialPath.addPoint(202, 72, 80);
								initialDrive.addCommands(new SetDrivePath(initialPath, false));

								Path secondPath = new Path(new Translation2d(200, 72));
								secondPath.addPoint(268, 105, 50);
								secondPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP),
										0.1);
								secondPath.addRoutine(scaleOuttakePosition, 0.2);
								secondPath.addPoint(264, 120, 40);
								secondPath.addCommand(
										new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING), 0.4);
								secondPath.addRoutine(stowPosition, 0.6);
								initialDrive.addCommands(new SetDrivePath(secondPath, true));

								overallRoutine.addRoutines(initialDrive);*/
								initialPath.addPoint(200, -115, 120);
								initialPath.addPoint(235, -90, 120);
								initialPath.addPoint(235, -40, 60);
								initialPath.addPoint(235, 40, 120);
								initialPath.addPoint(235, 70, 120);
								initialPath.addPoint(271, 97, 120); //-100 TO -92
								initialPath.addRoutine(highScaleOuttakePosition, 0.8);
								initialDrive.addCommands(new SetDrivePath(initialPath, false));
								initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(-25)));
								overallRoutine.addRoutines(initialDrive);
								overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING));
								overallRoutine.addCommands(new Delay(0.25));
								overallRoutine.addRoutines(stowPosition);
								overallRoutine.addCommands(new SetDriveAngle(new Translation2d(231, 85)));
								
								
								Path secondPath = new Path(new Translation2d(271, 97));
								secondPath.addPoint(228, 95, 35); 
								secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.OPEN), 0.0);
								secondPath.addRoutine(intakePosition, 0.0);
								//secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.85);
								overallRoutine.addCommands(new SetDrivePath(secondPath, false));
								
								Path thirdPath = new Path(new Translation2d(228, 95));
								thirdPath.addPoint(271, 100, 60); //97
								thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
								thirdPath.addRoutine(stowPosition, 0.2);
								thirdPath.addRoutine(highScaleOuttakePosition, 1.0);
								
								overallRoutine.addCommands(new SetDrivePath(thirdPath, true));
								overallRoutine.addCommands(new SetDriveAngle(Rotation.fromDegrees(-55)));
								//overallRoutine.addRoutines(highScaleOuttakePositionBlock);
								overallRoutine.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING));
								overallRoutine.addCommands(new Delay(0.5));

								
								Path fourthPath = new Path(new Translation2d(271, 97));
								fourthPath.addPoint(250, 110, 60);
								fourthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
								fourthPath.addRoutine(stowPosition, 0.2);;
								overallRoutine.addCommands(new SetDrivePath(fourthPath, true));
								overallRoutine.addCommands(new SetDriveAngle(new Translation2d(202, 44)));
							}
						} else {
							initialPath.addPoint(midFieldRightLeadUp, longDistanceSpeed);
							if (scalePos == Position.LEFT) {
								if (mindBusiness) {
									initialPath.addPoint(midFieldLeftPosition, longDistanceSpeed);
									overallRoutine.addCommands(new SetDrivePath(initialPath, false));
									break;
								}
								initialPath.addPoint(midFieldRightPosition, shortDistanceSpeed);
								initialPath.addPoint(midFieldLeftPosition, midFieldSpeed);
								initialPath.addPoint(leftScalePosition, scaleSpeed);
							} else {
								initialPath.addPoint(midFieldRightPosition, scaleSpeed);
								initialPath.addPoint(rightScalePosition, scaleSpeed);
							}
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
							if (scalePos == Position.RIGHT) {
								overallRoutine.addCommands(
										new DriveToPoints(reverseSpeed, true, midFieldRightBackUpPosition));
								overallRoutine.addRoutines(getRightSwitchCube);
								overallRoutine.addCommands(new SetArmAngle(80), new SetElevatorHeight(10),
										new DriveToPoints(reverseSpeed, true, midFieldRightBackUpPosition),
										new SetElevatorHeight(60), new DriveToPoints(scaleSpeed, false,
												midFieldRightPosition, rightScalePosition));
								overallRoutine.addRoutines(placeCubeOnScale);
							}
						}
						break;
					case BOTH:
						/*
						initialPath.addPoint(midFieldRightPosition, longDistanceSpeed);
						if (scalePos == Position.RIGHT) {
							initialPath.addPoint(rightScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						} else {
							
							 * if (mindBusiness) {
							 * initialPath.addPoint(midFieldLeftPosition,
							 * longDistanceSpeed);
							 * overallRoutine.addCommands(new
							 * SetDrivePath(initialPath, false)); break; }
							 
							initialPath.addPoint(midFieldLeftPosition, midFieldSpeed);
							initialPath.addPoint(leftScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						}
						if (switchPos == Position.RIGHT) {
							overallRoutine.addRoutines(toMidFieldRightReverse, getRightSwitchCube);
							overallRoutine.addCommands(new DriveToPoints(50, false, rightSwitchOuttakePositionFar));
						} else {
							overallRoutine.addRoutines(toMidFieldLeftReverse, getLeftSwitchCube);
							overallRoutine.addCommands(new DriveToPoints(50, false, leftSwitchOuttakePositionFar));
						}
						overallRoutine.addRoutines(placeCubeInSwitch);
						*/
						if(switchPos == Position.RIGHT) { //Near switch
							
							//Preload cube in switch
							initialPath.addPoint(new Translation2d(168, -110), 80);
							initialPath.addRoutine(switchOuttakePosition, 0.5);
							initialPath.addPoint(new Translation2d(168, -95), 80);
							initialPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_MIDDLE, SolenoidState.INTAKING), 0.95);
							
							
							//align with next cube 
							Path secondPath = new Path(new Translation2d(168, -90));
							secondPath.addPoint(new Translation2d(238, -90), 80);
							secondPath.addRoutine(intakePosition, 0.1);
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.OPEN), 0.1);
							//secondPath.addPoint(new Translation2d(238, 78), 80);
							secondPath.addPoint(new Translation2d(242, -78), 80); 
							
							//intake next cube
							Path thirdPath = new Path(new Translation2d(242, -78));
							thirdPath.addPoint(new Translation2d(228, -73), 35);
							thirdPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.9);
							
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(226, -73))); //228 75
							initialDrive.addCommands(new SetDrivePath(thirdPath, false));
							
							if(scalePos == Position.RIGHT) {
								//backtrack, static turn, release
								Path fourthPath = new Path(new Translation2d(228, -73));
								fourthPath.addPoint(270, -90, 60);
								fourthPath.addRoutine(highScaleOuttakePosition, 0.7);
								
								Path fifthPath = new Path(new Translation2d(228, -73));
								fifthPath.addPoint(220, -78, 50);
								fifthPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.INTAKING), 0.0);
								fifthPath.addRoutine(stowPosition, 0.1);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(60)));
								initialDrive.addCommands(new Delay(0.1));
								initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(35)));
								initialDrive.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN));
								initialDrive.addCommands(new Delay(1.0));
								
								initialDrive.addCommands(new SetDrivePath(fifthPath, true));
								
							} else if(scalePos == Position.LEFT) {
								//drive backward to the right scale, then drive forward and release
								/*
								Path fourthPath = new Path(new Translation2d(228, 78));
								fourthPath.addPoint(new Translation2d(242, -88), 80);
								fourthPath.addPoint(new Translation2d(238, -88), 80);
								
								Path fifthPath = new Path(new Translation2d(228, -88));
								fifthPath.addPoint(new Translation2d(248, -88), 35);
								fifthPath.addRoutine(scaleOuttakePosition, 0.0);
								fifthPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN), 0.9);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDrivePath(fifthPath, false));
								*/
							}
							
							
						} else { //far switch
							//Drive to the other side of the switch and release preload
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							//initialPath.addPoint(new Translation2d(168, 100), 80); may or may not be needed
							initialPath.addPoint(new Translation2d(237, -110), 80);
							initialPath.addCommand(new SetIntakeState(IntakeState.NEUTRAL, SolenoidState.CLAMP), 0.0);
							initialPath.addRoutine(switchOuttakePosition, 0.9);
							initialPath.addPoint(new Translation2d(237, 56), 80);
							
							
							//initialDrive.addCommands(new SetDrivePath(initialPath, false));
							
							//Grab cube right in front of robot and score
							Path secondPath = new Path(new Translation2d(237, 56));
							secondPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.INTAKING), 0.0);
							secondPath.addPoint(new Translation2d(237, 56), 20);
							secondPath.addRoutine(intakePosition, 0.9);
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.9);
							/*
							secondPath.addCommand(new SetIntakeState(IntakeState.INTAKE, SolenoidState.INTAKING), 0.0);
							secondPath.addRoutine(switchOuttakePosition, 0.9);
							secondPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN), 0.9);
							*/
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(160))); //did that
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							initialDrive.addCommands(new SetDriveAngle(new Translation2d(237,60)));
							
							/*
							if(scalePos == Position.LEFT) {
								Path fourthPath = new Path(new Translation2d(228, -78));
								fourthPath.addPoint(new Translation2d(242, 88), 80);
								fourthPath.addPoint(new Translation2d(238, 88), 80);
								
								Path fifthPath = new Path(new Translation2d(228, 88));
								fifthPath.addPoint(new Translation2d(248, 88), 35);
								fifthPath.addRoutine(scaleOuttakePosition, 0.0);
								fifthPath.addCommand(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN), 0.9);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDrivePath(fifthPath, false));
							} else if(scalePos == Position.RIGHT) {
								Path fourthPath = new Path(new Translation2d(228, -78));
								fourthPath.addPoint(270, -90, 60);
								fourthPath.addRoutine(highScaleOuttakePosition, 0.7);
								
								initialDrive.addCommands(new SetDrivePath(fourthPath, true));
								initialDrive.addCommands(new SetDriveAngle(Rotation.fromDegrees(15)));
								initialDrive.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST, SolenoidState.OPEN));
								
							} */
						}
						overallRoutine.addRoutines(initialDrive);
						break;
					case FORWARD:
						initialPath.addPoint(midFieldRightLeadUp, longDistanceSpeed);
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive);
						break;
					case NONE:
						break;
				}
				break;
		}

		return overallRoutine;
	}
}
