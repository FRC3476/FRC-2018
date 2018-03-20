package org.usfirst.frc.team3476.utility.auto;

import java.util.ArrayList;
import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class AutoRoutineGenerator {
	
	private static Translation2d robotRightStartPosition = new Translation2d(20, -115);
	private static Translation2d robotCenterStartPosition = new Translation2d(20, 0);
	private static Translation2d robotLeftStartPosition = new Translation2d(20, 115);
	
	private static Translation2d midFieldRightPosition = new Translation2d(240, -108);
	private static Translation2d midFieldLeftPosition = new Translation2d(240, 108);
	
	private static Translation2d midFieldRightBackUpPosition = new Translation2d(260, -128);
	private static Translation2d midFieldLeftBackUpPosition = new Translation2d(260, 128);
	
	private static Translation2d midFieldRightLeadUp = new Translation2d(120, -108);
	private static Translation2d midFieldLeftLeadUp = new Translation2d(120, 108);
	
	private static Translation2d rightScalePosition = new Translation2d(280, -94);
	private static Translation2d leftScalePosition = new Translation2d(280, 94);
	
	private static Translation2d rightSwitchCubePositionFar = new Translation2d(232, -86);
	private static Translation2d leftSwitchCubePositionFar = new Translation2d(232, 86);
	
	private static Translation2d rightSwitchOuttakePositionNear = new Translation2d(120, -50);
	private static Translation2d leftSwitchOuttakePositionNear = new Translation2d(120, 50);
	
	private static Translation2d rightSwitchOuttakeLeadUpNear = new Translation2d(70, -50);
	private static Translation2d leftSwitchOuttakeLeadUpNear = new Translation2d(70, 50);
	
	private static Translation2d rightSwitchOuttakePositionFar = new Translation2d(226, -46);
	private static Translation2d leftSwitchOuttakePositionFar = new Translation2d(226, 46);
	
	private static Translation2d rightSwitchLeadUpFar = new Translation2d(260, -72);
	private static Translation2d leftSwitchLeadUpFar = new Translation2d(260, 72);
	
	private static double switchSpeed = 40;
	private static double scaleSpeed = 50;
	private static double longDistanceSpeed = 140;
	private static double shortDistanceSpeed = 60;
	private static double midFieldSpeed = 70;
	private static double reverseSpeed = 60;
	
	private static AutoRoutine toMidFieldRightReverse;
	private static AutoRoutine toMidFieldLeftReverse;
	private static AutoRoutine placeCubeOnScale;
	private static AutoRoutine getRightSwitchCube;
	private static AutoRoutine getLeftSwitchCube;
	private static AutoRoutine placeCubeInSwitch;
	private static AutoRoutine initialDrive;
		
	private static Position switchPos;
	private static Position scalePos;
	
	public enum Position
	{
		LEFT, RIGHT
	}
	
	public enum PathOption
	{
		SCALE, SWITCH, BOTH, FORWARD, NONE
	}
	
	public enum StartPosition
	{
		LEFT, CENTER, RIGHT
	}
	
	static 
	{		
		toMidFieldRightReverse = new AutoRoutine(); //Drives to Mid Field Right Position from Current Location
		toMidFieldRightReverse.addCommands(new DriveToPoints(reverseSpeed, true, midFieldRightPosition));
		
		toMidFieldLeftReverse = new AutoRoutine();
		toMidFieldLeftReverse.addCommands(new DriveToPoints(reverseSpeed, true, midFieldLeftPosition));
		
		placeCubeOnScale = new AutoRoutine(); //Puts Cube on Right Scale from Mid Field Right Position, then backs up to Mid Field Right Position
		placeCubeOnScale.addCommands(new SetArmAngle(80, true), new SetElevatorHeight(60, true),
				new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.75), new SetElevatorHeight(10),
				new SetIntakeState(IntakeState.GRIP));
		

		getRightSwitchCube = new AutoRoutine(); //Grab Switch Cube, then back up to Mid Field Right Position
		getRightSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE), new SetElevatorHeight(Constants.ElevatorDownHeight),
				new SetArmAngle(Constants.ArmIntakeDegrees), new DriveToPoints(switchSpeed, false, rightSwitchCubePositionFar),
				new Delay(.5), new SetIntakeState(IntakeState.GRIP), new Delay(.5));
		
		getLeftSwitchCube = new AutoRoutine();
		getLeftSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE), new SetElevatorHeight(Constants.ElevatorDownHeight), 
				new SetArmAngle(Constants.ArmIntakeDegrees), new DriveToPoints(switchSpeed, false, leftSwitchCubePositionFar),
				new Delay(.5), new SetIntakeState(IntakeState.GRIP), new Delay(.5));
		
		placeCubeInSwitch = new AutoRoutine();
		placeCubeInSwitch.addCommands(new SetElevatorHeight(10), new SetArmAngle(40, true), new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.75), new SetIntakeState(IntakeState.GRIP), new SetElevatorHeight(10), new SetArmAngle(80));		
	}
	


	public static AutoRoutine generate(String gameMsg, PathOption option, StartPosition position) {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
		initialDrive.addCommands(new HomeElevator(), new SetElevatorHeight(0), new SetArmAngle(80, true));
		
		if(gameMsg.charAt(0) == 'l') {
			switchPos = Position.LEFT;
		} else {
			switchPos = Position.RIGHT;
		}
		if(gameMsg.charAt(1) == 'l') {
			scalePos = Position.LEFT;
		} else {
			scalePos = Position.RIGHT;
		}
		
		switch (position)
		{
			case LEFT:
				RobotTracker.getInstance().setInitialTranslation(robotLeftStartPosition);
				initialPath = new Path(robotLeftStartPosition);
				switch(option)
				{
					case SWITCH:
						if (switchPos == Position.LEFT)
						{
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(leftSwitchOuttakePositionNear, switchSpeed);
						}
						else
						{
							initialPath.addPoint(rightSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(rightSwitchOuttakePositionNear, switchSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
						break;
					case SCALE:
						initialPath.addPoint(midFieldLeftLeadUp, longDistanceSpeed);
						if (scalePos == Position.LEFT)
						{
							initialPath.addPoint(midFieldLeftPosition, scaleSpeed);
							initialPath.addPoint(leftScalePosition, scaleSpeed);
						}
						else
						{
							initialPath.addPoint(midFieldLeftPosition, shortDistanceSpeed);
							initialPath.addPoint(midFieldRightPosition, midFieldSpeed);
							initialPath.addPoint(rightScalePosition, scaleSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						if (scalePos == Position.LEFT)
						{
							overallRoutine.addCommands(new DriveToPoints(reverseSpeed, true, midFieldLeftBackUpPosition));
							overallRoutine.addRoutines(getLeftSwitchCube);
							overallRoutine.addCommands(new SetArmAngle(80), new SetElevatorHeight(10), new DriveToPoints(reverseSpeed, true, midFieldLeftBackUpPosition), new SetElevatorHeight(60), new DriveToPoints(scaleSpeed, false, midFieldLeftPosition, leftScalePosition));
							overallRoutine.addRoutines(placeCubeOnScale);
						}
						break;
					case BOTH:
						initialPath.addPoint(midFieldLeftPosition, longDistanceSpeed);
						if (scalePos == Position.RIGHT)
						{
							initialPath.addPoint(midFieldRightPosition, midFieldSpeed);
							initialPath.addPoint(rightScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						}
						else
						{
							initialPath.addPoint(leftScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						}
						
						if (switchPos == Position.RIGHT)
						{
							overallRoutine.addRoutines(toMidFieldRightReverse, getRightSwitchCube);
							overallRoutine.addCommands(new DriveToPoints(switchSpeed, false, rightSwitchOuttakePositionFar));
						}
						else
						{
							overallRoutine.addRoutines(toMidFieldLeftReverse, getLeftSwitchCube);
							overallRoutine.addCommands(new DriveToPoints(switchSpeed, false, leftSwitchOuttakePositionFar));
						}
						
						overallRoutine.addRoutines(placeCubeInSwitch);
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
				RobotTracker.getInstance().setInitialTranslation(robotCenterStartPosition);
				initialPath = new Path(robotCenterStartPosition);
				switch(option)
				{
					case SWITCH:
						if (switchPos == Position.LEFT)
						{
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(leftSwitchOuttakePositionNear, switchSpeed);
						}
						else
						{
							initialPath.addPoint(rightSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(rightSwitchOuttakePositionNear, switchSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
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
				RobotTracker.getInstance().setInitialTranslation(robotRightStartPosition);
				initialPath = new Path(robotRightStartPosition);
				switch(option)
				{
					case SWITCH:
						if (switchPos == Position.LEFT)
						{
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(leftSwitchOuttakePositionNear, switchSpeed);
						}
						else
						{
							initialPath.addPoint(rightSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(rightSwitchOuttakePositionNear, switchSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
						break;
					case SCALE:
						initialPath.addPoint(midFieldRightLeadUp, longDistanceSpeed);
						if (scalePos == Position.LEFT)
						{
							initialPath.addPoint(midFieldRightPosition, shortDistanceSpeed);
							initialPath.addPoint(midFieldLeftPosition, midFieldSpeed);
							initialPath.addPoint(leftScalePosition, scaleSpeed);
						}
						else
						{
							initialPath.addPoint(midFieldRightPosition, scaleSpeed);
							initialPath.addPoint(rightScalePosition, scaleSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						break;
					case BOTH:
						initialPath.addPoint(midFieldRightPosition, longDistanceSpeed);
						if (scalePos == Position.RIGHT)
						{
							initialPath.addPoint(rightScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						}
						else
						{
							initialPath.addPoint(midFieldLeftPosition, midFieldSpeed);
							initialPath.addPoint(leftScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						}						
						if (switchPos == Position.RIGHT)
						{
							overallRoutine.addRoutines(toMidFieldRightReverse, getRightSwitchCube);
							overallRoutine.addCommands(new DriveToPoints(50, false, rightSwitchOuttakePositionFar));
						}
						else
						{
							overallRoutine.addRoutines(toMidFieldLeftReverse, getLeftSwitchCube);
							overallRoutine.addCommands(new DriveToPoints(50, false, leftSwitchOuttakePositionFar));
						}
						overallRoutine.addRoutines(placeCubeInSwitch);
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
