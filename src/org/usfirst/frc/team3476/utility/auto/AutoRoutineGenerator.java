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
	
	private static Translation2d midFieldRightLeadUp = new Translation2d(120, -108);
	private static Translation2d midFieldLeftLeadUp = new Translation2d(120, 108);
	
	private static Translation2d rightScalePosition = new Translation2d(285, -94);
	private static Translation2d leftScalePosition = new Translation2d(285, 94);
	
	private static Translation2d rightSwitchCubePositionFar = new Translation2d(234, -54);
	private static Translation2d leftSwitchCubePositionFar = new Translation2d(234, 54);
	
	private static Translation2d rightSwitchOuttakePositionNear = new Translation2d(120, -50);
	private static Translation2d leftSwitchOuttakePositionNear = new Translation2d(120, 50);
	
	private static Translation2d rightSwitchOuttakeLeadUpNear = new Translation2d(70, -50);
	private static Translation2d leftSwitchOuttakeLeadUpNear = new Translation2d(70, 50);
	
	private static Translation2d rightSwitchOuttakePositionFar = new Translation2d(226, -54);
	private static Translation2d leftSwitchOuttakePositionFar = new Translation2d(226, 54);
	
	private static Translation2d rightSwitchOuttakeLeadUpFar = new Translation2d(260, -54);
	private static Translation2d leftSwitchOuttakeLeadUpFar = new Translation2d(260, 54);
	
	private static double switchSpeed = 40;
	private static double scaleSpeed = 50;
	private static double longDistanceSpeed = 100;
	private static double shortDistanceSpeed = 30;
	private static double midFieldSpeed = 40;
	
	
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
		toMidFieldRightReverse.addCommands(new DriveToPoints(50, true, midFieldRightPosition));
		
		toMidFieldLeftReverse = new AutoRoutine();
		toMidFieldLeftReverse.addCommands(new DriveToPoints(50, true, midFieldLeftPosition));
		
		placeCubeOnScale = new AutoRoutine(); //Puts Cube on Right Scale from Mid Field Right Position, then backs up to Mid Field Right Position
		placeCubeOnScale.addCommands(new SetArmAngle(80, true), new SetElevatorHeight(60, true),
				new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.75), new SetElevatorHeight(10),
				new SetIntakeState(IntakeState.GRIP));
		

		getRightSwitchCube = new AutoRoutine(); //Grab Switch Cube, then back up to Mid Field Right Position
		getRightSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE), new SetElevatorHeight(Constants.ElevatorDownHeight), new SetArmAngle(Constants.ArmDownDegrees), new DriveToPoints(50, false, midFieldRightPosition, rightSwitchOuttakeLeadUpFar, rightSwitchCubePositionFar),
				new Delay(1), new SetIntakeState(IntakeState.GRIP));
		
		getLeftSwitchCube = new AutoRoutine();
		getLeftSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE), new SetIntakeState(IntakeState.OPEN), new SetElevatorHeight(Constants.ElevatorDownHeight), 
				new SetArmAngle(Constants.ArmDownDegrees), new DriveToPoints(50, false, midFieldLeftPosition, leftSwitchOuttakeLeadUpFar, leftSwitchCubePositionFar),
				new Delay(1), new SetIntakeState(IntakeState.GRIP));
		
		placeCubeInSwitch = new AutoRoutine();
		placeCubeInSwitch.addCommands(new SetElevatorHeight(10), new SetArmAngle(40, true), new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.75), new SetIntakeState(IntakeState.GRIP), new SetElevatorHeight(10), new SetArmAngle(80));		
	}
	


	public static AutoRoutine generate(String gameMsg, PathOption option, StartPosition position) {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
		initialDrive.addCommands(new SetElevatorHeight(0), new SetArmAngle(80, true));
		
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
						initialPath.addPoint(midFieldLeftPosition, shortDistanceSpeed);
						if (scalePos == Position.LEFT)
						{
							initialPath.addPoint(leftScalePosition, scaleSpeed);
						}
						else
						{
							initialPath.addPoint(midFieldRightPosition, midFieldSpeed);
							initialPath.addPoint(rightScalePosition, scaleSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						break;
					case BOTH:
						initialPath.addPoint(midFieldLeftPosition, longDistanceSpeed);
						if (scalePos == Position.RIGHT)
						{
							initialPath.addPoint(midFieldRightPosition, midFieldSpeed);
							initialPath.addPoint(rightScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale, toMidFieldRightReverse, getRightSwitchCube);
						}
						else
						{
							initialPath.addPoint(leftScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale, toMidFieldLeftReverse, getLeftSwitchCube);
						}
						if (switchPos == Position.RIGHT)
						{
							//overallRoutine.addCommands(new DriveToPoints(50, false, rightSwitchOuttakePositionFar));
						}
						else
						{
							overallRoutine.addCommands(new DriveToPoints(50, false, leftSwitchOuttakePositionFar));
						}
						
						overallRoutine.addRoutines(placeCubeInSwitch);
						break;
					case FORWARD:
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
						initialPath.addPoint(midFieldRightPosition, shortDistanceSpeed);
						if (scalePos == Position.LEFT)
						{
							initialPath.addPoint(midFieldLeftPosition, midFieldSpeed);
							initialPath.addPoint(leftScalePosition, scaleSpeed);
						}
						else
						{
							initialPath.addPoint(rightScalePosition, scaleSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeOnScale);
						break;
					case BOTH:
						initialPath.addPoint(midFieldLeftPosition, longDistanceSpeed);
						if (scalePos == Position.RIGHT)
						{
							initialPath.addPoint(rightScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale, toMidFieldRightReverse, getRightSwitchCube);
						}
						else
						{
							initialPath.addPoint(midFieldLeftPosition, midFieldSpeed);
							initialPath.addPoint(leftScalePosition, scaleSpeed);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnScale, toMidFieldLeftReverse, getLeftSwitchCube);
						}
						if (switchPos == Position.RIGHT)
						{
							//overallRoutine.addCommands(new DriveToPoints(50, false, rightSwitchOuttakePositionFar));
						}
						else
						{
							overallRoutine.addCommands(new DriveToPoints(50, false, leftSwitchOuttakePositionFar));
						}
						overallRoutine.addRoutines(placeCubeInSwitch);
						break;
					case FORWARD:
						break;
					case NONE:
						break;
				}
				break;
		}
		
		return overallRoutine;
	}
	
}
