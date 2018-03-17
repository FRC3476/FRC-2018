package org.usfirst.frc.team3476.utility.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoRoutineGenerator {
	
	private static Translation2d robotRightStartPosition = new Translation2d(18, -108);
	private static Translation2d robotCenterStartPosition = new Translation2d(18, 0);
	private static Translation2d robotLeftStartPosition = new Translation2d(18, 108);
	
	private static Translation2d midFieldRightPosition = new Translation2d(240, -108);
	private static Translation2d midFieldLeftPosition = new Translation2d(240, 108);
	
	private static Translation2d rightScalePosition = new Translation2d(264, -100);
	private static Translation2d leftScalePosition = new Translation2d(264, 100);
	
	private static Translation2d rightSwitchCubePositionFar = new Translation2d(224, -90);
	private static Translation2d leftSwitchCubePositionFar = new Translation2d(224, 90);
	
	private static Translation2d rightSwitchOuttakePositionNear = new Translation2d(120, -50);
	private static Translation2d leftSwitchOuttakePositionNear = new Translation2d(120, 50);
	
	private static Translation2d rightSwitchOuttakeLeadUpNear = new Translation2d(60, -50);
	private static Translation2d leftSwitchOuttakeLeadUpNear = new Translation2d(60, 50);
	
	private static Translation2d rightSwitchOuttakePositionFar = new Translation2d(220, -54);
	private static Translation2d leftSwitchOuttakePositionFar = new Translation2d(220, 54);
	
	private static Translation2d rightSwitchOuttakeLeadUpFar = new Translation2d(260, -54);
	private static Translation2d leftSwitchOuttakeLeadUpFar = new Translation2d(260, 54);
	
	
	private static AutoRoutine toMidFieldRight;
	private static AutoRoutine toMidFieldLeft;
	private static AutoRoutine placeCubeOnRightScale;
	private static AutoRoutine placeCubeOnLeftScale;
	private static AutoRoutine getRightSwitchCube;
	private static AutoRoutine getLeftSwitchCube;
	private static AutoRoutine placeCubeInRightSwitchFar;
	private static AutoRoutine placeCubeInLeftSwitchFar;
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
		toMidFieldRight = new AutoRoutine(); //Drives to Mid Field Right Position from Current Location
		toMidFieldRight.addCommands(new DriveToPoints(100, false, midFieldRightPosition));
		
		toMidFieldLeft = new AutoRoutine();
		toMidFieldLeft.addCommands(new DriveToPoints(100, false, midFieldLeftPosition));
		
		placeCubeOnRightScale = new AutoRoutine(); //Puts Cube on Right Scale from Mid Field Right Position, then backs up to Mid Field Right Position
		placeCubeOnRightScale.addCommands(new SetElevatorHeight(60), new SetArmAngle(80), new Delay(1),
				new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.75), new SetElevatorHeight(10),
				new SetIntakeState(IntakeState.GRIP), new DriveToPoints(50, true, midFieldRightPosition));
		
		placeCubeOnLeftScale = new AutoRoutine();
		placeCubeOnLeftScale.addCommands(new DriveToPoints(50, false, midFieldLeftPosition, leftScalePosition), new SetElevatorHeight(60), 
				new SetArmAngle(80), new Delay(.5),	new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.5), new SetElevatorHeight(10),
				new SetIntakeState(IntakeState.GRIP), new DriveToPoints(50, true, midFieldLeftPosition));
		

		getRightSwitchCube = new AutoRoutine(); //Grab Switch Cube, then back up to Mid Field Right Position
		getRightSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE), new SetElevatorHeight(Constants.ElevatorDownHeight), new SetArmAngle(Constants.ArmDownDegrees), new DriveToPoints(50, false, midFieldRightPosition, rightSwitchCubePositionFar),
				new Delay(1), new SetIntakeState(IntakeState.GRIP), new DriveToPoints(50, true, midFieldRightPosition));
		
		getLeftSwitchCube = new AutoRoutine();
		getLeftSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE), new DriveToPoints(50, false, midFieldLeftPosition, leftSwitchCubePositionFar),
				new SetIntakeState(IntakeState.GRIP), new DriveToPoints(50, true, midFieldLeftPosition));

		
		placeCubeInRightSwitchFar = new AutoRoutine();
		placeCubeInRightSwitchFar.addCommands(new SetElevatorHeight(10), new SetArmAngle(80), new DriveToPoints(50, false, rightSwitchOuttakeLeadUpFar, rightSwitchOuttakePositionFar),
				new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(1), new SetIntakeState(IntakeState.GRIP));
		
		placeCubeInLeftSwitchFar = new AutoRoutine();
		placeCubeInLeftSwitchFar.addCommands(new SetElevatorHeight(10), new SetArmAngle(80), new DriveToPoints(50, false, leftSwitchOuttakeLeadUpFar, leftSwitchOuttakePositionFar),
				new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(1), new SetIntakeState(IntakeState.GRIP));
		
		placeCubeInSwitch = new AutoRoutine();
		placeCubeInSwitch.addCommands(new SetElevatorHeight(10), new SetArmAngle(80), new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.75), new SetIntakeState(IntakeState.GRIP));		
	}
	


	public static AutoRoutine generate(String gameMsg, PathOption option, StartPosition position) {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
		
		if(gameMsg.charAt(0) == 'l')
			switchPos = Position.LEFT;
		else
			switchPos = Position.RIGHT;
		
		if(gameMsg.charAt(1) == 'l')
			scalePos = Position.LEFT;
		else
			scalePos = Position.RIGHT;
				
		switch (position)
		{
			case LEFT:
				RobotTracker.getInstance().setInitialTranslation(robotLeftStartPosition);
				switch(option)
				{
					case SWITCH:
						initialPath = new Path(robotLeftStartPosition);
						if (switchPos == Position.LEFT)
						{
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, 80);
							initialPath.addPoint(leftSwitchOuttakePositionNear, 100);
						}
						else
						{
							initialPath.addPoint(rightSwitchOuttakeLeadUpNear, 80);
							initialPath.addPoint(rightSwitchOuttakePositionNear, 100);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
						break;
					case SCALE:
						break;
					case BOTH:
						initialPath = new Path(robotLeftStartPosition);
						initialPath.addPoint(midFieldLeftPosition, 100);
						if (scalePos == Position.RIGHT)
						{
							initialPath.addPoint(midFieldRightPosition, 100);
							initialPath.addPoint(rightScalePosition, 50);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnRightScale, getRightSwitchCube);
						}
						else
						{
							initialPath.addPoint(leftScalePosition, 50);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnLeftScale, getLeftSwitchCube);
						}
						if (switchPos == Position.RIGHT)
						{
							overallRoutine.addRoutines(placeCubeInRightSwitchFar);
						}
						else
						{
							overallRoutine.addRoutines(placeCubeInLeftSwitchFar);
						}
						break;
					case FORWARD:
						break;
					case NONE:
						break;
				}
				break;
			case CENTER:
				RobotTracker.getInstance().setInitialTranslation(robotCenterStartPosition);
				switch(option)
				{
					case SWITCH:
						initialPath = new Path(robotCenterStartPosition);
						if (switchPos == Position.LEFT)
						{
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, 80);
							initialPath.addPoint(leftSwitchOuttakePositionNear, 100);
						}
						else
						{
							initialPath.addPoint(rightSwitchOuttakeLeadUpNear, 80);
							initialPath.addPoint(rightSwitchOuttakePositionNear, 100);
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
						initialPath = new Path(robotRightStartPosition);
						if (switchPos == Position.LEFT)
						{
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, 80);
							initialPath.addPoint(leftSwitchOuttakePositionNear, 100);
						}
						else
						{
							initialPath.addPoint(rightSwitchOuttakeLeadUpNear, 80);
							initialPath.addPoint(rightSwitchOuttakePositionNear, 100);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
						break;
					case SCALE:
						break;
					case BOTH:
						initialPath.addPoint(midFieldRightPosition, 100);
						if (scalePos == Position.RIGHT)
						{
							initialPath.addPoint(rightScalePosition, 50);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnRightScale, getRightSwitchCube);
						}
						else
						{
							initialPath.addPoint(midFieldLeftPosition, 100);
							initialPath.addPoint(leftScalePosition, 50);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							overallRoutine.addRoutines(initialDrive, placeCubeOnLeftScale, getLeftSwitchCube);
						}
						if (switchPos == Position.RIGHT)
						{
							overallRoutine.addRoutines(placeCubeInRightSwitchFar);
						}
						else
						{
							overallRoutine.addRoutines(placeCubeInLeftSwitchFar);
						}
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
