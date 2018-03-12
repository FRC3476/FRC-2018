package org.usfirst.frc.team3476.utility.auto;

import java.util.HashMap;
import java.util.List;

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
	
	private static Translation2d rightScalePosition = new Translation2d(264, -108);
	private static Translation2d leftScalePosition = new Translation2d(264, 108);
	
	private static Translation2d rightSwitchCubePosition = new Translation2d(224, -90);
	private static Translation2d leftSwitchCubePosition = new Translation2d(224, 90);
	
	
	public enum PathOption
	{
		SCALE, SWITCH, BOTH, FORWARD, NONE
	}
	
	public enum StartPosition
	{
		LEFT, CENTER, RIGHT
	}


	public static AutoRoutine generate(String gameMsg, PathOption option, StartPosition position) {
		AutoRoutine overallRoutine = new AutoRoutine();	
		
		AutoRoutine toMidFieldRight = new AutoRoutine(); //Drives to Mid Field Right Position from Current Location
		toMidFieldRight.addCommands(new DriveToPoint(midFieldRightPosition, 100, false));
		
		AutoRoutine cubeOnRightScale = new AutoRoutine(); //Puts Cube on Right Scale from Mid Field Right Position, then backs up to Mid Field Right Position
		Path midFieldRightToScale = new Path(midFieldRightPosition);
		midFieldRightToScale.addPoint(rightScalePosition, 50);
		cubeOnRightScale.addCommands(new SetDrivePath(midFieldRightToScale, false, false), new SetElevatorHeight(60), new SetArmAngle(80), new Delay(.5),
				new SetIntakeState(IntakeState.OUTTAKE_FAST), new Delay(.5), new SetElevatorHeight(10), new SetIntakeState(IntakeState.GRIP), new DriveToPoint(midFieldRightPosition, 50, true));
		
		AutoRoutine getRightSwitchCube = new AutoRoutine(); //From Mid Field Right Position, grabs Switch Cube, then backs up to Mid Field Right Position
		Path midFieldRightToSwitchCube = new Path(midFieldRightPosition);
		midFieldRightToSwitchCube.addPoint(rightSwitchCubePosition, 50);
		getRightSwitchCube.addCommands(new SetIntakeState(IntakeState.INTAKE), new SetDrivePath(midFieldRightToSwitchCube, false), new SetIntakeState(IntakeState.GRIP));
		
		
		switch(option)
		{
			case SCALE:
				switch (position)
				{
					case LEFT:
						break;
					case RIGHT:
						RobotTracker.getInstance().setInitialTranslation(robotRightStartPosition);
						overallRoutine.addRoutines(toMidFieldRight, cubeOnRightScale);
						break;
					case CENTER:
						break;
				}
				break;
			case SWITCH:
				break;
			case BOTH:
				break;
			case FORWARD:
				break;
			case NONE:
				break;
		}
		
		
		if(gameMsg.charAt(0) == 'l') {
			
		} else {
			
		}
		
		if(gameMsg.charAt(1) == 'l') {
			
		} else {
			
		}
		
		return overallRoutine;
	}
	
}
