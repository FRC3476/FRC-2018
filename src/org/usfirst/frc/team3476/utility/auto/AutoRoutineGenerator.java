package org.usfirst.frc.team3476.utility.auto;

import java.util.ArrayList;
import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class AutoRoutineGenerator {
	
	private static Translation2d robotRightStartPosition = new Translation2d(20, -115);
	private static Translation2d robotCenterStartPosition = new Translation2d(20, -5);
	private static Translation2d robotLeftStartPosition = new Translation2d(20, 115);
	
	private static Translation2d midFieldRightPosition = new Translation2d(240, -118);
	private static Translation2d midFieldLeftPosition = new Translation2d(240, 108);
	
	private static Translation2d midFieldRightBackUpPosition = new Translation2d(260, -128);
	private static Translation2d midFieldLeftBackUpPosition = new Translation2d(260, 128);
	
	private static Translation2d midFieldRightLeadUp = new Translation2d(120, -108);
	private static Translation2d midFieldLeftLeadUp = new Translation2d(120, 108);
	
	private static Translation2d rightScalePosition = new Translation2d(270, -108);
	private static Translation2d leftScalePosition = new Translation2d(270, 86);
	private static Translation2d leftScalePositionSecondCube = new Translation2d(275, 86);
	
	private static Translation2d rightSwitchCubePositionFar = new Translation2d(226, -80);
	private static Translation2d leftSwitchCubePositionFar = new Translation2d(226, 80);
	
	private static Translation2d rightSwitchOuttakePositionNear = new Translation2d(118, -50);
	private static Translation2d leftSwitchOuttakePositionNear = new Translation2d(118, 50);
	
	private static Translation2d rightSwitchOuttakeLeadUpNear = new Translation2d(70, -50);
	private static Translation2d leftSwitchOuttakeLeadUpNear = new Translation2d(70, 50);
	
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
	private static AutoRoutine intakePosition;
	private static AutoRoutine secondIntakePosition;
	private static AutoRoutine switchOuttakePosition;
	private static AutoRoutine stowPosition;
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

		scaleOuttakePosition = new AutoRoutine();
		scaleOuttakePosition.addCommands(new SetElevatorHeight(50), new SetArmAngle(80));
		
		highScaleOuttakePosition = new AutoRoutine();
		highScaleOuttakePosition.addCommands(new SetElevatorHeight(60), new SetArmAngle(80));
		
		intakePosition = new AutoRoutine();
		intakePosition.addCommands(new SetElevatorHeight(Constants.ElevatorDownHeight), new SetArmAngle(Constants.ArmIntakeDegrees));
		
		secondIntakePosition = new AutoRoutine();
		secondIntakePosition.addCommands(new SetElevatorHeight(Constants.ElevatorDownHeight + 11), new SetArmAngle(Constants.ArmIntakeDegrees));
		
		switchOuttakePosition = new AutoRoutine();
		switchOuttakePosition.addCommands(new SetElevatorHeight(25), new SetArmAngle(0));
		
		stowPosition = new AutoRoutine();
		stowPosition.addCommands(new SetElevatorHeight(10), new SetArmAngle(80));
		
		placeCubeInSwitch = new AutoRoutine();
		placeCubeInSwitch.addRoutines(switchOuttakePosition);
		placeCubeInSwitch.addCommands(new SetIntakeState(IntakeState.OUTTAKE_FAST),
				new Delay(.6), new SetIntakeState(IntakeState.GRIP));
		placeCubeInSwitch.addRoutines(stowPosition);		
	}
	


	public static AutoRoutine generate(String gameMsg, PathOption option, StartPosition position, boolean mindBusiness) {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
		initialDrive.addCommands(new HomeElevator(), new SetElevatorHeight(0), new SetArmAngle(80, false));
		System.out.println(gameMsg);
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
		if(mindBusiness) {
			if(position == StartPosition.LEFT) {
				if(scalePos != Position.LEFT){
					if(switchPos != Position.LEFT){
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
				if(scalePos != Position.RIGHT){
					if(switchPos != Position.RIGHT){
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
		switch (position)
		{
			case LEFT:
				System.out.println("start left");
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
							//initialPath.addPoint(rightSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							//initialPath.addPoint(rightSwitchOuttakePositionNear, switchSpeed);
						}
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						overallRoutine.addRoutines(initialDrive, placeCubeInSwitch);
						break;
					case SCALE:
						/*
						initialPath.addPoint(midFieldLeftLeadUp, longDistanceSpeed);
						if (scalePos == Position.LEFT)
						{
							initialPath.addPoint(midFieldLeftPosition, shortDistanceSpeed);
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
							overallRoutine.addCommands(new SetArmAngle(80), new SetElevatorHeight(10), new DriveToPoints(reverseSpeed, true, midFieldLeftBackUpPosition), new SetElevatorHeight(60), new DriveToPoints(scaleSpeed - 15, false, midFieldLeftPosition, leftScalePositionSecondCube));
							overallRoutine.addRoutines(placeCubeOnScale);
						}
						*/
						if(scalePos == Position.LEFT) { //NEAR SCALE
							initialPath.addPoint(230, 135, 120);
							initialPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.8);
							initialPath.addPoint(260, 135, 60);
							initialPath.addPoint(290, 105, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FAST), 0.4);
							initialPath.addPoint(260, 85, 60);
							initialPath.addRoutineToCurrentSegment(intakePosition, 0.3);
							initialPath.addPoint(240, 85, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0.3);
							initialPath.addPoint(204, 72, 80);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							
							Path secondPath = new Path(new Translation2d(204, 72));
							secondPath.addPoint(268, 105, 50);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.1);
							secondPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.2);
							secondPath.addPoint(264, 120, 40);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FASTEST), 0.4);
							secondPath.addRoutineToCurrentSegment(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							
							Path thirdPath = new Path(new Translation2d(264, 120));
							thirdPath.addPoint(264, 105, 60);
							thirdPath.addPoint(230, 90, 60);
							thirdPath.addRoutineToCurrentSegment(intakePosition, 0.2);
							thirdPath.addPoint(190, 58, 60);
							thirdPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0.2);
							initialDrive.addCommands(new SetDrivePath(thirdPath, false));
							
							Path fourthPath = new Path(new Translation2d(190, 58));
							fourthPath.addPoint(230, 90, 60);
							fourthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.5);
							fourthPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.6);
							fourthPath.addPoint(268, 105, 60);
							fourthPath.addPoint(264, 120, 40);
							fourthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FASTEST), 0.4);
							fourthPath.addRoutineToCurrentSegment(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(fourthPath, true));
							
							
							overallRoutine.addRoutines(initialDrive);
						}
						else
						{ //FAR SCALE
							initialPath.addPoint(180, 115, 120);
							initialPath.addPoint(220, 115, 120);
							initialPath.addPoint(220, 75, 120);
							initialPath.addPoint(220, 20, 50);
							initialPath.addPoint(220, -20, 50);
							initialPath.addPoint(220, -120, 120);
							
							initialPath.addPoint(230, -135, 60);
							initialPath.addRoutineToCurrentSegment(highScaleOuttakePosition, 0.8);
							initialPath.addPoint(260, -135, 60);
							initialPath.addPoint(290, -105, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FAST), 0.4);
							initialPath.addPoint(260, -85, 60);
							initialPath.addRoutineToCurrentSegment(intakePosition, 0.3);
							initialPath.addPoint(240, -85, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0.3);
							initialPath.addPoint(202, -72, 80);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							
							Path secondPath = new Path(new Translation2d(200, -72));
							secondPath.addPoint(268, -105, 50);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.1);
							secondPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.2);
							secondPath.addPoint(264, -120, 40);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FASTEST), 0.4);
							secondPath.addRoutineToCurrentSegment(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							
							overallRoutine.addRoutines(initialDrive);
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
				System.out.println("start centers");
				RobotTracker.getInstance().setInitialTranslation(robotCenterStartPosition);
				initialPath = new Path(robotCenterStartPosition);
				switch(option)
				{
					case SWITCH:
						System.out.println("switch");
						if(switchPos == Position.LEFT) {
							System.out.println("left");
							initialPath.addPoint(leftSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(leftSwitchOuttakePositionNear, shortDistanceSpeed);
						} else {
							System.out.println("right");
							initialPath.addPoint(rightSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							initialPath.addPoint(rightSwitchOuttakePositionNear, shortDistanceSpeed);					
						}
						initialPath.addRoutineToCurrentSegment(switchOuttakePosition, 0.5);			
						initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE), 0.9);
						initialDrive.addCommands(new SetDrivePath(initialPath, false));
						Path secondPath;
						if(switchPos == Position.LEFT) {
							secondPath = new Path(leftSwitchOuttakePositionNear);
							secondPath.addPoint(50,  50, shortDistanceSpeed);
						} else {
							secondPath = new Path(rightSwitchOuttakePositionNear);
							secondPath.addPoint(50,  -50, shortDistanceSpeed);				
						}
						secondPath.addRoutineToCurrentSegment(intakePosition, 0.5);
						initialDrive.addCommands(new SetDrivePath(secondPath, true));
						Path thirdPath;
						if(switchPos == Position.LEFT) {
							thirdPath = new Path(new Translation2d(50, 50));
							thirdPath.addPoint(95, 8, shortDistanceSpeed);
							thirdPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0);
						} else {		
							thirdPath = new Path(new Translation2d(50, -50));
							thirdPath.addPoint(95, -8, shortDistanceSpeed);
							thirdPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0);
						}
						initialDrive.addCommands(new SetDrivePath(thirdPath, false));
						Path fourthPath;
						if(switchPos == Position.LEFT) {
							fourthPath = new Path(new Translation2d(97, 10));
							fourthPath.addPoint(60, 50, shortDistanceSpeed);
						} else {		
							fourthPath = new Path(new Translation2d(97, -10));
							fourthPath.addPoint(60, -50, shortDistanceSpeed);
						}
						fourthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.4);
						fourthPath.addRoutineToCurrentSegment(switchOuttakePosition, 0.5);
						initialDrive.addCommands(new SetDrivePath(fourthPath, true));
						Path fifthPath;
						if(switchPos == Position.LEFT) {
							fifthPath = new Path(new Translation2d(60, 50));
							fifthPath.addPoint(leftSwitchOuttakePositionNear, shortDistanceSpeed);
						} else {		
							fifthPath = new Path(new Translation2d(60, -50));
							fifthPath.addPoint(rightSwitchOuttakePositionNear, shortDistanceSpeed);
						}
						fifthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE), 0.9);
						initialDrive.addCommands(new SetDrivePath(fifthPath, false));
						
						Path sixthPath;
						if(switchPos == Position.LEFT) {
							sixthPath = new Path(leftSwitchOuttakePositionNear);
							sixthPath.addPoint(60,  50, shortDistanceSpeed);
						} else {
							sixthPath = new Path(rightSwitchOuttakePositionNear);
							sixthPath.addPoint(60,  -50, shortDistanceSpeed);				
						}
						sixthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.7);
						sixthPath.addRoutineToCurrentSegment(secondIntakePosition, 0.8);
						initialDrive.addCommands(new SetDrivePath(sixthPath, true));
						Path seventhPath;
						if(switchPos == Position.LEFT) {
							seventhPath = new Path(new Translation2d(60, 50));
							seventhPath.addPoint(108, 6, shortDistanceSpeed);
							seventhPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0);
						} else {		
							seventhPath = new Path(new Translation2d(60, -50));
							seventhPath.addPoint(108, -6, shortDistanceSpeed);
							seventhPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0);
						}
						initialDrive.addCommands(new SetDrivePath(seventhPath, false));
						Path eightPath;
						if(switchPos == Position.LEFT) {
							eightPath = new Path(new Translation2d(108, 6));
							eightPath.addPoint(50, 50, shortDistanceSpeed);
						} else {		
							eightPath = new Path(new Translation2d(108, -6));
							eightPath.addPoint(50, -50, shortDistanceSpeed);
						}
						eightPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.7);
						eightPath.addRoutineToCurrentSegment(switchOuttakePosition, 0.8);
						initialDrive.addCommands(new SetDrivePath(eightPath, true));
						Path ninthPath;
						if(switchPos == Position.LEFT) {
							ninthPath = new Path(new Translation2d(60, 50));
							ninthPath.addPoint(leftSwitchOuttakePositionNear, shortDistanceSpeed);
						} else {		
							ninthPath = new Path(new Translation2d(60, -50));
							ninthPath.addPoint(rightSwitchOuttakePositionNear, shortDistanceSpeed);
						}
						ninthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE), 0.9);
						initialDrive.addCommands(new SetDrivePath(ninthPath, false));
						overallRoutine.addRoutines(initialDrive);
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
				switch(option)
				{
					case SWITCH:
						if (switchPos == Position.LEFT)
						{
							//initialPath.addPoint(leftSwitchOuttakeLeadUpNear, shortDistanceSpeed);
							//initialPath.addPoint(leftSwitchOuttakePositionNear, switchSpeed);
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
						/*
						initialPath.addPoint(midFieldRightLeadUp, longDistanceSpeed);
						if (scalePos == Position.LEFT)
						{
							if (mindBusiness)
							{
								initialPath.addPoint(midFieldLeftPosition, longDistanceSpeed);
								overallRoutine.addCommands(new SetDrivePath(initialPath, false));
								break;
							}
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
						if (scalePos == Position.RIGHT)
						{
							overallRoutine.addCommands(new DriveToPoints(reverseSpeed, true, midFieldRightBackUpPosition));
							overallRoutine.addRoutines(getRightSwitchCube);
							overallRoutine.addCommands(new SetArmAngle(80), new SetElevatorHeight(10), new DriveToPoints(reverseSpeed, true, midFieldRightBackUpPosition), new SetElevatorHeight(60), new DriveToPoints(scaleSpeed, false, midFieldRightPosition, rightScalePosition));
							overallRoutine.addRoutines(placeCubeOnScale);
						}
						*/
						if(scalePos == Position.RIGHT) { //NEAR SCALE
							initialPath.addPoint(230, -135, 120);
							initialPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.8);
							initialPath.addPoint(260, -135, 60);
							initialPath.addPoint(290, -105, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FAST), 0.4);
							initialPath.addPoint(260, -85, 60);
							initialPath.addRoutineToCurrentSegment(intakePosition, 0.3);
							initialPath.addPoint(232, -85, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0.3);
							initialPath.addPoint(204, -72, 80);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							
							Path secondPath = new Path(new Translation2d(204, -72));
							secondPath.addPoint(232, -85, 60);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.8);
							secondPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.9);
							secondPath.addPoint(268, -105, 60);
							secondPath.addPoint(264, -120, 40);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FASTEST), 0.4);
							secondPath.addRoutineToCurrentSegment(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							
							Path thirdPath = new Path(new Translation2d(264, -120));
							thirdPath.addPoint(264, -105, 60);
							thirdPath.addPoint(232, -85, 60);
							thirdPath.addRoutineToCurrentSegment(intakePosition, 0.2);
							thirdPath.addPoint(204, -54, 60);
							thirdPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0.3);
							initialDrive.addCommands(new SetDrivePath(thirdPath, false));
							
							Path fourthPath = new Path(new Translation2d(204, -54));
							fourthPath.addPoint(232, -85, 60);
							fourthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.8);
							fourthPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.9);
							fourthPath.addPoint(268, -105, 60);
							fourthPath.addPoint(264, -120, 40);
							fourthPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FASTEST), 0.4);
							fourthPath.addRoutineToCurrentSegment(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(fourthPath, true));							
							
							overallRoutine.addRoutines(initialDrive);
						}
						else
						{ //FAR SCALE
							initialPath.addPoint(180, -115, 120);
							initialPath.addPoint(220, -115, 120);
							initialPath.addPoint(220, -75, 120);
							initialPath.addPoint(220, -20, 50);
							initialPath.addPoint(220, 20, 50);
							initialPath.addPoint(220, 120, 120);
							
							initialPath.addPoint(230, 135, 60);
							initialPath.addRoutineToCurrentSegment(highScaleOuttakePosition, 0.8);
							initialPath.addPoint(260, 135, 60);
							initialPath.addPoint(290, 105, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FAST), 0.4);
							initialPath.addPoint(260, 85, 60);
							initialPath.addRoutineToCurrentSegment(intakePosition, 0.3);
							initialPath.addPoint(240, 85, 60);
							initialPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.INTAKE), 0.3);
							initialPath.addPoint(202, 72, 80);
							initialDrive.addCommands(new SetDrivePath(initialPath, false));
							
							Path secondPath = new Path(new Translation2d(200, 72));
							secondPath.addPoint(268, 105, 50);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.GRIP), 0.1);
							secondPath.addRoutineToCurrentSegment(scaleOuttakePosition, 0.2);
							secondPath.addPoint(264, 120, 40);
							secondPath.addCommandToCurrentSegment(new SetIntakeState(IntakeState.OUTTAKE_FASTEST), 0.4);
							secondPath.addRoutineToCurrentSegment(stowPosition, 0.6);
							initialDrive.addCommands(new SetDrivePath(secondPath, true));
							
							overallRoutine.addRoutines(initialDrive);
						}
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
							/*if (mindBusiness)
							{
								initialPath.addPoint(midFieldLeftPosition, longDistanceSpeed);
								overallRoutine.addCommands(new SetDrivePath(initialPath, false));
								break;
							}*/
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
