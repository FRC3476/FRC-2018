package org.usfirst.frc.team3476.robot;

import java.time.Duration;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team3476.subsystem.Elevarm;
import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.OrangeDrive.DriveVelocity;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.utility.Controller;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.ThreadScheduler;
import org.usfirst.frc.team3476.utility.auto.AutoCommand;
import org.usfirst.frc.team3476.utility.auto.AutoRoutine;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator;
import org.usfirst.frc.team3476.utility.auto.Delay;
import org.usfirst.frc.team3476.utility.auto.SetArmAngle;
import org.usfirst.frc.team3476.utility.auto.SetDrivePath;
import org.usfirst.frc.team3476.utility.auto.SetElevatorHeight;
import org.usfirst.frc.team3476.utility.auto.SetIntakeState;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator.PathOption;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator.StartPosition;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Rotation;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends IterativeRobot {
	Controller xbox = new Controller(0);
	Controller buttonBox = new Controller(1);
	Controller joystick = new Controller(2);
	OrangeDrive drive = OrangeDrive.getInstance();
	Elevarm elevarm = Elevarm.getInstance();
	RobotTracker tracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	ExecutorService mainExecutor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();
	CameraServer camServer = CameraServer.getInstance();
	LazyTalonSRX climber = new LazyTalonSRX(21);
	
	boolean homed = false;

	Path autoPath;

	@Override
	public void robotInit() {
		scheduler.schedule(drive, Duration.ofMillis(5), mainExecutor);
		scheduler.schedule(tracker, Duration.ofMillis(5), mainExecutor);
		scheduler.schedule(elevarm, Duration.ofMillis(20), mainExecutor);
		camServer.startAutomaticCapture(0);
		camServer.startAutomaticCapture(1);
	}

	@Override
	public void autonomousInit() {
		scheduler.resume();
		drive.resetMotionProfile();
		elevarm.resetMotionProfile();
		elevarm.configArmEncoder();
		drive.stopMovement();
		elevarm.stopMovement();
		tracker.setInitialTranslation(new Translation2d(18, -108));
		tracker.resetOdometry();
		elevarm.homeElevator();
		/*
		AutoRoutine routine = AutoRoutineGenerator.generate("", PathOption.BOTH, StartPosition.RIGHT);
		routine.run();
		*/


		/*
		autoPath = new Path(new Translation2d(18,-108));
		autoPath.addPoint(50, -108, 100);
		autoPath.addPoint(50, -158, 100);
		autoPath = new Path(new Translation2d(18,-108));
		autoPath.addPoint(225, -124, 100);
		autoPath.addPoint(264, -108, 100);
		autoPath.addPoint(50, -108, 100);
		autoPath.addPoint(50, -150, 100);
		drive.setAutoPath(autoPath, false);
		
		while(!drive.isFinished()) {}
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(264, -124, 100);
		drive.setAutoPath(autoPath, true);
		while(!drive.isFinished()) {}
		*/
		/*
		Timer.delay(1);
		elevarm.setArmAngle(70);
		elevarm.setElevatorHeight(10);
		*/
		//Scale Switch
		autoPath = new Path(new Translation2d(18,-108));
		autoPath.addPoint(225, -124, 70);
		autoPath.addPoint(264, -108, 50);
		drive.setAutoPath(autoPath, false);
		while(!drive.isFinished()) {}
		//elevarm.setElevatorHeight(60);
		//Timer.delay(1.1);
		//intake.setIntake(IntakeState.OUTTAKE_FAST);
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(264, -124, 50);
		drive.setAutoPath(autoPath, true);
		//elevarm.setElevarmIntakePosition();
		while(!drive.isFinished()) {}
		//System.out.println("Arm Angle: " + elevarm.getArmAngle());
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(230, -100, 50);
		drive.setAutoPath(autoPath, false);
		//intake.setIntake(IntakeState.INTAKE);
		double start = Timer.getFPGATimestamp();
		while(!drive.isFinished() || Timer.getFPGATimestamp() - start < 2) {}
		//intake.setIntake(IntakeState.GRIP);
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(tracker.getOdometry().translationMat.getX() + 5, tracker.getOdometry().translationMat.getY() - 5, 30);
		drive.setAutoPath(autoPath, true);
		//elevarm.setElevatorHeight(10);
		//elevarm.setArmAngle(40);
		while(!drive.isFinished()) {}
		//intake.setIntake(IntakeState.OUTTAKE_FASTEST);
		

		//Scale To Right
		/*
		autoPath = new Path(new Translation2d(0, 0));
		autoPath.addPoint(210, 0, 100);
		autoPath.addPoint(250, -25, 100);
		drive.setAutoPath(autoPath, false);
		while(!drive.isFinished()){
			
		}
		
		elevarm.setArmAngle(80); //Scale Position
		elevarm.setElevatorHeight(Constants.ElevatorUpHeight);
		Timer.delay(1);
		intake.setIntake(IntakeState.OUTTAKE_FAST);
		*/
		
		//Left Switch from Center
		/*
		autoPath = new Path(new Translation2d(0,0));
		autoPath.addPoint(60, -65, 60);
		autoPath.addPoint(85, -60, 60);
		drive.setAutoPath(autoPath, false);
		elevarm.setArmAngle(40);
		elevarm.setElevatorHeight(20);
		while(!drive.isFinished()){
			if(!DriverStation.getInstance().isAutonomous()){
				break;
			}
		}
		intake.setIntake(IntakeState.OUTTAKE_FAST);
		Timer.delay(0.25);
		intake.setIntake(IntakeState.GRIP);
		elevarm.setElevarmIntakePosition();
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(65, -75, 80);
		drive.setAutoPath(autoPath, true);
		while(!drive.isFinished()){
			if(DriverStation.getInstance().isOperatorControl()){
				break;
			}
		}
		intake.setIntake(IntakeState.INTAKE);
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(90, -25, 60);
		drive.setAutoPath(autoPath, false);
		while(!drive.isFinished()){
			if(DriverStation.getInstance().isOperatorControl()){
				break;
			}
		}
		intake.setIntake(IntakeState.GRIP);
		elevarm.setArmAngle(40);
		elevarm.setElevatorHeight(20);
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(65, -75, 70);
		drive.setAutoPath(autoPath, true);
		while(!drive.isFinished()){
			if(DriverStation.getInstance().isOperatorControl()){
				break;
			}
		}		
		
		autoPath = new Path(tracker.getOdometry().translationMat);
		autoPath.addPoint(100, -65, 70);
		drive.setAutoPath(autoPath, false);
		while(!drive.isFinished()){
			if(DriverStation.getInstance().isOperatorControl()){
				break;
			}
		}
		intake.setIntake(IntakeState.OUTTAKE_FAST);
		Timer.delay(0.25);
		intake.setIntake(IntakeState.GRIP);
		*/
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		scheduler.resume();
		drive.resetMotionProfile();
		elevarm.resetMotionProfile();
		elevarm.configArmEncoder();
		drive.stopMovement();
		elevarm.stopMovement();
		tracker.resetOdometry();
		if (!homed)
		{
			elevarm.homeElevator();
			homed = true;
		}
	}

	double elevatorMaxCurrent = 150, armMaxCurrent = 40; // TEMP for testing

	@Override
	public void teleopPeriodic() {
		xbox.update();
		buttonBox.update();
		joystick.update();
		
		//drive.setWheelVelocity(new DriveVelocity(20, 20));
		drive.cheesyDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4), xbox.getRawAxis(2) > .3);
		//drive.arcadeDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4));
		//System.out.println("Angle: " + elevarm.getArmAngle()+ " Setpoint: " + elevarm.getTargetArmAngle());
		//System.out.println("Height: " + elevarm.getElevatorHeight() + " Setpoint: " + elevarm.getTargetElevatorHeight());

		if (joystick.getRawButton(2))
		{
			climber.set(ControlMode.PercentOutput, .75);
			System.out.println(climber.getOutputCurrent());
		}
		else
		{
			climber.set(ControlMode.PercentOutput, 0);
		}

		
		if(intake.getCurrent() > 15) {
			xbox.setRumble(RumbleType.kLeftRumble, 1);
			xbox.setRumble(RumbleType.kRightRumble, 1);
		} else {
			xbox.setRumble(RumbleType.kLeftRumble, 0);
			xbox.setRumble(RumbleType.kRightRumble, 0);
		}
		if (joystick.getRawButton(3))
		{
			intake.setIntake(IntakeState.INTAKE);
		}
		else if (joystick.getRawButton(4))
		{
			intake.setIntake(IntakeState.OUTTAKE);
		}
		else if (joystick.getRawButton(6))
		{
			intake.setIntake(IntakeState.OUTTAKE_FAST);
		}
		else if (joystick.getRawButton(5))
		{
			intake.setIntake(IntakeState.OPEN);
		}
		else
		{
			intake.setIntake(IntakeState.GRIP);
		}
		
		if (xbox.getRawAxis(3) > .3)
		{
			drive.setShiftState(true);
		}
		else
		{
			drive.setShiftState(false);
		}
		
		double nudge = joystick.getRawAxis(1);
		if (nudge > Constants.JoystickDeadzone)
		{
			elevarm.setElevatorHeight(elevarm.getElevatorHeight() - (nudge - Constants.JoystickDeadzone) * 4);
		}
		else if (nudge < -Constants.JoystickDeadzone)
		{
			elevarm.setElevatorHeight(elevarm.getElevatorHeight() - (nudge + Constants.JoystickDeadzone) * 4);
		}
		
		
		if (buttonBox.getRisingEdge(9))
		{
			elevarm.homeElevator();
		}
		if (buttonBox.getRisingEdge(10))
		{
			elevarm.configArmEncoder();
		}

		if (buttonBox.getRisingEdge(5))
		{
			elevarm.setElevarmIntakePosition();
		}
		else if (buttonBox.getRisingEdge(6))
		{
			elevarm.setArmAngle(80); //Switch Position - once PID is tuned better, make angle more vertical
			elevarm.setElevatorHeight(10);
		}
		else if (buttonBox.getRisingEdge(7))
		{
			elevarm.setArmAngle(80); //Scale Position
			elevarm.setElevatorHeight(50);
		}
		else if (buttonBox.getRisingEdge(8))
		{
			elevarm.setArmAngle(80); //Scale Horizontal Arm
			elevarm.setElevatorHeight(Constants.ElevatorUpHeight);
		}
		else if (buttonBox.getRisingEdge(4))
		{
			elevarm.setArmAngle(0);
		}
		
		if (buttonBox.getPOV() == 0)
		{
			elevarm.setOverallPosition(elevarm.getDistance() + 1, elevarm.getHeight());
		}
		else if (buttonBox.getPOV() == 180)
		{
			elevarm.setOverallPosition(elevarm.getDistance() - 1, elevarm.getHeight());
		}
		
		if(joystick.getRawButton(2)){
			climber.set(ControlMode.PercentOutput, 0.75);
			System.out.println(climber.getOutputCurrent());
		} else {
			climber.set(ControlMode.PercentOutput, 0);
		}
		
		//if (buttonBox.getRisingEdge(button))
		/*
		if (elevarm.getElevatorOutputCurrent() < elevatorMaxCurrent || elevarm.getArmOutputCurrent() < armMaxCurrent) //Prevent elevator from killing itself
		{
			if (xbox.getRisingEdge(7)) {
				elevarm.homeElevator();
				System.out.println("_______________________HOMING_________________________________");
			}

			// elevarm.setElevatorPercentOutput(xbox.getRawAxis(2)/2 - xbox.getRawAxis(3)/2);

			// Cube intake / outtake positions
			if (xbox.getRisingEdge(5)) {
				elevarm.setElevatorHeight(4.5);
				elevarm.setArmAngle(-26);
			}
			if (xbox.getRisingEdge(6)) {
				elevarm.setElevatorHeight(63);
				elevarm.setArmAngle(Constants.ArmHorizontalDegrees);
			}
		} else {
			elevarm.setElevatorPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}

		if (xbox.getRawButton(1)) {
			intakeMotor1.set(ControlMode.PercentOutput, 1);
			intakeMotor2.set(ControlMode.PercentOutput, 1);
		}
		else if (xbox.getRawAxis(3) > .3)
		{
			intakeMotor1.set(ControlMode.PercentOutput, -.45); //in
			intakeMotor2.set(ControlMode.PercentOutput, -.45); //in
			intakeSolenoid1.set(true);
			intakeSolenoid2.set(true);
		}
		else
		{
			intakeMotor1.set(ControlMode.PercentOutput, 0);
			intakeMotor2.set(ControlMode.PercentOutput, 0);
		}
		if (xbox.getRisingEdge(4)) {
			elevarm.homeElevator();
		}
		
		if (xbox.getRawAxis(2) > 0.3)
		{
			intakeSolenoid1.set(false);
			intakeSolenoid2.set(true);
		}
		else
		{
			intakeSolenoid1.set(true);
			intakeSolenoid2.set(false);
		}
		
		if (elevarm.getArmOutputCurrent() < armMaxCurrent) //Prevent arm from killing itself
		{
			//Manual Arm Control
			/*if (xbox.getPOV() == 0)
			{
				elevarm.setArmPercentOutput(.40);
			}
			else if (xbox.getPOV() == 180)
			{
				elevarm.setArmPercentOutput(-.40);
			}
			else
			{
				elevarm.setArmPercentOutput(0);
			}
			
			//Arm Position Control
			if (xbox.getRisingEdge(7))
			{
				elevarm.setArmAngle(Constants.ArmHorizontalDegrees);
			}
			if (xbox.getRisingEdge(8))
			{
				elevarm.setArmAngle(-27);
			}
		}
		else
		{
			elevarm.setArmPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}

		if (elevarm.getElevatorOutputCurrent() < elevatorMaxCurrent && elevarm.getArmOutputCurrent() < armMaxCurrent) {
			if (xbox.getRisingEdge(9)) {
				elevarm.setOverallPosition(20, 36);
			}
			if (xbox.getRisingEdge(10)) {
				elevarm.setOverallPosition(10, 72);
			}
		} else {
			elevarm.setElevatorPercentOutput(0);
			elevarm.setArmPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}

		if (xbox.getRisingEdge(11)) {
			elevarm.shiftElevatorGearbox(false);
		}
		if (xbox.getRisingEdge(12)) {
			elevarm.shiftElevatorGearbox(true);
		}

		if (xbox.getRisingEdge(3)) {
			drive.setShiftState(true);
		}
		if (xbox.getRisingEdge(4)) {
			drive.setShiftState(false);
		}*/
	}
	
	@Override
	public void disabledInit()
	{
		scheduler.pause();
		drive.stopMovement();
		elevarm.stopMovement();
	}

	@Override
	public void testInit() {
		//scheduler.pause();
		drive.stopMovement();
		elevarm.stopMovement();

	}

	@Override
	public void testPeriodic() {
		xbox.update();
		buttonBox.update();
		//Reset vbus of elevator to zero during test mode so homing can occur
		if (xbox.getRisingEdge(1))
		{
			drive.checkSubsystem();
		}
		if (xbox.getRisingEdge(2)) {
			elevarm.checkSubsystem();
		}
		if (xbox.getRisingEdge(3)) {
			elevarm.homeElevator();
		}
		if (xbox.getRisingEdge(4)) {
			System.out.println("Arm PWM Ticks: " + elevarm.getArmPWMPosition());
			System.out.println("Arm Encoder Ticks: " + elevarm.getArmEncoderPosition());
		}
		
		if (buttonBox.getRisingEdge(1))
		{
			elevarm.stopMovement();
			drive.stopMovement();
		}
		
		if (xbox.getPOV() == 0)
		{
			elevarm.setArmPercentOutput(.5);
		}
		else if (xbox.getPOV() == 180)
		{
			elevarm.setArmPercentOutput(-.3);
		}
		else
		{
			elevarm.setArmPercentOutput(0);
		}
	}
}
