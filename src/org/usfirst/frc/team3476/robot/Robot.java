package org.usfirst.frc.team3476.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team3476.subsystem.Elevarm;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.Controller;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.ThreadScheduler;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends IterativeRobot {
	Controller xbox = new Controller(0);
	Controller buttonBox = new Controller(1);
	OrangeDrive drive = OrangeDrive.getInstance();
	Elevarm elevarm = Elevarm.getInstance();
	RobotTracker tracker = RobotTracker.getInstance();
	ExecutorService mainExecutor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();
	LazyTalonSRX intakeMotor1 = new LazyTalonSRX(Constants.Intake1Id);
	LazyTalonSRX intakeMotor2 = new LazyTalonSRX(Constants.Intake2Id);
	Solenoid intakeSolenoid30Psi = new Solenoid(Constants.IntakeSolenoid30PsiId);
	Solenoid intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoid60PsiId);
	CameraServer camServer = CameraServer.getInstance();
	
	boolean homed = false;

	Path autoPath;

	@Override
	public void robotInit() {
		scheduler.schedule(drive, Duration.ofMillis(10), mainExecutor);
		//scheduler.schedule(tracker, Duration.ofMillis(10), mainExecutor);
		scheduler.schedule(elevarm, Duration.ofMillis(20), mainExecutor);
		camServer.startAutomaticCapture();
	}

	@Override
	public void autonomousInit() {
		scheduler.resume();
		tracker.resetOdometry();
		autoPath = new Path(new Translation2d(0, 0));
		autoPath.addPoint(250, -10, 100);
		autoPath.addPoint(300, 5, 100);
		autoPath.addPoint(209, 0, 100);
		autoPath.addPoint(250, -5, 100);
		autoPath.addPoint(300, 5, 100);
		autoPath.addPoint(209, 28.1, 100);
		autoPath.addPoint(250, -5, 100);
		autoPath.addPoint(300, 5, 100);
		drive.setAutoPath(autoPath, false);
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		scheduler.resume();
		drive.resetMotionProfile();
		elevarm.resetMotionProfile();
		elevarm.setArmPercentOutput(0);
		elevarm.setElevatorPercentOutput(0);
		if (!homed)
		{
			elevarm.homeElevator();
			homed = true;
		}
	}

	@Override
	public void disabledInit() {
		scheduler.pause();
	}

	double elevatorMaxCurrent = 150, armMaxCurrent = 40; // TEMP for testing

	@Override
	public void teleopPeriodic() {
		drive.arcadeDrive(xbox.getRawAxis(1), -xbox.getRawAxis(4));
		
		System.out.println("Angle: " + elevarm.getArmAngle()+ " Setpoint: " + elevarm.getTargetArmAngle());
		System.out.println("Height: " + elevarm.getElevatorHeight() + " Setpoint: " + elevarm.getTargetElevatorHeight());
		//double current = elevarm.getElevatorOutputCurrent();
		//if (current > 20)
			//System.out.println("Current: " + current);
		
		if (buttonBox.getRawButton(4))
		{
			intakeMotor1.set(ControlMode.PercentOutput, -.45); //Intake
			intakeMotor2.set(ControlMode.PercentOutput, -.45);
			intakeSolenoid30Psi.set(true);
			intakeSolenoid60Psi.set(true);
		}
		else if (buttonBox.getRawButton(3))
		{
			intakeMotor1.set(ControlMode.PercentOutput, .5); //Outtake
			intakeMotor2.set(ControlMode.PercentOutput, .5);
		}
		else if (buttonBox.getRawButton(2))
		{
			intakeMotor1.set(ControlMode.PercentOutput, 0);
			intakeMotor2.set(ControlMode.PercentOutput, 0);
			intakeSolenoid30Psi.set(true);
			intakeSolenoid60Psi.set(false);
		}
		else
		{
			intakeMotor1.set(ControlMode.PercentOutput, 0);
			intakeMotor2.set(ControlMode.PercentOutput, 0);
			intakeSolenoid30Psi.set(false);
			intakeSolenoid60Psi.set(true);
		}
		
		if (buttonBox.getPOV() == 0)
		{
			elevarm.setElevatorHeight(elevarm.getElevatorHeight() + 1);
		}
		if (buttonBox.getPOV() == 180)
		{
			elevarm.setElevatorHeight(elevarm.getElevatorHeight() - 1);
		}
		
		
		if (buttonBox.getRisingEdge(9))
		{
			elevarm.homeElevator();
		}
		if (buttonBox.getRisingEdge(10))
		{
			elevarm.homeArm();
		}
		
		
		if (buttonBox.getRisingEdge(6))
		{
			elevarm.setArmAngle(-15); //Intake Position
			elevarm.setElevatorHeight(3);
		}
		else if (buttonBox.getRisingEdge(7))
		{
			elevarm.setArmAngle(70); //Switch Position
			elevarm.setElevatorHeight(10);
		}
		else if (buttonBox.getRisingEdge(8))
		{
			elevarm.setArmAngle(60); //Scale Position
			elevarm.setElevatorHeight(63);
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
		buttonBox.update();
		xbox.update();
	}

	@Override
	public void testInit() {
		scheduler.pause();
		drive.stopSubsystem();
		elevarm.stopSubsystem();
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
			elevarm.homeArm();
		}
	}
}
