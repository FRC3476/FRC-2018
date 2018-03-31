package org.usfirst.frc.team3476.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team3476.subsystem.Elevarm;
import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.Controller;
import org.usfirst.frc.team3476.utility.ThreadScheduler;
import org.usfirst.frc.team3476.utility.auto.AutoRoutine;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator.PathOption;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator.StartPosition;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	Controller xbox = new Controller(0);
	Controller buttonBox = new Controller(1);
	Controller joystick = new Controller(2);

	OrangeDrive drive = OrangeDrive.getInstance();
	Elevarm elevarm = Elevarm.getInstance();
	RobotTracker tracker = RobotTracker.getInstance();
	Intake intake = Intake.getInstance();
	Solenoid fork = new Solenoid(Constants.ForkId);

	ExecutorService mainExecutor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();

	CameraServer camServer = CameraServer.getInstance();
	SendableChooser<String> posChooser = new SendableChooser<>();
	SendableChooser<String> optionChooser = new SendableChooser<>();
	SendableChooser<String> mInDbUsInEsS = new SendableChooser<>();

	@Override
	public void robotInit() {
		drive.setPeriod(Duration.ofMillis(5));
		tracker.setPeriod(Duration.ofMillis(5));
		elevarm.setPeriod(Duration.ofMillis(20));
		intake.setPeriod(Duration.ofMillis(20));
		mInDbUsInEsS.addDefault("Business", "Business");
		mInDbUsInEsS.addObject("mInDbUsInEsS", "mInDbUsInEsS");
		posChooser.addDefault("Left", "Left");
		posChooser.addObject("Center", "Center");
		posChooser.addObject("Right", "Right");
		optionChooser.addDefault("SCALE", "SCALE");
		optionChooser.addObject("SWITCH", "SWITCH");
		optionChooser.addObject("BOTH", "BOTH");
		optionChooser.addObject("FORWARD", "FORWARD");
		optionChooser.addObject("NONE", "NONE");
		SmartDashboard.putData("Position", posChooser);
		SmartDashboard.putData("Option", optionChooser);
		SmartDashboard.putData("business", mInDbUsInEsS);
		scheduler.schedule(drive, mainExecutor);
		scheduler.schedule(tracker, mainExecutor);
		scheduler.schedule(elevarm, mainExecutor);
		scheduler.schedule(intake, mainExecutor);
		camServer.startAutomaticCapture(0);
		camServer.startAutomaticCapture(1);
	}

	@Override
	public void autonomousInit() {
		configSubsytems();
		drive.resume();
		tracker.resume();
		String pos = posChooser.getSelected();
		String option = optionChooser.getSelected();
		String business = mInDbUsInEsS.getSelected();

		PathOption pOption = PathOption.NONE;
		StartPosition sPos = StartPosition.CENTER;
		boolean mind = false;
		switch (business) {
		case "Business":
			mind = false;
			break;
		case "mInDbUsInEsS":
			mind = true;
			break;
		}

		switch (pos) {
		case "Left":
			sPos = StartPosition.LEFT;
			break;
		case "Center":
			sPos = StartPosition.CENTER;
			break;
		case "Right":
			sPos = StartPosition.RIGHT;
			break;
		}

		switch (option) {
		case "SCALE":
			pOption = PathOption.SCALE;
			break;
		case "SWITCH":
			pOption = PathOption.SWITCH;
			break;
		case "BOTH":
			pOption = PathOption.BOTH;
			break;
		case "FORWARD":
			pOption = PathOption.FORWARD;
			break;
		case "mInD bUsInEsS":
			pOption = PathOption.SCALE;
			break;
		case "NONE":
			pOption = PathOption.NONE;
			break;
		}
		double start = Timer.getFPGATimestamp();
		while (DriverStation.getInstance().getGameSpecificMessage().isEmpty() && Timer.getFPGATimestamp() - start < 1) {

		}
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.isEmpty()) {
			gameData = "rr";
			if (pOption == PathOption.NONE) {
			} else if (sPos != StartPosition.CENTER) {
				pOption = PathOption.FORWARD;
			} else {
				pOption = PathOption.SWITCH;
			}
		}
		AutoRoutine routine = AutoRoutineGenerator.generate(gameData, pOption, sPos, mind);
		new Thread(routine).start();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		configSubsytems();
		drive.pause();
		tracker.pause();
	}

	double elevatorMaxCurrent = 150, armMaxCurrent = 40; // TEMP for testing

	@Override
	public void teleopPeriodic() {
		xbox.update();
		buttonBox.update();
		joystick.update();
		
		//drive.orangeDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4), xbox.getRawAxis(2) > .3);
		//drive.setWheelVelocity(new DriveVelocity(20, 20));
		drive.cheesyDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4), xbox.getRawButton(5));
		//drive.arcadeDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4));
		//System.out.println("Angle: " + elevarm.getArmAngle()+ " Setpoint: " + elevarm.getTargetArmAngle());
		//System.out.println("Height: " + elevarm.getElevatorHeight() + " Setpoint: " + elevarm.getTargetElevatorHeight());
		
		
		
		
		
		if (buttonBox.getRawButton(10))
		{
			elevarm.setClimberPercentOutput(.75);
			elevarm.setElevatorGearbox(true);
			elevarm.setElevatorPercentOutput(0);
			System.out.println("Climber: " + elevarm.getClimberCurrent());
		}
		else
		{
			elevarm.setClimberPercentOutput(0);
		}
		
		if (joystick.getRisingEdge(11))
		{
			elevarm.setElevatorGearbox(false);
			elevarm.homeElevator();
		}
	
		
		if (joystick.getRawButton(3) || xbox.getRawButton(6))
		{
			/*
			if (intake.getCubeSwitch())
			{
				xbox.setRumble(RumbleType.kLeftRumble, 1);
				xbox.setRumble(RumbleType.kRightRumble, 1);
			}
			else
			{
				xbox.setRumble(RumbleType.kLeftRumble, 0);
				xbox.setRumble(RumbleType.kRightRumble, 0);
			}
			*/
			intake.setIntake(IntakeState.INTAKE);
		}
		else if (joystick.getRawButton(6) || xbox.getRawAxis(3) > .9)
		{
			intake.setIntake(IntakeState.OUTTAKE_FAST);
		}
		else if (joystick.getRawButton(4) || xbox.getRawAxis(3) > .05)
		{
			intake.setIntake(IntakeState.OUTTAKE);
		}
		else if (joystick.getRawButton(5) || xbox.getRawButton(Controller.Xbox.A))
		{
			intake.setIntake(IntakeState.OPEN);
		}
		else
		{
			intake.setIntake(IntakeState.GRIP);
		}
		
		if (xbox.getRisingEdge(2, 0.3))
		{
			drive.setShiftState(true);
		}
		if (xbox.getFallingEdge(2, 0.3))
		{
			drive.setShiftState(false);
		}
		double nudge = joystick.getRawAxis(1);
		if (nudge > Constants.JoystickDeadzone)
		{
			elevarm.setElevatorHeight(elevarm.getTargetElevatorHeight() - (nudge - Constants.JoystickDeadzone) / 5);
		}
		else if (nudge < -Constants.JoystickDeadzone)
		{
			elevarm.setElevatorHeight(elevarm.getTargetElevatorHeight() - (nudge + Constants.JoystickDeadzone) / 5);
		}
		
		
		if (buttonBox.getRisingEdge(9))
		{
			elevarm.homeElevator();
		}

		
		if (joystick.getRisingEdge(9))
		{
			elevarm.setXRate(.01);
		}
		else if (joystick.getRisingEdge(10))
		{
			elevarm.setXRate(-.01);
		}
		else if (joystick.getFallingEdge(9) || joystick.getFallingEdge(10))
		{
			elevarm.setXRate(0);
			elevarm.resetRateLimits();
			elevarm.stopMovement();
			elevarm.setArmAngle(elevarm.getArmAngle());
			elevarm.setElevatorHeight(elevarm.getElevatorHeight());
		}
		else if (buttonBox.getRisingEdge(5))
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
			if (elevarm.getElevatorHeight() < 58)
				elevarm.setArmAngle(25);
			else
				elevarm.setArmAngle(60);
		}
		else if (buttonBox.getRisingEdge(3))
		{
			elevarm.setElevatorHeight(56.5);
			elevarm.setArmAngle(80);
		}
		
		if (buttonBox.getPOV() == 0)
		{
			//elevarm.setOverallPosition(elevarm.getDistance() + 1, elevarm.getHeight());
			elevarm.setArmAngle(elevarm.getArmAngle() - 3);
		}
		else if (buttonBox.getPOV() == 180)
		{
			//elevarm.setOverallPosition(elevarm.getDistance() - 1, elevarm.getHeight());
			elevarm.setArmAngle(elevarm.getArmAngle() + 3);
		}
		if(joystick.getRawButton(7) && joystick.getRawButton(8)){
			System.out.println("Forks");
			fork.set(true);
		}
		
		
	}

	@Override
	public void disabledInit() {
		scheduler.pause();
		drive.stopMovement();
		elevarm.stopMovement();
		fork.set(false);
		elevarm.setElevatorGearbox(false);
	}

	@Override
	public void testInit() {
		drive.stopMovement();
		elevarm.stopMovement();
	}

	public void configSubsytems() {
		scheduler.resume();
		drive.resetMotionProfile();
		elevarm.resetMotionProfile();
		elevarm.configArmEncoder();
		drive.stopMovement();
		elevarm.stopMovement();
	}

	@Override
	public void testPeriodic() {
		xbox.update();
		buttonBox.update();
		// Reset vbus of elevator to zero during test mode so homing can occur
		if (xbox.getRisingEdge(1)) {
			drive.checkSubsystem();
		}
		if (xbox.getRisingEdge(2)) {
			elevarm.checkElevator();
		}
		if (xbox.getRisingEdge(3)) {
			elevarm.homeElevator();
		}
		if (xbox.getRisingEdge(4)) {
			System.out.println("Arm PWM Ticks: " + elevarm.getArmPWMPosition());
			System.out.println("Arm Encoder Ticks: " + elevarm.getArmEncoderPosition());
		}

		if (buttonBox.getRisingEdge(1)) {
			elevarm.stopMovement();
			drive.stopMovement();
		}

		if (xbox.getPOV() == 0) {
			elevarm.setArmPercentOutput(.5);
		} else if (xbox.getPOV() == 180) {
			elevarm.setArmPercentOutput(-.3);
		} else {
			elevarm.setArmPercentOutput(0);
		}
		if (buttonBox.getRisingEdge(3)) {
			elevarm.checkClimber();
		}
	}
}
