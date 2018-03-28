package org.usfirst.frc.team3476.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team3476.subsystem.Elevarm;
import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.utility.Controller;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.ThreadScheduler;
import org.usfirst.frc.team3476.utility.auto.AutoRoutine;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator.PathOption;
import org.usfirst.frc.team3476.utility.auto.AutoRoutineGenerator.StartPosition;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

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
	
	boolean homed = false;

	Path autoPath;

	@Override
	public void robotInit() {
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
		scheduler.schedule(drive, Duration.ofMillis(5), mainExecutor);
		scheduler.schedule(tracker, Duration.ofMillis(5), mainExecutor);
		scheduler.schedule(elevarm, Duration.ofMillis(20), mainExecutor);
		camServer.startAutomaticCapture(0);
		camServer.startAutomaticCapture(1);
	}

	@Override
	public void autonomousInit() {
		String pos = posChooser.getSelected();
		String option = optionChooser.getSelected();
		String business = mInDbUsInEsS.getSelected();
		
		PathOption pOption = PathOption.NONE;
		StartPosition sPos = StartPosition.CENTER;
		boolean mind = false;
		switch(business){
		case "Business":
			mind = false;
			break;
		case "mInDbUsInEsS":
			mind = true;
			break;
		}
		
		switch(pos){
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
		
		switch(option){
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
		
		scheduler.resume();
		drive.resetMotionProfile();
		elevarm.resetMotionProfile();
		elevarm.configArmEncoder();
		drive.stopMovement();
		elevarm.stopMovement();
		double start = Timer.getFPGATimestamp();
		while (DriverStation.getInstance().getGameSpecificMessage().isEmpty() && Timer.getFPGATimestamp() - start < 1)
		{
			
		}
		String gameData = DriverStation.getInstance().getGameSpecificMessage().toLowerCase();
		if (gameData.isEmpty())
		{
			gameData = "rr";
			if (pOption == PathOption.NONE)
			{}
			else if (sPos != StartPosition.CENTER)
				pOption = PathOption.FORWARD;
			else
				pOption = PathOption.SWITCH;
		}
		System.out.println(pOption);
		System.out.println(sPos);
		AutoRoutine routine = AutoRoutineGenerator.generate(gameData, pOption, sPos, mind);
		new Thread(routine).start();

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
		Timer.delay(1);
		elevarm.setArmAngle(70);
		elevarm.setElevatorHeight(10);
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
		oldAxis = false;
	}

	double elevatorMaxCurrent = 150, armMaxCurrent = 40; // TEMP for testing
	
	boolean axis;
	boolean oldAxis;

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
		System.out.println("Height: " + elevarm.getElevatorHeight() + " Setpoint: " + elevarm.getTargetElevatorHeight());
		
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
		else if (joystick.getRawButton(6) || xbox.getRawAxis(2) > .9)
		{
			intake.setIntake(IntakeState.OUTTAKE_FAST);
		}
		else if (joystick.getRawButton(4) || xbox.getRawAxis(2) > .05)
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
		
		axis = xbox.getRawAxis(3) > .3;
		
		if (axis && !oldAxis)
		{
			drive.setShiftState(true);
		}
		if (oldAxis && !axis)
		{
			drive.setShiftState(false);
		}
		oldAxis = axis;
		
		double nudge = joystick.getRawAxis(1);
		if (nudge > Constants.JoystickDeadzone)
		{
			elevarm.setElevatorHeight(elevarm.getElevatorHeight() - (nudge - Constants.JoystickDeadzone) * 5);
		}
		else if (nudge < -Constants.JoystickDeadzone)
		{
			elevarm.setElevatorHeight(elevarm.getElevatorHeight() - (nudge + Constants.JoystickDeadzone) * 5);
		}
		
		
		if (buttonBox.getRisingEdge(9))
		{
			elevarm.homeElevator();
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
		fork.set(false);
		elevarm.setElevatorGearbox(false);
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
			elevarm.checkElevator();
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
		if (buttonBox.getRisingEdge(3))
		{
			elevarm.checkClimber();
		}
	}
}
