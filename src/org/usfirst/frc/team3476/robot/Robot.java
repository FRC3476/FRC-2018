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

	Path autoPath;

	@Override
	public void robotInit() {
		drive.setPeriod(Duration.ofMillis(5));
		tracker.setPeriod(Duration.ofMillis(5));
		elevarm.setPeriod(Duration.ofMillis(5));
		posChooser.addDefault("Left", "Left");
		posChooser.addObject("Center", "Center");
		posChooser.addObject("Right", "Right");
		optionChooser.addDefault("SCALE", "SCALE");
		optionChooser.addObject("SWITCH", "SWITCH");
		optionChooser.addObject("BOTH", "BOTH");
		optionChooser.addObject("FORWARD", "FORWARD");
		optionChooser.addObject("mInD bUsInEsS", "mInD bUsInEsS");
		optionChooser.addObject("NONE", "NONE");
		SmartDashboard.putData("Position", posChooser);
		SmartDashboard.putData("Option", optionChooser);
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
		switch(pos){
		case "Left":
			break;
		case "Center":
			break;
		case "Right":
			break;
		}
		
		switch(option){
		case "SCALE":
			break;
		case "SWITCH":
			break;
		case "BOTH":
			break;
		case "FORWARD":
			break;
		case "mInD bUsInEsS":
			break;
		case "NONE":
			break;
		}
		
		AutoRoutine routine = AutoRoutineGenerator.generate("ll", PathOption.SCALE, StartPosition.LEFT);
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
		
		drive.cheesyDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4), xbox.getRawAxis(2) > .3);
		System.out.println("Angle: " + elevarm.getArmAngle()+ " Setpoint: " + elevarm.getTargetArmAngle());
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
		}
	
		
		if (joystick.getRawButton(3) || xbox.getRawButton(Controller.Xbox.RightBumper))
		{
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
			intake.setIntake(IntakeState.INTAKE);
		}
		else if (joystick.getRawButton(4) || xbox.getRawButton(Controller.Xbox.LeftBumper))
		{
			intake.setIntake(IntakeState.OUTTAKE);
		}
		else if (joystick.getRawButton(6))
		{
			intake.setIntake(IntakeState.OUTTAKE_FAST);
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
			elevarm.setElevatorHeight(55);
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
		if (buttonBox.getRisingEdge(3))
		{
			elevarm.checkClimber();
		}
	}
}
