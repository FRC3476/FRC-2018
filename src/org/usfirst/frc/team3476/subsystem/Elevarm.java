package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.control.RateLimiter;

import edu.wpi.first.wpilibj.Timer;

public class Elevarm extends Threaded {

	private static final Elevarm instance = new Elevarm();

	Elevator elevator;
	Arm arm;

	public enum ElevatorState {
		MANUAL, POSITION, HOMING, INTAKE
	}
	//Practice Robot
	//Elevator
	//P .1
	//I .0001
	//D .0001
	//I Zone 1000
	//Arm
	//P .8
	//I .0005
	//D 0

	private ElevatorState currentElevatorState = ElevatorState.MANUAL;
	private RateLimiter elevatorLimiter, armLimiter;
	private volatile double elevatorSetpoint, armSetpoint;
	
	private boolean elevatorIntakePositionSet = false;

	private Elevarm() {
		elevatorLimiter = new RateLimiter(1000, 120);
		armLimiter = new RateLimiter(200, 500);
		elevator = Elevator.getInstance();
		arm = Arm.getInstance();
	}
	
	public double getClimberCurrent()
	{
		return elevator.getClimberCurrent();
	}

	public static Elevarm getInstance() {
		return instance;
	}

	synchronized public void setElevatorHeight(double height) {
		if (currentElevatorState != ElevatorState.HOMING && isValidAngleAndHeight(armSetpoint, height))
		{
			if(currentElevatorState != ElevatorState.POSITION){
				currentElevatorState = ElevatorState.POSITION;
				resetMotionProfile();
			}
			elevatorSetpoint = height;
		}
		else
		{
			System.out.println(currentElevatorState == ElevatorState.HOMING);
			System.out.println("------------------------------------------------------------");
			System.out.println("Homing or Not Valid Position");
			System.out.println("	Angle: " + arm.getAngle() + " Setpoint: " + arm.getTargetAngle());
			System.out.println("	Height: " + elevator.getHeight() + " Setpoint: " + elevator.getTargetHeight());
			System.out.println("------------------------------------------------------------");
		}
	}

	public int getElevatorEncoderTicks() {
		return elevator.getEncoderTicks();
	}

	public void setArmPercentOutput(double output) {
		arm.setPercentOutput(output);
	}

	public void setArmEncoderTicks(int position) {
		arm.setEncoderPosition(position);
	}

	public void setElevatorPercentOutput(double output) {
		if (currentElevatorState != ElevatorState.HOMING) {
			currentElevatorState = ElevatorState.MANUAL;
			elevator.setPercentOutput(output);
		}
	}

	public void setElevatorEncoderTicks(int position) {
		elevator.setEncoderPosition(position);
	}

	public double getElevatorHeight() {
		return elevator.getHeight();
	}

	public double getTargetElevatorHeight() {
		return elevatorSetpoint;
	}

	public void setArmAngle(double angle) {
		if (isValidAngleAndHeight(angle, elevatorSetpoint))
		{
			armSetpoint = angle;
		} else {
			System.out.println("------------------------------------------------------------");
			System.out.println("Collision detected. Arm not moving");
			System.out.println("	Angle: " + arm.getAngle() + " Setpoint: " + angle);
			System.out.println("	Height: " + elevator.getHeight() + " Setpoint: " + elevatorSetpoint);
			System.out.println("------------------------------------------------------------");
		}
	}

	public double getArmAngle() {
		return arm.getAngle();
	}

	public double getTargetArmAngle() {
		return armSetpoint;
	}
	
	
	public void setXRate(double xRate)
	{
		double armSpeed = -xRate * 57.44645 / (Constants.ArmLength * Math.sin(Math.toRadians(arm.getAngle())));
		double elevatorSpeed = -armSpeed * Constants.ArmLength * Math.cos(Math.toRadians(arm.getAngle()));
		
		if (isValidPosition(getX() + xRate, getY()))
		{
			arm.setSpeed(armSpeed * (1d / 360) * (1d / Constants.ArmRotationsPerMotorRotation)
				* Constants.SensorTicksPerMotorRotation);
		
			elevator.setSpeed(elevatorSpeed * (1d / Constants.ElevatorInchesPerMotorRotation)
				* Constants.SensorTicksPerMotorRotation);
		}
		else
		{
			arm.setSpeed(0);
			elevator.setSpeed(0);
		}
	}
	
	public void setOverallPosition(double distance, double height) {
		double armAngle = arm.getAngle();
		double elevatorHeight = elevator.getHeight();

		double heightByArm = Math.sqrt(Constants.ArmLength * Constants.ArmLength - distance * distance);

		double armAngle1 = Math.toDegrees(Math.asin(heightByArm / Constants.ArmLength));
		double elevatorHeight1 = height - heightByArm;

		double armAngle2 = -Math.toDegrees(Math.asin(heightByArm / Constants.ArmLength));
		double elevatorHeight2 = height + heightByArm;

		boolean position1Valid = isValidAngleAndHeight(armAngle1, elevatorHeight1);
		boolean position2Valid = isValidAngleAndHeight(armAngle2, elevatorHeight2);

		if (position1Valid && position2Valid) {
			double armTime1 = Math.abs(arm.getAngle() - armAngle1) * Constants.ArmSpeed;
			double elevatorTime1 = Math.abs(elevator.getHeight() - elevatorHeight1) * Constants.ElevatorSpeed;
			double timeToPosition1 = (armTime1 > elevatorTime1 ? armTime1 : elevatorTime1);

			double armTime2 = Math.abs(arm.getAngle() - armAngle2) * Constants.ArmSpeed;
			double elevatorTime2 = Math.abs(elevator.getHeight() - elevatorHeight2) * Constants.ElevatorSpeed;
			double timeToPosition2 = (armTime2 > elevatorTime2 ? armTime2 : elevatorTime2);

			armAngle = (timeToPosition1 < timeToPosition2 ? armAngle1 : armAngle2);
			elevatorHeight = (timeToPosition1 < timeToPosition2 ? elevatorHeight1 : elevatorHeight2);
			setElevatorHeight(elevatorHeight);
			setArmAngle(armAngle);

		} else if (position1Valid) {
			armAngle = armAngle1;
			elevatorHeight = elevatorHeight1;
			setElevatorHeight(elevatorHeight);
			setArmAngle(armAngle);
		} else if (position2Valid) {
			armAngle = armAngle2;
			elevatorHeight = elevatorHeight2;
			setElevatorHeight(elevatorHeight);
			setArmAngle(armAngle);
		} else {
			System.out.println("------------------------------------------------------------");
			System.out.println("Invalid Position. Not Moving");
			System.out.println("	Angle: " + arm.getAngle() + " Setpoint: " + armSetpoint);
			System.out.println("	Height: " + elevator.getHeight() + " Setpoint: " + elevatorSetpoint);
			System.out.println("------------------------------------------------------------");
		}
	}
	
	public void setElevarmIntakePosition()
	{
		currentElevatorState = ElevatorState.INTAKE;
	}

	public void shiftElevatorGearbox(boolean engaged) {
		elevator.shiftElevatorGearbox(engaged);
	}

	public void homeElevator() {
		elevator.homeStartTime = System.currentTimeMillis();
		currentElevatorState = ElevatorState.HOMING;
	}

	public void prepClimb() {
		setElevatorHeight(Constants.ElevatorUpHeight);
		setArmAngle(Constants.ArmDownDegrees);
		elevator.shiftElevatorGearbox(false);
	}

	public static boolean isValidAngleAndHeight(double armAngle, double elevatorHeight) {
		double x = Math.cos(Math.toRadians(armAngle)) * Constants.ArmLength;
		double y = elevatorHeight + Math.sin(Math.toRadians(armAngle)) * Constants.ArmLength;
		boolean armLow = armAngle < Constants.ArmLowerAngleLimit;
		boolean armHigh = armAngle > Constants.ArmUpperAngleLimit;
		boolean elevatorLow = elevatorHeight < Constants.ElevatorMinHeight;
		boolean elevatorHigh = elevatorHeight > Constants.ElevatorMaxHeight;
		boolean hittingElevator = (x < 14 && y < -10);
		if (armLow || armHigh || elevatorLow || elevatorHigh || hittingElevator)
			return false;
		return true;
	}
	
	public static boolean isValidPosition(double x, double y)
	{
		double armAngle = Math.toDegrees(Math.acos(x / Constants.ArmLength));
		double elevatorHeight = y - Math.sin(Math.toRadians(armAngle)) * Constants.ArmLength;
		return isValidAngleAndHeight(armAngle, elevatorHeight);
	}
	
	public void setClimberPercentOutput(double output)
	{
		elevator.setClimberPercentOutput(output);
	}
	
	public void setElevatorGearbox(boolean on)
	{
		elevator.setElevatorGearbox(on);
	}
	
	public double getX()
	{
		return Math.cos(Math.toRadians(getArmAngle())) * Constants.ArmLength;
	}
	
	public double getY()
	{
		return getElevatorHeight() + Math.sin(Math.toRadians(getArmAngle())) * Constants.ArmLength;
	}

	public double getElevatorOutputCurrent() {
		return elevator.getOutputCurrent();
	}

	public double getArmOutputCurrent() {
		return arm.getOutputCurrent();
	}

	@Override
	public void update() {
		switch (currentElevatorState) {
		case HOMING:
			if (!isValidAngleAndHeight(arm.getAngle(), 0)) {
				//setArmAngle(Constants.ArmHorizontalDegrees);
				System.out.println("------------------------------------------------------------");
				System.out.println("CAN'T HOME. INVALID POSITION");
				System.out.println("	Angle: " + arm.getAngle() + " Setpoint: " + arm.getTargetAngle());
				System.out.println("	Height: " + elevator.getHeight() + " Setpoint: " + elevator.getTargetHeight());
				System.out.println("------------------------------------------------------------");
				synchronized(this) {
					currentElevatorState = ElevatorState.MANUAL;					
				}
				break;
			}
			elevator.setPercentOutput(-.2); // Some slow speed
			if (elevator.getOutputCurrent() > Constants.ElevatorStallCurrent) {
				elevator.setPercentOutput(0);
				elevator.setEncoderPosition(0); // Sets encoder value to 0
				System.out.println("ELEVATOR HOMED");
				synchronized(this) {
					currentElevatorState = ElevatorState.MANUAL;					
				}
			} else if (System.currentTimeMillis() - elevator.homeStartTime > 3000) {
				System.out.println("------------------------------------------------------------");
				System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
				System.out.println("	Height: " + elevator.getHeight() + " Setpoint: " + elevator.getTargetHeight());
				System.out.println("------------------------------------------------------------");
				elevator.setPercentOutput(0);
				elevator.setEncoderPosition((int) (Constants.ElevatorMinHeight
						* (1 / Constants.ElevatorInchesPerMotorRotation) * Constants.SensorTicksPerMotorRotation));
				synchronized(this) {
					currentElevatorState = ElevatorState.MANUAL;					
				}
			}
			break;
		case POSITION:
			double setpoint = elevatorLimiter.update(elevatorSetpoint);
			elevator.setHeight(setpoint);
			arm.setAngle(armLimiter.update(armSetpoint));
			break;
		case INTAKE:
			elevator.setHeight(elevatorLimiter.update(Constants.ElevatorDownHeight));
			if (elevator.getHeight() < 20)
			{
				setArmAngle(Constants.ArmIntakeDegrees);
				arm.setAngle(armLimiter.update(armSetpoint));
			}
			break;
		case MANUAL:
			break;
		}
	}

	public boolean checkSubsystem() {
		return checkElevator() && checkArm();
	}

	public boolean checkElevator() {
		setArmAngle(Constants.ArmHorizontalDegrees); // Move arm out of the way before testing
		Timer.delay(0.75);
		return elevator.checkSubystem();
	}
	
	public void stopMovement() {
		setArmPercentOutput(0);
		setElevatorPercentOutput(0);
	}
	
	public boolean checkArm() {
		setElevatorHeight((Constants.ElevatorUpHeight + Constants.ElevatorDownHeight) / 2d); // Move elevator out of the
																							// way before testing
		Timer.delay(0.75);
		return arm.checkSubsytem();
	}
	
	public boolean checkClimber()
	{
		return elevator.checkClimber();
	}
	
	public void configArmEncoder()
	{
		arm.setEncoderFromPWM();
		System.out.println("Arm Encoder Set");
	}

	public int getArmPWMPosition()
	{
		return arm.getPWMPosition();
	}
	
	public int getArmEncoderPosition()
	{
		return arm.getEncoderPosition();
	}
	
	public void resetMotionProfile() {
		elevatorLimiter.reset();
		elevatorLimiter.setLatestValue(elevator.getHeight()); // reset elevator setpoint
		armLimiter.reset();
		armLimiter.setLatestValue(arm.getAngle());
	}
}
