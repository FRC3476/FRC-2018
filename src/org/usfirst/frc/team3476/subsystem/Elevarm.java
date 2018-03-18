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
	private RateLimiter elevatorLimiter;
	private volatile double elevatorSetpoint;
	
	private boolean elevatorIntakePositionSet = false;

	private Elevarm() {
		elevatorLimiter = new RateLimiter(1000, 120);
		elevator = Elevator.getInstance();
		arm = Arm.getInstance();
	}

	public static Elevarm getInstance() {
		return instance;
	}

	synchronized public void setElevatorHeight(double height) {
		if (currentElevatorState != ElevatorState.HOMING && isValidPosition(arm.getTargetAngle(), height))
		{
			if(currentElevatorState != ElevatorState.POSITION){
				currentElevatorState = ElevatorState.POSITION;
				resetMotionProfile();
			}
			elevatorSetpoint = height;
		}
		else
		{
			System.out.println("Homing or Not Valid Position");
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
		return elevator.getTargetHeight();
	}

	public void setArmAngle(double angle) {
		if (isValidPosition(angle, elevator.getTargetHeight()))
		{
			arm.setAngle(angle);
		} else {
			System.out.println("Collision detected. Arm not moving");
		}
	}

	public double getArmAngle() {
		return arm.getAngle();
	}

	public double getTargetArmAngle() {
		return arm.getTargetAngle();
	}

	public void setOverallPosition(double distance, double height) {
		double armAngle = arm.getAngle();
		double elevatorHeight = elevator.getHeight();

		double heightByArm = Math.sqrt(Constants.ArmLength * Constants.ArmLength - distance * distance);

		double armAngle1 = Math.toDegrees(Math.asin(heightByArm / Constants.ArmLength));
		double elevatorHeight1 = height - heightByArm;

		double armAngle2 = -Math.toDegrees(Math.asin(heightByArm / Constants.ArmLength));
		double elevatorHeight2 = height + heightByArm;

		boolean position1Valid = isValidPosition(armAngle1, elevatorHeight1);
		boolean position2Valid = isValidPosition(armAngle2, elevatorHeight2);

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
			System.out.println("Invalid Position. Not moving");
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

	public static boolean isValidPosition(double armAngle, double elevatorHeight) {
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
	
	public double getDistance()
	{
		return Math.cos(Math.toRadians(getArmAngle())) * Constants.ArmLength;
	}
	public double getHeight()
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
			if (!isValidPosition(arm.getAngle(), 0)) {
				//setArmAngle(Constants.ArmHorizontalDegrees);
				System.out.println("CAN'T HOME. INVALID POSITION");
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
				System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
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
			break;
		case INTAKE:
			elevator.setHeight(elevatorLimiter.update(Constants.ElevatorDownHeight));
			if (elevator.getHeight() < 20)
			{
				arm.setAngle(Constants.ArmIntakeDegrees);
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
	}
}
