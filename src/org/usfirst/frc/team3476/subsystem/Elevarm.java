package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.control.RateLimiter;

import edu.wpi.first.wpilibj.Timer;

public class Elevarm extends Threaded {

	private static final Elevarm instance = new Elevarm();

	Elevator elevator;
	Arm arm;

	public enum ElevarmState {
		HOMING, INTAKE, EXTERNAL, XRATE
	}
	
	public enum ElevatorState {
		MANUAL, POSITION, SPEED
	}

	public enum ArmState {
		POSITION, MANUAL, SPEED
	}

	private ElevarmState elevarmState = ElevarmState.HOMING;
	private ElevatorState elevatorState = ElevatorState.MANUAL;
	private ArmState armState = ArmState.MANUAL;
	private RateLimiter elevatorLimiter, armLimiter;
	private volatile double elevatorSetpoint, armSetpoint, elevatorSpeedSetpoint, armSpeedSetpoint, xRateSetpoint;
	public static final double maxElevatorSpeed = 100, maxArmSpeed = 100;

	private boolean elevatorIntakePositionSet = false;

	private Elevarm() {
		elevatorLimiter = new RateLimiter(1000, 250);
		armLimiter = new RateLimiter(200, 400);
		elevator = Elevator.getInstance();
		arm = Arm.getInstance();
	}

	public double getClimberCurrent() {
		return elevator.getClimberCurrent();
	}

	public static Elevarm getInstance() {
		return instance;
	}

	synchronized public void setElevatorHeight(double height) {
		if (elevarmState != ElevarmState.HOMING && isValidAngleAndHeight(armSetpoint, height)) {
			elevarmState = ElevarmState.EXTERNAL;
			elevatorState = ElevatorState.POSITION;
			elevatorSetpoint = height;
		} else {
			System.out.println(elevarmState == ElevarmState.HOMING);
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

	synchronized public void setArmPercentOutput(double output) {
		if (elevarmState != ElevarmState.HOMING) {
			elevarmState = ElevarmState.EXTERNAL;
			armState = ArmState.MANUAL;
			arm.setPercentOutput(output);
		}
	}

	synchronized public void setElevatorPercentOutput(double output) {
		if (elevarmState != ElevarmState.HOMING) {
			elevarmState = ElevarmState.EXTERNAL;
			elevatorState = ElevatorState.MANUAL;
			elevator.setPercentOutput(output);
		}
	}

	public void setElevatorEncoderTicks(int position) {
		elevator.setEncoderPosition(position);
	}

	public double getElevatorHeight() {
		return elevator.getHeight();
	}

	synchronized public double getTargetElevatorHeight() {
		return elevatorSetpoint;
	}

	synchronized public void setArmAngle(double angle) {
		if (elevarmState != ElevarmState.HOMING && isValidAngleAndHeight(angle, elevatorSetpoint)) {
			elevarmState = ElevarmState.EXTERNAL;
			armState = ArmState.POSITION;
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

	synchronized public double getTargetArmAngle() {
		return armSetpoint;
	}
	
	public void setArmSpeed(double speed)
	{
		armState = ArmState.SPEED;
		armSpeedSetpoint = speed;
	}
	
	public void setElevatorSpeed(double speed)
	{
		elevatorState = ElevatorState.SPEED;
		elevatorSpeedSetpoint = speed;
	}
	
	public void setXRate(double xRate) {
		xRateSetpoint = xRate;
		elevarmState = ElevarmState.XRATE;
	}
	
	public void resetRateLimits()
	{
		elevatorLimiter.setMaxAccel(Constants.ElevatorVelocityLimit);
		elevatorLimiter.setMaxJerk(Constants.ElevatorAccelerationLimit);
		armLimiter.setMaxAccel(Constants.ArmVelocityLimit);
		armLimiter.setMaxJerk(Constants.ArmAccelerationLimit);
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
		System.out.println("Angle Set: " + armAngle);
		System.out.println("Elevator Set: " + elevatorHeight);
	}

	synchronized public void setElevarmIntakePosition() {
		if(elevarmState != ElevarmState.HOMING) {
			elevarmState = ElevarmState.INTAKE;
			armState = ArmState.POSITION;
			elevatorState = ElevatorState.POSITION;
		}
	}

	public void shiftElevatorGearbox(boolean engaged) {
		elevator.shiftElevatorGearbox(engaged);
	}

	synchronized public void homeElevator() {
		elevator.homeStartTime = System.currentTimeMillis();
		elevarmState = ElevarmState.HOMING;
		elevatorState = ElevatorState.MANUAL;
		armState = ArmState.MANUAL;
	}

	public void prepClimb() {
		setElevatorHeight(Constants.ElevatorUpHeight);
		setArmAngle(Constants.ArmDownDegrees);
		elevator.shiftElevatorGearbox(false);
	}

	public static boolean isValidAngleAndHeight(double armAngle, double elevatorHeight) {
		double x = Math.cos(Math.toRadians(armAngle)) * Constants.ArmLength;
		double y = elevatorHeight + Math.sin(Math.toRadians(armAngle)) * Constants.ArmLength;
		boolean isNaN = Double.isNaN(armAngle) || Double.isNaN(elevatorHeight);
		boolean tooFar = x > Constants.ArmLength;
		boolean armLow = armAngle < Constants.ArmLowerAngleLimit;
		boolean armHigh = armAngle > Constants.ArmUpperAngleLimit;
		boolean elevatorLow = elevatorHeight < Constants.ElevatorMinHeight;
		boolean elevatorHigh = elevatorHeight > Constants.ElevatorMaxHeight;
		boolean hittingElevator = (x < 14 && y < -10);
		if (isNaN || tooFar || armLow || armHigh || elevatorLow || elevatorHigh || hittingElevator) {
			return false;
		}
		return true;
	}

	public static boolean isValidPosition(double x, double y) {
		double armAngle = Math.toDegrees(Math.acos(x / Constants.ArmLength));
		double elevatorHeight = y - Math.sin(Math.toRadians(armAngle)) * Constants.ArmLength;
		return isValidAngleAndHeight(armAngle, elevatorHeight);
	}

	public void setClimberPercentOutput(double output) {
		elevator.setClimberPercentOutput(output);
	}

	public void setElevatorGearbox(boolean on) {
		elevator.setElevatorGearbox(on);
	}

	public double getX() {
		return Math.cos(Math.toRadians(getTargetArmAngle())) * Constants.ArmLength;
	}

	public double getY() {
		System.out.println("_____________________________");
		System.out.println("Angle: " + getTargetArmAngle());
		System.out.println("Angle Radians: " + Math.toRadians(getTargetArmAngle()));
		System.out.println("Angle Radians Sine: " + Math.sin(Math.toRadians(getTargetArmAngle())));
		System.out.println("Elevator Height: " + getTargetElevatorHeight());
		System.out.println("_____________________________");
		return getTargetElevatorHeight() + Math.sin(Math.toRadians(getTargetArmAngle())) * Constants.ArmLength;
	}

	public double getElevatorOutputCurrent() {
		return elevator.getOutputCurrent();
	}

	public double getArmOutputCurrent() {
		return arm.getOutputCurrent();
	}

	@Override
	public void update() {
		ElevatorState snapElevState;
		ArmState snapArmState;
		ElevarmState snapElevarmState;
		double snapElevSetpoint;
		double snapArmSetpoint;
		synchronized(this) {
			snapElevState = elevatorState;
			snapArmState = armState;
			snapElevarmState = elevarmState;
			snapElevSetpoint = elevatorSetpoint;
			snapArmSetpoint = armSetpoint;
		}
		
		
		
		switch (snapElevarmState) 	{
		case HOMING:
			if (!isValidAngleAndHeight(arm.getTargetAngle(), 0)) {
				// setArmAngle(Constants.ArmHorizontalDegrees);
				System.out.println("------------------------------------------------------------");
				System.out.println("CAN'T HOME. INVALID POSITION");
				System.out.println("	Angle: " + arm.getAngle() + " Setpoint: " + arm.getTargetAngle());
				System.out.println("	Height: " + elevator.getHeight() + " Setpoint: " + elevator.getTargetHeight());
				System.out.println("------------------------------------------------------------");
				synchronized(this){
					elevarmState = ElevarmState.EXTERNAL;					
				}
				break;
			}
			elevator.setPercentOutput(-.2); // Some slow speed
			if (elevator.getOutputCurrent() > Constants.ElevatorStallCurrent) {
				elevator.setPercentOutput(0);
				elevator.setEncoderPosition(0); // Sets encoder value to 0
				System.out.println("ELEVATOR HOMED");
				synchronized(this){
					elevarmState = ElevarmState.EXTERNAL;					
				}
			} else if (System.currentTimeMillis() - elevator.homeStartTime > 1500) {
				System.out.println("------------------------------------------------------------");
				System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
				System.out.println("	Height: " + elevator.getHeight() + " Setpoint: " + elevator.getTargetHeight());
				System.out.println("------------------------------------------------------------");
				elevator.setPercentOutput(0);
				elevator.setEncoderPosition((int) (Constants.ElevatorMinHeight
						* (1 / Constants.ElevatorInchesPerMotorRotation) * Constants.SensorTicksPerMotorRotation));
				synchronized(this){
					elevarmState = ElevarmState.EXTERNAL;					
				}
			}
			break;
		case INTAKE:
			synchronized(this) {				
				elevatorSetpoint = Constants.ElevatorDownHeight;
				if (elevator.getHeight() < 20) {
					armSetpoint = Constants.ArmIntakeDegrees;
				}
			}
			break;
		case EXTERNAL:
			break;
		case XRATE:
			double armSpeed = -xRateSetpoint * (180d / Math.PI) / (Constants.ArmLength * Math.sin(Math.toRadians(getTargetArmAngle())));
			double elevatorSpeed = (Math.cos(Math.toRadians(getTargetArmAngle())) / Math.sin(Math.toRadians(getTargetArmAngle()))) * xRateSetpoint;

			if (xRateSetpoint < 0)
			{
				armSpeed = Math.copySign(armSpeed, 1);
				elevatorSpeed = Math.copySign(elevatorSpeed, -1);
			}
			
			if (armSpeed > 1000 || elevatorSpeed > 1000 ||
					getArmAngle() < -30 || getArmAngle() > 94 ||
					getElevatorHeight() < 5 || getElevatorHeight() > 63 ||
					(xRateSetpoint > 0 && elevatorSpeed > 0 && getArmAngle() < 2) ||
					(xRateSetpoint > 0 && elevatorSpeed < 0 && getArmAngle() > -2)
					) //If we fix the math, get rid of this. Otherwise, this prevents near zero angles from messing everything up
			{
				armSpeed = 0;
				elevatorSpeed = 0;
			}
			
			System.out.println("Elevator Speed: " + elevatorSpeed);
			System.out.println("Arm Speed: " + armSpeed);
			
			elevatorSetpoint = getTargetElevatorHeight() + elevatorSpeed * 1.2 / 10d;
			armSetpoint = getTargetArmAngle() + armSpeed / 10d;
			
			//Set correct setpoint for arm and elevator to move to
			
			/*if (elevatorSpeed > 0)
				setElevatorHeight((getElevatorHeight() + 22) > Constants.ElevatorMaxHeight ? Constants.ElevatorMaxHeight : getElevatorHeight() + 22);
			else
				setElevatorHeight((getElevatorHeight() - 22) < Constants.ElevatorMinHeight ? Constants.ElevatorMinHeight : getElevatorHeight() - 22);
			
			if (xRateSetpoint > 0)
				setArmAngle(0);
			else
			{
				if (getArmAngle() >= 0)
					setArmAngle(Constants.ArmUpperAngleLimit);
				else
					setArmAngle(Constants.ArmLowerAngleLimit);
			}*/
			
			
			//Scale back speeds if too fast
			/*if (Math.abs(armSpeeed) > Constants.ArmVelocityLimit)
			{
				elevatorSpeed = elevatorSpeed * Constants.ArmVelocityLimit / armSpeed;
				armSpeed = Constants.ArmVelocityLimit;
			}
			if (Math.abs(elevatorSpeed) > Constants.ElevatorVelocityLimit)
			{
				armSpeed = armSpeed * Constants.ElevatorVelocityLimit / elevatorSpeed;
				elevatorSpeed = Constants.ElevatorVelocityLimit;
			}*/
			
	 		elevatorSpeedSetpoint = elevatorSpeed;
			armSpeedSetpoint = armSpeed;
			
			elevatorState = ElevatorState.POSITION;
			armState = ArmState.POSITION;
			
			//elevatorState = ElevatorState.SPEED;
			//armState = ArmState.SPEED;
			break;
		}
		
		switch(snapElevState) {
			case POSITION:
				double setpoint = elevatorLimiter.update(snapElevSetpoint);
				elevator.setHeight(setpoint);
				break;
			case SPEED:
				elevatorLimiter.setMaxAccel(elevatorSpeedSetpoint);
 				elevator.setHeight(armLimiter.update(snapElevSetpoint));
 				System.out.println("Elevator Speed: " + elevatorLimiter.getMaxAccel());
				//elevator.setSpeed(elevatorSpeedSetpoint);
				break;
			case MANUAL:
				elevatorLimiter.update(elevator.getHeight());
				break;
		}

		switch (snapArmState) {
			case POSITION:
				arm.setAngle(armLimiter.update(snapArmSetpoint));
				break;
			case SPEED:
				armLimiter.setMaxAccel(armSpeedSetpoint);
				arm.setAngle(armLimiter.update(snapArmSetpoint));
				System.out.println("Arm Speed: " + armLimiter.getMaxAccel());
				//arm.setSpeed(armSpeedSetpoint);
				break;
			case MANUAL:
				armLimiter.update(arm.getAngle());
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
		setClimberPercentOutput(0);
	}

	public boolean checkArm() {
		setElevatorHeight((Constants.ElevatorUpHeight + Constants.ElevatorDownHeight) / 2d); // Move elevator out of the
																								// way before testing
		Timer.delay(0.75);
		return arm.checkSubsytem();
	}

	public boolean checkClimber() {
		return elevator.checkClimber();
	}

	public void configArmEncoder() {
		arm.setEncoderFromPWM();
		System.out.println("Arm Encoder Set");
	}

	public int getArmPWMPosition() {
		return arm.getPWMPosition();
	}

	public int getArmEncoderPosition() {
		return arm.getEncoderPosition();
	}

	synchronized public void resetMotionProfile() {
		elevatorLimiter.reset();
		armLimiter.reset();
		elevatorLimiter.setLatestValue(elevator.getHeight());
		elevatorSetpoint = elevator.getHeight();
		armLimiter.setLatestValue(arm.getAngle());
		armSetpoint = arm.getAngle();
		elevarmState = ElevarmState.EXTERNAL;
		armState = ArmState.MANUAL;
		elevatorState = ElevatorState.MANUAL;
	}

	synchronized public boolean isHomed() {
		return elevarmState != ElevarmState.HOMING;
	}
}