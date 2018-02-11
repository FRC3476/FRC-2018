package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;

public class Elevarm extends Threaded {

	private static final Elevarm instance = new Elevarm();
	
	Elevator elevator;
	Arm arm;
	
	public enum ElevatorState {MANUAL, HOMING}
	public enum ArmState{MANUAL, HOMING}
	
	private ElevatorState currentElevatorState = ElevatorState.MANUAL;
	private ArmState currentArmState = ArmState.MANUAL;
	
	private Elevarm() {
		elevator = Elevator.getInstance();
		arm = Arm.getInstance();
	}
	
	public static Elevarm getInstance()
	{
		return instance;
	}
	
	public void setElevatorHeight(double height)
	{
		if (isValidPosition(arm.getClosedLoopTarget(), height)) //If no collisions with the final positions, move the elevator to the position
		{
			elevator.setHeight(height);
		}
		else
		{
			System.out.println("Collision detected. Elevator not moving");
		}
	}
	 	
	public void setArmAngle(double angle)
	{
		if (isValidPosition(angle, elevator.getClosedLoopTarget())) //If no collisions with the final positions, move the arm to the position
		{
			arm.setAngle(angle);
		}
		else
		{ 
			System.out.println("Collision detected. Arm not moving");
		}
	}
	
	public void setOverallPosition(double distance, double height)
	{	
		if (distance > Constants.ArmLength || height > Constants.ElevatorHeight)
		{
			System.out.println("Position not reachable by elevator.");
			return;
		}
		
		double armAngle = arm.getAngle();
		double elevatorHeight = elevator.getHeight();
		
		double armAngle1 = 180 - Math.asin(distance / Constants.ArmLength);
		double elevatorHeight1 = height - Math.sqrt(Constants.ArmLength * Constants.ArmLength - distance * distance);
		
		double armAngle2 = Math.asin(distance / Constants.ArmLength);
		double elevatorHeight2 = height + Math.sqrt(Constants.ArmLength * Constants.ArmLength - distance * distance);
		
		boolean position1Valid = isValidPosition(armAngle1, elevatorHeight1);
		boolean position2Valid = isValidPosition(armAngle2, elevatorHeight2);
		
		if (position1Valid && position2Valid)
		{
			double armTime1 = Math.abs(arm.getAngle() - armAngle1) * Constants.ArmSpeed;
			double elevatorTime1 = Math.abs(elevator.getHeight() - elevatorHeight1) * Constants.ElevatorSpeed;
			double timeToPosition1 = (armTime1 > elevatorTime1 ? armTime1 : elevatorTime1);
			
			double armTime2 = Math.abs(arm.getAngle() - armAngle2) * Constants.ArmSpeed;
			double elevatorTime2 = Math.abs(elevator.getHeight() - elevatorHeight2) * Constants.ElevatorSpeed;
			double timeToPosition2 = (armTime2 > elevatorTime2 ? armTime2 : elevatorTime2);
			
			armAngle = (timeToPosition1 < timeToPosition2 ? armAngle1 : armAngle2);
			elevatorHeight = (timeToPosition1 < timeToPosition2 ? elevatorHeight1: elevatorHeight2);
			
		}
		else if (position1Valid)
		{ 
			armAngle = armAngle1;
			elevatorHeight = elevatorHeight1;
		}
		else if (position2Valid)
		{
			armAngle = armAngle2;
			elevatorHeight = elevatorHeight2;
		}
		else
		{
			System.out.println("Invalid Position. Not moving");
		}
		
		setElevatorHeight(elevatorHeight);
		setArmAngle(armAngle);
	}
	
	public void homeElevator()
	{
		elevator.homeStartTime = System.currentTimeMillis();
		currentElevatorState = ElevatorState.HOMING;
	}
	
	public void homeArm()
	{
		arm.homeStartTime = System.currentTimeMillis();
		currentArmState = ArmState.HOMING;
	}
	
	public static boolean isValidPosition(double armAngle, double elevatorHeight)
	{
		double x = Math.sin(armAngle) * Constants.ArmLength;
		double y = elevatorHeight - Constants.ArmLength * Math.cos(armAngle);
		
		return !(armAngle < Constants.ArmLowerAngleLimit			//Checks if
				|| armAngle > Constants.ArmUpperAngleLimit			//limits of
				|| elevatorHeight < Constants.ElevatorMinPosition	//elevator or arm
				|| elevatorHeight > Constants.ElevatorMaxPosition);	//are exceeded
				//Add more constraints if needed
	}

	@Override
	public void update() {
		switch (currentElevatorState)
		{
			case HOMING:
				if (!isValidPosition(arm.getAngle(), 0))
				{
					setArmAngle(arm.HORIZONTAL);
					break;
				}
				elevator.setPercentOutput(.1); //Some slow speed
				if (elevator.getOutputCurrent() > Constants.ElevatorStallCurrent)
					{
						elevator.setPercentOutput(0);
						elevator.setEncoderPosition(0); //Sets encoder value to 0
						//elevator.setHeight(elevator.DOWN); Add this back in if we need to go to a certain position after homing
						currentElevatorState = ElevatorState.MANUAL;
					}
				else if (System.currentTimeMillis() - elevator.homeStartTime > 1000)
				{
					System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
					elevator.setPercentOutput(0);
					elevator.setEncoderPosition(0);
					//elevator.setHeight(elevator.DOWN); Add this back in if we need to go to a certain position after homing
					currentElevatorState = ElevatorState.MANUAL;
				}
				break;
			case MANUAL:
				break;
		}
		switch (currentArmState)
		{
			case HOMING:
				if (!isValidPosition(0, elevator.getHeight()))
				{
					setElevatorHeight(elevator.HOMING_HEIGHT);
					break;
				}
				arm.setPercentOutput(0.3);
				if (arm.getOutputCurrent() > Constants.ElevatorStallCurrent)
				{
					arm.setPercentOutput(0);
					arm.setEncoderPosition(0);
					//arm.setAngle(arm.DOWN); Add this back in if we need to go to a certain position after homing
					currentArmState = ArmState.MANUAL;
				}
				else if (System.currentTimeMillis() - arm.homeStartTime > 1000)
				{
					System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
					arm.setPercentOutput(0);
					arm.setEncoderPosition(0);
					//arm.setAngle(arm.DOWN); Add this back in if we need to go to a certain position after homing
					currentArmState = ArmState.MANUAL;
				}
				break;
			case MANUAL:
				break;
		}
	}

}
