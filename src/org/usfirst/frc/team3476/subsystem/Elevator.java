package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

public class Elevator extends Threaded {

	public enum ElevatorState
	{
		HOMING, UP, DOWN, MANUAL
	}
	
	public final double UP = 100000, DOWN = 0; //Replace with actual values
	
	private Elevator() {
		elevatorTalon = new CANTalon(Constants.ElevatorId);
		armTalon = new CANTalon(Constants.ArmId);
		
		elevatorTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		armTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);

		elevatorTalon.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
		armTalon.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);

		elevatorTalon.configEncoderCodesPerRev(1024);
		armTalon.configEncoderCodesPerRev(1024);
		
		
	}
	
	private CANTalon elevatorTalon, armTalon;
	private ElevatorState currentState = ElevatorState.MANUAL;
	private double homeStartTime;
	
	private static final Elevator instance = new Elevator();
		
	
	public void homeElevator()
	{
		homeStartTime = System.currentTimeMillis();
		currentState = ElevatorState.HOMING;
	}
	
	public void setElevatorHeight(double height)
	{
		elevatorTalon.changeControlMode(TalonControlMode.Position);
		elevatorTalon.setSetpoint(height * Constants.ElevatorHeightToEncoderTicks);
	}
	
	public void setArmDistance(double distance)
	{
		
	}
	
	public void setOverallPosition(double distance, double height)
	{	
		if (distance > Constants.ArmLength || height > Constants.ElevatorHeight)
		{
			System.out.println("Position not reachable by elevator.");
			return;
		}
		
		double armAngle = Math.asin(distance / Constants.ArmLength);
		double elevatorPosition = height - Math.sqrt(Constants.ArmLength * Constants.ArmLength - distance * distance);
		
		setElevatorHeight(elevatorPosition);
		
		
		//theta = arcsin(distance/armlength)
		//y = height - sqrt(arm^2 - distance^2)
	}

	@Override
	public void update() {
		switch(currentState)
		{
		case MANUAL:
			break;
		case UP:
			setElevatorHeight(UP);
			break;
		case DOWN:
			setElevatorHeight(DOWN);
			break;
		case HOMING:
			elevatorTalon.changeControlMode(TalonControlMode.PercentVbus);
			elevatorTalon.setSetpoint(0.3);
			if (elevatorTalon.getOutputCurrent() > Constants.ElevatorStallCurrent)
				{
					elevatorTalon.setSetpoint(0);
					elevatorTalon.setPosition(0); //Set Encoder value to 0
					currentState = ElevatorState.DOWN;
				}
			else if (System.currentTimeMillis() - homeStartTime > 1000)
			{
				System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
				elevatorTalon.setSetpoint(0);
				elevatorTalon.setPosition(0);
				currentState = ElevatorState.DOWN;
			}
			break;
		
		}

	}

}
