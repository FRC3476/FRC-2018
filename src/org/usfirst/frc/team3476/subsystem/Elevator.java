package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

public class Elevator extends Threaded {

	private CANTalon elevatorTalon;
	public enum ElevatorState { HOMING, MANUAL }
	private ElevatorState currentState = ElevatorState.MANUAL;
	private double homeStartTime;
	private double setpoint;
	public final double UP = 100000, DOWN = 0; //Replace with actual values
	
	private static final Elevator instance = new Elevator();
	
	private Elevator() {
		elevatorTalon= new CANTalon(Constants.ElevatorId);
		elevatorTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		elevatorTalon.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
		elevatorTalon.configEncoderCodesPerRev(1024);
	}
	
	public static Elevator getInstance()
	{
		return instance;
	}
	
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
	
	/*
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
		setArmAngle(armAngle);
	}
*/
	@Override
	public void update() {
		switch(currentState)
		{
		case MANUAL:
			setElevatorHeight(setpoint);
			break;
		case HOMING:
			elevatorTalon.changeControlMode(TalonControlMode.PercentVbus);
			elevatorTalon.setSetpoint(0.3);
			if (elevatorTalon.getOutputCurrent() > Constants.ElevatorStallCurrent)
				{
					elevatorTalon.setSetpoint(0);
					elevatorTalon.setPosition(0); //Sets encoder value to 0
					currentState = ElevatorState.MANUAL;
				}
			else if (System.currentTimeMillis() - homeStartTime > 1000)
			{
				System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
				elevatorTalon.setSetpoint(0);
				elevatorTalon.setPosition(0);
				currentState = ElevatorState.MANUAL;
			}
			break;
		}

	}

}
