package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.Elevator.ElevatorState;
import org.usfirst.frc.team3476.utility.Threaded;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

public class Arm extends Threaded {

	private CANTalon armTalon;
	public enum ArmState { HOMING, MANUAL }
	private ArmState currentState = ArmState.MANUAL;
	private double homeStartTime;
	private double setpoint;
	public final double HORIZONTAL = 100000000, DOWN = 0; //Update with real values
	
	public static final Arm instance = new Arm();
	
	private Arm() {
		armTalon = new CANTalon(Constants.ArmId);
		armTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		armTalon.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
		armTalon.configEncoderCodesPerRev(1024);
	}
	
	public Arm getInstance()
	{
		return instance;
	}
	
	public void homeElevator()
	{
		homeStartTime = System.currentTimeMillis();
		currentState = ArmState.HOMING;
	}
	
	public void setArmAngle(double degreesFromVertical)
	{
		
	}

	@Override
	public void update() {
		switch(currentState)
		{
		case MANUAL:
			setArmAngle(setpoint);
			break;
		case HOMING:
			armTalon.changeControlMode(TalonControlMode.PercentVbus);
			armTalon.setSetpoint(0.3);
			if (armTalon.getOutputCurrent() > Constants.ElevatorStallCurrent)
				{
					armTalon.setSetpoint(0);
					armTalon.setPosition(0); //Sets encoder value to 0
					currentState = ArmState.MANUAL;
				}
			else if (System.currentTimeMillis() - homeStartTime > 1000)
			{
				System.out.println("FAILED TO HOME. USING CURRENT POSITION AS HOME");
				armTalon.setSetpoint(0);
				armTalon.setPosition(0);
				currentState = ArmState.MANUAL;
			}
			break;
		}

	}

}
