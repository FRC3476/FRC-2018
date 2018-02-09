package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {

	private TalonSRX elevatorTalon, slaveTalon;
	public final double HIGHEST = 1000000000, LOWEST = -1000000000; //Replace with actual values
	
	private static final Elevator instance = new Elevator();
	
	private Elevator() {
		elevatorTalon= new TalonSRX(Constants.ElevatorMotorId);
		slaveTalon = new TalonSRX(Constants.ElevatorSlaveMotorId);
		elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		slaveTalon.set(ControlMode.Follower, elevatorTalon.getDeviceID());		
	}
	
	public static Elevator getInstance() {
		return instance;
	}
	
	public void setElevatorHeight(double height) {	
		elevatorTalon.set(ControlMode.Position, height * Constants.ElevatorHeightToMotorRotations * Constants.SensorTicksPerRev);
	}
	
}
