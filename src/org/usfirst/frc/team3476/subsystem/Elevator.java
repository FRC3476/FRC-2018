package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Elevator {

	private LazyTalonSRX elevatorTalon, slaveTalon;
	public long homeStartTime;

	private static final Elevator instance = new Elevator();
	protected final double DOWN = 0, UP = -0, HOMING_HEIGHT = -0;

	private Elevator() {
		elevatorTalon = new LazyTalonSRX(Constants.ElevatorMotorId);
		slaveTalon = new LazyTalonSRX(Constants.ElevatorSlaveMotorId);
		elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		slaveTalon.set(ControlMode.Follower, elevatorTalon.getDeviceID());
	}

	public static Elevator getInstance() {
		return instance;
	}

	public void setPercentOutput(double output) {
		elevatorTalon.set(ControlMode.PercentOutput, output);
	}

	protected void setEncoderPosition(int position) {
		elevatorTalon.setSelectedSensorPosition(position, 0, 10);
	}

	protected void setHeight(double height) {
		elevatorTalon.set(ControlMode.Position, height * Constants.ElevatorHeightToMotorRotations
				* Constants.SensorTicksPerRev);
	}

	public double getHeight() {
		return elevatorTalon.getSelectedSensorPosition(0) * Constants.ElevatorHeightToMotorRotations
				* Constants.SensorTicksPerRev;
	}

	public double getOutputCurrent() {
		return elevatorTalon.getOutputCurrent();
	}

	public double getClosedLoopTarget() {
		return elevatorTalon.getClosedLoopError(0);
	}

	protected LazyTalonSRX[] getTalons() {
		return new LazyTalonSRX[] { elevatorTalon, slaveTalon };
	}
}
