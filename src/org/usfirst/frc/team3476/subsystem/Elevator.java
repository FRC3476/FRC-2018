package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Elevator {

	private LazyTalonSRX elevatorTalon, slaveTalon;
	protected long homeStartTime;

	private static final Elevator instance = new Elevator();
	public static final double DOWN = 0, UP = -0, ARM_HOMING_HEIGHT = -0;

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
		elevatorTalon.set(ControlMode.Position, height * (1 / Constants.ElevatorInchesPerMotorRotation)
				* Constants.SensorTicksPerMotorRotation);
	}

	public double getHeight() {
		return elevatorTalon.getSelectedSensorPosition(0) * (1 / Constants.SensorTicksPerMotorRotation) * Constants.ElevatorInchesPerMotorRotation;
	}
	
	public double getTargetHeight() {
		return elevatorTalon.getClosedLoopTarget(0) * (1 / Constants.SensorTicksPerMotorRotation) * Constants.ElevatorInchesPerMotorRotation;
	}

	public double getOutputCurrent() {
		return elevatorTalon.getOutputCurrent();
	}

	protected LazyTalonSRX[] getTalons() {
		return new LazyTalonSRX[] { elevatorTalon, slaveTalon };
	}
	
	public boolean checkSubystem() {
		return OrangeUtility.checkMotors(.25, Constants.ExpectedElevatorCurrent, Constants.ExpectedElevatorRPM, Constants.ExpectedElevatorPosition, elevatorTalon, elevatorTalon, slaveTalon);
	}
}
