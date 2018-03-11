package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;

public class Elevator {

	private LazyTalonSRX elevatorTalon, slaveTalon;
	private Solenoid gearboxSolenoid;
	protected long homeStartTime;

	private static final Elevator instance = new Elevator();

	private Elevator() {
		elevatorTalon = new LazyTalonSRX(Constants.ElevatorMotorId);
		slaveTalon = new LazyTalonSRX(Constants.ElevatorSlaveMotorId);
		elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		elevatorTalon.setInverted(true);
		elevatorTalon.setSensorPhase(true); // flip encoder direction
		slaveTalon.set(ControlMode.Follower, elevatorTalon.getDeviceID());
		slaveTalon.setInverted(true);

		gearboxSolenoid = new Solenoid(Constants.ElevatorGearboxShifterId);
	}

	protected static Elevator getInstance() {
		return instance;
	}

	public void setPercentOutput(double output) {
		elevatorTalon.set(ControlMode.PercentOutput, output);
	}

	public int getEncoderTicks() {
		return elevatorTalon.getSelectedSensorPosition(0);
	}

	protected void setEncoderPosition(int position) {
		elevatorTalon.setSelectedSensorPosition(position, 0, 10);
	}

	protected void setHeight(double height) {
		elevatorTalon.set(ControlMode.Position, height * (1d / Constants.ElevatorInchesPerMotorRotation)
				* Constants.SensorTicksPerMotorRotation);
	}

	public void shiftElevatorGearbox(boolean engaged) {
		gearboxSolenoid.set(engaged);
	}

	public double getHeight() {
		return elevatorTalon.getSelectedSensorPosition(0) * (1d / Constants.SensorTicksPerMotorRotation)
				* Constants.ElevatorInchesPerMotorRotation;
	}

	public double getTargetHeight() {
		return elevatorTalon.getSetpoint() * (1d / Constants.SensorTicksPerMotorRotation)
				* Constants.ElevatorInchesPerMotorRotation;
	}

	public double getOutputCurrent() {
		return (elevatorTalon.getOutputCurrent() + slaveTalon.getOutputCurrent()) / 2d;
	}

	public void configMotors() {
		slaveTalon.set(ControlMode.Follower, elevatorTalon.getDeviceID());		
	}

	protected LazyTalonSRX[] getTalons() {
		return new LazyTalonSRX[] { elevatorTalon, slaveTalon };
	}

	public boolean checkSubystem() {
		boolean success = OrangeUtility.checkMotors(.05, Constants.ExpectedElevatorCurrent, Constants.ExpectedElevatorRPM, Constants.ExpectedElevatorPosition, elevatorTalon, elevatorTalon, slaveTalon);
		configMotors();
		return success;
	}
}
