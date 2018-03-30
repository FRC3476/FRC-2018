package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;

public class Elevator {

	private LazyTalonSRX elevatorTalon, slaveTalon, climber1Talon, climber2Talon;
	private Solenoid gearboxSolenoid;
	protected long homeStartTime;

	private static final Elevator instance = new Elevator();

	//P .1
	//I .0001
	//D .0001
	//I Zone 1000
	private Elevator() {
		elevatorTalon = new LazyTalonSRX(Constants.ElevatorMotorId);
		slaveTalon = new LazyTalonSRX(Constants.ElevatorSlaveMotorId);
		elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		elevatorTalon.setInverted(true);
		elevatorTalon.setSensorPhase(true); // flip encoder direction
		slaveTalon.set(ControlMode.Follower, elevatorTalon.getDeviceID());
		slaveTalon.setInverted(true);

		climber1Talon = new LazyTalonSRX(Constants.Climber1TalonId);
		climber2Talon = new LazyTalonSRX(Constants.Climber2TalonId);
		climber1Talon.setInverted(true);
		climber2Talon.setInverted(true);

		gearboxSolenoid = new Solenoid(Constants.ElevatorGearboxShifterId);
		
		elevatorTalon.config_kP(0, 0.125, 10);
		elevatorTalon.config_kI(0, 0.0, 10);
		elevatorTalon.config_kD(0, 0.0, 10);
		elevatorTalon.config_IntegralZone(0, 1000, 10);
	}

	public void setElevatorGearbox(boolean on) {
		gearboxSolenoid.set(on);
	}

	public void setClimberPercentOutput(double output) {
		climber1Talon.set(ControlMode.PercentOutput, output);
		climber2Talon.set(ControlMode.PercentOutput, output);
	}

	public double getClimberCurrent() {
		return (climber1Talon.getOutputCurrent() + climber2Talon.getOutputCurrent()) / 2d;
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

	public void setSpeed(double speed) {
		elevatorTalon.set(ControlMode.Velocity, speed * (1d / Constants.ElevatorInchesPerMotorRotation)
				* Constants.SensorTicksPerMotorRotation);
	}

	public double getSpeed() {
		return elevatorTalon.getSelectedSensorVelocity(0) * (1d / Constants.SensorTicksPerMotorRotation)
				* Constants.ElevatorInchesPerMotorRotation;
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
		boolean success = OrangeUtility.checkMotors(.25, Constants.ExpectedElevatorCurrent, Constants.ExpectedElevatorRPM, Constants.ExpectedElevatorPosition, elevatorTalon, elevatorTalon, slaveTalon);
		configMotors();
		return success;
	}

	public boolean checkClimber() {
		return OrangeUtility.checkMotors(.25, Constants.ExpectedClimberCurrent, 0, 0, null, climber1Talon, climber2Talon);
	}
}
