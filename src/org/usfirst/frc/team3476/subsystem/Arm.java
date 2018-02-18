package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Arm {

	private LazyTalonSRX armTalon;
	public static final double HORIZONTAL = 100000000, DOWN = 0; // Update with real values
	protected long homeStartTime;

	private static final Arm instance = new Arm();

	private Arm() {
		armTalon = new LazyTalonSRX(Constants.ArmId);
		armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	}

	public static Arm getInstance() {
		return instance;
	}

	public void setPercentOutput(double output) {
		armTalon.set(ControlMode.PercentOutput, output);
	}

	protected void setEncoderPosition(int position) {
		armTalon.setSelectedSensorPosition(position, 0, 10);
	}

	protected void setAngle(double angle) {
		armTalon.set(ControlMode.Position, angle * (1 / Constants.ArmRotationsPerMotorRotation) * Constants.SensorTicksPerMotorRotation);
	}

	public double getAngle() {
		return armTalon.getSelectedSensorPosition(0) * (1 / Constants.SensorTicksPerMotorRotation) * Constants.ArmRotationsPerMotorRotation;
	}
	
	public double getTargetAngle() {
		return armTalon.getClosedLoopTarget(0) * (1 / Constants.SensorTicksPerMotorRotation) * Constants.ArmRotationsPerMotorRotation;
	}

	public double getOutputCurrent() {
		return armTalon.getOutputCurrent();
	}
	
	public boolean checkSubsytem() {
		return OrangeUtility.checkMotors(0.25, Constants.ExpectedArmCurrent, Constants.ExpectedArmRPM, Constants.ExpectedArmPosition, armTalon, armTalon);
	}
}
