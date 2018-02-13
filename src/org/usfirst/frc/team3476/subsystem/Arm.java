package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Arm {

	private LazyTalonSRX armTalon;
	protected final double HORIZONTAL = 100000000, DOWN = 0; // Update with real values
	public long homeStartTime;

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
		armTalon.set(ControlMode.Position, angle * Constants.ArmAngleToMotorRotations * Constants.SensorTicksPerRev);
	}

	public double getAngle() {
		return armTalon.getSelectedSensorPosition(0);
	}

	public double getOutputCurrent() {
		return armTalon.getOutputCurrent();
	}

	public double getClosedLoopTarget() {
		return armTalon.getClosedLoopTarget(0);
	}
}
