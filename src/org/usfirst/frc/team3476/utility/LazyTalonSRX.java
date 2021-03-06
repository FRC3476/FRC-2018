package org.usfirst.frc.team3476.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Sends only new commands to the Talon to reduce CAN usage.
 *
 */
public class LazyTalonSRX extends TalonSRX {

	private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	public LazyTalonSRX(int deviceNumber) {
		super(deviceNumber);
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
		}
	}

	public double getSetpoint() {
		return prevValue;
	}

}
