package org.usfirst.frc.team3476.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {

	private double prevValue = 0;

	public LazyTalonSRX(int deviceNumber) {
		super(deviceNumber);
		enableVoltageCompensation(true);
		configVoltageCompSaturation(12, 10);
	}	

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		if (outputValue != prevValue) {
			set(controlMode, outputValue);
			prevValue = outputValue;
		}
	}

}
