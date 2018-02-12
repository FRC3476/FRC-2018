package org.usfirst.frc.team3476.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {

	double prevValue = 0;
	
	public LazyTalonSRX(int deviceNumber) {
		super(deviceNumber);
		this.enableVoltageCompensation(true);
	}
	
	@Override
	public void set(ControlMode controlMode, double outputValue)
	{
		if (outputValue != prevValue)
		{
			super.set(controlMode, outputValue);
			prevValue = outputValue;
		}
	}

}
