package org.usfirst.frc.team3476.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {

	public LazyTalonSRX(int deviceNumber) {
		super(deviceNumber);
	}
	
	@Override
	public void set(ControlMode controlMode, double outputValue)
	{
		if (controlMode != super.getControlMode() || outputValue != super.getClosedLoopTarget(0)) //Only deals with pid profile 0
			super.set(controlMode, outputValue);
	}

}
