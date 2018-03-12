package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Elevarm;

public class SetArmAngle extends AutoCommand {

	private double angle;
	public SetArmAngle(double angle)
	{
		this.angle = angle;
	}
	
	@Override
	public void start() {
		Elevarm.getInstance().setArmAngle(angle);
		
	}

	@Override
	public boolean isFinished() {
		return Math.abs(Elevarm.getInstance().getArmAngle() - angle) < 5;
	}
	
}
