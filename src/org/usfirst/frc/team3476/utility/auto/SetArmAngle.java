package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Elevarm;

public class SetArmAngle extends AutoCommand {

	private double angle;
	public SetArmAngle(double angle)
	{
		this.angle = angle;
	}
	
	public SetArmAngle(double angle, boolean isBlocking)
	{
		this.angle = angle;
		setBlocking(isBlocking);
	}
	
	@Override
	public void start() {
		System.out.println("Set Arm Angle");
		Elevarm.getInstance().setArmAngle(angle);
		
	}

	@Override
	public boolean isFinished() {
		return Math.abs(Elevarm.getInstance().getArmAngle() - angle) < 10;
	}
	
}
