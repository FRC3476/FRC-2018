package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Elevarm;

public class SetElevatorHeight extends AutoCommand {

	private double height;
	
	public SetElevatorHeight(double height) {
		this.height = height;
	}
	
	public boolean isFinished() {
		return Math.abs(Elevarm.getInstance().getElevatorHeight() - height) < 5;
	}

	@Override
	public void start() {
		Elevarm.getInstance().setElevatorHeight(height);
	}
}
