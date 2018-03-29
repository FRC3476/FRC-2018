package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Elevarm;

public class SetElevatorHeight extends AutoCommand {

	private double height;

	public SetElevatorHeight(double height) {
		this.height = height;
	}

	public SetElevatorHeight(double height, boolean isBlocking) {
		this.height = height;
		setBlocking(isBlocking);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(Elevarm.getInstance().getElevatorHeight() - height) < 10;
	}

	@Override
	public void start() {
		System.out.println("Set Elevator Height");
		Elevarm.getInstance().setElevatorHeight(height);
	}
}
