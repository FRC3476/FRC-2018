package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Elevarm;

public class SetElevatorHeight implements AutoCommand {

	private double height;
	
	public SetElevatorHeight(double height) {
		this.height = height;
	}
	
	public void run() {
		Elevarm.getInstance().setElevatorHeight(height);
	}
}
