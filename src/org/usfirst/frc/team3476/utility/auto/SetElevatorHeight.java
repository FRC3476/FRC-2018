package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Elevarm;

public class SetElevatorHeight implements AutoCommand {

	private double height;
	private boolean isBlocking = false;
	
	public SetElevatorHeight(double height) {
		this.height = height;
	}
	
	public void run() {
		Elevarm.getInstance().setElevatorHeight(height);
		if(isBlocking) {
			while(Math.abs(Elevarm.getInstance().getElevatorHeight() - height) < 5) {
				//Do nothing
			}
		}
	}

	@Override
	public void setBlocking(boolean isBlocking) {
				
	}
}
