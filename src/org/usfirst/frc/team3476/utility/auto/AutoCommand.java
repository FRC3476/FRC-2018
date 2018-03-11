package org.usfirst.frc.team3476.utility.auto;

import edu.wpi.first.wpilibj.DriverStation;

public abstract class AutoCommand {
	
	public abstract void start();
	public abstract boolean isFinished();
	
	private boolean isBlocking = false;
	
	public void run() {
		start();
		if(isBlocking) {
			while(!isFinished() && DriverStation.getInstance().isAutonomous()){
				
			}
		}
	}

	public void setBlocking(boolean isBlocking) {
		this.isBlocking = isBlocking;		
	}
	
}
