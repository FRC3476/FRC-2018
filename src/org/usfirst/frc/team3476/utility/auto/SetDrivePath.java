package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.utility.control.Path;

public class SetDrivePath implements AutoCommand {

	private Path robotPath;
	private boolean isReversed;
	
	public SetDrivePath (Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
	}
	
	public void run() {
		OrangeDrive.getInstance().setAutoPath(robotPath, isReversed);
	}
}
