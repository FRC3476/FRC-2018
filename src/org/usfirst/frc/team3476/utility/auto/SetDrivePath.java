package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.utility.control.motion.Path;

public class SetDrivePath extends AutoCommand {

	private Path robotPath;
	private boolean isReversed;

	public SetDrivePath(Path robotPath, boolean isReversed) {
		this(robotPath, isReversed, true);
	}

	public SetDrivePath(Path robotPath, boolean isReversed, boolean isBlocking) {
		System.out.println("Set Drive Path");
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		this.setBlocking(isBlocking);
	}

	@Override
	public boolean isFinished() {
		if(OrangeDrive.getInstance().isFinished()) {
			System.out.println("pathcomplete");
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		OrangeDrive.getInstance().setAutoPath(robotPath, isReversed);

	}

}
