package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Elevarm;

public class HomeElevator extends AutoCommand {

	@Override
	public void start() {
		Elevarm.getInstance().homeElevator();
		setBlocking(true);
	}

	@Override
	public boolean isFinished() {
		return Elevarm.getInstance().isHomed();
	}

}
