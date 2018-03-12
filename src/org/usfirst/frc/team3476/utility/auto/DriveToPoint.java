package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class DriveToPoint extends AutoCommand{

	private Translation2d point;
	private double speed;
	private boolean isReversed;
	public DriveToPoint(Translation2d point, double speed, boolean isReversed)
	{
		this.point = point;
		this.speed = speed;
		this.isReversed = isReversed;
	}
	
	@Override
	public void start() {
		Path drivePath = new Path(RobotTracker.getInstance().getOdometry().translationMat);
		drivePath.addPoint(point, speed);
		OrangeDrive.getInstance().setAutoPath(drivePath, isReversed);
	}

	@Override
	public boolean isFinished() {
		return OrangeDrive.getInstance().isFinished();
	}

}
