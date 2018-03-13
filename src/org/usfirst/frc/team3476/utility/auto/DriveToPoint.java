package org.usfirst.frc.team3476.utility.auto;

import java.util.ArrayList;

import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class DriveToPoint extends AutoCommand{

	private ArrayList<Translation2d> points;
	private double speed;
	private boolean isReversed;
	public DriveToPoint(Translation2d point, double speed, boolean isReversed)
	{
		points.add(point);
		this.speed = speed;
		this.isReversed = isReversed;
	}
	
	@Override
	public void start() {
		Path drivePath = new Path(RobotTracker.getInstance().getOdometry().translationMat);
		for (Translation2d point : points)
		{
			drivePath.addPoint(point, speed);
		}
		OrangeDrive.getInstance().setAutoPath(drivePath, isReversed);
	}

	@Override
	public boolean isFinished() {
		return OrangeDrive.getInstance().isFinished();
	}

}
