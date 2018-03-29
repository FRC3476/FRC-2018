package org.usfirst.frc.team3476.utility.auto;

import java.util.ArrayList;

import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class DriveToPoints extends AutoCommand {

	private ArrayList<Translation2d> points;
	private double speed;
	private boolean isReversed;

	public DriveToPoints(double speed, boolean isReversed, Translation2d... points) {
		this.points = new ArrayList<Translation2d>();
		this.speed = speed;
		this.isReversed = isReversed;
		setBlocking(true);
		for (Translation2d point : points) {
			this.points.add(point);
		}
	}

	@Override
	public void start() {
		System.out.println("Drive To Points");
		Path drivePath = new Path(RobotTracker.getInstance().getOdometry().translationMat);
		for (Translation2d point : points) {
			drivePath.addPoint(point.getX(), point.getY(), speed);
		}
		OrangeDrive.getInstance().setAutoPath(drivePath, isReversed);
	}

	@Override
	public boolean isFinished() {
		return OrangeDrive.getInstance().isFinished();
	}

}