package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.utility.CircularQueue;
import org.usfirst.frc.team3476.utility.InterpolableValue;
import org.usfirst.frc.team3476.utility.RigidTransform;
import org.usfirst.frc.team3476.utility.Rotation;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.Translation2d;

public class RobotTracker extends Threaded {

	private static final RobotTracker trackingInstance = new RobotTracker();

	public static RobotTracker getInstance() {
		return RobotTracker.trackingInstance;
	}

	private OrangeDrive driveBase;
	private RigidTransform currentOdometry;
	private CircularQueue<RigidTransform> vehicleHistory;
	private CircularQueue<Rotation> gyroHistory;

	private double currentDistance, oldDistance, deltaDistance;

	private RobotTracker() {
		vehicleHistory = new CircularQueue<>(100);
		gyroHistory = new CircularQueue<>(200);
		driveBase = OrangeDrive.getInstance();
		driveBase.zeroSensors();
		currentOdometry = new RigidTransform(new Translation2d(), driveBase.getGyroAngle());
	}

	public Rotation getGyroAngle(long time) {
		return gyroHistory.getInterpolatedKey(time);
	}

	public synchronized RigidTransform getOdometry() {
		return currentOdometry;
	}

	public synchronized void resetOdometry() {
		driveBase.zeroSensors();
		currentOdometry = new RigidTransform(new Translation2d(), driveBase.getGyroAngle());
		oldDistance = 0;
	}

	@Override
	public void update() {
		currentDistance = (driveBase.getLeftDistance() + driveBase.getRightDistance()) / 2;
		deltaDistance = currentDistance - oldDistance;
		synchronized(this){
			Rotation deltaRotation = driveBase.getGyroAngle();
			Translation2d deltaPosition = new Translation2d(deltaRotation.sin() * deltaDistance, deltaRotation.cos() * deltaDistance);
			//currentOdometry = currentOdometry.transform(new RigidTransform(deltaPosition, deltaRotation));
			currentOdometry = new RigidTransform(currentOdometry.translationMat.translateBy(deltaPosition), driveBase.getGyroAngle());
			oldDistance = currentDistance;
			//vehicleHistory.add(new InterpolableValue<>(System.nanoTime(), currentOdometry));
			//gyroHistory.add(new InterpolableValue<>(System.nanoTime(), driveBase.getGyroAngle()));
		}
		/*
		double sTBT;
		double cTBT;
		if (Math.abs(deltaRotation.getRadians()) < 1E-9) {
			sTBT = 1.0 - 1.0 / 6.0 * deltaRotation.getRadians() * deltaRotation.getRadians();
			cTBT = 0.5 * deltaRotation.getRadians() - 1.0 / 24.0 * Math.pow(deltaRotation.getRadians(), 3);
		} else {
			sTBT = deltaRotation.sin() / deltaRotation.getRadians();
			cTBT = (1 - deltaRotation.cos()) / deltaRotation.getRadians();
		}
		Translation2d deltaPosition = new Translation2d(cTBT * deltaDistance, sTBT * deltaDistance);
		synchronized (this) {
			currentOdometry = currentOdometry.transform(new RigidTransform(deltaPosition, deltaRotation));
			oldDistance = currentDistance;
		}
		*/
	}
}

/*
 * How we calculate curvature
 * 
 * From https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp //Group
 * exponential
 * 
 */