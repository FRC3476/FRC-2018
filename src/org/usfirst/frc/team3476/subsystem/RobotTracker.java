package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.utility.CircularQueue;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.math.InterpolablePair;
import org.usfirst.frc.team3476.utility.math.RigidTransform;
import org.usfirst.frc.team3476.utility.math.Rotation;
import org.usfirst.frc.team3476.utility.math.Translation2d;

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
	private Rotation offset;

	private RobotTracker() {
		vehicleHistory = new CircularQueue<>(100);
		gyroHistory = new CircularQueue<>(200);
		driveBase = OrangeDrive.getInstance();
		currentOdometry = new RigidTransform(new Translation2d(), driveBase.getGyroAngle());
		offset = Rotation.fromDegrees(0);
	}

	public Rotation getGyroAngle(long time) {
		return gyroHistory.getInterpolatedKey(time);
	}

	public synchronized RigidTransform getOdometry() {
		return currentOdometry;
	}

	public synchronized void resetOdometry() {
		driveBase.zeroSensors();
		currentOdometry = new RigidTransform(new Translation2d(), Rotation.fromDegrees(0));
		oldDistance = 0;
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant curvature is assumed.
	 */
	@Override
	public void update() {
		currentDistance = driveBase.getDistance();
		Rotation deltaRotation = driveBase.getGyroAngle().inverse().rotateBy(offset);
		deltaDistance = currentDistance - oldDistance;
		Translation2d deltaPosition = new Translation2d(deltaDistance, 0);
        Rotation positionAngle = Rotation.fromRadians(deltaRotation.getRadians() / 2.0);
		synchronized (this) {			
			deltaRotation = currentOdometry.rotationMat.inverse().rotateBy(deltaRotation);
			currentOdometry = currentOdometry.transform(new RigidTransform(deltaPosition.rotateBy(positionAngle), deltaRotation));
			oldDistance = currentDistance;
			vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentOdometry));
			gyroHistory.add(new InterpolablePair<>(System.nanoTime(), driveBase.getGyroAngle()));
		}
		System.out.println("Position: " + currentOdometry.translationMat.getX() + "   " + currentOdometry.translationMat.getY());
		//System.out.println("Gyro: " + currentOdometry.rotationMat.getDegrees());
	}

	/**
	 *
	 * @param offset
	 */
	synchronized public void setRotationOffset(Rotation offset) {
		this.offset = offset;
	}
}