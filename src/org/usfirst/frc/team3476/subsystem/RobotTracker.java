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
	private Rotation rotationOffset;
	private Translation2d translationOffset;

	private RobotTracker() {
		vehicleHistory = new CircularQueue<>(100);
		gyroHistory = new CircularQueue<>(200);
		driveBase = OrangeDrive.getInstance();
		currentOdometry = new RigidTransform(new Translation2d(), driveBase.getGyroAngle());
		rotationOffset = Rotation.fromDegrees(0);
		translationOffset = new Translation2d();
	}

	synchronized public Rotation getGyroAngle(long time) {
		return gyroHistory.getInterpolatedKey(time);
	}

	synchronized public RigidTransform getOdometry() {
		return currentOdometry;
	}

	synchronized public void resetOdometry() {
		driveBase.resetGyro();
		currentOdometry = new RigidTransform(new Translation2d().translateBy(translationOffset), Rotation.fromDegrees(0).rotateBy(rotationOffset));
		oldDistance = driveBase.getDistance();
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant curvature is assumed
	 */
	@Override
	public void update() {
		double leftDist = driveBase.getLeftDistance();
		double rightDist = driveBase.getRightDistance();
		/*
		 * Solve problem where Talon returns 0 for distance due to an error
		 * This causes an abnormal deltaPosition
		 */
		if(leftDist != 0 && rightDist != 0) {
			currentDistance = (leftDist + rightDist) / 2;
		} else {
			return;
		}
		deltaDistance = currentDistance - oldDistance;
		Translation2d deltaPosition = new Translation2d(deltaDistance, 0);
		Rotation deltaRotation = driveBase.getGyroAngle().inverse().rotateBy(rotationOffset);
		synchronized (this) {
			deltaRotation = currentOdometry.rotationMat.inverse().rotateBy(deltaRotation);
			Rotation halfRotation = Rotation.fromRadians(deltaRotation.getRadians() / 2.0);
			currentOdometry = currentOdometry.transform(new RigidTransform(deltaPosition.rotateBy(halfRotation), deltaRotation));
			vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentOdometry));
			gyroHistory.add(new InterpolablePair<>(System.nanoTime(), driveBase.getGyroAngle()));
		}
		oldDistance = currentDistance;

		// System.out.println("Position: " + currentOdometry.translationMat.getX() + " " +
		// currentOdometry.translationMat.getY());
		// System.out.println("Gyro: " + currentOdometry.rotationMat.getDegrees());
	}

	/**
	 *
	 * @param offset
	 */
	synchronized public void setInitialRotation(Rotation offset) {
		this.rotationOffset = offset;
	}

	synchronized public void setInitialTranslation(Translation2d offset) {
		this.translationOffset = offset;
		resetOdometry();
	}
}