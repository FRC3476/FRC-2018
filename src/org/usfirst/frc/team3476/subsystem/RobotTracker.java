package org.usfirst.frc.team3476.subsystem;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.usfirst.frc.team3476.utility.CircularQueue;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.UDP;
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
	private volatile Rotation rotationOffset;
	private volatile Translation2d translationOffset;

	private RobotTracker() {
		vehicleHistory = new CircularQueue<>(100);
		gyroHistory = new CircularQueue<>(200);
		driveBase = OrangeDrive.getInstance();
		currentOdometry = new RigidTransform(new Translation2d(), driveBase.getGyroAngle());
		rotationOffset = Rotation.fromDegrees(0);
		translationOffset = new Translation2d();
	}

	public Rotation getGyroAngle(long time) {
		return gyroHistory.getInterpolatedKey(time);
	}

	public synchronized RigidTransform getOdometry() {
		return currentOdometry;
	}

	public synchronized void resetOdometry() {
		driveBase.resetGyro();
		currentOdometry = new RigidTransform(new Translation2d().translateBy(translationOffset), Rotation.fromDegrees(0).rotateBy(rotationOffset));
		oldDistance = driveBase.getDistance();
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant curvature is assumed.
	 */
	@Override
	public void update() {
		currentDistance = driveBase.getDistance();
		deltaDistance = currentDistance - oldDistance;
		Translation2d deltaPosition = new Translation2d(deltaDistance, 0);
		Rotation deltaRotation = driveBase.getGyroAngle().inverse().rotateBy(rotationOffset);
		deltaRotation = currentOdometry.rotationMat.inverse().rotateBy(deltaRotation);
        Rotation halfRotation = Rotation.fromRadians(deltaRotation.getRadians() / 2.0);
        synchronized(this) {
    		currentOdometry = currentOdometry.transform(new RigidTransform(deltaPosition.rotateBy(halfRotation), deltaRotation)); 
    		vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentOdometry));
    		gyroHistory.add(new InterpolablePair<>(System.nanoTime(), driveBase.getGyroAngle()));       	
        }
		oldDistance = currentDistance;
	
		
		//System.out.println("Position: " + currentOdometry.translationMat.getX() + "   " + currentOdometry.translationMat.getY());
		//System.out.println("Gyro: " + currentOdometry.rotationMat.getDegrees());
	}

	/**
	 *
	 * @param offset
	 */
	public void setInitialRotation(Rotation offset) {
		this.rotationOffset = offset;
	}
	
	public void setInitialTranslation(Translation2d offset) {
		this.translationOffset = offset;
	}
}