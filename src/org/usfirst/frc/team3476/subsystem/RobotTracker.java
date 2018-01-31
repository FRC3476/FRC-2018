package org.usfirst.frc.team3476.subsystem;

import org.json.simple.JSONObject;
import org.usfirst.frc.team3476.utility.CircularQueue;
import org.usfirst.frc.team3476.utility.RigidTransform;
import org.usfirst.frc.team3476.utility.Rotation;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.Translation2d;
import org.usfirst.frc.team3476.utility.UDP;

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
		driveBase.zeroSensors();
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
		currentOdometry = new RigidTransform(new Translation2d(), driveBase.getGyroAngle());
		oldDistance = 0;
	}

	/**
	 * Integrates the encoders and gyro to figure out robot position. We don't calculate a circular path.
	 */
	@Override
	public void update() {
		currentDistance = (driveBase.getLeftDistance() + driveBase.getRightDistance()) / 2;
		Rotation deltaRotation = driveBase.getGyroAngle().inverse().rotateBy(offset);
		deltaDistance = currentDistance - oldDistance;		
		Translation2d deltaPosition = new Translation2d(deltaRotation.cos() * deltaDistance, deltaRotation.sin() * deltaDistance);
		synchronized(this){
			//currentOdometry = currentOdometry.transform(new RigidTransform(deltaPosition, deltaRotation));
			currentOdometry = new RigidTransform(currentOdometry.translationMat.translateBy(deltaPosition), deltaRotation);
			oldDistance = currentDistance;
			//vehicleHistory.add(new InterpolableValue<>(System.nanoTime(), currentOdometry));
			//gyroHistory.add(new InterpolableValue<>(System.nanoTime(), driveBase.getGyroAngle()));
		}
	}
	
	/**
	 * 
	 * @param offset
	 */
	synchronized public void setRotationOffset(Rotation offset){
		this.offset = offset;
	}
}