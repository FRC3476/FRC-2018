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
	private volatile Rotation offset;

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
		driveBase.resetGyro();
		currentOdometry = new RigidTransform(new Translation2d(), Rotation.fromDegrees(0));
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
		Rotation deltaRotation = driveBase.getGyroAngle().inverse().rotateBy(offset);
		deltaRotation = currentOdometry.rotationMat.inverse().rotateBy(deltaRotation);
        Rotation halfRotation = Rotation.fromRadians(deltaRotation.getRadians() / 2.0);
        synchronized(this) {
    		currentOdometry = currentOdometry.transform(new RigidTransform(deltaPosition.rotateBy(halfRotation), deltaRotation)); 
    		vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentOdometry));
    		gyroHistory.add(new InterpolablePair<>(System.nanoTime(), driveBase.getGyroAngle()));       	
        }
		oldDistance = currentDistance;
		JSONObject message = new JSONObject();
		JSONArray pose = new JSONArray();
		JSONArray lookAhead = new JSONArray();
		JSONArray closest = new JSONArray();

		lookAhead.add(0);
		lookAhead.add(0);
		pose.add(currentOdometry.translationMat.getX());
		pose.add(currentOdometry.translationMat.getY());
		message.put("lookAhead", lookAhead);
		message.put("pose", pose);
		UDP.getInstance().send("10.34.76.5", message.toJSONString(), 5801);
		
		//System.out.println("Position: " + currentOdometry.translationMat.getX() + "   " + currentOdometry.translationMat.getY());
		//System.out.println("Gyro: " + currentOdometry.rotationMat.getDegrees());
	}

	/**
	 *
	 * @param offset
	 */
	public void setRotationOffset(Rotation offset) {
		this.offset = offset;
	}
}