package org.usfirst.frc.team3476.utility.control.motion;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.OrangeDrive.AutoDriveSignal;
import org.usfirst.frc.team3476.subsystem.OrangeDrive.DriveSignal;
import org.usfirst.frc.team3476.utility.OrangeUtility;
import org.usfirst.frc.team3476.utility.UDP;
import org.usfirst.frc.team3476.utility.control.RateLimiter;
import org.usfirst.frc.team3476.utility.control.SynchronousPid;
import org.usfirst.frc.team3476.utility.control.motion.Path.DrivingData;
import org.usfirst.frc.team3476.utility.math.RigidTransform;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Find angle to path relative to robot 3. Drive towards point
	 */

	private Path robotPath;
	private boolean isReversed, turnToHeading;
	private SynchronousPid turnPID;
	private RateLimiter speedProfiler;

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		this.turnToHeading = false;
		turnPID = new SynchronousPid(0.2, 0, 5, 0);
		turnPID.setInputRange(180, -180);
		turnPID.setOutputRange(Constants.HighDriveSpeed, -Constants.HighDriveSpeed);
		turnPID.setTolerance(1);
		speedProfiler = new RateLimiter(100, 1000);
		if(robotPath.isEmpty()){
			
		}
		
	}

	/**
	 * Calculates the look ahead and the desired speed for each side of the robot.
	 *
	 * @param robotPose
	 *            Robot position and gyro angle.
	 * @return
	 * 		Speed for each side of the robot.
	 *
	 */
	@SuppressWarnings("unchecked")
	public synchronized AutoDriveSignal calculate(RigidTransform robotPose) {
		if (isReversed) {
			robotPose = new RigidTransform(robotPose.translationMat, robotPose.rotationMat.flip());
		}
		double lookAheadDist = OrangeUtility.coercedNormalize(speedProfiler.getLatestValue(), Constants.MinPathSpeed,
		Constants.MaxPathSpeed, Constants.MinLookAheadDistance, Constants.MaxLookAheadDistance);
		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, lookAheadDist);
		
		if(data.remainingDist == 0.0) { //If robot passes point, remaining distance is 0
			return new AutoDriveSignal(new DriveSignal(0, 0), true);
		}
		double robotSpeed = speedProfiler.update(data.maxSpeed, data.remainingDist);
		if (robotSpeed < 20) {
			robotSpeed = 20;
		}
		Translation2d robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
		/*
		 * TEST
		 * Translation2d closestToEnd = data.currentSegEnd.translateBy(data.closestPoint.inverse());
		 * double angleToPath = robotPose.rotationMat.inverse().rotateBy(closestToEnd.getAngleFromOffset(new
		 * Translation2d(0, 0))).getDegrees();
		 * if(Math.abs(angleToPath) > 135) {
		 * turnToHeading = true;
		 * }
		 * if(turnToHeading) {
		 * double angleToLookAhead = robotToLookAhead.getAngleFromOffset(new Translation2d(0, 0)).getDegrees();
		 * double deltaSpeed = turnPID.update(angleToLookAhead);
		 * if(turnPID.isDone()) {
		 * turnToHeading = false;
		 * } else {
		 * return new DriveVelocity(deltaSpeed, deltaSpeed);
		 * }
		 * }
		 */

		double radius;
		radius = getRadius(robotToLookAhead);
		double delta = (robotSpeed / radius);
		double deltaSpeed = Constants.TrackRadius * delta;

		JSONObject message = new JSONObject();
		JSONArray pose = new JSONArray();
		JSONArray lookAhead = new JSONArray();
		JSONArray closest = new JSONArray();

		if (Constants.LOGGING) {
			closest.add(data.closestPoint.getX());
			closest.add(data.closestPoint.getY());
			lookAhead.add(data.lookAheadPoint.getX());
			lookAhead.add(data.lookAheadPoint.getY());
			pose.add(robotPose.translationMat.getX());
			pose.add(robotPose.translationMat.getY());
			message.put("closest", closest);
			message.put("lookAhead", lookAhead);
			message.put("pose", pose);
			UDP.getInstance().send("10.34.76.5", message.toJSONString(), 5801);
		}

		if (isReversed) {
			robotSpeed *= -1;
		}
		double maxSpeed = Math.abs(robotSpeed) + Math.abs(deltaSpeed);
		if(maxSpeed > Constants.MaxPathSpeed) {
			robotSpeed -= Math.copySign(maxSpeed - Constants.MaxPathSpeed, robotSpeed);
		}
		return new AutoDriveSignal(new DriveSignal(robotSpeed + deltaSpeed, robotSpeed - deltaSpeed), false);
	}

	private double getRadius(Translation2d robotToLookAheadPoint) {
		// Hypotenuse^2 / (2 * X)
		double radius = Math.pow(Math.hypot(robotToLookAheadPoint.getX(), robotToLookAheadPoint.getY()), 2)
				/ (2 * robotToLookAheadPoint.getY());
		return radius;
	}

	private Translation2d getRobotToLookAheadPoint(RigidTransform robotPose, Translation2d lookAheadPoint) {
		Translation2d lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);
		lookAheadPointToRobot = lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		return lookAheadPointToRobot;
		
	}

	/**
	 * Resets the time for the speed profiler.
	 */
	public void resetTime() {
		// TODO: Big Bang
		speedProfiler.reset();
	}
	
}
