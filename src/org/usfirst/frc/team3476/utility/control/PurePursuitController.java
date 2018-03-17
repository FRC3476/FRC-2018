package org.usfirst.frc.team3476.utility.control;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.OrangeDrive.DriveVelocity;
import org.usfirst.frc.team3476.utility.UDP;
import org.usfirst.frc.team3476.utility.control.Path.DrivingData;
import org.usfirst.frc.team3476.utility.math.RigidTransform;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Find angle to path relative to robot 3. Drive towards point
	 */

	private Path robotPath;
	private boolean isReversed;
	private SynchronousPid turnPID;
	private RateLimiter speedProfiler;

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		//0.2, 0, 5, 0 values
		turnPID = new SynchronousPid(0.15, 0, 0.5, 0);
		turnPID.setInputRange(180, -180);
		turnPID.setOutputRange(1, -1);
		speedProfiler = new RateLimiter(120, 1000);
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
	public DriveVelocity calculate(RigidTransform robotPose) {
		if (isReversed) {
			robotPose = new RigidTransform(robotPose.translationMat, robotPose.rotationMat.flip());
		}

		/*
		 * Adaptive Lookahead
		 * double lookAheadDist = OrangeUtility.coercedNormalize(speedProfiler.getLatestValue(), Constants.MinPathSpeed,
		 * Constants.MaxPathSpeed, Constants.MinLookAheadDistance, Constants.MaxLookAheadDistance);
		 */
		// Motion Profiling
		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, Constants.LookAheadDistance);
		if(data.remainingDist == 0) {
			OrangeDrive.getInstance().setFinished();
			return new DriveVelocity(0, 0);
		}
		double robotSpeed = speedProfiler.update(data.maxSpeed, data.remainingDist);	
		if(robotSpeed < 10) {
			robotSpeed = 10;
		}
		Translation2d robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
		double angleToLookAhead = robotToLookAhead.getAngleFromOffset(new Translation2d(0, 0)).getDegrees();
		//System.out.println("turn: " + turn + " angle: " + angleToLookAhead);
		
		//double deltaSpeed =  turn * robotSpeed;
	
		double radius;
		
		radius = getRadius(robotToLookAhead);

		double delta =  (robotSpeed / radius);
		
		double deltaSpeed = Constants.TrackRadius * delta;
		//Constants.TurnScrubCoeff = deltaRotation * Constants.TrackDiameter / (deltaPos * 2)
		
		
		JSONObject message = new JSONObject();
		JSONArray pose = new JSONArray();
		JSONArray lookAhead = new JSONArray();
		JSONArray closest = new JSONArray();

		closest.add(data.closestPoint.getX());
		closest.add(data.closestPoint.getY());
		pose.add(robotPose.translationMat.getX());
		pose.add(robotPose.translationMat.getY());
		message.put("lookAhead", closest);
		message.put("pose", pose);
		UDP.getInstance().send("10.34.76.5", message.toJSONString(), 5801);
	
		if (isReversed) {
			robotSpeed *= -1;
		}
		return new DriveVelocity(robotSpeed + deltaSpeed, robotSpeed - deltaSpeed);
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
