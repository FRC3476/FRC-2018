package org.usfirst.frc.team3476.utility;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.OrangeDrive.DriveVelocity;
import org.usfirst.frc.team3476.utility.Path.DrivingData;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Check x offset * angle to see
	 * if past tolerance 3. Create circle 4. Follow circle path 5. Actual path
	 * looks like a spline
	 */

	private volatile Path robotPath;
	private boolean isReversed;
	private SynchronousPid turnPID;

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		turnPID = new SynchronousPid(0.01612346, 0, 0, 0);
		turnPID.setIzone(15);
		turnPID.setInputRange(180, -180);
		turnPID.setOutputRange(1, -1);
	}

	@SuppressWarnings("unchecked")
	public DriveVelocity calculate(RigidTransform robotPose) {	
		if (isReversed) {
			robotPose = new RigidTransform(robotPose.translationMat,
					robotPose.rotationMat.rotateBy(Rotation.fromDegrees(180)));
		}
		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, 20);
		
		//TODO: Slow down
		if(data.remainingDist < 1){
			return new DriveVelocity(0, 0);
		}

		Translation2d robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
		double angleToLookAhead = robotToLookAhead.getAngleFromOffset(new Translation2d(0, 0)).getDegrees();
		double deltaSpeed = turnPID.update(angleToLookAhead) * Constants.MaxTurningSpeed;
		double robotSpeed = data.maxSpeed;

		JSONObject message = new JSONObject();
		JSONArray pose = new JSONArray();
		JSONArray lookAhead = new JSONArray();
		JSONArray closest = new JSONArray();
		pose.add(robotPose.translationMat.getX());
		pose.add(robotPose.translationMat.getY());
		lookAhead.add(data.lookAheadPoint.getX());
		lookAhead.add(data.lookAheadPoint.getY());
		closest.add(data.closestPoint.getX());
		closest.add(data.closestPoint.getY());		
		message.put("pose", pose);
		message.put("lookAhead", lookAhead);
		message.put("closest", closest);
		UDP.getInstance().send("10.34.76.5", message.toJSONString(), 5801);
		
		if (isReversed) {
			robotSpeed *= -1;
			deltaSpeed *= -1;
		}
		return new DriveVelocity(robotSpeed, deltaSpeed);
	}
	
	@Deprecated
	public double getRadius(RigidTransform robotPose, Translation2d lookAheadPoint) {
		Translation2d robotToLookAheadPoint = getRobotToLookAheadPoint(robotPose, lookAheadPoint);
		//Hypotenuse^2 / (2 * X)
		double radius = Math.pow(Math.hypot(robotToLookAheadPoint.getX(), robotToLookAheadPoint.getY()), 2)	/ (2 * robotToLookAheadPoint.getX());
		return radius;
	}
	
	public Translation2d getRobotToLookAheadPoint(RigidTransform robotPose, Translation2d lookAheadPoint) {
		Translation2d lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);
		lookAheadPointToRobot = lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		return lookAheadPointToRobot;
	}
}
