package org.usfirst.frc.team3476.utility.control;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.OrangeDrive.DriveVelocity;
import org.usfirst.frc.team3476.utility.UDP;
import org.usfirst.frc.team3476.utility.control.Path.DrivingData;
import org.usfirst.frc.team3476.utility.math.RigidTransform;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import edu.wpi.first.wpilibj.Timer;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Find angle to path relative to robot 3. Drive towards point
	 */

	private Path robotPath;
	private boolean isReversed;
	private SynchronousPid turnPID;
	private RateLimiter speedProfiler;
	private double lastTime;

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		turnPID = new SynchronousPid(0.0252346, 0, 0, 0);
		turnPID.setIzone(15);
		turnPID.setInputRange(180, -180);
		turnPID.setOutputRange(1, -1);
		speedProfiler = new RateLimiter(40, 120);
		lastTime = Timer.getFPGATimestamp();
	}

	/**
	 * Calculates the look ahead and the desired speed for each side of the robot.
	 * 
	 * @param robotPose
	 * 			Robot position and gyro angle.
	 * @return
	 * 			Speed for each side of the robot.
	 * 			
	 */
	@SuppressWarnings("unchecked")
	public DriveVelocity calculate(RigidTransform robotPose) {
		if (isReversed) {
			robotPose = new RigidTransform(robotPose.translationMat, robotPose.rotationMat.flip());
		}
		
		/*
		Adaptive Lookahead
		double lookAheadDist = OrangeUtility.coercedNormalize(speedProfiler.getLatestValue(), Constants.MinPathSpeed,
				Constants.MaxPathSpeed, Constants.MinLookAheadDistance, Constants.MaxLookAheadDistance);
		*/
		//Motion Profiling
		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, 20);
		double robotSpeed = getProfiledSpeed(data.maxSpeed, data.remainingDist);
		
		Translation2d robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
		double angleToLookAhead = robotToLookAhead.getAngleFromOffset(new Translation2d(0, 0)).getDegrees();
		double deltaSpeed = turnPID.update(angleToLookAhead) * robotSpeed;

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
		}
		return new DriveVelocity(robotSpeed + deltaSpeed, robotSpeed - deltaSpeed);
	}

	@Deprecated
	public double getRadius(RigidTransform robotPose, Translation2d lookAheadPoint) {
		Translation2d robotToLookAheadPoint = getRobotToLookAheadPoint(robotPose, lookAheadPoint);
		// Hypotenuse^2 / (2 * X)
		double radius = Math.pow(Math.hypot(robotToLookAheadPoint.getX(), robotToLookAheadPoint.getY()), 2)
				/ (2 * robotToLookAheadPoint.getX());
		return radius;
	}

	private Translation2d getRobotToLookAheadPoint(RigidTransform robotPose, Translation2d lookAheadPoint) {
		Translation2d lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);
		lookAheadPointToRobot = lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		return lookAheadPointToRobot;
	}
	
	/**
	 * Returns the speed after it's been passed through the motion profiler. It slows down the robot once decelerating will make it stop approximately at the end.
	 * 
	 * @param setpoint
	 * 			Maximum speed possible for the robot
	 * @param remainingDist
	 * 			The distance on the path remaining for the robot.
	 * @return
	 * 			Profiled speed for the robot.
	 */
	private double getProfiledSpeed(double setpoint, double remainingDist){
		double timeToSwitchAcc = (speedProfiler.getAcc() / speedProfiler.getMaxJerk())
				+ (speedProfiler.getMaxAccel() / speedProfiler.getMaxJerk());
		double timeToDecel = speedProfiler.getLatestValue() / speedProfiler.getMaxAccel();
		double distanceTillStop = (timeToSwitchAcc + timeToDecel) * speedProfiler.getLatestValue();
		double dt = Timer.getFPGATimestamp() - lastTime;
		dt = Math.min(20, dt);
		lastTime = System.currentTimeMillis();
		if (distanceTillStop > remainingDist) {
			return speedProfiler.update(0, dt);
		} else {
			return speedProfiler.update(setpoint, dt);
		}		
	}
	
	/**
	 * Resets the time for the speed profiler.
	 */
	public void resetTime(){
		//TODO: Big Bang
		lastTime = System.currentTimeMillis();
	}
}
