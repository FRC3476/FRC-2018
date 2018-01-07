package org.usfirst.frc.team3476.utility;

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

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
	}

	public DriveVelocity calculate(RigidTransform robotPose) {

		if (isReversed) {
			robotPose = new RigidTransform(robotPose.translationMat,
					robotPose.rotationMat.rotateBy(Rotation.fromDegrees(180)));
		}

		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat,20); 
		if(data.remainingDist < 5){
			return new DriveVelocity(0, 0);
		}
		double radius = getRadius(robotPose, data.lookAheadPoint);
		double robotSpeed = data.maxSpeed;

		if (isReversed) {
			robotSpeed *= -1;
		}
		
		if (radius != 0) {
			return new DriveVelocity(robotSpeed,
					23 * (robotSpeed / radius) / (2));
		} else {
			return new DriveVelocity(robotSpeed, 0);
		}
	}

	public double getRadius(RigidTransform robotPose, Translation2d lookAheadPoint) {
		Translation2d lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);	
		lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		double radius = Math.pow(Math.hypot(lookAheadPointToRobot.getX(), lookAheadPointToRobot.getY()), 2)	/ (2 * -lookAheadPointToRobot.getX());
		return radius;
	}
}
