package org.usfirst.frc.team3476.utility;

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
		System.out.println(robotPose.translationMat.getX() + "  " + robotPose.translationMat.getY());
		double angleToLookAhead = robotToLookAhead.getAngleFromOffset(new Translation2d(0, 0)).getDegrees();
		double deltaSpeed = turnPID.update(angleToLookAhead) * Constants.MaxTurningSpeed;
		double robotSpeed = data.maxSpeed;
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
