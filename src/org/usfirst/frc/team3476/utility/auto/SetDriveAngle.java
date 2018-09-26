package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.math.Rotation;
import org.usfirst.frc.team3476.utility.math.Translation2d;

public class SetDriveAngle extends AutoCommand {

	private Rotation angle;
	private Translation2d point;

	public SetDriveAngle(Rotation angle) {
		this.angle = angle;
		this.setBlocking(true);
	}

	public SetDriveAngle(Translation2d point) {
		this.point = point;
		this.setBlocking(true);
	}

	@Override
	public void start() {
		if (angle == null) {
			angle = RobotTracker.getInstance().getOdometry().translationMat.getAngle(point);
		}
		OrangeDrive.getInstance().setRotation(angle);
		System.out.println("Angle: " + angle.getDegrees());
	}

	@Override
	public boolean isFinished() {
		return OrangeDrive.getInstance().isFinished();
	}

}
