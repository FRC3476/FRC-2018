package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.math.Rotation;

public class Superstructure extends Threaded {

	public enum SuperstructureState {
		HOMING, MANUAL
	}

	private SuperstructureState superstructureState;

	private Arm arm;
	private Elevator elevator;

	private Superstructure() {
		arm = Arm.getInstance();
		elevator = Elevator.getInstance();

	}

	@Override
	public void update() {
		switch (superstructureState) {
		case HOMING:
			// set arm angle
			// move down
			break;
		case MANUAL:
			// do some rate limiting or somethang idk
			break;
		}
	}

	// TODO: change to using a setpoint and using ratelimiter
	public void setElevatorHeight(double height) {
		elevator.setHeight(height);
	}

	// TODO: change to using a setpoint and using ratelimiter
	public void setArmAngle(Rotation angle) {
		// arm.setAngle(angle);
	}

	public void setOverallPosition(double distance, double height) {
		if (distance > Constants.ArmLength || height > Constants.ElevatorHeight) {
			System.out.println("Position not reachable by elevator.");
			return;
		}

		double armAngle = Math.asin(distance / Constants.ArmLength);
		double elevatorPosition = height - Math.sqrt(Constants.ArmLength * Constants.ArmLength - distance * distance);

		setElevatorHeight(elevatorPosition);
		setArmAngle(Rotation.fromRadians(armAngle));
	}

}
