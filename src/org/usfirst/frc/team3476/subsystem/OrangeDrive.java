package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.Path;
import org.usfirst.frc.team3476.utility.PurePursuitController;
import org.usfirst.frc.team3476.utility.Rotation;
import org.usfirst.frc.team3476.utility.Threaded;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/* Inspiration from Team 254 */

public class OrangeDrive extends Threaded {	
	public enum DriveState {
		TELEOP, AUTO
	}

	public static class DriveVelocity {
		/*
		 * Inches per second for speed
		 */
		public double wheelSpeed;
		public double deltaSpeed;

		public DriveVelocity(double wheelSpeed, double deltaSpeed) {
			this.wheelSpeed = wheelSpeed;
			this.deltaSpeed = deltaSpeed;
		}
	}


	private static final OrangeDrive instance = new OrangeDrive();

	public static OrangeDrive getInstance() {
		return OrangeDrive.instance;
	}

	private double quickStopAccumulator;
	private double lastTime;
	private double lastValue;

	private double driveMultiplier;
	private boolean drivePercentVbus;

	private ADXRS450_Gyro gyroSensor = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	private CANTalon leftTalon, rightTalon, leftSlaveTalon, rightSlaveTalon;
	private PurePursuitController autonomousDriver;

	private DriveVelocity autoDriveVelocity;
	private DriveState driveState;

	private OrangeDrive() {
		leftTalon = new CANTalon(Constants.LeftMasterDriveId);
		rightTalon = new CANTalon(Constants.RightMasterDriveId);

		leftTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);

		// Quadrature updates at 100ms

		leftTalon.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
		rightTalon.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);

		leftTalon.configEncoderCodesPerRev(1024);
		rightTalon.configEncoderCodesPerRev(1024);

		leftTalon.reverseOutput(false);
		leftTalon.reverseSensor(true);
		rightTalon.reverseOutput(true);
		rightTalon.reverseSensor(false);

		leftSlaveTalon = new CANTalon(Constants.LeftSlaveDriveId);
		rightSlaveTalon = new CANTalon(Constants.RightSlaveDriveId);

		leftSlaveTalon.changeControlMode(TalonControlMode.Follower);
		leftSlaveTalon.set(leftTalon.getDeviceID());
		rightSlaveTalon.changeControlMode(TalonControlMode.Follower);
		rightSlaveTalon.set(rightTalon.getDeviceID());

		leftTalon.changeControlMode(TalonControlMode.Speed);
		rightTalon.changeControlMode(TalonControlMode.Speed);
		
		//TODO: Find constants of new drivebase
		driveMultiplier = 0;
		drivePercentVbus = true;
		driveState = DriveState.TELEOP;
	}

	public synchronized void arcadeDrive(double moveValue, double rotateValue) {
		driveState = DriveState.TELEOP;
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		if (moveValue >= 0.0) {
			moveValue = moveValue * moveValue;
		} else {
			moveValue = -(moveValue * moveValue);
		}
		if (rotateValue >= 0.0) {
			rotateValue = rotateValue * rotateValue;
		} else {
			rotateValue = -(rotateValue * rotateValue);
		}
		// Get highest correct speed for left/right wheels
		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				leftMotorSpeed = -Math.max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			} else {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		if (drivePercentVbus) {
			double moveSpeed = (leftMotorSpeed + rightMotorSpeed) / 2;
			double turnSpeed = (leftMotorSpeed - rightMotorSpeed) / 2;
			setWheelPower(new DriveVelocity(moveSpeed, turnSpeed));
		} else {

			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;

			// get acceleration
			// assumes that wheel speed pid works
			// works by limiting how much higher/lower we can set the speed
			double now = Timer.getFPGATimestamp();
			double dt = (now - lastTime);

			double moveSpeed = (leftMotorSpeed + rightMotorSpeed) / 2;
			double turnSpeed = (leftMotorSpeed - rightMotorSpeed) / 2;

			double accel = (moveSpeed - lastValue) / dt;
			if (accel < -Constants.MaxAcceleration) {
				moveSpeed = lastValue - Constants.MaxAcceleration * dt;
			} else if (accel > Constants.MaxAcceleration) {
				moveSpeed = lastValue + Constants.MaxAcceleration * dt;
			}

			lastTime = now;
			lastValue = moveSpeed;
			setWheelVelocity(new DriveVelocity(moveSpeed, turnSpeed));
		}
	}

	public void calibrateGyro() {
		gyroSensor.calibrate();
	}

	public void cheesyDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		driveState = DriveState.TELEOP;
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);

		double leftMotorSpeed;
		double rightMotorSpeed;
		double angularPower = 1;

		double overPower;

		if (isQuickTurn) {
			overPower = 1;
			if (moveValue < 0.2) {
				quickStopAccumulator = quickStopAccumulator + rotateValue * 2;
			}
			angularPower = rotateValue;
		} else {
			overPower = 0;
			angularPower = Math.abs(moveValue) * rotateValue - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0;
			}
		}

		leftMotorSpeed = moveValue - angularPower;
		rightMotorSpeed = moveValue + angularPower;

		angularPower = Math.abs(moveValue) * rotateValue - quickStopAccumulator;

		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= overPower * (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= overPower * (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += overPower * (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += overPower * (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}
		leftMotorSpeed *= driveMultiplier;
		rightMotorSpeed *= driveMultiplier;

		setWheelVelocity(
				new DriveVelocity((leftMotorSpeed + rightMotorSpeed) / 2, (leftMotorSpeed - rightMotorSpeed) / 2));
	}

	public double getAngle() {
		return gyroSensor.getAngle();
	}

	public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	public Rotation getGyroAngle() {
		// -180 through 180
		return Rotation.fromDegrees(gyroSensor.getAngle());
	}

	public double getLeftDistance() {
		return leftTalon.getPosition() * Constants.WheelDiameter * Math.PI;
	}

	public double getRightDistance() {
		return rightTalon.getPosition() * Constants.WheelDiameter * Math.PI;
	}

	public double getSpeed() {
		return ((leftTalon.getSpeed() + rightTalon.getSpeed()) / 120) * Constants.WheelDiameter * Math.PI;
	}

	public void resetGyro() {
		gyroSensor.reset();
	}
	
	public double scaleJoystickValues(double rawValue) {
		return scaleValues(rawValue, Constants.MinimumControllerInput, Constants.MaximumControllerInput,
				Constants.MinimumControllerOutput, Constants.MaximumControllerOutput);
	}

	public double scaleValues(double rawValue, double minInput, double maxInput, double minOutput, double maxOutput) {
		// scales ranges IE. 0.15 - 1 to 0 - 1
		// the absolute value of the rawValue under minimum are treated as 0
		// negative values also work
		// values higher than maxInput returns maxOutput
		if (Math.abs(rawValue) >= minInput) {
			double norm = (rawValue - minInput) / (maxInput - minInput);
			norm = norm * (maxOutput - minOutput) + ((norm * minOutput) / Math.abs(norm));
			return norm;
		} else {
			return 0;
		}
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.AUTO;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		updateAutoPath();
	}

	public void setBrakeState(boolean isBraked) {
		leftTalon.enableBrakeMode(isBraked);
		rightTalon.enableBrakeMode(isBraked);
		leftSlaveTalon.enableBrakeMode(isBraked);
		rightSlaveTalon.enableBrakeMode(isBraked);
	}
	
	public void setInvert() {
		driveMultiplier = -1;
	}

	public void setNormal() {
		driveMultiplier = 1;
	}

	private void setWheelPower(DriveVelocity setVelocity) {
		leftTalon.changeControlMode(TalonControlMode.PercentVbus);
		rightTalon.changeControlMode(TalonControlMode.PercentVbus);
		leftTalon.set(setVelocity.wheelSpeed + setVelocity.deltaSpeed);
		// power is reversed for right side
		rightTalon.set(-(setVelocity.wheelSpeed - setVelocity.deltaSpeed));
	}

	private void setWheelVelocity(DriveVelocity setVelocity) {
		leftTalon.changeControlMode(TalonControlMode.Speed);
		rightTalon.changeControlMode(TalonControlMode.Speed);
		// inches per sec to rotations per min
		if (setVelocity.wheelSpeed > 216) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over 216!", false);
			return;
		}
		// in/s -> (in / pi) * 15
		leftTalon.setSetpoint((setVelocity.wheelSpeed + setVelocity.deltaSpeed) / Math.PI * 15);
		rightTalon.setSetpoint((setVelocity.wheelSpeed - setVelocity.deltaSpeed) / Math.PI * 15);

	}

	public synchronized void setSimpleDrive(boolean setting) {
		drivePercentVbus = setting;
	}

	@Override
	public synchronized void update() {
		if(driveState == DriveState.TELEOP) {
			
		} else {
			updateAutoPath();
		}
	}

	private synchronized void updateAutoPath() {
		autoDriveVelocity = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
		setWheelVelocity(autoDriveVelocity);
	}
	
	public void zeroSensors() {
		gyroSensor.reset();
		leftTalon.setPosition(0);
		rightTalon.setPosition(0);
	}
}
