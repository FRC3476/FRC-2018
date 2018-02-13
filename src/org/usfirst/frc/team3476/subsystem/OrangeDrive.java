package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.control.PurePursuitController;
import org.usfirst.frc.team3476.utility.control.RateLimiter;
import org.usfirst.frc.team3476.utility.math.Rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class OrangeDrive extends Threaded {
	public enum DriveState {
		TELEOP, AUTO
	}

	public static class DriveVelocity {
		/*
		 * Inches per second for speed
		 */
		public double rightWheelSpeed;
		public double leftWheelSpeed;

		public DriveVelocity(double left, double right) {
			this.rightWheelSpeed = left;
			this.leftWheelSpeed = right;
		}
	}

	private static final OrangeDrive instance = new OrangeDrive();

	public static OrangeDrive getInstance() {
		return instance;
	}

	private double quickStopAccumulator;
	private double lastTime;

	private boolean drivePercentVbus;

	private ADXRS450_Gyro gyroSensor = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon, rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;

	private DriveVelocity autoDriveVelocity;
	private DriveState driveState;
	private RateLimiter leftProfiler, rightProfiler;

	private OrangeDrive() {
		leftTalon = new LazyTalonSRX(Constants.LeftMasterDriveId);
		rightTalon = new LazyTalonSRX(Constants.RightMasterDriveId);

		/*
		 * leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		 * rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		 */

		leftSlaveTalon = new LazyTalonSRX(Constants.LeftSlaveDriveId);
		leftSlave2Talon = new LazyTalonSRX(Constants.LeftSlave2DriveId);
		rightSlaveTalon = new LazyTalonSRX(Constants.RightSlaveDriveId);
		rightSlave2Talon = new LazyTalonSRX(Constants.RightSlave2DriveId);
		leftTalon.setInverted(false);
		rightTalon.setInverted(false);
		leftSlaveTalon.setInverted(true);
		leftSlave2Talon.setInverted(true);
		rightSlaveTalon.setInverted(true);
		rightSlave2Talon.setInverted(true);

		leftTalon.setSensorPhase(true);
		rightTalon.setSensorPhase(false);

		leftSlaveTalon.set(ControlMode.Follower, leftTalon.getDeviceID());
		leftSlave2Talon.set(ControlMode.Follower, leftTalon.getDeviceID());
		rightSlaveTalon.set(ControlMode.Follower, rightTalon.getDeviceID());
		rightSlave2Talon.set(ControlMode.Follower, rightTalon.getDeviceID());

		// TODO: Find constants of new drivebase
		drivePercentVbus = true;
		driveState = DriveState.TELEOP;

		rightTalon.config_kP(0, 0.2, 10);
		rightTalon.config_kF(0, 0.1453, 10);
		leftTalon.config_kP(0, 0.2, 10);
		leftTalon.config_kF(0, 0.1453, 10);
		leftProfiler = new RateLimiter(Constants.TeleopAccLimit);
		rightProfiler = new RateLimiter(Constants.TeleopAccLimit);
	}

	public synchronized void arcadeDrive(double moveValue, double rotateValue) {
		driveState = DriveState.TELEOP;
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);

		// Get highest correct speed for left/right wheels
		// Positive rotateValue turns right
		if (drivePercentVbus) {
			leftMotorSpeed = OrangeUtility.coerce(moveValue + rotateValue, 1, -1);
			rightMotorSpeed = OrangeUtility.coerce(moveValue - rotateValue, 1, -1);
			setWheelPower(new DriveVelocity(leftMotorSpeed, rightMotorSpeed));
		} else {
			moveValue *= Constants.MaxDriveSpeed;
			rotateValue *= Constants.MaxDriveSpeed;

			leftMotorSpeed = OrangeUtility.coerce(moveValue
					+ rotateValue, Constants.MaxDriveSpeed, -Constants.MaxDriveSpeed);
			rightMotorSpeed = OrangeUtility.coerce(moveValue
					- rotateValue, Constants.MaxDriveSpeed, -Constants.MaxDriveSpeed);

			double now = Timer.getFPGATimestamp();
			double dt = (now - lastTime);
			leftMotorSpeed = leftProfiler.update(leftMotorSpeed, dt);
			rightMotorSpeed = rightProfiler.update(rightMotorSpeed, dt);
			lastTime = now;

			setWheelVelocity(new DriveVelocity(leftMotorSpeed, rightMotorSpeed));
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
		// TODO: make pretty - coerce
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
		leftMotorSpeed *= Constants.MaxDriveSpeed;
		rightMotorSpeed *= Constants.MaxDriveSpeed;

		setWheelVelocity(new DriveVelocity(leftMotorSpeed, rightMotorSpeed));
	}

	public void resetMotionProfile() {
		lastTime = Timer.getFPGATimestamp();
		leftProfiler.reset();
		rightProfiler.reset();
	}

	public double getAngle() {
		return gyroSensor.getAngle();
	}

	public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	// TODO: rename
	public Rotation getGyroAngle() {
		// -180 through 180
		return Rotation.fromDegrees(gyroSensor.getAngle());
	}

	// TODO: Constant for 1024
	public double getLeftDistance() {
		return leftTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerRev * Constants.WheelDiameter * Math.PI;
	}

	public double getRightDistance() {
		return rightTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerRev * Constants.WheelDiameter
				* Math.PI;
	}

	public double getSpeed() {
		return ((leftTalon.getSelectedSensorVelocity(0) + rightTalon.getSelectedSensorVelocity(0))
				/ Constants.SensorTicksPerRev) / 60 / 2 * Constants.WheelDiameter * Math.PI;
	}

	public void resetGyro() {
		gyroSensor.reset();
	}

	public double scaleJoystickValues(double rawValue) {
		return Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinimumControllerInput, Constants.MaximumControllerInput, Constants.MinimumControllerOutput, Constants.MaximumControllerOutput), rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.AUTO;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		updateAutoPath();
	}

	public void setBrakeState(NeutralMode mode) {
		leftTalon.setNeutralMode(mode);
		rightTalon.setNeutralMode(mode);
		leftSlaveTalon.setNeutralMode(mode);
		rightSlaveTalon.setNeutralMode(mode);
	}

	private void setWheelPower(DriveVelocity setVelocity) {
		leftTalon.set(ControlMode.PercentOutput, setVelocity.leftWheelSpeed);
		rightTalon.set(ControlMode.PercentOutput, -(setVelocity.rightWheelSpeed));
	}

	private void setWheelVelocity(DriveVelocity setVelocity) {
		// inches per sec to rotations per min
		if (Math.abs(setVelocity.leftWheelSpeed) > Constants.MaxDriveSpeed
				|| Math.abs(setVelocity.rightWheelSpeed) > Constants.MaxDriveSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.MaxDriveSpeed + " !", false);
			return;
		}
		// in/s -> (in / pi) * 15
		// positive deltaSpeed turns right by making left wheels faster than right
		leftTalon.set(ControlMode.Velocity, (setVelocity.leftWheelSpeed) / Math.PI * 15);
		rightTalon.set(ControlMode.Velocity, (setVelocity.rightWheelSpeed) / Math.PI * 15);
	}

	public synchronized void setSimpleDrive(boolean setting) {
		drivePercentVbus = setting;
	}

	@Override
	public synchronized void update() {
		if (driveState == DriveState.TELEOP) {

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
		leftTalon.setSelectedSensorPosition(0, 0, 10);
		rightTalon.setSelectedSensorPosition(0, 0, 10);
	}

	public boolean checkSubsystem() { 
		// TODO: Get accurate thresholds
		// TODO: Use PDP to get current
		boolean success = true;
		success = checkMotor(leftTalon, leftTalon, 0.25);
		success = checkMotor(leftSlaveTalon, leftTalon, 0.25);
		success = checkMotor(leftSlave2Talon, leftTalon, 0.25);
		success = checkMotor(rightTalon, rightTalon, 0.25);
		success = checkMotor(rightSlaveTalon, rightTalon, 0.25);
		success = checkMotor(rightSlave2Talon, rightTalon, 0.25);
		return success;
	}
	
	public boolean checkMotor(LazyTalonSRX checkedTalon, LazyTalonSRX sensorTalon, double output) {
		checkedTalon.set(ControlMode.PercentOutput, output);
		Timer.delay(0.5);
		boolean success = true;
		if (Math.abs(checkedTalon.getOutputCurrent() - 0.1) > .1) {
			DriverStation.reportError("Motor leftSlaveTalon current outside expected range.", false);
			success = false;
		} 
		if (Math.abs(sensorTalon.getSelectedSensorVelocity(0) - 10) > 10) {
			DriverStation.reportError("Motor leftSlaveTalon current outside expected range.", false);
			success = false;
		}
		if (Math.abs(sensorTalon.getSelectedSensorPosition(0) - 10) > 10) {
			DriverStation.reportError("Motor leftSlaveTalon current outside expected range.", false);
			success = false;
		}
		checkedTalon.set(ControlMode.PercentOutput, 0);
		return success;
	}
}
