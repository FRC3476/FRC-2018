package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.OrangeUtility;
import org.usfirst.frc.team3476.utility.Path;
import org.usfirst.frc.team3476.utility.PurePursuitController;
import org.usfirst.frc.team3476.utility.RateLimiter;
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
	private CANTalon leftTalon, rightTalon, leftSlaveTalon, rightSlaveTalon;
	private PurePursuitController autonomousDriver;

	private DriveVelocity autoDriveVelocity;
	private DriveState driveState;
	private RateLimiter leftProfiler, rightProfiler;
	
	private OrangeDrive() {
		leftTalon = new CANTalon(Constants.LeftMasterDriveId);
		rightTalon = new CANTalon(Constants.RightMasterDriveId);

		leftTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		rightTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);

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
		drivePercentVbus = false;
		driveState = DriveState.TELEOP;

		rightTalon.setP(0.2); // 0.45 on practice
		rightTalon.setF(0.1453);
		leftTalon.setP(0.2);
		leftTalon.setF(0.1453);
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
		leftMotorSpeed = OrangeUtility.coerce(moveValue + rotateValue, 1, -1);
		rightMotorSpeed = OrangeUtility.coerce(moveValue - rotateValue, 1, -1);
		if (drivePercentVbus) {
			setWheelPower(new DriveVelocity(leftMotorSpeed, rightMotorSpeed));
		} else {
			moveValue *= Constants.MaxDriveSpeed;
			rotateValue *= Constants.MaxDriveSpeed;		
			
			leftMotorSpeed = OrangeUtility.coerce(moveValue + rotateValue, Constants.MaxDriveSpeed, -Constants.MaxDriveSpeed);
			rightMotorSpeed = OrangeUtility.coerce(moveValue - rotateValue, Constants.MaxDriveSpeed, -Constants.MaxDriveSpeed);

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

	public void resetMotionProfile(){
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
		return Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinimumControllerInput, Constants.MaximumControllerInput,
				Constants.MinimumControllerOutput, Constants.MaximumControllerOutput), rawValue);
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

	private void setWheelPower(DriveVelocity setVelocity) {
		leftTalon.changeControlMode(TalonControlMode.PercentVbus);
		rightTalon.changeControlMode(TalonControlMode.PercentVbus);
		leftTalon.set(setVelocity.leftWheelSpeed);
		rightTalon.set(-(setVelocity.rightWheelSpeed));
	}

	private void setWheelVelocity(DriveVelocity setVelocity) {
		leftTalon.changeControlMode(TalonControlMode.Speed);
		rightTalon.changeControlMode(TalonControlMode.Speed);
		// inches per sec to rotations per min
		if (Math.abs(setVelocity.leftWheelSpeed) > Constants.MaxDriveSpeed || Math.abs(setVelocity.rightWheelSpeed) > Constants.MaxDriveSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over 216!", false);
			return;
		}
		// in/s -> (in / pi) * 15
		// positive deltaSpeed turns right by making left wheels faster than right
		leftTalon.setSetpoint((setVelocity.leftWheelSpeed) / Math.PI * 15);
		rightTalon.setSetpoint((setVelocity.rightWheelSpeed) / Math.PI * 15);
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
