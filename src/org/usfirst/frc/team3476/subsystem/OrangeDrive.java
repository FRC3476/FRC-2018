package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;
import org.usfirst.frc.team3476.utility.Threaded;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.control.PurePursuitController;
import org.usfirst.frc.team3476.utility.control.RateLimiter;
import org.usfirst.frc.team3476.utility.control.PurePursuitController.AutoDriveSignal;
import org.usfirst.frc.team3476.utility.math.Rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

public class OrangeDrive extends Threaded {
	public enum DriveState {
		TELEOP, AUTO, DONE
	}

	public static class DriveSignal {
		/*
		 * Inches per second for speed
		 */
		public double rightWheelSpeed;
		public double leftWheelSpeed;

		public DriveSignal(double left, double right) {
			this.rightWheelSpeed = left;
			this.leftWheelSpeed = right;
		}
	}

	private static final OrangeDrive instance = new OrangeDrive();

	public static OrangeDrive getInstance() {
		return instance;
	}

	private double quickStopAccumulator;

	private boolean drivePercentVbus;

	private ADXRS450_Gyro gyroSensor = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon, rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;
	private volatile double driveMultiplier;
	private DriveState driveState;
	private RateLimiter moveProfiler;
	private Solenoid shifter;

	private OrangeDrive() {
		shifter = new Solenoid(Constants.DriveShifterId);
		leftTalon = new LazyTalonSRX(Constants.LeftMasterDriveId);
		rightTalon = new LazyTalonSRX(Constants.RightMasterDriveId);

		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		leftSlaveTalon = new LazyTalonSRX(Constants.LeftSlaveDriveId);
		leftSlave2Talon = new LazyTalonSRX(Constants.LeftSlave2DriveId);
		rightSlaveTalon = new LazyTalonSRX(Constants.RightSlaveDriveId);
		rightSlave2Talon = new LazyTalonSRX(Constants.RightSlave2DriveId);
		configMotors();
		
		drivePercentVbus = false;
		driveState = DriveState.TELEOP;

		moveProfiler = new RateLimiter(Constants.TeleopAccLimit);
		
		configHigh();
	}

	private void configHigh() {
		rightTalon.config_kP(0, Constants.kRightHighP, 10);
		rightTalon.config_kI(0, Constants.kRightHighI, 10);
		rightTalon.config_kD(0, Constants.kRightHighD, 10);
		rightTalon.config_kF(0, Constants.kRightHighF, 10);
		leftTalon.config_kP(0, Constants.kLeftHighP, 10);
		leftTalon.config_kI(0, Constants.kLeftHighI, 10);
		leftTalon.config_kD(0, Constants.kRightHighD, 10);
		leftTalon.config_kF(0, Constants.kLeftHighF, 10);
		driveMultiplier = Constants.HighDriveSpeed;
	}
	
	private void configLow() {
		rightTalon.config_kP(0, Constants.kRightLowP, 10);
		rightTalon.config_kF(0, Constants.kRightLowF, 10);	
		leftTalon.config_kP(0, Constants.kLeftLowP, 10);
		leftTalon.config_kF(0, Constants.kLeftLowF, 10);
		driveMultiplier = Constants.LowDriveSpeed;	
	}
	
	public void arcadeDrive(double moveValue, double rotateValue) {
		synchronized(this) {
			driveState = DriveState.TELEOP;			
		}
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
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			moveValue *= Constants.HighDriveSpeed;
			rotateValue *= Constants.HighDriveSpeed;

			leftMotorSpeed = OrangeUtility.coerce(moveValue
					+ rotateValue, Constants.HighDriveSpeed, -Constants.HighDriveSpeed);
			rightMotorSpeed = OrangeUtility.coerce(moveValue
					- rotateValue, Constants.HighDriveSpeed, -Constants.HighDriveSpeed);

			//leftMotorSpeed = leftProfiler.update(leftMotorSpeed);
			//rightMotorSpeed = rightProfiler.update(rightMotorSpeed);

			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void calibrateGyro() {
		gyroSensor.calibrate();
	}

	public void cheesyDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized(this) {
			driveState = DriveState.TELEOP;			
		}
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);

		double leftMotorSpeed;
		double rightMotorSpeed;
		double angularPower = 1;

		double overPower;

		if (isQuickTurn) {
			overPower = 1;
			if (moveValue < 0.2) {
				quickStopAccumulator = 0.9 * quickStopAccumulator + 0.1 * rotateValue * 2;
			}
			angularPower = rotateValue * 0.8;
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

		moveValue = moveProfiler.update(moveValue * driveMultiplier) / driveMultiplier;
		leftMotorSpeed = moveValue + angularPower;
		rightMotorSpeed = moveValue - angularPower;

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
		if(drivePercentVbus){
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));			
		} else {			
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			if(leftMotorSpeed == 0 && rightMotorSpeed == 0) {
				setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));	
			}
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void orangeDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized(this) {
			driveState = DriveState.TELEOP;			
		}
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);
		//50 is min turn radius
		double radius = (1 / rotateValue) + Math.copySign(24, rotateValue);
		double deltaSpeed = (Constants.TrackRadius * ((moveValue * driveMultiplier) / radius));
		deltaSpeed /= driveMultiplier;
		if(isQuickTurn){
			deltaSpeed = rotateValue;
		}
		double leftMotorSpeed = moveValue + deltaSpeed;
		double rightMotorSpeed = moveValue - deltaSpeed;
		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}		
		if(drivePercentVbus){
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));			
		} else {			
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			if(leftMotorSpeed == 0 && rightMotorSpeed == 0) {
				setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));	
			}
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}
	
	private void configMotors() {
		leftSlaveTalon.set(ControlMode.Follower, Constants.LeftMasterDriveId);
		leftSlave2Talon.set(ControlMode.Follower, Constants.LeftMasterDriveId);
		rightSlaveTalon.set(ControlMode.Follower, Constants.RightMasterDriveId);
		rightSlave2Talon.set(ControlMode.Follower, Constants.RightMasterDriveId);
		setBrakeState(NeutralMode.Coast);
		rightTalon.configNominalOutputForward(0.05, 10);
		rightTalon.configNominalOutputReverse(-0.05, 10);
		
		leftTalon.setInverted(true);
		leftSlaveTalon.setInverted(true);
		leftSlave2Talon.setInverted(true);

		rightTalon.setInverted(false);
		rightSlaveTalon.setInverted(false);
		rightSlave2Talon.setInverted(false);

		leftTalon.setSensorPhase(false);
		rightTalon.setSensorPhase(false);
	}

	public void resetMotionProfile() {
		moveProfiler.reset();
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
		return leftTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerMotorRotation * Constants.WheelDiameter
				* Math.PI * 22d/62d / 3d;
	}

	public double getRightDistance() {
		return rightTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerMotorRotation * Constants.WheelDiameter
				* Math.PI * 22d/62d / 3d;
	}

	public double getSpeed() {
		return ((leftTalon.getSelectedSensorVelocity(0) + rightTalon.getSelectedSensorVelocity(0))
				/ Constants.SensorTicksPerMotorRotation) / 10 / 2 * Constants.WheelDiameter * Math.PI;
	}
	
	public double getLeftSpeed() {
		return leftTalon.getSelectedSensorVelocity(0) / Constants.SensorTicksPerMotorRotation * 10 * Constants.WheelDiameter * Math.PI * 22d/62d / 3d;
	}
	
	public double getRightSpeed() {
		return rightTalon.getSelectedSensorVelocity(0) / Constants.SensorTicksPerMotorRotation * 10 * Constants.WheelDiameter * Math.PI * 22d/62d / 3d;
	}
	
	public double scaleJoystickValues(double rawValue) {
		return Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinimumControllerInput, Constants.MaximumControllerInput, Constants.MinimumControllerOutput, Constants.MaximumControllerOutput), rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.AUTO;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		autonomousDriver.resetTime();
		updateAutoPath();
	}

	public void setBrakeState(NeutralMode mode) {
		leftTalon.setNeutralMode(mode);
		rightTalon.setNeutralMode(mode);
		leftSlaveTalon.setNeutralMode(mode);
		rightSlaveTalon.setNeutralMode(mode);
	}

	private void setWheelPower(DriveSignal setVelocity) {
		leftTalon.set(ControlMode.PercentOutput, setVelocity.leftWheelSpeed);
		rightTalon.set(ControlMode.PercentOutput, setVelocity.rightWheelSpeed);
	}

	private void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.leftWheelSpeed) > Constants.HighDriveSpeed
				|| Math.abs(setVelocity.rightWheelSpeed) > Constants.HighDriveSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.HighDriveSpeed + " !", false);
			return;
		}
		// inches per sec to rotations per min
		double leftSetpoint = (setVelocity.leftWheelSpeed) * 4096 / (Constants.WheelDiameter * Math.PI * 10) * (62d/22d) * 3d;
		double rightSetpoint = (setVelocity.rightWheelSpeed) * 4096 / (Constants.WheelDiameter * Math.PI * 10)  * (62/22d) * 3d;
		leftTalon.set(ControlMode.Velocity, leftSetpoint);
		rightTalon.set(ControlMode.Velocity, rightSetpoint);
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

	public void setShiftState(boolean state) {
		shifter.set(state);
		if(state) {
			configLow();
		} else {
			configHigh();
		}
	}

	private void updateAutoPath() {
		AutoDriveSignal signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
		if(signal.isDone){
			synchronized(this){
				driveState = DriveState.DONE;				
			}
		}
		setWheelVelocity(signal.command);
	}

	public void resetGyro() {
		gyroSensor.reset();
	}

	public boolean checkSubsystem() {

		// TODO: Get accurate thresholds
		// TODO: Use PDP to get current
		// boolean success =
		boolean success = leftTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0;
		success = rightTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0 && success;
		success = OrangeUtility.checkMotors(.25, Constants.ExpectedDriveCurrent, Constants.ExpectedDriveRPM, Constants.ExpectedDrivePosition, rightTalon, rightTalon, rightSlaveTalon, rightSlave2Talon);
		success = OrangeUtility.checkMotors(.25, Constants.ExpectedDriveCurrent, Constants.ExpectedDriveRPM, Constants.ExpectedDrivePosition, leftTalon, leftTalon, leftSlaveTalon, leftSlave2Talon)
				&& success;
		configMotors();
		return success;
	}
	
	public void stopMovement() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
		driveState = DriveState.TELEOP;
	}

	synchronized public boolean isFinished() {
		return driveState == DriveState.DONE;
	}
}
