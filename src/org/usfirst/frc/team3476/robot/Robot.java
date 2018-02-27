package org.usfirst.frc.team3476.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team3476.subsystem.Elevarm;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.Controller;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.ThreadScheduler;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends IterativeRobot {
	Controller xbox = new Controller(0);
	OrangeDrive drive = OrangeDrive.getInstance();
	Elevarm elevarm = Elevarm.getInstance();
	RobotTracker tracker = RobotTracker.getInstance();
	ExecutorService mainExecutor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();
	LazyTalonSRX intakeMotor1 = new LazyTalonSRX(Constants.Intake1Id);
	LazyTalonSRX intakeMotor2 = new LazyTalonSRX(Constants.Intake2Id);
	Solenoid intakeSolenoid = new Solenoid(Constants.IntakeSolenoidId);

	boolean homed = false;

	Path autoPath;

	@Override
	public void robotInit() {
		scheduler.schedule(drive, Duration.ofMillis(10), mainExecutor);
		scheduler.schedule(tracker, Duration.ofMillis(10), mainExecutor);
		scheduler.schedule(elevarm, Duration.ofMillis(20), mainExecutor);
	}

	@Override
	public void autonomousInit() {
		scheduler.resume();
		tracker.resetOdometry();
		autoPath = new Path(new Translation2d(0, 0));
		autoPath.addPoint(250, -10, 100);
		autoPath.addPoint(300, 5, 100);
		autoPath.addPoint(209, 0, 100);
		autoPath.addPoint(250, -5, 100);
		autoPath.addPoint(300, 5, 100);
		autoPath.addPoint(209, 28.1, 100);
		autoPath.addPoint(250, -5, 100);
		autoPath.addPoint(300, 5, 100);
		drive.setAutoPath(autoPath, false);
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		scheduler.resume();
		drive.resetMotionProfile();
		elevarm.resetMotionProfile();
		if (!homed) {
			elevarm.homeElevator();
			homed = true;
		}
	}

	@Override
	public void disabledInit() {
		scheduler.pause();
	}

	double elevatorMaxCurrent = 150, armMaxCurrent = 40; // TEMP for testing

	@Override
	public void teleopPeriodic() {
		drive.arcadeDrive(xbox.getRawAxis(1), -xbox.getRawAxis(4));

		// System.out.println("Angle: " + elevarm.getArmAngle()+ " Setpoint: " + elevarm.getTargetArmAngle());
		// System.out.println("Height: " + elevarm.getElevatorHeight() + " Setpoint: " +
		// elevarm.getTargetElevatorHeight());
		double current = elevarm.getElevatorOutputCurrent();
		if (current > 20) {
			System.out.println("Current: " + current);
		}

		if (elevarm.getElevatorOutputCurrent() < elevatorMaxCurrent || elevarm.getArmOutputCurrent() < armMaxCurrent) // Prevent
																														// elevator
																														// from
																														// killing
																														// itself
		{
			if (xbox.getRisingEdge(7)) {
				elevarm.homeElevator();
				System.out.println("_______________________HOMING_________________________________");
			}

			// elevarm.setElevatorPercentOutput(xbox.getRawAxis(2)/2 - xbox.getRawAxis(3)/2);

			// Cube intake / outtake positions
			if (xbox.getRisingEdge(5)) {
				elevarm.setElevatorHeight(4.5);
				elevarm.setArmAngle(-26);
			}
			if (xbox.getRisingEdge(6)) {
				elevarm.setElevatorHeight(63);
				elevarm.setArmAngle(Constants.ArmHorizontalDegrees);
			}
		} else {
			elevarm.setElevatorPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}

		if (xbox.getRawButton(1)) {
			intakeMotor1.set(ControlMode.PercentOutput, 1);
			intakeMotor2.set(ControlMode.PercentOutput, 1);
		} else if (xbox.getRawAxis(3) > .3) {
			intakeMotor1.set(ControlMode.PercentOutput, -.45); // out
			intakeMotor2.set(ControlMode.PercentOutput, -.45); // out
		} else {
			intakeMotor1.set(ControlMode.PercentOutput, 0);
			intakeMotor2.set(ControlMode.PercentOutput, 0);
		}
		if (xbox.getRisingEdge(4)) {
			elevarm.homeElevator();
		}

		if (xbox.getRawAxis(2) > 0.3) {
			intakeSolenoid.set(true);
		} else {
			intakeSolenoid.set(false);
		}

		if (elevarm.getArmOutputCurrent() < armMaxCurrent) // Prevent arm from killing itself
		{
			// Manual Arm Control
			/*
			 * if (xbox.getPOV() == 0)
			 * {
			 * elevarm.setArmPercentOutput(.40);
			 * }
			 * else if (xbox.getPOV() == 180)
			 * {
			 * elevarm.setArmPercentOutput(-.40);
			 * }
			 * else
			 * {
			 * elevarm.setArmPercentOutput(0);
			 * }
			 * //Arm Position Control
			 * if (xbox.getRisingEdge(7))
			 * {
			 * elevarm.setArmAngle(Constants.ArmHorizontalDegrees);
			 * }
			 * if (xbox.getRisingEdge(8))
			 * {
			 * elevarm.setArmAngle(-27);
			 * }
			 */
		} else {
			elevarm.setArmPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}

		if (elevarm.getElevatorOutputCurrent() < elevatorMaxCurrent && elevarm.getArmOutputCurrent() < armMaxCurrent) {
			if (xbox.getRisingEdge(9)) {
				elevarm.setOverallPosition(20, 36);
			}
			if (xbox.getRisingEdge(10)) {
				elevarm.setOverallPosition(10, 72);
			}
		} else {
			elevarm.setElevatorPercentOutput(0);
			elevarm.setArmPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}

		if (xbox.getRisingEdge(11)) {
			elevarm.shiftElevatorGearbox(false);
		}
		if (xbox.getRisingEdge(12)) {
			elevarm.shiftElevatorGearbox(true);
		}

		if (xbox.getRisingEdge(3)) {
			drive.setShiftState(true);
		}
		if (xbox.getRisingEdge(4)) {
			drive.setShiftState(false);
		}
		xbox.update();
	}

	@Override
	public void testInit() {
		scheduler.pause();
		drive.stopSubsystem();
		elevarm.stopSubsystem();
	}

	@Override
	public void testPeriodic() {
		xbox.update();
		if (xbox.getRisingEdge(1)) {
			drive.checkSubsystem();
		}
		if (xbox.getRisingEdge(2)) {
			elevarm.checkSubsystem();
		}
		if (xbox.getRisingEdge(3)) {
			elevarm.homeElevator();
		}
		if (xbox.getRisingEdge(4)) {
			elevarm.homeArm();
		}
	}
}
