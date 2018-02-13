package org.usfirst.frc.team3476.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team3476.subsystem.Elevator;
import org.usfirst.frc.team3476.subsystem.OrangeDrive;
import org.usfirst.frc.team3476.subsystem.RobotTracker;
import org.usfirst.frc.team3476.utility.Controller;
import org.usfirst.frc.team3476.utility.ThreadScheduler;
import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Rotation;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Controller xbox = new Controller(0);
	OrangeDrive drive = OrangeDrive.getInstance();
	Elevator elevator = Elevator.getInstance();
	RobotTracker tracker = RobotTracker.getInstance();
	ExecutorService mainExecutor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();

	Path autoPath = new Path(new Translation2d(0, 0));

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		scheduler.schedule(drive, Duration.ofMillis(10), mainExecutor);
		// scheduler.schedule(tracker, Duration.ofMillis(10), mainExecutor);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		scheduler.resume();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		tracker.resetOdometry();
		autoPath.addPoint(100, 0, 40);
		autoPath.addPoint(75, -85, 35);
		autoPath.addPoint(50, -85, 30);
		autoPath.addPoint(50, -40, 30);
		autoPath.addPoint(0, -40, 30);
		autoPath.setAngle(Rotation.fromDegrees(90));
		autoPath.processPoints();
		drive.setAutoPath(autoPath, false);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		scheduler.resume();
		drive.resetMotionProfile();
	}

	@Override
	public void disabledInit() {
		scheduler.pause();
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		drive.arcadeDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4));

	}

	@Override
	public void testInit() {
		drive.checkSubsystem();
	}
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		
	}
}
