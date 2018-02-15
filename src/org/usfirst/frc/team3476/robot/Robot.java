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

public class Robot extends IterativeRobot {
	Controller xbox = new Controller(0);
	OrangeDrive drive = OrangeDrive.getInstance();
	Elevator elevator = Elevator.getInstance();
	RobotTracker tracker = RobotTracker.getInstance();
	ExecutorService mainExecutor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();

	Path autoPath;

	@Override
	public void robotInit() {
		scheduler.schedule(drive, Duration.ofMillis(10), mainExecutor);
		scheduler.schedule(tracker, Duration.ofMillis(10), mainExecutor);
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
	}

	@Override
	public void disabledInit() {
		scheduler.pause();
	}
	
	@Override
	public void teleopPeriodic() {
		drive.arcadeDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4));
		
		
	}

	@Override
	public void testInit() {
		drive.checkSubsystem();
	}
	
	@Override
	public void testPeriodic() {
		
	}
}
