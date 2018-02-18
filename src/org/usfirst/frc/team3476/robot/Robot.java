package org.usfirst.frc.team3476.robot;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team3476.subsystem.Arm;
import org.usfirst.frc.team3476.subsystem.Elevarm;
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
	Elevarm elevarm = Elevarm.getInstance();
	RobotTracker tracker = RobotTracker.getInstance();
	ExecutorService mainExecutor = Executors.newFixedThreadPool(4);
	ThreadScheduler scheduler = new ThreadScheduler();

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
	}

	@Override
	public void disabledInit() {
		scheduler.pause();
	}
	
	double elevatorMaxCurrent = 10, armMaxCurrent = 2; //TEMP for testing
	
	@Override
	public void teleopPeriodic() {
		drive.arcadeDrive(xbox.getRawAxis(1), -xbox.getRawAxis(4));

		if (elevarm.getElevatorOutputCurrent() < elevatorMaxCurrent) //Prevent elevator from killing itself
		{
			//Manual Elevator Control
			/*if (xbox.getRawButton(1))
			{
				elevator.setPercentOutput(.1);
			}
			else if (xbox.getRawaButton(2))
			{
				elevator.setPercentOutput(-.5);
			}
			else
			{
				elevator.setPercentOutput(0);
			}*/
			//Elevator Manual Control
			elevarm.setElevatorPercentOutput(xbox.getRawAxis(3)/2 - xbox.getRawAxis(2)/2);
			
			//Elevator Position Control
			if (xbox.getRisingEdge(5))
			{
				elevarm.setElevatorHeight(50);
			}
			if (xbox.getRisingEdge(6))
			{
				elevarm.setElevatorHeight(5);
			}
		}
		else
		{
			elevarm.setElevatorPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}
		
		if (elevarm.getArmOutputCurrent() < armMaxCurrent) //Prevent arm from killing itself
		{
			//Manual Arm Control
			if (xbox.getPOV() == 0)
			{
				elevarm.setArmPercentOutput(.5);
			}
			else if (xbox.getPOV() == 4)
			{
				elevarm.setArmPercentOutput(-.5);
			}
			else
			{
				elevarm.setArmPercentOutput(0);
			}
			
			//Arm Position Control
			if (xbox.getRisingEdge(7))
			{
				elevarm.setArmAngle(Constants.ArmHorizontal);
			}
			if (xbox.getRisingEdge(8))
			{
				elevarm.setArmAngle(55);
			}
		}
		else
		{
			elevarm.setArmPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}

		if (elevarm.getElevatorOutputCurrent() < elevatorMaxCurrent && elevarm.getArmOutputCurrent() < armMaxCurrent)
		{
			if (xbox.getRisingEdge(9))
			{
				elevarm.setOverallPosition(20, 36);
			}
			if (xbox.getRisingEdge(10))
			{
				elevarm.setOverallPosition(10, 72);
			}
		}
		else
		{
			elevarm.setElevatorPercentOutput(0);
			elevarm.setArmPercentOutput(0);
			System.out.println("---------------------------Current Threshold Reached ---------------------------");
		}
		
		if (xbox.getRisingEdge(11))
		{
			elevarm.shiftElevatorGearbox(false);
		}
		if (xbox.getRisingEdge(12))
		{
			elevarm.shiftElevatorGearbox(true);
		}
		
		if (xbox.getRisingEdge(3))
		{
			drive.setShiftState(true);
		}
		if (xbox.getRisingEdge(4))
		{
			drive.setShiftState(false);
		}
		
	}

	@Override
	public void testInit() {
		drive.checkSubsystem();
		elevarm.checkSubsystem();
	}
	
	@Override
	public void testPeriodic() {
		
	}
}
