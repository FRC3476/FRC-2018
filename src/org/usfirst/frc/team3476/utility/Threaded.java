package org.usfirst.frc.team3476.utility;

import edu.wpi.first.wpilibj.Timer;

/**
 * Classes that want to be threaded using ThreadScheduler need to implement this class
 */
public abstract class Threaded implements Runnable {

	private volatile boolean isDone = true;
	private volatile double lastRuntime = 0;
	
	@Override
	public void run() {
		isDone = false;
		double start = Timer.getFPGATimestamp();
		update();
		lastRuntime = Timer.getFPGATimestamp() - start;
		isDone = true;
	}

	public abstract void update();
	
	public boolean isDone(){
		return isDone;
	}
	
	public double getLastRuntime() {
		return lastRuntime;
	}
}
