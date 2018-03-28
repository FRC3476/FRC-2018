package org.usfirst.frc.team3476.utility;

import java.time.Duration;

import edu.wpi.first.wpilibj.Timer;

/**
 * Classes that want to be threaded using ThreadScheduler need to implement this class
 */
public abstract class Threaded implements Runnable {

	private boolean isUpdated = true;
	private volatile boolean isPaused = false;
	private double lastRuntime = 0;
	private volatile long period;

	@Override
	public void run() {
		if(!isPaused) {
			isUpdated = false;
			double start = Timer.getFPGATimestamp();
			update();
			lastRuntime = Timer.getFPGATimestamp() - start;
			isUpdated = true;
		}
	}

	public abstract void update();

	public boolean isUpdated() {
		return isUpdated;
	}

	public double getLastRuntime() {
		return lastRuntime;
	}
	
	public double getPeriod() {
		return period;
	}
	
	public void setPeriod(Duration duration) {
		this.period = duration.getNano();
	}
	
	public void pause() {
		isPaused = true;
	}
	
	public void resume() {
		isPaused = false;
	}
}
