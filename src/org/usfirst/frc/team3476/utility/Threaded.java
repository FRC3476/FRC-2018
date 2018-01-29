package org.usfirst.frc.team3476.utility;

/**
 * Classes that want to be threaded using ThreadScheduler need ot implement this class
 */
public abstract class Threaded implements Runnable {

	@Override
	public void run() {
		update();
	}

	public abstract void update();
}
