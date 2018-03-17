package org.usfirst.frc.team3476.utility;

import java.time.Duration;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.LockSupport;

/**
 * Keeps track of scheduled tasks and checks every 1ms to execute the task with
 * given threads. Tasks that have not finished in the given time will not be ran
 * again until it finishes.
 */
// TODO: Add remove function
public class ThreadScheduler implements Runnable {

	private Vector<Schedule> schedules;
	private volatile boolean isRunning;
	private volatile boolean paused;

	public ThreadScheduler() {
		schedules = new Vector<Schedule>();
		isRunning = true;
		paused = true;
		ExecutorService schedulingThread = Executors.newSingleThreadExecutor();
		schedulingThread.execute(this);
	}

	@Override
	public void run() {
		while (isRunning) {
			long waitTime = Duration.ofMillis(5).toNanos();
			if (!paused) {
				synchronized (this) {
					for (Schedule schedule : schedules) {
						schedule.executeIfReady();
					}
				}
			}
			LockSupport.parkNanos(waitTime);
		}
	}

	public void schedule(Threaded task, Duration period, ExecutorService thread) {
		schedules.add(new Schedule(task, period.toNanos(), System.nanoTime(), thread));
	}

	public void pause() {
		paused = true;
	}

	public void resume() {
		paused = false;
	}

	public void shutdown() {
		isRunning = false;
	}

	private static class Schedule {
		Threaded task;
		public long taskPeriod, taskTime;
		ExecutorService thread;

		private Schedule(Threaded task, long taskPeriod, long taskTime, ExecutorService thread) {
			this.task = task;
			this.taskPeriod = taskPeriod;
			this.taskTime = taskTime;
			this.thread = thread;
		}

		public void executeIfReady() {
			if (task.isUpdated()) {
				if (System.nanoTime() - taskTime > taskPeriod) {
					thread.submit(task);
					taskTime = System.nanoTime();
				}
			}
		}

		public Threaded getTask() {
			return task;
		}
	}
}
