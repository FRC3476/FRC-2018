package org.usfirst.frc.team3476.utility;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.locks.LockSupport;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Keeps track of scheduled tasks and checks every 1ms to execute the task with
 * given threads. Tasks that have not finished in the given time will not be ran
 * again until it finishes.
 */
//TODO: Add remove function
public class ThreadScheduler implements Runnable {

	private ArrayList<Threaded> scheduledTasks;
	private ArrayList<Long> taskPeriods, taskTimes;
	private ArrayList<Future<?>> scheduledFutures;
	private ArrayList<ExecutorService> threadPools;
	private volatile boolean isRunning;

	public ThreadScheduler() {
		scheduledTasks = new ArrayList<>();
		taskPeriods = new ArrayList<>();
		taskTimes = new ArrayList<>();
		scheduledFutures = new ArrayList<>();
		threadPools = new ArrayList<>();
		isRunning = true;

		ExecutorService schedulingThread = Executors.newSingleThreadExecutor();
		schedulingThread.execute(this);
	}

	@Override
	public void run() {
		while (isRunning) {
			long waitTime = 1000000;
			synchronized (this) {
				for (int task = 0; task < scheduledTasks.size(); task++) {
					long duration = System.nanoTime() - taskTimes.get(task);
					long timeUntilCalled = taskPeriods.get(task) - duration;
					if (timeUntilCalled <= 0) {
						if (scheduledFutures.get(task).isDone()) {
							scheduledFutures.set(task, threadPools.get(task).submit(scheduledTasks.get(task)));
							taskTimes.set(task, System.nanoTime());
						} else {
							// TODO: log error
						}
					} else {
						if (timeUntilCalled < waitTime) {
							waitTime = timeUntilCalled;
						}
					}
				}
			}
			LockSupport.parkNanos(waitTime);
		}
	}

	public void schedule(Threaded task, long period, ExecutorService threadPool) {
		synchronized (this) {
			scheduledTasks.add(task);
			scheduledFutures.add(threadPool.submit(task));
			taskPeriods.add(period);
			taskTimes.add(System.nanoTime());
			threadPools.add(threadPool);
		}
	}

	public void shutdown() {
		isRunning = false;
	}
}
