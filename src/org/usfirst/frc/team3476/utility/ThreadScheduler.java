package org.usfirst.frc.team3476.utility;

import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.LockSupport;


/**
 * Keeps track of scheduled tasks and checks every 1ms to execute the task with
 * given threads. Tasks that have not finished in the given time will not be ran
 * again until it finishes.
 */
//TODO: Add remove function
public class ThreadScheduler implements Runnable {

	private Vector<Schedule> schedules;
	private long msToNs = 1000000;
	private volatile boolean isRunning;

	public ThreadScheduler() {
		schedules = new Vector<Schedule>();
		isRunning = true;

		ExecutorService schedulingThread = Executors.newSingleThreadExecutor();
		schedulingThread.execute(this);
	}

	@Override
	public void run() {		
		while (isRunning) {
			long waitTime = 1 * msToNs;
			synchronized (this) {
				for(Schedule schedule : schedules) {
					schedule.executeIfReady();
				}				
			}
			LockSupport.parkNanos(waitTime);
		}
	}

	public void schedule(Threaded task, long period, ExecutorService thread) {
		schedules.add(new Schedule(task, period, System.nanoTime(), thread));
	}

	public void shutdown() {
		isRunning = false;
	}
	
	public void remove (Threaded task){
		for(Schedule schedule : schedules){
			if(task == schedule.getTask()){
				schedules.remove(schedule);
				return;
			}
		}
		System.out.println("Task not found");
	}
	
	private static class Schedule {
		Threaded task;
		public long taskPeriod, taskTime;
		ExecutorService thread;
		private Schedule (Threaded task, long taskPeriod, long taskTime, ExecutorService thread) {
			this.task = task;
			this.taskPeriod = taskPeriod;
			this.taskTime = taskTime;
			this.thread = thread;
		}
		
		public void executeIfReady() {
			if(task.isDone()) {
				if(System.nanoTime() - taskTime > taskPeriod){
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
