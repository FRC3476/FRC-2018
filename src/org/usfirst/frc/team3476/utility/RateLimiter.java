package org.usfirst.frc.team3476.utility;

/**
 * Limits acceleration and optionally jerk
 */
public class RateLimiter {
	
	private double maxAccel, maxJerk, latestValue;
	private double accValue;
	
	public RateLimiter(double accel){
		this(accel, Double.NEGATIVE_INFINITY);
	}
	
	public RateLimiter(double accel, double jerk){
		this.maxAccel = accel;
		this.maxJerk = jerk;
		latestValue = 0;
	}
	
	public double update(double setpoint, double dt){
		double diff = setpoint - latestValue;
		double area = (Math.pow(accValue, 2) / maxJerk);
		if(Math.abs(diff) >= Math.abs(area)){
			accValue = accValue + Math.copySign(maxJerk * dt, diff);
			if(Math.abs(accValue) > maxAccel){
				accValue = Math.copySign(maxAccel, accValue);
			}
			latestValue += accValue * dt;
		} else {
			accValue = accValue - Math.copySign(maxJerk * dt, diff);
			if(Math.abs(accValue) > maxAccel){
				accValue = Math.copySign(maxAccel, accValue);
			}
			latestValue += accValue * dt;
		}
		if(Math.signum(setpoint - latestValue) !=  Math.signum(diff)){
			latestValue = setpoint;
			accValue = 0;
		}
		return latestValue;
	}
	
	public double getAcc(){
		return accValue;
	}
	
	public double getMaxJerk(){
		return maxJerk;
	}
	
	public double getMaxAccel(){
		return maxAccel;
	}
	
	public double getLatestValue(){
		return latestValue;
	}
	
	public void setLatestValue(double val){
		latestValue = val;
	}
	
	public void reset(){
		latestValue = 0;
		accValue = 0;
	}
}
