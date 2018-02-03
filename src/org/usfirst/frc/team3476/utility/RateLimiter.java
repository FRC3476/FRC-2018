package org.usfirst.frc.team3476.utility;

/**
 * Limits acceleration and optionally jerk
 */
public class RateLimiter {

	private double maxAccel, maxJerk, latestValue;
	private double accValue;

	public RateLimiter(double accel) {
		this(accel, Double.POSITIVE_INFINITY);
	}

	/**
	 * 
	 * @param accel
	 *            Maximum acceleration in units with an arbitrary time unit. The
	 *            units match whatever you send in update()
	 * @param jerk
	 *            Maximum jerk in units with an arbitrary time unit. The units
	 *            match whatever you send in update()
	 */
	public RateLimiter(double accel, double jerk) {
		this.maxAccel = accel;
		this.maxJerk = jerk;
		latestValue = 0;
	}

	/**
	 * 
	 * @param setpoint
	 *            What value to accelerate towards
	 * @param dt
	 *            How much time has past between iterations
	 * @return Calculated latest value
	 */
	public double update(double setpoint, double dt) {
		double diff = setpoint - latestValue;
		double area = (Math.pow(accValue, 2) / maxJerk);// Trapezoidal
														// acceleration area at
														// decrease in jerk ->
														// total velocity
		/*
		 * Check if we need to start decelerating
		 */
		if (Math.abs(diff) >= Math.abs(area)) {
			accValue = accValue + Math.copySign(maxJerk * dt, diff);
			/*
			 * Limit accValue to the maximum acceleration
			 */
			OrangeUtility.coerce(accValue, maxAccel, -maxAccel);
			latestValue += accValue * dt;
		} else {
			accValue = accValue - Math.copySign(maxJerk * dt, diff);
			latestValue += accValue * dt;
		}
		/*
		 * Makes sure latestValue isn't greater than setpoint. accValue is moved
		 * to 0 because the else part of the statement above changes the
		 * accValue without despite it not needing to be changed.
		 */
		if (Math.signum(setpoint - latestValue) != Math.signum(diff)) {
			latestValue = setpoint;
			accValue = 0;
		}
		return latestValue;
	}

	/**
	 * 
	 * @return Current acceleration value
	 */
	public double getAcc() {
		return accValue;
	}

	/**
	 * 
	 * @return Current maximum jerk value
	 */
	public double getMaxJerk() {
		return maxJerk;
	}

	/**
	 * 
	 * @return Current maximum acceleration value
	 */
	public double getMaxAccel() {
		return maxAccel;
	}

	/**
	 * 
	 * @return Latest value calculated
	 */
	public double getLatestValue() {
		return latestValue;
	}

	/**
	 * 
	 * @param val
	 *            Value to set the latest value to
	 */
	public void setLatestValue(double val) {
		latestValue = val;
	}

	/**
	 * Sets the latest value to 0 and current acceleration value to 0
	 */
	public void reset() {
		latestValue = 0;
		accValue = 0;
	}

	/**
	 * 
	 * @param maxAccel
	 *            Wanted max acceleration
	 */
	public void setMaxAccel(double maxAccel) {
		this.maxAccel = maxAccel;
	}

	/**
	 * 
	 * @param maxJerk
	 *            Wanted max Jerk
	 */
	public void setMaxJerk(double maxJerk) {
		this.maxJerk = maxJerk;
	}

	/**
	 * 
	 * @param accValue
	 *            Wanted acceleration value
	 */
	public void setAccValue(double accValue) {
		this.accValue = accValue;
	}
}
