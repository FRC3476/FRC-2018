package org.usfirst.frc.team3476.utility;

/**
 * This class uses a double to interpolate between Interpolable<T>
 * 
 * @param <T>
 * 			Class that implements Interpolable<T>
 */

public class InterpolableValue<T extends Interpolable<T>> {

	private T value;
	private double key;

	public InterpolableValue(double key, T value) {
		this.key = key;
		this.value = value;
	}

	public double getKey() {
		return key;
	}

	public T getValue() {
		return value;
	}
}
