package org.usfirst.frc.team3476.utility;

/**
 * Stores an x and y value
 */
public class Translation2d implements Interpolable<Translation2d> {

	public static Translation2d fromAngleDistance(double distance, Rotation angle) {
		return new Translation2d(angle.sin() * distance, angle.cos() * distance);
	}

	private double x;

	private double y;

	public Translation2d() {
		x = 0;
		y = 0;
	}

	public Translation2d(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public Rotation getAngleFromOffsetFromYAxis(Translation2d offset) {
		return offset.getAngleFromYAxis(this);
	}
	
	public Rotation getAngleFromOffset(Translation2d offset) {
		return offset.getAngle(this);
	}

	public Rotation getAngleFromYAxis(Translation2d nextPoint) {
		double angleOffset = Math.asin((x - nextPoint.getX()) / getDistanceTo(nextPoint));
		return Rotation.fromRadians(angleOffset);
	}
	
	public Rotation getAngle(Translation2d nextPoint){
		double angleOffset = Math.atan2(nextPoint.getY() - y, nextPoint.getX() - x);
		return Rotation.fromRadians(angleOffset);		
	}

	public double getDistanceTo(Translation2d nextPoint) {
		return Math.sqrt(Math.pow((x - nextPoint.getX()), 2) + Math.pow(y - nextPoint.getY(), 2));
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public Translation2d inverse() {
		return new Translation2d(-x, -y);
	}

	public Translation2d rotateBy(Rotation rotationMat) {
		double x2 = x * rotationMat.cos() - y * rotationMat.sin();
		double y2 = x * rotationMat.sin() + y * rotationMat.cos();
		return new Translation2d(x2, y2);
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public Translation2d translateBy(Translation2d delta) {

		return new Translation2d(x + delta.getX(), y + delta.getY());
	}
	
	@Override
	public Translation2d interpolate(Translation2d other, double percentage) {
		// TODO Auto-generated method stub
		return null;
	}
}
