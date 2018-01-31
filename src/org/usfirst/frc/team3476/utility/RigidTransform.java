package org.usfirst.frc.team3476.utility;

/**
 * Stores a Translation2d and a Rotation 
 */
public class RigidTransform implements Interpolable<RigidTransform> {

	public Rotation rotationMat;
	public Translation2d translationMat;

	public RigidTransform() {
		rotationMat = new Rotation();
		translationMat = new Translation2d();
	}

	public RigidTransform(Translation2d translation, Rotation rotation) {
		rotationMat = rotation;
		translationMat = translation;
	}

	/*
	 * Doesn't work yet
	 */	
	@Override
	public RigidTransform interpolate(RigidTransform other, double percentage) {
		return null;
	}

	/**
	 * Translates delta rotated by our rotation matrix and rotates our rotation matrix by the other rotation matrix
	 * 
	 * @param delta
	 * 			
	 * @return
	 */
	public RigidTransform transform(RigidTransform delta) {
		return new RigidTransform(translationMat.translateBy(delta.translationMat.rotateBy(rotationMat)),
				rotationMat.rotateBy(delta.rotationMat));
	}
}
