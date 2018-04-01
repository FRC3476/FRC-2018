
package org.usfirst.frc.team3476.utility.control;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.auto.AutoCommand;
import org.usfirst.frc.team3476.utility.auto.AutoRoutine;
import org.usfirst.frc.team3476.utility.math.Rotation;
import org.usfirst.frc.team3476.utility.math.Translation2d;

/**
 * Contains a list of points and generated segments from these points. This can
 * be used to calculate lookahead in this class.
 */
public class Path {

	/**
	 * Two points that function as a line.
	 */
	public static class PathSegment {

		private Translation2d start, end, delta;
		private double maxSpeed, deltaDist, deltaDistSquared;
		
		private ArrayList<AutoRoutineTrigger> commands = new ArrayList<>();

		private PathSegment(double xStart, double yStart, double xEnd, double yEnd, double maxSpeed) {
			this(new Translation2d(xStart, yStart), new Translation2d(xEnd, yEnd), maxSpeed);
		}

		private PathSegment(Translation2d start, Translation2d end, double maxSpeed) {
			this.start = start;
			this.end = end;
			this.maxSpeed = maxSpeed;
			delta = start.inverse().translateBy(end);
			deltaDist = Math.hypot(delta.getX(), delta.getY());
			deltaDistSquared = Math.pow(deltaDist, 2);
		}
		
		public void addAutoRoutineTrigger(AutoRoutineTrigger routine) {
			this.commands.add(routine);
			Collections.sort(this.commands);
		}
	
		private Translation2d getStart() {
			return start;
		}

		private Translation2d getEnd() {
			return end;
		}

		/**
		 * Gets the point on the line closest to the point defined in the
		 * argument
		 *
		 * @param point
		 *            Point to find the closest point to
		 * @return The closest point on the line to the point specified
		 */
		private Translation2d getClosestPoint(Translation2d point) {
			double u = ((point.getX() - start.getX()) * delta.getX() + (point.getY() - start.getY()) * delta.getY())
					/ deltaDistSquared;
			u = Math.max(Math.min(u, 1), 0);
			return new Translation2d(start.getX() + delta.getX() * u, start.getY() + delta.getY() * u);
		}
		
		private double getPercentageOnSegment(Translation2d point) {
			double u = ((point.getX() - start.getX()) * delta.getX() + (point.getY() - start.getY()) * delta.getY())
					/ deltaDistSquared;
			u = Math.max(Math.min(u, 1), 0);
			return u;
		}

		/**
		 * Returns the point on the line which is some distance away from the
		 * start. The point travels on the line towards the end. More efficient
		 * than calling interpolate in Translation2d because delta is
		 * pre-computed before.
		 *
		 * @param distance
		 *            Distance from the start of the line.
		 * @return Point on the line that is the distance away specified.
		 */
		private Translation2d getPointByDistance(double distance) {
			distance = Math.pow(distance, 2);
			double u = Math.sqrt(distance / deltaDistSquared);
			return new Translation2d(start.getX() + delta.getX() * u, start.getY() + delta.getY() * u);
		}

		/**
		 * Returns the point on the line which is some distance away from the
		 * end. The point travels on the line towards the start.
		 *
		 * @param distance
		 *            Distance from the end of the line.
		 * @return Point on the line that is the distance away from the end
		 */
		private Translation2d getPointByDistanceFromEnd(double distance) {
			distance = Math.pow(distance, 2);
			double u = Math.sqrt(distance / deltaDistSquared);
			return new Translation2d(end.getX() - delta.getX() * u, end.getY() - end.getY() * u);
		}

		/**
		 *
		 * @return Maximum speed for the path segment.
		 */
		private double getMaxSpeed() {
			return maxSpeed;
		}

		/**
		 *
		 * @return Total distance from start of the segment to the end.
		 */
		private double getDistance() {
			return deltaDist;
		}

		/**
		 *
		 * @return X and Y offset from the start to the end of the segment.
		 */
		private Translation2d getDelta() {
			return delta;
		}
	}

	public static class DrivingData {
		public double remainingDist, maxSpeed;
		public Translation2d lookAheadPoint;
		public Translation2d closestPoint;
		public Translation2d currentSegEnd;
	}
	
	public static class AutoRoutineTrigger implements Comparable<AutoRoutineTrigger>{
		private double percentage;
		private AutoRoutine routine;
		
		//Should not be blocking nerds
		public AutoRoutineTrigger(AutoRoutine routine, double percentage) {
			this.routine = routine;
			this.percentage = percentage;
		}

		@Override
		public int compareTo(AutoRoutineTrigger other) {
			if (this.percentage > other.percentage)
				return 1;
			if (this.percentage < other.percentage)
				return -1;
			return -0;
		}
	}

	private List<PathSegment> segments;
	private Translation2d lastPoint;
	private Rotation endAngle = null;
	private volatile boolean isEmpty;

	/**
	 * Contains a list of points and can create path segments with them.
	 * Lookahead is calculated with using this.
	 *
	 * @param start
	 *            Initial point in path.
	 */
	public Path(Translation2d start) {
		segments = new ArrayList<PathSegment>();
		lastPoint = start;
		isEmpty = true;
	}

	/**
	 * Add another point to the path.
	 *
	 * @param x
	 * @param y
	 * @param speed
	 */
	public void addPoint(double x, double y, double speed) {
		segments.add(new PathSegment(lastPoint.getX(), lastPoint.getY(), x, y, speed));
		lastPoint = new Translation2d(x, y);
		isEmpty = false;
	}
	
	public void addPoint(Translation2d point, double speed)
	{
		addPoint(point.getX(), point.getY(), speed);	
		isEmpty = false;
	}

	public void addRoutineToCurrentSegment(AutoRoutine routine, double percentage){
		segments.get(segments.size() - 1).addAutoRoutineTrigger(new AutoRoutineTrigger(routine, percentage));
	}
	public void addCommandToCurrentSegment(AutoCommand command, double percentage){
		AutoRoutine routine = new AutoRoutine();
		routine.addCommands(command);
		segments.get(segments.size() - 1).addAutoRoutineTrigger(new AutoRoutineTrigger(routine, percentage));
	}
	/**
	 * Sets the desired angle for the robot to end in. It does this by placing
	 * up to two points that have a 90 degree angle to allow the robot to
	 * converge on the path.
	 *
	 * @param angle
	 *            Angle for the robot to end in.
	 */
	public void setAngle(Rotation angle) {
		endAngle = angle;
	}


	public boolean isEmpty(){
		return isEmpty;
	}
	/**
	 *
	 */
	public void processPoints() {
		if (endAngle != null) {
			PathSegment lastSegment = segments.get(segments.size() - 1);
			Rotation angleOfPath = lastSegment.getStart().getAngle(lastSegment.getEnd());
			Rotation rotatedEndAngle = angleOfPath.inverse().rotateBy(endAngle);
			boolean rotateLeft = rotatedEndAngle.sin() > 0;

			Translation2d finalSegmentStart = new Translation2d(Constants.MinimumTurningRadius, 0).rotateBy(endAngle.flip());
			Translation2d secondSegmentStart = finalSegmentStart.rotateBy(Rotation.fromDegrees(rotateLeft ? 90 : -90));
			// The two points we potentially add
			finalSegmentStart = finalSegmentStart.translateBy(lastSegment.getEnd());
			secondSegmentStart = secondSegmentStart.translateBy(finalSegmentStart);

			segments.remove(segments.size() - 1);
			// Add both points if it is
			// sharper than a 90 degree turn
			if (rotatedEndAngle.cos() < 0) {
				segments.add(new PathSegment(lastSegment.getStart(), secondSegmentStart, lastSegment.getMaxSpeed()));
				segments.add(new PathSegment(secondSegmentStart, finalSegmentStart, lastSegment.getMaxSpeed()));
			} else {
				segments.add(new PathSegment(lastSegment.getStart(), finalSegmentStart, lastSegment.getMaxSpeed()));
			}
			segments.add(new PathSegment(finalSegmentStart, lastSegment.getEnd(), lastSegment.getMaxSpeed()));
		}
	}

	/**
	 * Prints all the points on the path.
	 */
	synchronized public void printAllPoints() {
		for (PathSegment segment : segments) {
			System.out.println(segment.getStart().getX() + "    " + segment.getStart().getY());
		}
		System.out.println(segments.get(segments.size()).getEnd().getX() + "   "
				+ segments.get(segments.size()).getEnd().getY());
	}

	/**
	 * TODO: Explain what's goin on in this function. Review with uncool kids(mentor)
	 *
	 * @param pose
	 *            Current robot position
	 * @param lookAheadDistance
	 *            Distance on the path to get the look ahead.
	 * @return
	 */
	public DrivingData getLookAheadPoint(Translation2d pose, double lookAheadDistance) {
		DrivingData data = new DrivingData();
		Translation2d closestPoint = segments.get(0).getClosestPoint(pose);
		Translation2d closestToRobot = closestPoint.inverse().translateBy(pose);
		while (segments.size() > 1) {
			double distToClosest = Math.hypot(closestToRobot.getX(), closestToRobot.getY());
			Translation2d closestNextPoint = segments.get(1).getClosestPoint(pose);
			Translation2d closestNextToRobot = closestNextPoint.inverse().translateBy(pose);
			double distToNext = Math.hypot(closestNextToRobot.getX(), closestNextToRobot.getY());
			if (distToClosest > distToNext) {
				//Run commands that didn't run yet in segments being deleted
				while(!segments.get(0).commands.isEmpty()) {
					segments.get(0).commands.remove(0).routine.run();
				}
				segments.remove(0);				
				closestPoint = closestNextPoint;
				closestToRobot = closestNextToRobot;
			} else {
				break;
			}
		}
		//Run commands when we zoom past
		double percentage = segments.get(0).getPercentageOnSegment(pose);
		while(!segments.get(0).commands.isEmpty()) {
			if(segments.get(0).commands.get(0).percentage <= percentage) {
				segments.get(0).commands.remove(0).routine.run();
			} else {
				break;
			}
		}
		data.closestPoint = closestPoint;
		data.currentSegEnd = segments.get(0).getEnd();
		Translation2d closestToEnd = closestPoint.inverse().translateBy(segments.get(0).getEnd());
		Translation2d closestToStart = segments.get(0).getStart().inverse().translateBy(closestPoint);

		lookAheadDistance += Math.hypot(closestToRobot.getX(), closestToRobot.getY());
		double remainingSegDist = Math.hypot(closestToEnd.getX(), closestToEnd.getY());
		data.remainingDist = remainingSegDist;
		data.maxSpeed = segments.get(0).getMaxSpeed();
		for (int i = 1; i < segments.size(); i++) {
			data.remainingDist += segments.get(i).getDistance();
		}
		if (lookAheadDistance > remainingSegDist && segments.size() > 1) {
			lookAheadDistance -= remainingSegDist;
			PathSegment lastSegment;
			for (int i = 1; i < segments.size(); i++) {
				if (lookAheadDistance > segments.get(i).getDistance() && i != (segments.size() - 1)) {
					lookAheadDistance -= segments.get(i).getDistance();
				} else {
					data.lookAheadPoint = segments.get(i).getPointByDistance(lookAheadDistance);
					break;
				}
			}
		} else {
			double distanceOnPath = segments.get(0).getDistance() - remainingSegDist + lookAheadDistance;
			data.lookAheadPoint = segments.get(0).getPointByDistance(distanceOnPath);
		}
		/*
		 * UDPServer.getInstance().send(data.lookAheadPoint.getX() + "," +
		 * data.lookAheadPoint.getY() + "," + pose.getX() + "," + pose.getY(),
		 * 5801);
		 */
		return data;
	}
}
