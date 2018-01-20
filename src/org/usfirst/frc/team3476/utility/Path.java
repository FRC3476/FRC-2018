package org.usfirst.frc.team3476.utility;


import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.usfirst.frc.team3476.robot.Constants;

public class Path {

	public static class PathSegment {	
		
		private Translation2d start, end, delta;
		private double maxSpeed, deltaDist, deltaDistSquared;
		
		private PathSegment(double xStart, double yStart, double xEnd, double yEnd, double maxSpeed){
			start = new Translation2d(xStart, yStart);
			end = new Translation2d(xEnd, yEnd);
			this.maxSpeed = maxSpeed;
			delta = start.inverse().translateBy(end);
			deltaDist = Math.hypot(delta.getX(), delta.getY());
			deltaDistSquared = Math.pow(deltaDist, 2);
		}
		
		private PathSegment(Translation2d start, Translation2d end, double maxSpeed){
			this.start = start;
			this.end = end;
			this.maxSpeed = maxSpeed;
			delta = start.inverse().translateBy(end);
			deltaDist = Math.hypot(delta.getX(), delta.getY());
			deltaDistSquared = Math.pow(deltaDist, 2);
		}
		
		private Translation2d getStart(){
			return start;
		}
		
		private Translation2d getEnd(){
			return end;
		}
		
		private Translation2d getClosestPoint(Translation2d point){
			double u = ((point.getX() - start.getX()) * delta.getX() + (point.getY() - start.getY()) * delta.getY()) / deltaDistSquared;
			u = Math.max(Math.min(u, 1), 0);
			return new Translation2d(start.getX() + delta.getX() * u, start.getY() + delta.getY() * u);	
		}
		
		private Translation2d getPointByDistance(double distance){
			distance = Math.pow(distance, 2);
			double u = Math.sqrt(distance / deltaDistSquared);
			return new Translation2d(start.getX() + delta.getX() * u, start.getY() + delta.getY() * u);
		}
		
		private Translation2d getPointByDistanceFromEnd(double distance){
			distance = Math.pow(distance, 2);
			double u = Math.sqrt(distance / deltaDistSquared);
			return new Translation2d(end.getX() - delta.getX() * u, end.getY() - end.getY() * u);
		}
		
		private double getSpeed(){
			return maxSpeed;
		}
		
		private double getDistance(){
			return deltaDist;
		}		
		
		private Translation2d getDelta(){
			return delta;
		}
	}
	
	public static class DrivingData {
		public double remainingDist, maxSpeed;
		public Translation2d lookAheadPoint;
		public Translation2d closestPoint;
	}
	
	private List<PathSegment> segments;
	private Translation2d lastPoint;
	private Rotation endAngle = null;
	public Path(Translation2d start) {
		segments = new ArrayList<PathSegment>();
		lastPoint = start;
	}
	
	public void addPoint(double x, double y, double speed){
		segments.add(new PathSegment(lastPoint.getX(), lastPoint.getY(), x, y, speed));
		lastPoint = new Translation2d(x, y);
	}	
	
	public void setAngle(Rotation angle){
		endAngle = angle;
	}
	
	public void processPoints(){
		if(endAngle != null){
			PathSegment lastSegment = segments.get(segments.size() - 1);
			Rotation angleOfPath = lastSegment.getStart().getAngle(lastSegment.getEnd());
			Rotation rotatedEndAngle = angleOfPath.inverse().rotateBy(endAngle);
			boolean rotateLeft = rotatedEndAngle.sin() > 0;
		
			Translation2d finalSegmentStart = new Translation2d(Constants.MinimumTurningRadius, 0).rotateBy(endAngle.flip());
			Translation2d secondSegmentStart = finalSegmentStart.rotateBy(Rotation.fromDegrees(rotateLeft ? 90 : -90));
			finalSegmentStart = finalSegmentStart.translateBy(lastSegment.getEnd());
			secondSegmentStart = secondSegmentStart.translateBy(finalSegmentStart);

			segments.remove(segments.size() - 1);
			//sharper than a 90 degree turn
			if(rotatedEndAngle.cos() < 0){
				segments.add(new PathSegment(lastSegment.getStart(), secondSegmentStart, lastSegment.getSpeed()));
				segments.add(new PathSegment(secondSegmentStart, finalSegmentStart, lastSegment.getSpeed()));
			} else {
				segments.add(new PathSegment(lastSegment.getStart(), finalSegmentStart, lastSegment.getSpeed()));
			}
			segments.add(new PathSegment(finalSegmentStart, lastSegment.getEnd(), lastSegment.getSpeed()));
			
			/*
			PathSegment lastSegment = segments.get(segments.size() - 1);
			Rotation angleOfPath = lastSegment.getStart().getAngle(lastSegment.getEnd());
			Rotation rotatedEndAngle = angleOfPath.inverse().rotateBy(endAngle);
			boolean rotateRight = rotatedEndAngle.sin() > 0;
			Rotation angleToCircle;
			if(rotateRight){
				angleToCircle = endAngle.rotateBy(Rotation.fromDegrees(-90));
			} else {
				angleToCircle = endAngle.rotateBy(Rotation.fromDegrees(90));				
			}
			Translation2d endToCircle = new Translation2d(Constants.MinimumTurningRadius, 0).rotateBy(angleToCircle);
			Translation2d circleCenter = lastSegment.getEnd().translateBy(endToCircle);
			double startToCircleDistance = circleCenter.getDistanceTo(lastSegment.getStart());
			Rotation angleToNewPoint = Rotation.fromRadians(Math.acos(Constants.MinimumTurningRadius / startToCircleDistance));
			PathSegment startToCircle = new PathSegment(lastSegment.getStart().getX(), lastSegment.getStart().getX(), circleCenter.getX(), circleCenter.getY(), 0);
			Translation2d circleIntersection = startToCircle.getPointByDistanceFromEnd(Constants.MinimumTurningRadius);
			Translation2d circleIntersectionDelta = lastSegment.getEnd().inverse().translateBy(circleIntersection);
			Translation2d newPointDelta;
			if(rotateRight) {
				newPointDelta = circleIntersectionDelta.rotateBy(angleToNewPoint);
			} else {
				newPointDelta = circleIntersectionDelta.rotateBy(angleToNewPoint.inverse());				
			}
			Translation2d additionalPoint = lastSegment.getEnd().translateBy(newPointDelta);
			PathSegment 
			*/
		}
	}
	
	synchronized public void printAllPoints() {
		for(PathSegment segment : segments) {
			System.out.println(segment.getStart().getX() + "    " + segment.getStart().getY());
		}
		System.out.println(segments.get(segments.size()).getEnd().getX() + "   " + segments.get(segments.size()).getEnd().getY());
	}
	
	synchronized public DrivingData getLookAheadPoint(Translation2d pose, double lookAheadDistance){
		DrivingData data = new DrivingData();
		Translation2d closestPoint = segments.get(0).getClosestPoint(pose);
		Translation2d closestToRobot = closestPoint.inverse().translateBy(pose);
		while(segments.size() > 1){
			double distToClosest = Math.hypot(closestToRobot.getX(), closestToRobot.getY());
			Translation2d closestNextPoint = segments.get(1).getClosestPoint(pose);
			Translation2d closestNextToRobot = closestNextPoint.inverse().translateBy(pose);
			double distToNext = Math.hypot(closestNextToRobot.getX(), closestNextToRobot.getY());
			if(distToClosest > distToNext){
				segments.remove(0);
				closestPoint = closestNextPoint;
				closestToRobot = closestNextToRobot;
			} else {
				break;
			}
		}
		data.closestPoint = closestPoint;
		Translation2d closestToEnd = closestPoint.inverse().translateBy(segments.get(0).getEnd());			
		Translation2d closestToStart = segments.get(0).getStart().inverse().translateBy(closestPoint);
		
		lookAheadDistance += Math.hypot(closestToRobot.getX(), closestToRobot.getY());
		double remainingSegDist = Math.hypot(closestToEnd.getX(), closestToEnd.getY());
		data.remainingDist = remainingSegDist;
		data.maxSpeed = segments.get(0).getSpeed();
		for(int i = 1; i < segments.size(); i++){
			data.remainingDist += segments.get(i).getDistance();
		}
		if(lookAheadDistance > remainingSegDist && segments.size() > 1){
			lookAheadDistance -= remainingSegDist;
			for(int i = 1; i < segments.size(); i++){
				if(lookAheadDistance > segments.get(i).getDistance()){
					lookAheadDistance -= segments.get(i).getDistance();
				} else {
					data.lookAheadPoint = segments.get(i).getPointByDistance(lookAheadDistance);
					break;
				}
			}
		} else {
			lookAheadDistance += Math.hypot(closestToStart.getX(), closestToStart.getY());
			data.lookAheadPoint = segments.get(0).getPointByDistance(lookAheadDistance);
		}
		/*
		UDPServer.getInstance().send(data.lookAheadPoint.getX() + "," +
		data.lookAheadPoint.getY() + "," +
		pose.getX() + "," +
		pose.getY(), 5801);
		*/
		return data;
	}
}
