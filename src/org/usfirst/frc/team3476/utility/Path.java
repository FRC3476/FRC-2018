package org.usfirst.frc.team3476.utility;


import java.util.ArrayList;
import java.util.List;

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
		
		private Translation2d getPointByDistance(double lookAheadDistance){
			lookAheadDistance = Math.pow(lookAheadDistance, 2);
			double u = Math.sqrt(lookAheadDistance / deltaDistSquared);
			return new Translation2d(start.getX() + delta.getX() * u, start.getY() + delta.getY() * u);
		}
		
		private double getSpeed(){
			return maxSpeed;
		}
		
		private double getDistance(){
			return deltaDist;
		}		
	}
	
	public static class DrivingData {
		public double remainingDist, maxSpeed;
		public Translation2d lookAheadPoint;
	}
	
	private List<PathSegment> segments;
	private Translation2d lastPoint;
	
	public Path(Translation2d start) {
		segments = new ArrayList<PathSegment>();
		lastPoint = start;
	}
	
	public void addPoint(double x, double y, double speed){
		segments.add(new PathSegment(lastPoint.getX(), lastPoint.getY(), x, y, speed));
		lastPoint = new Translation2d(x, y);		
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
			System.out.println(distToClosest + "  " + distToNext);
			if(distToClosest > distToNext){
				System.out.println("removed");
				segments.remove(0);
				closestPoint = closestNextPoint;
				closestToRobot = closestNextToRobot;
			} else {
				break;
			}
		}	
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
