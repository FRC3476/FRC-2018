package org.usfirst.frc.team3476.utility.control.motion;

import java.util.ArrayList;
import java.util.Arrays;

import org.usfirst.frc.team3476.utility.math.Translation2d;

public class BezierCurve {
	
	public static class BezierPoint {
		
		private Translation2d start, prevTangent, nextTangent;
		private double speed;
		
		public BezierPoint(Translation2d start, Translation2d prevTangent, Translation2d nextTangent, double speed) {
			this.start = start;
			this.prevTangent = prevTangent;
			this.nextTangent = nextTangent;
			this.speed = speed;
		}
	}
	
	private ArrayList<BezierPoint> points;
	
	public BezierCurve(BezierPoint... points) {
		this.points = new ArrayList<BezierPoint>();
		this.points.addAll(Arrays.asList(points));
	}
	
	public void addPoints(BezierPoint... points) {
		this.points.addAll(Arrays.asList(points));
	}
	
	public Path computePath(double step) {
		if(points.size() > 2) {
			Path generatedPath = new Path(points.get(0).start);
			double startSpeed = points.get(0).speed;
			double endSpeed = points.get(1).speed;
			double diffSpeed = startSpeed - endSpeed;
			for(int i = 0; i < points.size() - 1; i++) {
				BezierPoint firstPoint = points.get(i - 1);
				BezierPoint secondPoint = points.get(i);
				Segment firstSeg = new Segment(firstPoint.start, firstPoint.nextTangent, 0);
				Segment secondSeg = new Segment(firstPoint.nextTangent, secondPoint.prevTangent, 0);
				Segment thirdSeg = new Segment(firstPoint.nextTangent, secondPoint.prevTangent, 0);
				for(double j = 0; j <= 1.0; j += step) {
					Translation2d A = firstSeg.getPointByPercentage(j);
					Translation2d B = secondSeg.getPointByPercentage(j);
					Translation2d C = thirdSeg.getPointByPercentage(j);
					Segment AB = new Segment(A, B, 0);
					Segment BC = new Segment(B, C, 0);
					Translation2d D = AB.getPointByPercentage(j);
					Translation2d E = BC.getPointByPercentage(j);
					Segment DE = new Segment(D, E, 0);				
					Translation2d F = DE.getPointByPercentage(j);
					generatedPath.addPoint(F, (diffSpeed * j) + startSpeed);				
				}			
			}
			return generatedPath;
		} else {
			//Print error
			return null;
		}
	}
}
