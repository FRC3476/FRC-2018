package org.usfirst.frc.team3476.utility.control.motion;

import org.usfirst.frc.team3476.utility.math.Translation2d;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class TrajectoryUtil {

	private TrajectoryUtil() {
	}
	
	static public Path generatePathFromWaypoints(double speed, Waypoint...points) {
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
		Trajectory trajectory = Pathfinder.generate(points, config);
		Path generatedPath = new Path(new Translation2d(trajectory.segments[0].x, trajectory.segments[0].y));
		for(int i = 1; i < trajectory.segments.length; i++) {
			generatedPath.addPoint(trajectory.segments[i].x, trajectory.segments[i].y, speed);
		}
		return generatedPath;
	}
}
