package org.usfirst.frc.team3476.utility.auto;

import java.util.HashMap;
import java.util.List;

import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoPathGenerator {

	private HashMap<String, Path> pathTable;
	private Path robotPath;
	private List<Translation2d> cubeLocations;
	
	public enum PathOption {
		Scale, Switch, Optimal, Forward
	}

	public AutoPathGenerator() {
		// Add paths to table;
	}

	public Path generate(String gameMsg) {
		Translation2d switchLocation, scaleLocation;
		if(gameMsg.charAt(0) == 'l') {
			
		} else {
			
		}
		
		if(gameMsg.charAt(1) == 'l') {
			
		} else {
			
		}
		
		
		return null;
	}
}
