package org.usfirst.frc.team3476.utility.auto;

import java.util.HashMap;
import java.util.List;

import org.usfirst.frc.team3476.utility.control.Path;
import org.usfirst.frc.team3476.utility.math.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoRoutineGenerator {

	private Path robotPath;
	private List<Translation2d> cubeLocations;
	private PathOption wantedOption;
	private List<Translation2d> scalePath;
	private List<Translation2d> switchPath;
	
	public enum PathOption {
		SCALE, SWITCH, OPTIMAL, FORWARD, NONE
	}

	public AutoRoutineGenerator() {
		wantedOption = PathOption.SCALE;
		// Add paths to table;
	}

	public AutoRoutine generate(String gameMsg) {
		Translation2d switchLocation, scaleLocation;
		//Create switch and scale locations
		if(gameMsg.charAt(0) == 'l') {
			
		} else {
			
		}
		
		if(gameMsg.charAt(1) == 'l') {
			
		} else {
			
		}
		//Create routine
		AutoRoutine generatedRoutine = new AutoRoutine();
		switch(wantedOption) {
		case SCALE:
			break;
		case SWITCH:
			break;
		case OPTIMAL:
			break;
		case FORWARD:
			generatedRoutine.addCommand(new SetDrivePath(new Path(new Translation2d(100, 0)), false));
			break;
		case NONE:
			break;
		}		
		return generatedRoutine;
	}
	
	private Path flip(Path oldPath){
		
	}
}
