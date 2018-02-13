package org.usfirst.frc.team3476.robot;

import java.util.HashMap;

import org.usfirst.frc.team3476.utility.control.Path;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoPathGenerator {
	
	HashMap<String, Path> pathTable;
	
	public AutoPathGenerator() {
		//Add paths to table;
	}
	
	public Path generate() {
		String colors = DriverStation.getInstance().getGameSpecificMessage();
		return pathTable.get(colors.substring(0, 2));
	}
}
