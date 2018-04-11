package org.usfirst.frc.team3476.utility.auto;

import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team3476.utility.control.motion.Path;

public abstract class PathDatabase {
	
	public Path getPath(String name) {
		
		return null;
	}
	
	public void updatePath(String name, Path newPath) {
		try {
			FileWriter writer = new FileWriter("home/lvuser/auto" + name + ".csv");
			
		} catch (IOException e) {
			e.printStackTrace();
		}
				
	}
}
