package org.usfirst.frc.team3476.utility.auto;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoRoutine implements Runnable {

	private ArrayList<AutoCommand> routine = new ArrayList<AutoCommand>();
	
	synchronized public void addCommand(AutoCommand...commands) {
		routine.addAll(Arrays.asList(commands));
	}
	
	synchronized public void run() {
		for(AutoCommand command : routine) {
			command.run();
		}
	}
}
