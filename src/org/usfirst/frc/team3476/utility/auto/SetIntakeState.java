package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;

public class SetIntakeState extends AutoCommand {
	
	private IntakeState state;
	private boolean isBlocking = false;
	
	public SetIntakeState(IntakeState state) {
		this.state = state;
	}
	

	@Override
	public void start() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
}
