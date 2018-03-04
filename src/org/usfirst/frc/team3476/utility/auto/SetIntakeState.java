package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;

public class SetIntakeState implements AutoCommand {
	
	private IntakeState state;
	
	public SetIntakeState(IntakeState state) {
		this.state = state;
	}
	
	@Override
	public void run() {
		Intake.getInstance().setIntake(state);
	}
}
