package org.usfirst.frc.team3476.utility.auto;

import org.usfirst.frc.team3476.subsystem.Intake;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.subsystem.Intake.SolenoidState;

public class SetIntakeState extends AutoCommand {

	private IntakeState intakeState;
	private SolenoidState solenoidState;

	public SetIntakeState(IntakeState intakeState, SolenoidState solenoidState) {
		this.intakeState = intakeState;
		this.solenoidState = solenoidState;
	}

	@Override
	public void start() {
		System.out.println("Set Intake State");
		Intake.getInstance().setIntake(intakeState, solenoidState);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
