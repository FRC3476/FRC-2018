package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
	
	private Solenoid intakeSolenoid30Psi;
	private Solenoid intakeSolenoid60Psi;
	private LazyTalonSRX intakeMotor1;
	private LazyTalonSRX intakeMotor2;
	private static Intake intakeInstance = new Intake();
	
	public enum SolenoidState
	{
		OPEN,
		CLAMP,
		INTAKING
	}
	
	public enum IntakeState
	{
		INTAKE,
		OUTTAKE,
		GRIP,
		OPEN
	}
	
	private Intake()
	{
		intakeMotor1 = new LazyTalonSRX(Constants.Intake1Id);
		intakeMotor2 = new LazyTalonSRX(Constants.Intake2Id);
		intakeSolenoid30Psi = new Solenoid(Constants.IntakeSolenoid30PsiId);
		intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoid60PsiId);
	}
	
	public static Intake getInstance()
	{
		return intakeInstance;
	}
	
	public void setIntake(IntakeState state)
	{
		switch(state)
		{
		case INTAKE:
			intakeMotor1.set(ControlMode.PercentOutput, -.45);
			intakeMotor2.set(ControlMode.PercentOutput, -.45);
			setIntakeSolenoid(SolenoidState.INTAKING);
			break;
		case OUTTAKE:
			intakeMotor1.set(ControlMode.PercentOutput, .5);
			intakeMotor2.set(ControlMode.PercentOutput, .5);
			break;
		case GRIP:
			setIntakeSolenoid(SolenoidState.CLAMP);
			break;
		case OPEN:
			setIntakeSolenoid(SolenoidState.OPEN);
			break;
		}
	}
	
	private void setIntakeSolenoid(SolenoidState state)
	{
		switch(state)
		{
		case OPEN:
			intakeSolenoid30Psi.set(true);
			intakeSolenoid60Psi.set(true);
			break;
		case CLAMP:
			intakeSolenoid30Psi.set(false);
			intakeSolenoid60Psi.set(false);
		case INTAKING:
			intakeSolenoid30Psi.set(true);
			intakeSolenoid60Psi.set(false);
		}
	}
}
