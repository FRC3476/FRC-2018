package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {

	private Solenoid intakeSolenoid30Psi;
	private Solenoid intakeSolenoid60Psi;
	private LazyTalonSRX intakeMotor1;
	private LazyTalonSRX intakeMotor2;
	private static Intake intakeInstance = new Intake();
	private DigitalInput cubeSwitch = new DigitalInput(Constants.CubeSwitchId);

	public enum SolenoidState {
		OPEN, CLAMP, INTAKING
	}

	public enum IntakeState {
		INTAKE, OUTTAKE, GRIP, OPEN, OUTTAKE_FAST, OUTTAKE_FASTEST, INTAKE_OPEN
	}

	private Intake() {
		intakeMotor1 = new LazyTalonSRX(Constants.Intake1Id);
		intakeMotor2 = new LazyTalonSRX(Constants.Intake2Id);
		intakeSolenoid30Psi = new Solenoid(Constants.IntakeSolenoid30PsiId);
		intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoid60PsiId);
	}

	public static Intake getInstance() {
		return intakeInstance;
	}

	public boolean getCubeSwitch() {
		return !cubeSwitch.get();
	}

	public void setIntake(IntakeState state) {
		switch (state) {
		case INTAKE:

			intakeMotor1.set(ControlMode.PercentOutput, -.7);
			intakeMotor2.set(ControlMode.PercentOutput, -.3);
			/*
			 * if(getCubeSwitch()){
			 * setIntakeSolenoid(SolenoidState.CLAMP);
			 * } else {
			 * setIntakeSolenoid(SolenoidState.OPEN);
			 * }
			 */
			setIntakeSolenoid(SolenoidState.INTAKING);
			break;
		case OUTTAKE:
			intakeMotor1.set(ControlMode.PercentOutput, .25);
			intakeMotor2.set(ControlMode.PercentOutput, .25);
			setIntakeSolenoid(SolenoidState.INTAKING);
			break;
		case OUTTAKE_FAST:
			intakeMotor1.set(ControlMode.PercentOutput, .5);
			intakeMotor2.set(ControlMode.PercentOutput, .5);
			break;
		case OUTTAKE_FASTEST:
			intakeMotor1.set(ControlMode.PercentOutput, 1);
			intakeMotor2.set(ControlMode.PercentOutput, 1);
			break;
		case GRIP:
			setIntakeSolenoid(SolenoidState.CLAMP);
			intakeMotor1.set(ControlMode.PercentOutput, 0);
			intakeMotor2.set(ControlMode.PercentOutput, 0);
			break;
		case OPEN:
			setIntakeSolenoid(SolenoidState.OPEN);
			intakeMotor1.set(ControlMode.PercentOutput, 0);
			intakeMotor2.set(ControlMode.PercentOutput, 0);
			break;
		case INTAKE_OPEN:
			setIntakeSolenoid(SolenoidState.OPEN);
			intakeMotor1.set(ControlMode.PercentOutput, -.15);
			intakeMotor2.set(ControlMode.PercentOutput, -.15);
			break;
		}
	}

	private void setIntakeSolenoid(SolenoidState state) {
		switch (state) {
		case OPEN:
			intakeSolenoid30Psi.set(true);
			intakeSolenoid60Psi.set(true);
			break;
		case CLAMP:
			intakeSolenoid30Psi.set(false);
			intakeSolenoid60Psi.set(false);
			break;
		case INTAKING:
			intakeSolenoid30Psi.set(true);
			intakeSolenoid60Psi.set(false);
			break;
		}
	}

	public double getCurrent() {
		return (intakeMotor1.getOutputCurrent() + intakeMotor2.getOutputCurrent()) / 2d;
	}

	public boolean isFinished() {
		return true;
	}
}
