package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;
import org.usfirst.frc.team3476.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends Threaded {

	private Solenoid intakeSolenoid30Psi;
	private Solenoid intakeSolenoid60Psi;
	private LazyTalonSRX intakeMotor1;
	private LazyTalonSRX intakeMotor2;
	private IntakingState intakeState;
	private BiasState biasState;
	private double biasTimer;
	private static Intake intakeInstance = new Intake();
	//private DigitalInput cubeSwitch = new DigitalInput(Constants.CubeSwitchId);
	
	public enum SolenoidState
	{
		OPEN,
		CLAMP,
		INTAKING
	}

	public enum IntakeState {
		INTAKE, OUTTAKE, GRIP, OPEN, OUTTAKE_FAST, OUTTAKE_FASTEST, INTAKE_OPEN
	}
	
	private enum IntakingState {
		INTAKE, MANUAL
	}
	
	private enum BiasState {
		LEFT, RIGHT
	}

	private Intake() {
		intakeMotor1 = new LazyTalonSRX(Constants.Intake1Id);
		intakeMotor2 = new LazyTalonSRX(Constants.Intake2Id);
		intakeSolenoid30Psi = new Solenoid(Constants.IntakeSolenoid30PsiId);
		intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoid60PsiId);
		intakeState = IntakingState.MANUAL;
		biasState = biasState.LEFT;
	}

	public static Intake getInstance() {
		return intakeInstance;
	}
	/*
	public boolean getCubeSwitch()
	{
		return !cubeSwitch.get();
	}
	*/
	public void setIntake(IntakeState state)
	{
		switch(state)
		{
		case INTAKE:
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
		
		if(state == IntakeState.INTAKE){
			synchronized(this){
				intakeState = IntakingState.INTAKE;
			}
		} else {
			synchronized(this){
				intakeState = IntakingState.MANUAL;
			}
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

	@Override
	public void update() {
		IntakingState snapState;
		synchronized(this) {
			snapState = intakeState;			
		}
		
		switch(snapState){
			case INTAKE:
				double currentRight = intakeMotor1.getOutputCurrent();
				double currentLeft = intakeMotor2.getOutputCurrent();
				System.out.println(currentLeft + "   " + currentRight);
				double powerLeft = OrangeUtility.coercedNormalize(currentLeft, 1.5, 20, 0.3, 0.7);
				double powerRight = OrangeUtility.coercedNormalize(currentRight, 1.5, 20, 0.3, 0.7);
				double bias = 0;
				if(getCurrent() > 20) {
					bias = OrangeUtility.coercedNormalize(getCurrent(), 20, 40, 0.1, 0.3);
					if(Timer.getFPGATimestamp() - biasTimer > 1) {
						swapBias();
						biasTimer = Timer.getFPGATimestamp();
					}
				} else {
					biasTimer = Timer.getFPGATimestamp();
				}
				if(biasState == BiasState.RIGHT) {
					bias *= -1;
				}

				intakeMotor1.set(ControlMode.PercentOutput, -powerRight + bias);
				intakeMotor2.set(ControlMode.PercentOutput, -powerLeft - bias);
				break;
			case MANUAL:
				break;
		}
	}
	
	synchronized private void swapBias(){
		if(biasState == BiasState.LEFT) {
			biasState = BiasState.RIGHT;
		} else {
			biasState = BiasState.LEFT;
		}
	}
}
