package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.subsystem.Intake.IntakeState;
import org.usfirst.frc.team3476.subsystem.Intake.SolenoidState;
import org.usfirst.frc.team3476.utility.LazyTalonSRX;
import org.usfirst.frc.team3476.utility.OrangeUtility;
import org.usfirst.frc.team3476.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends Threaded {

	private Solenoid intakeSolenoid30Psi;
	private Solenoid intakeSolenoid60Psi;
	private LazyTalonSRX intakeMotor1;
	private LazyTalonSRX intakeMotor2;
	private DigitalInput cubeSensor = new DigitalInput(1);
	private IntakeState intakeState;
	private SolenoidState solenoidState;
	private BiasState biasState;
	private double biasTimer;
	private double autoGripTimeout;
	private static Intake intakeInstance = new Intake();
	// private DigitalInput cubeSwitch = new
	// DigitalInput(Constants.CubeSwitchId);

	public enum SolenoidState {
		OPEN, CLAMP, INTAKING, AUTO
	}

	public enum IntakeState {
		INTAKE, OUTTAKE, OUTTAKE_MIDDLE, OUTTAKE_FAST, OUTTAKE_FASTEST, NEUTRAL
	}

	private enum BiasState {
		REVERSE, NORMAL
	}

	private Intake() {
		intakeMotor1 = new LazyTalonSRX(Constants.Intake1Id);
		intakeMotor2 = new LazyTalonSRX(Constants.Intake2Id);
		intakeSolenoid30Psi = new Solenoid(Constants.IntakeSolenoid30PsiId);
		intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoid60PsiId);
		intakeState = IntakeState.NEUTRAL;
		solenoidState = SolenoidState.CLAMP;
		biasState = BiasState.NORMAL;
	}

	public static Intake getInstance() {
		return intakeInstance;
	}

	/*
	 * public boolean getCubeSwitch() { return !cubeSwitch.get(); }
	 */
	public void setIntake(IntakeState intakeState, SolenoidState solenoidState) {
		synchronized (this) {
			this.intakeState = intakeState;
			this.solenoidState = solenoidState;
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
		IntakeState snapIntake;
		SolenoidState snapSolenoid;
		synchronized (this) {
			snapIntake = intakeState;
			snapSolenoid = solenoidState;
		}

		switch (snapIntake) {
			case INTAKE:
				if(Constants.OldIntake) {

					double currentRight =  intakeMotor1.getOutputCurrent();
					double currentLeft = intakeMotor2.getOutputCurrent();
					// System.out.println(currentLeft + " " + currentRight);
					double powerLeft = OrangeUtility.coercedNormalize(currentLeft, 1.5, 20, 0.2, 1);
					double powerRight = OrangeUtility.coercedNormalize(currentRight, 1.5, 20, 0.2, 1);
					double bias = 0;
					/*
					 * if(!DriverStation.getInstance().isAutonomous()) {
					 * if(getCurrent() > 10) { if(biasState == BiasState.NORMAL &&
					 * Timer.getFPGATimestamp() - biasTimer > 0.8) { biasState =
					 * BiasState.REVERSE; biasTimer = Timer.getFPGATimestamp();
					 * System.out.println("its a reverse"); } if(biasState ==
					 * BiasState.REVERSE && Timer.getFPGATimestamp() - biasTimer >
					 * 0.5) { biasState = BiasState.NORMAL; biasTimer =
					 * Timer.getFPGATimestamp(); System.out.println("its a normal");
					 * } } else { biasTimer = Timer.getFPGATimestamp(); biasState =
					 * BiasState.NORMAL; } }
					 */
					if (biasState == BiasState.NORMAL) {
						intakeMotor1.set(ControlMode.PercentOutput, -powerRight);
						intakeMotor2.set(ControlMode.PercentOutput, -powerLeft);
					} else {
						intakeMotor1.set(ControlMode.PercentOutput, -powerRight);
						intakeMotor2.set(ControlMode.PercentOutput, -(-powerLeft));
					}
				} else {

					intakeMotor1.set(ControlMode.PercentOutput, -1);
					intakeMotor2.set(ControlMode.PercentOutput, -1);
				}
				break;
			case OUTTAKE:
				intakeMotor1.set(ControlMode.PercentOutput, .275);
				intakeMotor2.set(ControlMode.PercentOutput, .275);
				break;
			case OUTTAKE_MIDDLE:
				intakeMotor1.set(ControlMode.PercentOutput, .35);
				intakeMotor2.set(ControlMode.PercentOutput, .35);
				break;
			case OUTTAKE_FAST:
				intakeMotor1.set(ControlMode.PercentOutput, .5);
				intakeMotor2.set(ControlMode.PercentOutput, .5);
				break;
			case OUTTAKE_FASTEST:
				intakeMotor1.set(ControlMode.PercentOutput, 1);
				intakeMotor2.set(ControlMode.PercentOutput, 1);
				break;
			case NEUTRAL:
				intakeMotor1.set(ControlMode.PercentOutput, 0);
				intakeMotor2.set(ControlMode.PercentOutput, 0);
				break;
		}

		if (snapSolenoid == SolenoidState.AUTO) {
			if (!cubeSensor.get()) {
				autoGripTimeout = Timer.getFPGATimestamp();
				setIntakeSolenoid(SolenoidState.INTAKING);
			} else {
				if (Timer.getFPGATimestamp() - autoGripTimeout > 0.5) {
					setIntakeSolenoid(SolenoidState.OPEN);
				}
			}
		} else {
			setIntakeSolenoid(snapSolenoid);
		}
	}
}
