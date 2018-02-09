package org.usfirst.frc.team3476.subsystem;

import org.usfirst.frc.team3476.robot.Constants;
import org.usfirst.frc.team3476.utility.math.Rotation;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Arm {

	private TalonSRX armTalon;
	public final double HORIZONTAL = 100000000, DOWN = 0; //Update with real values
	
	private static final Arm instance = new Arm();
	
	private Arm() {
		armTalon = new TalonSRX(Constants.ArmId);
		armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	}
	
	public static Arm getInstance() {
		return instance;
	}
	
	public void setArmAngle(Rotation angle)	{
		
	}
	//undecided

}
