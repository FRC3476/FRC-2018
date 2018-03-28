package org.usfirst.frc.team3476.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class stores the int sent back from the Driver Station and uses it to check for rising or falling edges
 */
public class Controller extends Joystick {

	public static class Xbox {
		public static int A = 1;
		public static int B = 2;
		public static int X = 3;
		public static int Y = 4;
		public static int LeftBumper = 5;
		public static int RightBumper = 6;
		public static int Back = 7;
		public static int Start = 8;
		public static int LeftClick = 9;
		public static int RightClick = 10;

		public static int LeftX = 0;
		public static int LeftY = 1;
		public static int LeftTrigger = 2;
		public static int RightTrigger = 3;
		public static int RightX = 4;
		public static int RightY = 5;
	}

	/*
	 * The Driver Station sends back an int(32 bits) for buttons
	 * Shifting 1 left (button - 1) times and ANDing it with
	 * int sent from the Driver Station will either give you
	 * 0 or a number not zero if it is true
	 */
	private int oldButtons;
	private int currentButtons;
	private int buttonCount;
	private int axisCount;
	private double[] oldAxis;

	public Controller(int port) {
		super(port);
		axisCount = DriverStation.getInstance().getStickAxisCount(port);
		buttonCount = DriverStation.getInstance().getStickButtonCount(getPort());
		oldAxis = new double[axisCount];
	}

	/**
	 * Only works if update() is called in each iteration
	 *
	 * @param button
	 *            Joystick button ID
	 * @return
	 * 		Falling edge state of the button
	 */
	public boolean getFallingEdge(int button) {
		if (button <= buttonCount) {
			boolean oldVal = ((0x1 << (button - 1)) & oldButtons) != 0;
			boolean currentVal = ((0x1 << (button - 1)) & currentButtons) != 0;
			if (oldVal == true && currentVal == false) {
				return true;
			} else {
				return false;
			}
		}
		return false;
	}

	/**
	 * Only works if update() is called in each iteration
	 *
	 * @param button
	 *            Joystick button ID
	 * @return
	 * 		Rising edge state of the button
	 */
	public boolean getRisingEdge(int button) {
		if (button <= buttonCount) {
			boolean oldVal = ((0x1 << (button - 1)) & oldButtons) != 0;
			boolean currentVal = ((0x1 << (button - 1)) & currentButtons) != 0;

			if (oldVal == false && currentVal == true) {
				return true;
			} else {
				return false;
			}
		}
		return false;
	}
	
	public boolean getRisingEdge(int axis, double threshold) {
		if(axis <= axisCount) {
			boolean oldVal = oldAxis[axis] > threshold;
			boolean currentVal = super.getRawAxis(axis) > threshold;
			if (oldVal == false && currentVal == true) {
				return true;
			} else {
				return false;
			}
		} 
		return false;
	}
	
	public boolean getFallingEdge(int axis, double threshold) {
		if(axis <= axisCount) {
			boolean oldVal = oldAxis[axis] > threshold;
			boolean currentVal = super.getRawAxis(axis) > threshold;
			if (oldVal == true && currentVal == false) {
				return true;
			} else {
				return false;
			}
		} 
		return false;
	}

	/**
	 * This method needs to be called for each iteration of the teleop loop
	 */
	boolean next = false;

	public void update() {
		if (Math.random() > .999 || next)
		{
			this.setRumble(RumbleType.kLeftRumble, 1);
			this.setRumble(RumbleType.kRightRumble, 1);
			if (next = true) {
				next = false;
			} else {
				next = true;
			}
		} else {
			this.setRumble(RumbleType.kLeftRumble, 0);
			this.setRumble(RumbleType.kRightRumble, 0);
		}
		oldButtons = currentButtons;
		currentButtons = DriverStation.getInstance().getStickButtons(getPort());
		for(int i = 0; i < axisCount; i++){
			oldAxis[i] = super.getRawAxis(i);
		}
	}

}
