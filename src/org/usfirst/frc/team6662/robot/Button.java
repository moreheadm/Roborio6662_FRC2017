package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Button {
	
	private boolean pressed, on;
	private Joystick joystick;
	private int buttonId;
	
	public Button (Joystick joystick, int buttonId) {
		this.joystick = joystick;
		this.buttonId = buttonId;
		on = false;
		pressed = false;
	}
	
	public boolean update () {
		boolean pressed = joystick.getRawButton(buttonId);
		boolean changed = pressed != this.pressed;
		this.pressed = pressed;
		
		if (changed) {
			on = !on;
		}
		
		return changed;
	}
	
	public boolean isOn () {
		return on;
	}
	
	public boolean isPressed () {
		return pressed;
	}
}
