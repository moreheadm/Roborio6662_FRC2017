package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
	private final double MAX_LOW_SPEED = 6.56, MAX_HIGH_SPEED = 17.89,
						UPSHIFT_THRESH = 5.00, DOWNSHIFT_THRESH = 8.00;
	private final boolean HIGH_GEAR = true;
	private final boolean LOW_GEAR = false;
	
	private RobotDrive dt;
	private Encoder leftEncoder, rightEncoder;
	private DoubleSolenoid shifter;
	private boolean shiftState;
	//private boolean automatic;
	
	public Drivetrain (RobotDrive dt, DoubleSolenoid shifter, 
					Encoder leftEncoder, Encoder rightEncoder) {
		this.dt = dt;
		this.rightEncoder = rightEncoder;
		this.leftEncoder = leftEncoder;
		this.shifter = shifter;
		
		
		this.rightEncoder.setDistancePerPulse(0.5 * Math.PI * 3.0 / 12.0);
		this.leftEncoder.setDistancePerPulse(0.5 * Math.PI * 3.0 / 12.0);
		
		//automatic = true;
		shiftState = LOW_GEAR;
		shifter.set(DoubleSolenoid.Value.kReverse);
	}
	
	private void shift() {
		if (shiftState == HIGH_GEAR) {
			shifter.set(DoubleSolenoid.Value.kReverse);
			shiftState = LOW_GEAR;
		} else {
			shifter.set(DoubleSolenoid.Value.kForward);
			shiftState = HIGH_GEAR;
		}
	}
	
	private void shiftHigh () {
		shifter.set(DoubleSolenoid.Value.kForward);
		shiftState = HIGH_GEAR;
	}
	
	private void shiftLow () {
		shifter.set(DoubleSolenoid.Value.kReverse);
		shiftState = LOW_GEAR;
	}
	
	public void teleOpDrivetrain (Joystick joystick, Button modeButton, Button shiftButton) {
		if (modeButton.isOn()) {
			SmartDashboard.putBoolean("Manual Mode", true);
			manualDrivetrain (joystick, shiftButton);
		} else {
			SmartDashboard.putBoolean("Manual Mode", false);
			automaticDrivetrain (joystick);
		}
		
		SmartDashboard.putBoolean("High Gear", shiftState == HIGH_GEAR);
	}
	
	public void manualDrivetrain (Joystick joystick, Button shiftButton) {
		if (shiftButton.isOn()) {
			shiftHigh();
		} else {
			shiftLow();
		}
		
		dt.arcadeDrive(joystick);
	}
	
	public void automaticDrivetrain (Joystick joystick) {
		double leftSpeed = leftEncoder.getRate();
		double rightSpeed = rightEncoder.getRate();
		double speed = (leftSpeed + rightSpeed) / 2.0; // TODO: Maybe this can be improved
		
		SmartDashboard.putNumber("Left Wheel Speed", leftSpeed);
		SmartDashboard.putNumber("Right Wheel Speed", rightSpeed);
		SmartDashboard.putNumber("Speed", speed);
		

		double jsY = joystick.getY(), jsX = joystick.getX();

		
		//squared input target speed
		double targetSpeed = MAX_HIGH_SPEED * (jsY * jsY - jsX * jsX);
		
		
		
		if (targetSpeed > MAX_LOW_SPEED) {
			if (targetSpeed > DOWNSHIFT_THRESH) {
				if (speed > UPSHIFT_THRESH) {
					shiftHigh();
				} else {
					shiftLow();
				}
			}
		} else {
			if (speed < DOWNSHIFT_THRESH) {
				shiftLow();
			} else {
				shiftHigh();
			}
		}
		
		if (shiftState == HIGH_GEAR) {
			dt.arcadeDrive(jsY, jsX);
		} else {
			dt.arcadeDrive(jsY * Math.sqrt(MAX_HIGH_SPEED / MAX_LOW_SPEED), 
					jsX * jsX);
		}
	}
}
