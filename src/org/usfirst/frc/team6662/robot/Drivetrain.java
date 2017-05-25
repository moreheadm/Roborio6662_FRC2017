package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
	private final double MAX_LOW_SPEED = 8.00, MAX_HIGH_SPEED = 16.00,
						UPSHIFT_THRESH = 5.00, DOWNSHIFT_THRESH = 10.00;
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
		
		
		this.rightEncoder.setDistancePerPulse(- 6. * Math.PI * 3.0 / 12.0 / 360.);
		this.leftEncoder.setDistancePerPulse(6. * Math.PI * 3.0 / 12.0 / 360.);
		this.rightEncoder.setSamplesToAverage(7);
		this.leftEncoder.setSamplesToAverage(7);
		
		//automatic = true;
		shiftState = LOW_GEAR;
		shifter.set(DoubleSolenoid.Value.kForward);
	}
	
	public void shift() {
		if (shiftState == HIGH_GEAR) {
			shifter.set(DoubleSolenoid.Value.kForward);
			shiftState = LOW_GEAR;
		} else {
			shifter.set(DoubleSolenoid.Value.kReverse);
			shiftState = HIGH_GEAR;
		}
	}
	
	public void shiftHigh () {
		shifter.set(DoubleSolenoid.Value.kReverse);
		shiftState = HIGH_GEAR;
	}
	
	public void shiftLow () {
		shifter.set(DoubleSolenoid.Value.kForward);
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
		double leftSpeed = leftEncoder.getRate();
		double rightSpeed = rightEncoder.getRate();
		double speed = (leftSpeed + rightSpeed) / 2.0;
		SmartDashboard.putNumber("Left Wheel Speed", leftSpeed);
		SmartDashboard.putNumber("Right Wheel Speed", rightSpeed);
		SmartDashboard.putNumber("Speed", speed);
		
		if (shiftButton.isOn()) {
			shiftHigh();
		} else {
			shiftLow();
		}
		
		dt.arcadeDrive(-joystick.getY(), -joystick.getZ());
	}
	
	public double getLeftDistance () {
		return leftEncoder.getDistance();
	}
	
	public double getRightDistance () {
		return rightEncoder.getDistance();
	}
	
	public void drive (double magnitude, double curve) {
		dt.drive(magnitude, curve);
	}
	
	double pow1, pow2;
	public void automaticDrivetrain (Joystick joystick) {
		double leftSpeed = leftEncoder.getRate();
		double rightSpeed = rightEncoder.getRate();
		//double speed = Math.abs((leftSpeed + rightSpeed) / 2.0); // TODO: Maybe this can be improved
		
		
		
		double speed = Math.abs((pow1 + pow2) / (2. * Math.max(Math.abs(pow1), Math.abs(pow2)))) * leftSpeed;
		SmartDashboard.putNumber("Left Wheel Speed", leftSpeed);
		SmartDashboard.putNumber("Right Wheel Speed", rightSpeed);
		SmartDashboard.putNumber("Speed", speed);
		

		//joystick x, y is reversed
		double jsY = -joystick.getY(), jsX = -joystick.getZ();

		
		//squared input target speed
		double targetSpeed = MAX_HIGH_SPEED * (jsY * jsY - jsX * jsX);
		
		SmartDashboard.putNumber("targetSpeed", targetSpeed);

		if (Math.abs(jsX) > 0.15) {
			shiftLow();
		} else if (targetSpeed > MAX_LOW_SPEED) {
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
			/*double rotate;
			if (jsX >= 0.0) {
				rotate = jsX * jsX;
			} else {
				rotate = - jsX * jsX;
			}*/
			dt.arcadeDrive(jsY, jsX);
		} else {
			dt.arcadeDrive(jsY * Math.sqrt(MAX_HIGH_SPEED / MAX_LOW_SPEED), 
					jsX);
		}
		
		double moveValue = jsY;
		double rotateValue = jsX;
		
	   	if (moveValue >= 0.0) {
	        moveValue = moveValue * moveValue;
	      } else {
	        moveValue = -(moveValue * moveValue);
	      }
	    if (rotateValue >= 0.0) {
	      rotateValue = rotateValue * rotateValue;
	      } else {
	        rotateValue = -(rotateValue * rotateValue);
	      }
	    

	    if (moveValue > 0.0) {
	      if (rotateValue > 0.0) {
	        pow1 = moveValue - rotateValue;
	        pow2 = Math.max(moveValue, rotateValue);
	      } else {
	        pow1 = Math.max(moveValue, -rotateValue);
	        pow2 = moveValue + rotateValue;
	      }
	    } else {
	      if (rotateValue > 0.0) {
	        pow1 = -Math.max(-moveValue, rotateValue);
	        pow2 = moveValue + rotateValue;
	      } else {
	        pow1 = moveValue - rotateValue;
	        pow2 = -Math.max(-moveValue, -rotateValue);
	      }
	    }
	}
	
}
