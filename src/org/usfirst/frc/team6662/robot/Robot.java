package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	RobotDrive driveTrain = new RobotDrive(0, 1, 2, 3);
	Joystick joystick = new Joystick(0);
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	Compressor compressor = new Compressor(0);
	DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);
	
	@Override
	public void robotInit() {
		compressor.setClosedLoopControl(true);
	}
	
	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopPeriodic() {
		boolean trigger = joystick.getRawButton(1);
		
		double rotate = joystick.getX();
		
		double motorCurrent0 = pdp.getCurrent(0),
			   motorCurrent1 = pdp.getCurrent(1),
			   motorCurrent2 = pdp.getCurrent(14),
			   motorCurrent3 = pdp.getCurrent(15);
		
		double compressorCurrent = compressor.getCompressorCurrent();
		
		/** SMARTDASHBOARD **/
		
		SmartDashboard.putNumber("Motor 0 Current", motorCurrent0);
		SmartDashboard.putNumber("Motor 1 Current", motorCurrent1);
		SmartDashboard.putNumber("Motor 2 Current", motorCurrent2);
		SmartDashboard.putNumber("Motor 3 Current", motorCurrent3);
		
		SmartDashboard.putNumber("Compressor Current", compressorCurrent);
		
		/** CONTROLLER **/
		
		if (trigger) {
			solenoid.set(DoubleSolenoid.Value.kForward);
		}
		else {
			solenoid.set(DoubleSolenoid.Value.kReverse);
		}
		
		if (joystick.getY() <= 0)
			rotate = -rotate;
		
		driveTrain.arcadeDrive(-joystick.getY(), rotate);
	}

	@Override
	public void testPeriodic() {
	}
}

