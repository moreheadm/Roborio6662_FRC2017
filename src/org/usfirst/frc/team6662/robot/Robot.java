package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Robot extends IterativeRobot {
	RobotDrive driveTrain = new RobotDrive(0, 1, 2, 3);
	Joystick joystick = new Joystick(0);
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	Compressor compressor = new Compressor(0);
	DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);
	Victor liftMotor = new Victor(4);
	Gyro gyro;
	
	@Override
	public void robotInit() {
		gyro = new ADXRS450_Gyro();
	  	compressor.setClosedLoopControl(true);
	//	CameraServer.getInstance().startAutomaticCapture();
	}
	
	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}
	
	@Override
	public void teleopInit() {
		gyro.calibrate();
	}

	@Override
	public void testPeriodic() {
	}
}

