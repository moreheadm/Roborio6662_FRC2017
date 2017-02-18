package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Robot extends IterativeRobot {
	RobotDrive drive = new RobotDrive(0, 1, 2, 3);
	Joystick joystick = new Joystick(0);
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	Compressor compressor = new Compressor(0);
	DoubleSolenoid shifter = new DoubleSolenoid(0, 1);
	Solenoid gearage = new Solenoid(2);
	Victor liftMotor = new Victor(4);
	Gyro gyro = new ADXRS450_Gyro();
	Encoder leftWheelEncoder = new Encoder(0, 1);
	Encoder rightWheelEncoder = new Encoder(2, 3);
	Drivetrain drivetrain = new Drivetrain(drive, shifter,
			leftWheelEncoder, rightWheelEncoder);
	
	Button driveModeButton, driveShiftButton, liftStopButton;
	
	@Override
	public void robotInit() {
		
		driveModeButton = new Button(joystick, 11);
		driveShiftButton = new Button (joystick, 12);
		liftStopButton = new Button (joystick, 10);
		
	  	compressor.setClosedLoopControl(true);
	//	CameraServer.getInstance().startAutomaticCapture();
	  	gyro.calibrate();
	  	
	}
	
	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}
	
	@Override
	public void teleopInit() {
		
	}

	@Override
	public void teleopPeriodic() {
		boolean trigger        = joystick.getRawButton(1),
				overrideButton = joystick.getRawButton(9);
			
		
		//double rotate = joystick.getX(),
		double angle = gyro.getAngle();
		
		double throttle = joystick.getThrottle();
		
		double motor0Current    = pdp.getCurrent(0),
			   motor1Current    = pdp.getCurrent(1),
			   motor2Current    = pdp.getCurrent(14),
			   motor3Current    = pdp.getCurrent(15),
			   liftMotorCurrent = pdp.getCurrent(6);
		
		double compressorCurrent = compressor.getCompressorCurrent();
		
		final double safeLiftCurrent = 35;
		
		boolean liftMotorShutoff  = false,
				liftMotorOverride = false,
				stopLift          = false;
		
		/******** SMARTDASHBOARD ********/
		
		/** MOTORS **/
		
		SmartDashboard.putNumber("Motor 0 Current", motor0Current);
		SmartDashboard.putNumber("Motor 1 Current", motor1Current);
		SmartDashboard.putNumber("Motor 2 Current", motor2Current);
		SmartDashboard.putNumber("Motor 3 Current", motor3Current);
		
		SmartDashboard.putNumber("Lift Motor Current", liftMotorCurrent);
		
		/** PNEUMATICS **/
		
		SmartDashboard.putNumber("Compressor Current", compressorCurrent);
		
		/** GYROSCOPE **/
		
		SmartDashboard.putNumber("Gyroscope Angle", angle);
		
		/******** CONTROLS ********/
		
		/** PNEUMATICS **/
		SmartDashboard.putBoolean("Gearage Open", trigger );
		if (trigger) { gearage.set(true); }
		else { gearage.set(false); }
		
		/** LIFT **/
		
		
		//if () { stopLift = !stopLift; }
		
		//if(overrideButton) { liftMotorOverride = true; }
		
		liftStopButton.update();
		
		if((!liftMotorShutoff || overrideButton /*liftMotorOverride*/) && !liftStopButton.isOn()) {	
			liftMotor.set((throttle + 1.0) / 2);
		} else {
			liftMotor.set(0);
		}
		
		if(liftMotorCurrent > safeLiftCurrent) { liftMotorShutoff = true; }
		
		/** DRIVE **/
		
		driveModeButton.update();
		driveShiftButton.update();
		
		drivetrain.teleOpDrivetrain(joystick, driveModeButton, driveShiftButton);
	}

	@Override
	public void testPeriodic() {
	}
}

