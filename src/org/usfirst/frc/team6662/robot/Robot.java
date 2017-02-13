package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	public void teleopPeriodic() {
		boolean trigger        = joystick.getRawButton(1),
				overrideButton = joystick.getRawButton(9),
				stopButton     = joystick.getRawButton(10),
				modeButton     = joystick.getRawButton(11);
		
		double rotate = joystick.getX(),
				angle = gyro.getAngle();
		
		double throttle = joystick.getThrottle();
		
		double motor0Current    = pdp.getCurrent(0),
			   motor1Current    = pdp.getCurrent(1),
			   motor2Current    = pdp.getCurrent(14),
			   motor3Current    = pdp.getCurrent(15),
			   liftMotorCurrent = pdp.getCurrent(6);
		
		double compressorCurrent = compressor.getCompressorCurrent();
		
		final double safeLiftCurrent = 35;
		
		boolean gameDrive         = false,
				liftMotorShutoff  = false,
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
		if (trigger) {
			solenoid.set(DoubleSolenoid.Value.kForward);
		}
		else {
			solenoid.set(DoubleSolenoid.Value.kReverse);
		}
		
		/** LIFT **/
		
		if(stopButton) {
			stopLift = !stopLift;
		}
		
		if(overrideButton) {
			liftMotorOverride = true;
		}
		
		if((!liftMotorShutoff || liftMotorOverride) && !stopLift) {
			liftMotor.set(throttle);
		}
		
		if(liftMotorCurrent > safeLiftCurrent) {
			liftMotorShutoff = true;
		}
		
		/** DRIVE **/
		
		if(modeButton) {
			gameDrive = gameDrive ? false : true;
		}
		
		if(gameDrive) {
			if (joystick.getY() <= 0) {
				rotate = -rotate;
			}
		}
		
		driveTrain.arcadeDrive(joystick.getY(), rotate);
	}

	@Override
	public void testPeriodic() {
	}
}

