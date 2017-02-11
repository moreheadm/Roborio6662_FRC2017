package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Robot extends IterativeRobot {
	RobotDrive driveTrain = new RobotDrive(0, 1, 2, 3);
	Joystick joystick = new Joystick(0);
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	Compressor compressor = new Compressor(0);
	DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);
	Victor liftMotor = new Victor(4);
	Accelerometer accel = /*new ADXL362(SPI.Port.kOnboardCS0, Accelerometer.Range.k4G);*/ new BuiltInAccelerometer(Accelerometer.Range.k4G);
	Gyro gyro;
	
	int loop;
	double lastGyroAngle;
	double lastTime;
	double currTime;
	double averageGyroDrift;
	
	@Override
	public void disabledInit() {
		gyro.reset();
		lastGyroAngle = gyro.getAngle();
		
		averageGyroDrift = 0;
		loop = 0;
	}
	
	
	@Override
	public void disabledPeriodic() {
		double currGyroAngle = gyro.getAngle();
		//update time
		if (loop < 20) {
			averageGyroDrift += ((currGyroAngle - lastGyroAngle) / (currTime - lastTime)) / 20.0;
		}
			
		
	}
	
	@Override
	public void robotInit() {
		gyro = new ADXRS450_Gyro();
		compressor.setClosedLoopControl(true);
	//	CameraServer.getInstance().startAutomaticCapture();
		loop = 0;
	}
	
	@Override
	public void autonomousInit() {
		loop = 0;
	}

	@Override
	public void autonomousPeriodic() {
		
		loop++;
	}
	
	@Override
	public void teleopInit() {
		gyro.calibrate();
		loop = 0;
	}

	@Override
	public void teleopPeriodic() {
		boolean trigger = joystick.getRawButton(1);
		double throttle = joystick.getThrottle();
		
		double rotate = joystick.getX();
		
		double angle = gyro.getAngle();
		
		double accelX = accel.getX();
		double accelY = accel.getY();
		double accelZ = accel.getZ();
		
		double motorCurrent0 = pdp.getCurrent(0),
			   motorCurrent1 = pdp.getCurrent(1),
			   motorCurrent2 = pdp.getCurrent(14),
			   motorCurrent3 = pdp.getCurrent(15);
		
		double compressorCurrent = compressor.getCompressorCurrent();
		
		/******** SMARTDASHBOARD ********/
		
		/** MOTORS **/
		
		SmartDashboard.putNumber("Motor 0 Current", motorCurrent0);
		SmartDashboard.putNumber("Motor 1 Current", motorCurrent1);
		SmartDashboard.putNumber("Motor 2 Current", motorCurrent2);
		SmartDashboard.putNumber("Motor 3 Current", motorCurrent3);
		
		/** PNEUMATICS **/
		
		SmartDashboard.putNumber("Compressor Current", compressorCurrent);
		
		/** GYROSCOPE **/
		
		SmartDashboard.putNumber("Gyroscope Angle", angle);
		SmartDashboard.putNumber("Accel X", accelX);
		SmartDashboard.putNumber("Accel Y", accelY);
		SmartDashboard.putNumber("Accel Z", accelZ);
		
		/******** CONTROLLER ********/
		
		/** PNEUMATICS **/
		
		if (trigger) {
			solenoid.set(DoubleSolenoid.Value.kForward);
		}
		else {
			solenoid.set(DoubleSolenoid.Value.kReverse);
		}
		
		/** LIFT **/
		
		liftMotor.set(throttle);
		
		/** DRIVE **/
		
		if (joystick.getY() <= 0)
			rotate = -rotate;
		
		driveTrain.arcadeDrive(joystick.getY(), rotate);
		
		loop++;
	}

	@Override
	public void testPeriodic() {
	}
}

