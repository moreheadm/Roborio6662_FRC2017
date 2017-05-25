package org.usfirst.frc.team6662.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	Encoder leftWheelEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);
	Encoder rightWheelEncoder = new Encoder(2, 3);
	Drivetrain drivetrain = new Drivetrain(drive, shifter,
			leftWheelEncoder, rightWheelEncoder);
	
	Button driveModeButton, driveShiftButton, liftStopButton, calibrationButton;
	
	SendableChooser<Integer> autonRoutine = new SendableChooser<Integer>(); 
	
	@Override
	public void robotInit() {
		
		driveModeButton = new Button(joystick, 11);
		driveShiftButton = new Button (joystick, 12);
		liftStopButton = new Button (joystick, 10);
		calibrationButton = new Button (joystick, 7);
		
	  	compressor.setClosedLoopControl(true);
	//	CameraServer.getInstance().startAutomaticCapture();
	  	gyro.calibrate();
	  
	  	
		autonRoutine.addObject("Blue Boiler or Red Loading", 0);
		autonRoutine.addObject("Red Boiler or Blue Loading", 1);
		autonRoutine.addObject("Middle", 2);
		autonRoutine.addDefault("Straight", 3);
		SmartDashboard.putData("Autonomous Routine", autonRoutine);
	}
	
	private double startRight, startLeft;
	
	@Override
	public void autonomousInit() {
		
		startRight = drivetrain.getRightDistance();
		startLeft = drivetrain.getLeftDistance();
		
		switch (autonRoutine.getSelected().intValue()) {
		case 0:
			turnRightAuto();
			break;
		case 1:
			turnLeftAuto();
			break;
		case 2:
			middleAuto();
			break;
		default:
			straightAuto();
			break;
		}
				
		
	}
	
	@Override
	public void disabledInit() {
		SmartDashboard.putBoolean("Calibrated", false);
	}
	
	private boolean calibrated = false;
	@Override
	public void disabledPeriodic () {
		calibrationButton.update();
		if (!calibrated && calibrationButton.isOn()) {
			SmartDashboard.putBoolean("Calibrated", true);
			gyro.calibrate();
			calibrated = true;
		}
		Timer.delay(0.02);
	}
	
	private void turnRightAuto() {
		while (isAutonomous() && isEnabled()) {
			drivetrain.shiftLow();
			drivetrain.drive(1, 0);
			if (drivetrain.getRightDistance() - startRight > (100. - 13) / 12. ||
					drivetrain.getLeftDistance() - startLeft > (100. - 13) / 12.) {
				break;
			}
			Timer.delay(0.02);
		}
		
		double startAngle = gyro.getAngle();
		while (isAutonomous () && isEnabled()) {
			drivetrain.drive(0.5, 1);
			if (gyro.getAngle() - startAngle < -60.) {
				break;
			}
			Timer.delay(0.02);
		}
		
		startRight = drivetrain.getRightDistance();
		startLeft = drivetrain.getLeftDistance();
		while (isAutonomous() && isEnabled()) {
			drivetrain.shiftLow();
			drivetrain.drive(1, 0);
			if (drivetrain.getRightDistance() - startRight > 100. / 12. ||
					drivetrain.getLeftDistance() - startLeft > 100. / 12.) {
				break;
			}
			Timer.delay(0.02);
		}
		
		
		startRight = drivetrain.getRightDistance();
		startLeft = drivetrain.getLeftDistance();
		gearage.set(true);
		Timer.delay(1);
		while (isAutonomous() && isEnabled()) {
			drivetrain.drive(-1,0);
			if (drivetrain.getRightDistance() - startRight < - 100. / 12. ||
					drivetrain.getLeftDistance() - startLeft < - 100. / 12.) {
				break;
			}
			Timer.delay(0.02);
		}
		gearage.set(false);
	}
	
	private void turnLeftAuto() {
		while (isAutonomous() && isEnabled()) {
			drivetrain.shiftLow();
			drivetrain.drive(1, 0);
			if (drivetrain.getRightDistance() - startRight > (110. - 13.) / 12. ||
					drivetrain.getLeftDistance() - startLeft > (110. - 13.) / 12.) {
				break;
			}
			Timer.delay(0.02);
		}
		
		double startAngle = gyro.getAngle();
		while (isAutonomous () && isEnabled()) {
			drivetrain.drive(0.5, 1);
			if (gyro.getAngle() - startAngle > 60.) {
				break;
			}
			Timer.delay(0.02);
		}
		
		startRight = drivetrain.getRightDistance();
		startLeft = drivetrain.getLeftDistance();
		while (isAutonomous() && isEnabled()) {
			drivetrain.shiftLow();
			drivetrain.drive(1, 0);
			if (drivetrain.getRightDistance() - startRight > 25. / 12. ||
					drivetrain.getLeftDistance() - startLeft > 25. / 12.) {
				break;
			}
			Timer.delay(0.02);
		}
		
		
		startRight = drivetrain.getRightDistance();
		startLeft = drivetrain.getLeftDistance();
		//gearage.set(true);
		Timer.delay(1);
		while (isAutonomous() && isEnabled()) {
			drivetrain.drive(-1,0);
			if (drivetrain.getRightDistance() - startRight < - 100. / 12. ||
					drivetrain.getLeftDistance() - startLeft < - 100. / 12.) {
				break;
			}
			Timer.delay(0.02);
		}
		//gearage.set(false);
	}
	
	private void middleAuto () {
		int count = 0;
		while (isAutonomous() && isEnabled()) {
			drivetrain.shiftLow();
			drivetrain.drive(1, 0);
			if (drivetrain.getRightDistance() - startRight > 100. / 12. ||
					drivetrain.getLeftDistance() - startLeft > 100. / 12.) {
				break;
			}
			Timer.delay(0.02);
			
			if (count++ > 200) {
				break;
			}
		}
		
		
		startRight = drivetrain.getRightDistance();
		startLeft = drivetrain.getLeftDistance();
		gearage.set(true);
		Timer.delay(1);
		while (isAutonomous() && isEnabled()) {
			drivetrain.drive(-1,0);
			if (drivetrain.getRightDistance() - startRight < - 100. / 12. ||
					drivetrain.getLeftDistance() - startLeft < - 100. / 12.) {
				break;
			}
			Timer.delay(0.02);
		}
		gearage.set(false);
	}
	
	private void straightAuto() {
		int count = 0;
		while (isAutonomous() && isEnabled()) {
			drivetrain.shiftLow();
			drivetrain.drive(1, 0);
			
			SmartDashboard.putNumber("Distance",
					(drivetrain.getRightDistance() - startRight 
							+ drivetrain.getLeftDistance()) - startLeft / 2.);
			if (drivetrain.getRightDistance() - startRight > 100. / 12. ||
					drivetrain.getLeftDistance() - startLeft > 100. / 12.) {
				break;
			}
			Timer.delay(0.02);
			
			if (count++ > 100) {
				break;
			}
		}
		
		
		while (isAutonomous() && isEnabled()) {
			drivetrain.drive(0., 0.);
			Timer.delay(0.02);
		}
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
		
		//throttle is reversed
		double throttle = -joystick.getThrottle();
		
		double motor0Current    = pdp.getCurrent(0),
			   motor1Current    = pdp.getCurrent(1),
			   motor2Current    = pdp.getCurrent(14),
			   motor3Current    = pdp.getCurrent(15),
			   liftMotorCurrent = pdp.getCurrent(6);
		
		//double compressorCurrent = compressor.getCompressorCurrent();
		
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
		
		//SmartDashboard.putNumber("Compressor Current", compressorCurrent);
		
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
			liftMotor.set((1.0 + throttle) / 2);
			SmartDashboard.putBoolean("Lift Motor On", true);
		} else {
			liftMotor.set(0);
			SmartDashboard.putBoolean("Lift Motor On", false);
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

