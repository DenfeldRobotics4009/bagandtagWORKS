package org.usfirst.frc.team4009.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;

import org.usfirst.frc.team4009.robot.commands.ExampleCommand;
import org.usfirst.frc.team4009.robot.subsystems.ExampleSubsystem;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	public static OI oi;

	Command autonomousCommand;
	SendableChooser chooser;
	CANTalon can0, can1, can2, can3, can4, can5, can6, can7;
	RobotDrive ac, eg, db, hf;
	Joystick joystick, joystick2;
	Trigger two;
	double autonomousMovementTime = 1;
	double autonomousMotorPower = -.5;
	// DigitalInput limitswitch1,limitswitch2,limitswitchGEAR;
	// AnalogInput potentiometer1;
	// SpeedController motor1;
	TalonSRX motor0, motor2, motor3, motor4, motor5, motor6;
	// Encoder encoder1;
	PIDController shooterWheel;

	// PID variables
	double flyPterm; // flywheel proportional constant
	double flyDterm; // flywheel derivative constant
	double flyError; // flywheel error
	double flyTarget; // flywheel target rotation rate
	double flyPrevError; // flywheel previous error
	double flyCP; // flywheel current power
	boolean fly; // flywheel on/off

	// variables
	PIDSource pidsource1;
	double rotationRate;
	double RotationCount;

	// camera variables
	public static I2C pixyi2c;

	public Robot() {
		RobotMap.table = NetworkTable.getTable("GRIP/myContoursReport");
	}

	public AnalogInput potentiometer1;
	public AnalogInput currentsensor;

	public DigitalInput limitswitchGEAR;

	public Timer lastReset;
	public Timer gearDelay;
	public Timer shooterReverse;
	public Timer autonomousTimer; 
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		oi = new OI();
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// ::: Initialize objects! Each CANTalon # is the same channel number
		// ::: ex. can0 is on channel, can1 is on channel 1
		/**
		 * private CANTalon can0, can1,can2,can3,can4,can5,can6,can7; private
		 * RobotDrive ac,eg,db,hf; private Joystick joystick;
		 */
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		joystick = new Joystick(0);
		joystick2 = new Joystick(1);
		// CAN
		can0 = new CANTalon(0);
		can1 = new CANTalon(1);
		can2 = new CANTalon(2);
		can3 = new CANTalon(3);
		can4 = new CANTalon(4);
		can5 = new CANTalon(5);
		can6 = new CANTalon(6);
		can7 = new CANTalon(7);

		two = new JoystickButton(joystick2, 2);

		ac = new RobotDrive(can7, can4);
		eg = new RobotDrive(can6, can2);
		db = new RobotDrive(can1, can3);
		hf = new RobotDrive(can5, can0);
		

		limitswitchGEAR = new DigitalInput(3);
		potentiometer1 = new AnalogInput(0);
		currentsensor = new AnalogInput(3);
		// DIO
		// limitswitch1 = new DigitalInput(0);
		// limitswitch2 = new DigitalInput(1);
		// limitswitchGEAR = new DigitalInput(2);
		// encoder1 = new Encoder(2,3, false, Encoder.EncodingType.k4X);
		// encoder1.setMaxPeriod(.1);
		// encoder1.setMinRate(10);
		// encoder1.setDistancePerPulse(5);
		// encoder1.setReverseDirection(false);
		// encoder1.setSamplesToAverage(7);

		// ANALOG IN
		// potentiometer1 = new AnalogInput(0);
		
		// Timer
		lastReset = new Timer();
		lastReset.start();
		gearDelay = new Timer();
		gearDelay.start();
		shooterReverse = new Timer();
		shooterReverse.start();
		autonomousTimer = new Timer();
		autonomousTimer.start();
		// PWM
		motor6 = new TalonSRX(6); // Shooter
		motor5 = new TalonSRX(5); // Intake
		motor4 = new TalonSRX(4); // Climb 1
		motor3 = new TalonSRX(3); // Climb 2
		motor2 = new TalonSRX(2); // Josilator*?
		motor0 = new TalonSRX(0); // Gear Placer

		// Network Table
		RobotMap.table = NetworkTable.getTable("datatable");
		double[] defaultValue = new double[0];
		while (isEnabled()) {
			double[] areas = RobotMap.table.getNumberArray("area", defaultValue);
			RobotMap.table.setIPAddress("10.40.9.1");
			System.out.print("areas: ");
			for (double area : areas) {
				System.out.print(area + " ");
			}
			System.out.println();
			Timer.delay(1);
		}
		NetworkTable.setIPAddress("10.40.9.1");
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {

	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyroa
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */

	public void autonomousInit() {
		autonomousCommand = (Command) chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	// No variable for time because I don't know how to make variables
	// No variable for motors because I don't know how to make variables
	public void autonomousPeriodic() {  
		if(autonomousTimer.hasPeriodPassed(autonomousMovementTime)){ //Time measured in seconds
			ac.setLeftRightMotorOutputs(autonomousMotorPower, autonomousMotorPower); //Number in parenthesis = motor power 1 = 100% -1 = -100%
			db.setLeftRightMotorOutputs(autonomousMotorPower, autonomousMotorPower);
			hf.setLeftRightMotorOutputs(autonomousMotorPower, autonomousMotorPower);
		}
		else{
			ac.setLeftRightMotorOutputs(0, 0);
			db.setLeftRightMotorOutputs(0, 0);
			hf.setLeftRightMotorOutputs(0, 0);
		}
	}

	// Other Network Table
	public void operatorControl() {
		double x = 0;
		double y = 0;
		while (isOperatorControl() && isEnabled()) {
			Timer.delay(0.25);
			RobotMap.table.putNumber("X", x);
			RobotMap.table.putNumber("Y", y);
			x += 0.05;
			y += 1.0;
		}
	}
	/*
	 * public void operatorControl() { }
	 * 
	 * /** This function is called periodically during operator control
	 */

	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		// lets make a new record object, it will feed the stuff we record into
		// the .csv file

		// Set Variables
		final double DZ = 0.1;
		final double twistDZ = 0.5;
		final double twistShift = 0; // Shift of twist range if one direction is
										// more sensitive than the other
		final double pMin = 0.25; // Precision Minimum: Must be between 0-1 and
									// less than pMax
		final double pMax = 0.75; // Precision Maximum: Must be between 0-1 and
									// more than pMin

		// Calculated Variables
		double xPwr;
		double yPwr;
		double zPwr;
		double highPwr;
		double oddQuad;
		double evenQuad;
		double pScale;

		double intakeSpeed = 0;
		double xCur;
		double yCur;
		double zCur;
		double highCur;
		double oddCur;
		double evenCur;
		boolean isPressed;
		double pMag;
		double fwsc; // front wheel strafe correction
		double jostSpeed;

		// Corrections for 6 wheel drive
		double sixWheelCorrect; // correction factor for 6 wheel bot
		double yPwr6W;
		double highPwr6W;
		double oddQuad6W;
		double evenQuad6W;
		double yCur6W;
		double highCur6W;
		double oddCur6W;
		double evenCur6W;

		while (isOperatorControl() && isEnabled()) {
			// printPixyCam();
			// encoder
			// SmartDashboard.putNumber("potentiometer",
			// potentiometer1.getVoltage());
			// SmartDashboard.putBoolean("limit switch 1", limitswitch1.get() );
			// SmartDashboard.putNumber("encoder count", encoder1.get());
			// SmartDashboard.putNumber("encoder rate", encoder1.getRate());
			// SmartDashboard.putNumber("encoder raw", encoder1.getRaw());
			// SmartDashboard.putBoolean("encoder direction",
			// encoder1.getDirection());
			// System.out.println("Encoder Rate: " + encoder1.getRaw());
			// if(joystick2.getRawButton(11)==true) {encoder1.reset();}
			// rotationRate=(encoder1.getRate()/360);
			// RotationCount=(encoder1.getRaw()/360);
			// SmartDashboard.putNumber("Rotation rate" , rotationRate);
			// SmartDashboard.putNumber("Rotation count", RotationCount);
			SmartDashboard.putNumber("flyCP", flyCP);
			SmartDashboard.putNumber("flyError", flyError);
			// SmartDashboard.putNumber("pixyCam", SPI.Port.kOnboardCS0);
			// System.out.println("PixyCam port: " + SPI.Port.values());
			// System.out.println("PixyCam ?: " +
			// SPI.Port.valueOf(kOnboardCS0));
			SmartDashboard.putNumber("Current Sensor", currentsensor.getVoltage());

			// drive functions
			xPwr = joystick.getX();
			yPwr = -joystick.getY();
			zPwr = joystick.getTwist(); // twistShift*(1/(1+(Math.signum(joystick.getTwist())*twistShift)));
										// // Add .15 because Twist was drifting
										// to the left.
			highPwr = Math.max(Math.abs(xPwr), Math.abs(yPwr));
			oddQuad = yPwr - xPwr;
			evenQuad = yPwr + xPwr;
			isPressed = joystick.getTrigger();
			pMag = (joystick.getThrottle() + 1) / 2;
			fwsc = (1 - .3 * Math.abs(xPwr)); // Front wheel strafe correction
												// -- change the .3 higher for
												// more correction or lower for
												// less correction
			jostSpeed = 1;

			sixWheelCorrect = (Math.abs(xPwr) * (-yPwr / 100)) / Math.sqrt(2);
			yPwr6W = yPwr + sixWheelCorrect;
			highPwr6W = Math.max(Math.abs(xPwr), Math.abs(yPwr6W));
			oddQuad6W = yPwr6W - xPwr;
			evenQuad6W = yPwr6W + xPwr;

			// Precision Mode
			
			/**
			 * the reason we add .5 to pMag then divide by 2 is to change the
			 * effective range of pMag from the throttle range of 0-1 to
			 * 0.25-0.75 A scale factor of zero would just be stopped, and a
			 * scale factor of 1 would be full speed and thus pointless for
			 * precision mode. You can change the function to create whatever
			 * precision range driver finds useful.
			 */
			if (isPressed == true) {
				pScale = (pMag * (pMax - pMin) + pMin);
			} else {
				pScale = 1;
			}

			SmartDashboard.putNumber("Precision Scale", (pMag * (pMax - pMin) + pMin) );
			
			// The Cur Variables
			/**
			 * Cur variables are meant to smooth the drive speed coming in and
			 * out of the dead zone so that the robot drives less "choppy" at
			 * slower speeds
			 */

			// xCur
			if (Math.abs(xPwr) < DZ) {
				xCur = 0;
			} else {
				xCur = Math.signum(xPwr) * pScale * ((Math.abs(xPwr) - DZ) * (1 / (1 - DZ)));
			}

			// yCur
			if (Math.abs(yPwr) < DZ) {
				yCur = 0;
			} else {
				yCur = Math.signum(yPwr) * pScale * ((Math.abs(yPwr) - DZ) * (1 / (1 - DZ)));
			}

			// yCur6W
			/*
			 * if (Math.abs(yPwr6W)<DZ){ yCur6W=0; } else{ yCur6W=
			 * Math.signum(yPwr6W)*pScale*((Math.abs(yPwr6W)-DZ)*(1/(1-DZ))); }
			 */

			// zCur
			if (Math.abs(zPwr) < twistDZ) {
				zCur = 0;
			} else {
				zCur = Math.signum(zPwr) * pScale * ((Math.abs(zPwr) - twistDZ) * (1 / (1 - twistDZ)));
			}

			// highCur
			if (Math.abs(highPwr) < DZ) {
				highCur = 0;
			} else {
				highCur = Math.signum(highPwr) * pScale * ((Math.abs(highPwr) - DZ) * (1 / (1 - DZ)));
			}

			// highCur6W
			/*
			 * if (Math.abs(highPwr6W)<DZ){ highCur6W=0; } else{ highCur6W=
			 * Math.signum(highPwr6W)*pScale*((Math.abs(highPwr6W)-DZ)*(1/(1-DZ)
			 * )); }
			 */

			// oddCur
			if (Math.abs(oddQuad) < DZ) {
				oddCur = 0;
			} else {
				oddCur = Math.signum(oddQuad) * pScale * ((Math.abs(oddQuad) - DZ) * (1 / (1 - DZ)));
			}

			// oddCur6W
			/*
			 * if (Math.abs(oddQuad6W)<DZ){ oddCur6W=0; }else{ oddCur6W=
			 * Math.signum(oddQuad6W)*pScale*((Math.abs(oddQuad6W)-DZ)*(1/(1-DZ)
			 * )); }
			 */

			// evenCur
			if (Math.abs(evenQuad) < DZ) {
				evenCur = 0;
			} else {
				evenCur = Math.signum(evenQuad) * pScale * ((Math.abs(evenQuad) - DZ) * (1 / (1 - DZ)));
			}

			// evenCur6W
			/*
			 * if (Math.abs(evenQuad6W)<DZ){ evenCur6W=0; } else{ evenCur6W=
			 * Math.signum(evenQuad6W)*pScale*((Math.abs(evenQuad6W)-DZ)*(1/(1-
			 * DZ))); }
			 */

			// DRIVE CONTROL
			// Code assumes motors c,g,b,f are reversed in direction

			// Spin
			if (Math.abs(zPwr) > twistDZ) {
				ac.setLeftRightMotorOutputs(zCur, -zCur);
				// ac.setLeftRightMotorOutputs(zCur, -zCur); // ProtoBot wheel
				// backwards fixed
				eg.setLeftRightMotorOutputs(zCur * 0.777, -zCur); // Not needed
																	// if
																	// switched
				db.setLeftRightMotorOutputs(zCur, -zCur);
				// db.setLeftRightMotorOutputs(zCur, -zCur); // ProtoBot wheel
				// backwards fixed
				hf.setLeftRightMotorOutputs(zCur, -zCur);

			} else {

				// DeadZone is built into the Cur variables so separate DeadZone
				// section not required

				// X-Positive, Y-Positive
				if (xPwr >= 0 && yPwr >= 0) {
					/*
					 * 
					 * ac.drive(highCur,0); eg.drive(xCur,0);
					 * db.drive(oddCur,0); hf.drive(yCur,0);
					 */
					ac.setLeftRightMotorOutputs(highCur * fwsc, highCur);
					// ac.setLeftRightMotorOutputs(highCur6W*fwsc, highCur6W);
					eg.setLeftRightMotorOutputs(xCur * fwsc, xCur); // Not
																	// needed if
																	// switched
					db.setLeftRightMotorOutputs(oddCur, oddCur * fwsc);
					// db.setLeftRightMotorOutputs(oddCur6W, oddCur6W*fwsc);
					hf.setLeftRightMotorOutputs(yCur, yCur); // Stays the same

				}

				// X-Negative, Y-Positive
				if (xPwr < 0 && yPwr >= 0) {
					/*
					 * ac.drive(evenCur,0); eg.drive(xCur,0);
					 * db.drive(highCur,0); hf.drive(yCur,0);
					 */
					ac.setLeftRightMotorOutputs(evenCur * fwsc, evenCur);
					// ac.setLeftRightMotorOutputs(evenCur6W*fwsc, evenCur6W);
					eg.setLeftRightMotorOutputs(xCur * fwsc, xCur); // Not
																	// needed if
																	// switched
					db.setLeftRightMotorOutputs(highCur, highCur * fwsc);
					// db.setLeftRightMotorOutputs(highCur6W, highCur6W*fwsc);
					hf.setLeftRightMotorOutputs(yCur, yCur); // Stays the same

				}

				// X-Negative, Y-Negative
				if (xPwr < 0 && yPwr < 0) {
					/*
					 * ac.drive(-highCur,0); eg.drive(xCur,0);
					 * db.drive(oddCur,0); hf.drive(yCur,0);
					 */
					ac.setLeftRightMotorOutputs(-highCur * fwsc, -highCur);
					// ac.setLeftRightMotorOutputs(-highCur6W*fwsc, -highCur6W);
					eg.setLeftRightMotorOutputs(xCur * fwsc, xCur); // Not
																	// needed if
																	// switched
					db.setLeftRightMotorOutputs(oddCur, oddCur * fwsc);
					// db.setLeftRightMotorOutputs(oddCur6W, oddCur6W*fwsc);
					hf.setLeftRightMotorOutputs(yCur, yCur); // Stays the same

				}

				// X-Positive, Y-Negative
				if (xPwr >= 0 && yPwr < 0) {
					/*
					 * ac.drive(evenCur,0); eg.drive(xCur,0);
					 * db.drive(-highCur,0); hf.drive(yCur,0);
					 */
					ac.setLeftRightMotorOutputs(evenCur * fwsc, evenCur);
					// ac.setLeftRightMotorOutputs(evenCur6W*fwsc, evenCur6W);
					eg.setLeftRightMotorOutputs(xCur * fwsc, xCur); // Not
																	// needed if
																	// switched
					db.setLeftRightMotorOutputs(-highCur, -highCur * fwsc);
					// db.setLeftRightMotorOutputs(-highCur6W, -highCur6W*fwsc);
					hf.setLeftRightMotorOutputs(yCur, yCur); // Stays the same

				}

			}

			// OPERATOR CONTROL

			/*
			 * Limit switches return true when open and return false when
			 * pressed
			 */

			/*
			 * //Limited ends: Joystick control between limit switches
			 * if((joystick2.getY()>0 && limitswitch1.get() == true) ||
			 * (joystick2.getY()<0 && limitswitch2.get() == true)){
			 * motor4.set(joystick2.getY()); } else{ motor4.set(0); }
			 */

			// Gear Placer
			// SmartDashboard.putNumber("Potentiometer",
			// potentiometer1.getVoltage() );
			if ((joystick.getRawButton(2) == true || !limitswitchGEAR.get()) && gearDelay.get() > 6.0) {
				gearDelay.reset();
			}
			if (joystick.getRawButton(2) == true || !limitswitchGEAR.get()) {
				if (gearDelay.get() <= 0.44) {
					motor0.set(1);
				}
				if (gearDelay.get() > 0.44 && gearDelay.get() <= 5.0) {
					motor0.set(0);
				}
				if (gearDelay.get() > 5.0) {
					motor0.set(1);
				}
			}

			// else if (joystick.getRawButton(3) == true){motor0.set(0.35);}
			else {
				motor0.set(0);
			}

			// Intake
			if (joystick2.getRawButton(5) == true) { // Intake
				intakeSpeed = -1;

			}
			if (joystick2.getRawButton(3) == true) { // Intake Stop
				intakeSpeed = 0;

			}
			if (joystick2.getRawButton(11) == true) { // Intake Reverse
				intakeSpeed = 1;
			}
			motor5.set(intakeSpeed);

			// Climb
			if (joystick2.getRawButton(7) == true) {
				motor4.set(-1);
				motor3.set(1);
			} else if (joystick2.getRawButton(8) && joystick.getRawButton(8)) {
				motor4.set(1);
				motor3.set(-1);
			}

			else {
				motor4.set(0);
				motor3.set(0);
			}

			// Button Controlled Motor
			/*
			 * if(joystick2.getRawButton(7) == true){
			 * 
			 * motor2.set((joystick2.getThrottle()+1)/2); }else
			 * {if(joystick2.getRawButton(8) == true){
			 * motor2.set(-(joystick2.getThrottle()+1)/2); }else{ motor2.set(0);
			 * } }
			 */
			// Operator Motors

			// Shooter
			if (joystick2.getTrigger() == true) {
				if (shooterReverse.get()<1){motor6.set(-1);}
				else {motor6.set((joystick2.getThrottle() + 1) / 2);}}
			else {
				motor6.set(0);
				shooterReverse.reset();
			}
			
			
			SmartDashboard.putNumber("Shooter Speed", (joystick2.getThrottle() + 1) / 2);
			// Josilator
			if (joystick2.getTrigger() == true && lastReset.get() >= 0.5 && shooterReverse.get()>1.1) {
				motor2.set(jostSpeed);
				if (Math.abs(currentsensor.getVoltage() - .55) > 10) {
					jostSpeed = (jostSpeed * (-1));
					lastReset.reset();
				}
			} else {
				motor2.set(0);
			}
		}
	}

	// SmartDashboard.putNumber("Current Sensor: ",
	// RobotMap.currentsensor.getVoltage());

	// ++ send out current sensor value, if current sensor > x, reverse

	/*
	 * if(joystick2.getRawButton(2) == true){
	 * 
	 * ;
	 * 
	 * } else{
	 * 
	 * }
	 */

	/*
	 * //Potentiometer Controlled Motor if(joystick2.getRawButton(9) == true &&
	 * potentiometer1.getVoltage() > 1){ motor3.set(1); } else{
	 * if(joystick2.getRawButton(10) == true && potentiometer1.getVoltage() <
	 * 3){ motor3.set(-1); } else{ motor3.set(0); } }
	 */

	// PD Velocity Control
	/*
	 * flyTarget = 50; flyPterm = 0.004; flyDterm = 0.004; //flyPterm =
	 * SmartDashboard.getNumber("DB/Slider 0", 0.005); //flyDterm =
	 * SmartDashboard.getNumber("DB/Slider 1", 0.005); flyError =
	 * (flyTarget-rotationRate); if(joystick2.getRawButton(3)== true){fly =
	 * true;} if(joystick2.getRawButton(4)== true){fly = false;} if(fly== true){
	 * flyCP=(double)Math.min(Math.max(flyCP+(flyPterm*flyError)+(flyDterm*(
	 * flyError-flyPrevError)),-1),1); flyPrevError=flyError; }else{ flyCP=0;
	 * flyPrevError=0; } motorX.set(flyCP); }
	 */
	// PID Controlled Motor
	/*
	 * if(joystick2.getRawButton(3) == true){ shooterWheel.enable();
	 * //shooterWheel.setSetpoint(75); //motor1.set(shooterWheel.get());
	 * 
	 * }
	 * 
	 * if(joystick2.getRawButton(4) == true){ shooterWheel.disable();
	 * //motor1.set(0); }
	 */

	/*
	 * if((potentiometer1.getVoltage() < 1) || (potentiometer1.getVoltage() >
	 * 3)){ motor3.set(0); }
	 */

	/*
	 * First try limit switch code commented out //limitswitch1.get()
	 * if(limitswitch1.get() == true){ motor1.set(joystick2.getY()); }
	 * if(limitswitch1.get() == false){ if(joystick2.getY()<0){
	 * motor1.set(joystick2.getY()); } } if(limitswitch2.get() != true){
	 * motor2.set(1); }else{ motor2.set(0); }
	 */

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
}