package org.usfirst.frc.team4009.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
	public RobotMap(){
		can0 = new CANTalon(0);
		can1 = new CANTalon(1);
		can2 = new CANTalon(2);
		can3 = new CANTalon(3);
		can4 = new CANTalon(4);
		can5 = new CANTalon(5);
		can6 = new CANTalon(6);
		can7 = new CANTalon(7);
		ac = new RobotDrive(can7,can4);
		eg = new RobotDrive(can6,can2);
		db = new RobotDrive(can1,can3);
		hf = new RobotDrive(can5,can0);
		limitswitch1 = new DigitalInput(0);
		limitswitch2 = new DigitalInput(1);
		motor1 = new Spark(0);
		motor2 = new Spark(1);
		motor3 = new TalonSRX(3);
		motor4 = new TalonSRX(4);
		timer = new Timer();
		ipAddress = new String("10.40.9.35");
		motor5 = new TalonSRX(2);
	}
	
	public static CANTalon can0;
	public static CANTalon can1;
	public static CANTalon can2;
	public static CANTalon can3;
	public static CANTalon can4;
	public static CANTalon can5;
	public static CANTalon can6;
	public static CANTalon can7;
	//can1,can2,can3,can4,can5,can6,can7;
	public static RobotDrive ac;
	public static RobotDrive eg;
	public static RobotDrive db;
	public static RobotDrive hf;
	
	public static DigitalInput limitswitch1; 
	public static DigitalInput limitswitch2;
	
	public static SpeedController motor1;
	public static SpeedController motor2;
	public static TalonSRX motor3;
	public static TalonSRX motor4;
	public static SpeedController motor5;
	
	public static Encoder encoder1;
	
	public static PIDController shooterWheel;
	
	public static NetworkTable table;
	public static String ipAddress;
	public static Timer timer;

}
