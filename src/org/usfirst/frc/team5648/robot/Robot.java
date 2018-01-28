///*----------------------------------------------------------------------------*/
///* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/


package org.usfirst.frc.team5648.robot;

import org.usfirst.frc.team5648.robot.subsystems.ExampleSubsystem;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static final ExampleSubsystem kExampleSubsystem
		= new ExampleSubsystem();
	public static OI m_oi;

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	
	private static final int usbPort = 0; // DriverStation Port
	private static final int leftMotorPortFront = 0; // PWM port
	private static final int rightMotorPortFront = 1; // PWM port
	private static final int leftMotorPortBack = 2; // PWM port
	private static final int rightMotorPortBack = 3; // PWM port
	private static final int sparkMotorPort = 8; // PWM port
	private static final int relayPort = 0;	// DIO port
	
	private static final double distancePerAutoPeriodic = 0.0095; //estimated travel per autonomous period
	
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";

	private static final String distanceKey = "Distance Traveled in Auto mode";
	private static final String autoPeriodKey = "Number of Auto mode periods we have driven for";
	private static final String driveRotation = "DriveRotation";
	private static final String driveMoveValue = "DriveMove";
	private static final String xValue = "xValue";
	private static final String yValue = "yValue";
	private static final String throttle = "throttle";
	private static final String scaledThrottle = "scaledThrottle";
	
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	Relay slideRelayRelayPort;
	Joystick joystick;
	XboxController xboxController;
	Talon driveLeftFront;
	Talon driveRightFront;
	Talon driveLeftBack;
	Talon driveRightBack;
	DifferentialDrive driveFront;
	DifferentialDrive driveBack;
	DigitalOutput slideRelay;
	NetworkTable joystickTable;
	NetworkTable motorTable;
	// distance we estimate we have traveled
	double autoAccumulatedDistance;
	// how many AutoPeriodic periods have executed that we have driven forward for
	int autoAccumulatedPeriods;
	Spark climbingMechanism;
	private NetworkTableInstance ntInstance;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);		
		
		try {
			//joystick = new Joystick(usbPort);
			xboxController = new XboxController(usbPort);
		} catch (Exception e)
		{
			System.err.println("Issue with joystick");
			throw e;
		}
		
		System.out.println("Robot Initialised!");

		try {
			driveLeftBack = new Talon(leftMotorPortBack);
			driveLeftFront = new Talon(leftMotorPortFront);
			driveRightBack = new Talon(rightMotorPortBack);
			driveRightFront = new Talon(rightMotorPortFront);
			
			driveFront = new DifferentialDrive (driveLeftFront, driveRightFront);
			driveBack = new DifferentialDrive (driveLeftBack, driveRightBack);
		}
		catch (Exception e)
		{
			System.err.println("Issue with robot drive");
			throw e;
		}
		
		try {
			climbingMechanism = new Spark(sparkMotorPort);
		}
		catch (Exception e)
		{
			System.err.println("Issue with Spark device!");
			throw e;
		}
		
		slideRelay = new DigitalOutput(relayPort);

		try {
			// start camera capture
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// set appropriate mode for the camera
			VideoMode[] mode = camera.enumerateVideoModes();
			camera.setVideoMode(mode[0]);
		}
		catch (Exception e)
		{
			System.err.println("No USB Camera detected, continuing init.");		
		}
		
		// initialise auto values
		autoAccumulatedDistance = 0.0;
		autoAccumulatedPeriods = 0;
		
		try {
			// create debug tables
			ntInstance = NetworkTableInstance.getDefault();
			NetworkTable debugTable = ntInstance.getTable("Debug");
			joystickTable = debugTable.getSubTable("Joystick Values");
			motorTable = debugTable.getSubTable("Motor Input");
			
			// initialise the Joystick/Motor input tables
			motorTable.getEntry(distanceKey).setNumber(autoAccumulatedDistance);
			motorTable.getEntry(autoPeriodKey).setNumber(autoAccumulatedPeriods); 		
			motorTable.getEntry(driveRotation).setNumber(0); 
			motorTable.getEntry(driveMoveValue).setNumber(0);	
			
			joystickTable.getEntry(xValue).setNumber(0);
			joystickTable.getEntry(yValue).setNumber(0);
			joystickTable.getEntry(throttle).setNumber(0);
			joystickTable.getEntry(scaledThrottle).setNumber(0.0);
			joystickTable.getEntry("climbing motor trigger").setBoolean(false);
		}
		catch (Exception e)
		{
			System.err.println("Issue creating network table entries, continuing init.");	
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		
		// enable slide relay
		slideRelayRelayPort = new Relay(0);
		slideRelayRelayPort.set(Value.kReverse);
		
		slideRelay.set(true);
		
		/*try {
			Thread.sleep(100);
		} catch (Exception e) {
			// TODO: handle exception
		}*/
				
		
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		// move forward 2.37 meters, for extra 5 points
		if (autoAccumulatedDistance <= 2.37)
		{
			// continue driving
			autoAccumulatedDistance += distancePerAutoPeriodic;
			autoAccumulatedPeriods += 1;
			DriveMotors(0.5, 0);
		} else {
			// stop driving
			DriveMotors(0, 0);

			slideRelay.set(false);
			slideRelayRelayPort.set(Value.kForward);
		}
		
     	motorTable.getEntry(distanceKey).setNumber(autoAccumulatedDistance);
		motorTable.getEntry(autoPeriodKey).setNumber(autoAccumulatedPeriods); 
		
	}

	private void DriveMotors(double moveValue, double rotateValue) {
		driveFront.arcadeDrive(moveValue, rotateValue, true);
		driveBack.arcadeDrive(moveValue, rotateValue, true);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		autoAccumulatedDistance = 0;
		
		// get joystick position/variables
		// double xValue = joystick.getX();
		// double yValue = joystick.getY();
		// double throttle = joystick.getThrottle();
		// double scaledThrottle = (throttle - 1)*(0.5); // limited to min of 0 and max of 1
		
		// boolean trigger = joystick.getTrigger();
		
		double xValue = xboxController.getX(Hand.kLeft);
		double yValue = xboxController.getY(Hand.kLeft);
		double throttle = 0.0;
		double triggerAxis = xboxController.getTriggerAxis(Hand.kLeft);
		double scaledThrottle = 0.5 + (triggerAxis / 2);
		System.out.println("Scaled throtttle ttttt " + scaledThrottle);
		boolean trigger = false;
		
		// update joystick values in Network Tables for display in the dashboard
		// note that new values will not show in the Dashboard unless it is restarted
		joystickTable.getEntry(Robot.xValue).setNumber(xValue);
		joystickTable.getEntry(Robot.yValue).setNumber(yValue);
		joystickTable.getEntry(Robot.throttle).setNumber(throttle);
		joystickTable.getEntry("climbing motor trigger").setBoolean(trigger);
		joystickTable.getEntry(Robot.scaledThrottle).setNumber(scaledThrottle);
		
		// calculate drive values - modify power by scaled throttle
		double driveRotatation = -xValue * scaledThrottle;
		double driveMoveValue = -yValue * scaledThrottle;
		
		// update motor drive values in Network Tables for display in the dashboard		
		motorTable.getEntry(Robot.driveRotation).setNumber(driveRotatation);
		motorTable.getEntry(Robot.driveMoveValue).setNumber(driveMoveValue);		
		
		// send drive values to motor
		DriveMotors(driveMoveValue, driveRotatation);
			
		if (trigger == true)
		{
			// if trigger is pressed activate climbing robot
			climbingMechanism.set(scaledThrottle*2);
		} else {
			// if trigger is not pressed deactivate climbing robot
			climbingMechanism.set(0);
		}
			
	}
	
	public void disabledInit()
	{
		DriveMotors(0, 0);
		
		// disable climbing mech
		climbingMechanism.set(0);
	}
}



