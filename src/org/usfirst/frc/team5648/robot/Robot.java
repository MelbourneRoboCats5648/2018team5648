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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

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

	private static final int usbPort0 = 0; // DriverStation Primary Port
	private static final int usbPort1 = 1; // DriverStation Secondary Port
	private static final int leftMotorPortFront = 0; // PWM port
	private static final int rightMotorPortFront = 1; // PWM port
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
	Compressor compressor;
	XboxController primaryController, secondaryController;
	Talon driveLeftFront;
	Talon driveRightFront;
	DifferentialDrive driveFront;
	DigitalOutput slideRelay;
	NetworkTable joystickTable;
	NetworkTable motorTable;
	int robotlocation;
	char switchtarget;
	// distance we estimate we have traveled
	double autoAccumulatedDistance;
	// how many AutoPeriodic periods have executed that we have driven forward for
	int autoAccumulatedPeriods;
	Spark climbingMechanism;
	private NetworkTableInstance ntInstance;
	Timer timer, testTimer;
	double distancePerSecond = 63.5; // cm/s travel speed in auto period 2sec @ .5 = 127cm, 4 sec @ .5 = 232cm
	double rotationPerSecond = 63.3; // degrees/s rotation speed in auto period 1.5 sec @ .5 = 95 deg 
	private boolean useSecondaryController;
	boolean clampOpen;
	boolean bucketPush;
	DoubleSolenoid clampSolenoid, bucketSolenoid;
	Solenoid solenoid0,	solenoid1, solenoid2, solenoid3;

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
			primaryController = new XboxController(usbPort0);
		} catch (Exception e)
		{
			System.err.println("Can't find primary controller");
			throw e;
		}
		
		try {
			secondaryController = new XboxController(usbPort1);
		} catch (Exception e)
		{
			System.err.println("Can't find secondary controller");
		}
		
		if (DriverStation.getInstance().getJoystickIsXbox(1) == true)
		{
			useSecondaryController = true;			
		}
		else 
		{
			useSecondaryController = false;
		}
		
		System.out.println("Robot Initialised!");

		try {
			driveLeftFront = new Talon(leftMotorPortFront);
			driveRightFront = new Talon(rightMotorPortFront);
			
			driveFront = new DifferentialDrive (driveLeftFront, driveRightFront);
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
		
		try {
			compressor = new Compressor(0);
		}
		catch (Exception e)
		{
			System.err.println("Issue with compressor");
			throw e;
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
			joystickTable.getEntry("Use Secondary Controller").setBoolean(useSecondaryController);
		}
		catch (Exception e)
		{
			System.err.println("Issue creating network table entries, continuing init.");	
		}

		clampSolenoid = new DoubleSolenoid(0,1);
		bucketSolenoid = new DoubleSolenoid(2,3);
		
		clampOpen = true;
		bucketPush = false;
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
		// start time
		timer = new Timer();
		timer.start();
		
		// get robot location (1, 2, or 3)
		DriverStation driveStation = DriverStation.getInstance();
		driveStation.getLocation(); 
		robotlocation = driveStation.getLocation();
		
		// get alliance switch location ('R' or 'L')
		char[] GameSpecificMessage = driveStation.getGameSpecificMessage().toCharArray();
		switchtarget = GameSpecificMessage[0];
		
		// TODO: turn light to blue or red
		driveStation.getAlliance();
		
		// set up air compressor, and turn on
		compressor.setClosedLoopControl(true);
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		if (switchtarget == 'L') // if target is left
		{
			// L1 Straight 427cm, turn right 90 degrees, go forward 175cm, drop cube and reverse
			if (robotlocation == 1)
			{
				if(timer.get() < 427.0/distancePerSecond)
				{
					// motors go forward
					DriveMotors(-0.5,0.25);
				}
				else
				{
					if (timer.get() < 90.0/rotationPerSecond + 427.0/distancePerSecond)
					{
						// turn right
						DriveMotors(0,0.5);
					} 
					else
					{
						if (timer.get() < 90.0/rotationPerSecond + 427.0/distancePerSecond + 175.0/distancePerSecond)
						{
							//go forward (super fast for dumb box!)
							DriveMotors(-1,0.25);
							// TODO pneumatics
							
						}
						else 
						{
							// Stop motors	
							DriveMotors(0,0);
						}
					}
				}
			} 
			// L2 Straight 135.75cm, turn left 90 degrees, straight 344cm, turn right 90 degrees, straight 245.75cm
			if (robotlocation == 2)
			{
				if (timer.get() < 135.75/distancePerSecond)
				{
					//move forward 
					DriveMotors(-0.5,0);
				}
				else
				{
					if (timer.get() < 90.0/rotationPerSecond + 135.75/distancePerSecond)
					{
						//turn left
						DriveMotors(0,-0.5);
					}
					else
					{
						if (timer.get() < 90.0/rotationPerSecond + 135.75/distancePerSecond + 344.0/distancePerSecond)
						{
							//move forward
							DriveMotors(-0.5,0.25);
						}
						else
						{
							if (timer.get() < 90.0/rotationPerSecond + 90.0/rotationPerSecond + 135.75/distancePerSecond + 344.0/distancePerSecond)
							{
								//turn right 
								DriveMotors(0,0.5);
							}
							else
							{
								if (timer.get() < 90.0/rotationPerSecond + 90.0/rotationPerSecond + 135.75/distancePerSecond + 344.0/distancePerSecond + 245.75/distancePerSecond)
								{
									//go forward 
									DriveMotors(-1,0.25);
									// TODO pneumatics
								}
								else
								{
									//stop motors
									DriveMotors(0,0);
								}
							}
						}
					}
				}
			}
			// R1 & L3 Straight 400cm
			if (robotlocation == 3)
			{
				if (timer.get() < 400.0/distancePerSecond)
				{
					//move forward 
					DriveMotors(-1,0.25);
					// TODO pneumatics
				}
				else
				{
					//stop motors 
					DriveMotors(0,0);
				}
			}
		} else { // target is right
			// R1 & L3 Straight 400cm
			if (robotlocation == 1)
			{
				if (timer.get() < 400.0/distancePerSecond)
				{
					//move forward 
					DriveMotors(-1,0.25);
					// TODO pneumatics
				}
				else
				{
					//stop motors 
					DriveMotors(0,0);
				}	
			}
			
			// R2 Straight 427cm
			if (robotlocation == 2)
			{
				if (timer.get() < 427.0/distancePerSecond)
				{
					//move forward
					DriveMotors(-1,0.25);
					// TODO pneumatics
				}
				else
				{
					//stop motors
					DriveMotors(0,0);
				}
			}
				
			// R3 Straight 427cm, turn left 90 degrees, go forward 175cm, drop cube and reversing (possible extent) 
			if (robotlocation == 3)
			{
				if (timer.get() < 427.0/distancePerSecond)
				{
					//move forward
					DriveMotors(-0.5,0.25);
				}
				else
				{
					if (timer.get() < 90.0/rotationPerSecond + 427.0/distancePerSecond)
					{
						// turn left
						DriveMotors(0,-0.5);
					} 
					else
					{
						if (timer.get() < 90.0/rotationPerSecond + 427.0/distancePerSecond + 175.0/distancePerSecond)
						{
							//go forward (super fast for dumb box!)
							DriveMotors(-1,0.25);
							// TODO pneumatics
						}
						else 
						{
							// Stop motors	
							DriveMotors(0,0);
						}
					}
				}
				
			}
			
		}	
	}

	/**
	 * DriveMotors will drive our robot
	 * @param moveValue speed along X axis [-1, 1], positive is forward
	 * @param rotateValue rotation rate [-1, 1], clockwise is positive
	 */  		
	private void DriveMotors(double moveValue, double rotateValue) {
		driveFront.arcadeDrive(moveValue, rotateValue, true);
	}

	@Override
	public void teleopInit() {
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
	
		drivingControl(primaryController);
		
		if(useSecondaryController == true)
		{
			pneumaticsControl(secondaryController);
		}
		else
		{
			pneumaticsControl(primaryController);
		}
	
		// TODO: 5 climbing mechanism control
			
	}

	private void pneumaticsControl(XboxController pneumaticsController) {
	
		// Clamp arms - B button
		// beginning of time clampOpen = true
		// if B button is pressed and clampOpen == true
		// close clamp and set clampOpen to false
		// if B button is press and clampOpen == false
		// open clamp and set clampOpen to true
		if (pneumaticsController.getBButtonPressed())
		{
			if (clampOpen == true)
			{
				clampSolenoid.set(DoubleSolenoid.Value.kReverse);      
				clampOpen = false;
			}
			else
			{
				clampSolenoid.set(DoubleSolenoid.Value.kForward);      
				clampOpen = true;
			}
		}
		
		// Bucket - A button
		// beginning of time bucketPush = false
		// if a button is pressed and bucketPush == true
		// set bucket pnuematics to retract and set bucketPush to false
		// if A button is pressed and bucketPush == false
		// push block and bucketPush set to true
		if (pneumaticsController.getAButtonPressed())
		{
			if (bucketPush == true)
			{
				bucketSolenoid.set(DoubleSolenoid.Value.kReverse);
				bucketPush = false;
			}
			else
			{
				bucketSolenoid.set(DoubleSolenoid.Value.kForward);
				bucketPush = true;
			}
		}
		
		
	}

	private void drivingControl(XboxController drivingController) {
		// get xbox controller position/variables		
		double xValue = drivingController.getX(Hand.kLeft);
		double yValue = drivingController.getY(Hand.kLeft);
		double throttle = 0.0;
		double triggerAxis = drivingController.getTriggerAxis(Hand.kLeft);
		double scaledThrottle = 0.5 + (triggerAxis / 2); // limited to min of 0 and max of 1
		System.out.println("Scaled throttle ttttt " + scaledThrottle);
		
		// update joystick values in Network Tables for display in the dashboard
		// note that new values will not show in the Dashboard unless it is restarted
		joystickTable.getEntry(Robot.xValue).setNumber(xValue);
		joystickTable.getEntry(Robot.yValue).setNumber(yValue);
		joystickTable.getEntry(Robot.throttle).setNumber(throttle);
		joystickTable.getEntry(Robot.scaledThrottle).setNumber(scaledThrottle);
		
		// calculate drive values - modify power by scaled throttle
		double driveRotatation = -xValue * scaledThrottle;
		double driveMoveValue = -yValue * scaledThrottle;
		
		// update motor drive values in Network Tables for display in the dashboard		
		motorTable.getEntry(Robot.driveRotation).setNumber(driveRotatation);
		motorTable.getEntry(Robot.driveMoveValue).setNumber(driveMoveValue);		
		
		// send drive values to motor
		DriveMotors(driveMoveValue, driveRotatation);
	}
	
	public void disabledInit()
	{
		DriveMotors(0, 0);
		
		// disable climbing mech
		climbingMechanism.set(0);
	}
	
	/**
	 * This function is called once before test periodic state
	 */
	@Override
	public void testInit()
	{
		testTimer = new Timer();
		testTimer.start();
				
		//clampSolenoid = new DoubleSolenoid(0,1);
		//bucketSolenoid = new DoubleSolenoid(2,3);

//		solenoid0 = new Solenoid(0);
//		solenoid1 = new Solenoid(1);
//		solenoid2 = new Solenoid(2);
//		solenoid3 = new Solenoid(3);
//		
//		solenoid0.set(false);    
//		solenoid1.set(false);
//		solenoid2.set(false);
//		solenoid3.set(false);
	}
	
	/**
	 * This function is called periodically during test
	 */
	@Override
	public void testPeriodic()
	{
		if (testTimer.get() < 1.5)
		{
			//DriveMotors(-0.5,0.25); forward
			DriveMotors(0, 0.5);
		}
		else
		{
			DriveMotors(0,0);
		}
		
//
//		
//		// set up air compressor, and turn on
//		compressor.setClosedLoopControl(true);
//		
//		if (primaryController.getBButtonPressed()) // clamp
//		{
//			solenoid0.set(true);                                                        
//		}
//		
//		if (primaryController.getXButtonPressed()) // clamp
//		{
//			solenoid1.set(true);
//		}
//		
//		if (primaryController.getAButtonPressed()) // bucket
//		{
//			solenoid2.set(true);
//		}
//
//		if (primaryController.getYButtonPressed()) // bucket
//		{
//			solenoid3.set(true);
//		}
//		if (primaryController.getBButtonReleased()) // clamp
//		{
//			solenoid0.set(false);                                                        
//		}
//		
//		if (primaryController.getXButtonReleased()) // clamp
//		{
//			solenoid1.set(false);
//		}
//		
//		if (primaryController.getAButtonReleased()) // bucket
//		{
//			solenoid2.set(false);
//		}
//
//		if (primaryController.getYButtonReleased()) // bucket
//		{
//			solenoid3.set(false);
//		}
	}
}



