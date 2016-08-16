package org.usfirst.frc.team4028.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URISyntaxException;
import java.net.UnknownHostException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.time.ZonedDateTime;
import java.util.Date;
import java.util.TimeZone;

import org.usfirst.frc.team4028.robot.Constants.RobotMap;
import org.usfirst.frc.team4028.robot.RobotData.AutonMode;
import org.usfirst.frc.team4028.robot.RobotData.Auton_Drive_Throttle_Percent;
import org.usfirst.frc.team4028.robot.RobotData.Infeed_Tilt_Zero_State;
import org.usfirst.frc.team4028.robot.RobotData.InputData;
import org.usfirst.frc.team4028.robot.RobotData.OutputData;
import org.usfirst.frc.team4028.robot.RobotData.WorkingData;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{
	// ===========================================================
	//   Define class level instance variables for Robot Runtime controllable objects  
	// ===========================================================
	
	// Driver & Operator station gamepads
	private Joystick _driverGamepad;
	private Joystick _operatorGamepad;

	// CIM DC Motors on Talon SRX Speed Controllers (via CAN Bus)
	private CANTalon _leftDriveMasterMtr;
	private CANTalon _leftDriveSlaveMtr;
	private CANTalon _leftDriveSlave2Mtr;
	private CANTalon _rightDriveMasterMtr;
	private CANTalon _rightDriveSlaveMtr;
	private CANTalon _rightDriveSlave2Mtr;
	private CANTalon _infeedTiltMtr;
	
	// CIM DC Motors on Victor SP Speed Controllers (via PWM Ports)
	private VictorSP _infeedAcqMtr;
	private VictorSP _winchMtr;
	
	// Arcade Drive with four drive motors
	private RobotDrive _robotDrive;
	
	// Pneumatic Solenoids for Air Cylinders
	private DoubleSolenoid _pumaFrontSolenoid;
	private DoubleSolenoid _pumaBackSolenoid;
	private DoubleSolenoid _shifterSolenoid;
	
	// Camera
	DynamicCameraServer server;
	private String _currentCameraName;
	
	// Vision Server Client
	private VisionClient _visionClient;
	
	// Servo
	private Servo _cupidServo;
	
	// ===========================================================
	//   Define class level working variables 
	// ===========================================================
		
	// DTO (Data Transfer Object) holding all live Robot Data Values
	RobotData _robotLiveData;
	VisionData _visionLiveData;
	
	// Wrapper around data logging (if it is enabled)
	DataLogger _dataLogger;
	
	// Smart Dashboard chooser
	SendableChooser autonModeChooser;
	SendableChooser autonDriveTimeChooser;
	SendableChooser autonDriveThrottleChooser;
	SendableChooser autonCrossDefenseTypeChooser;
	SendableChooser autonCrossDefensePositionChooser;
	
	double _autonTargetDriveTimeMSecs = 0;
	double _autonTargetDriveThrottlePercent = 0;
	
	boolean _isInfeedTiltAxisZeroedYet = false;
	boolean _isInfeedPeriodZeroMode = false;
	boolean _isInfeedTiltAxisZeroTimedOut = false;
	
	Infeed_Tilt_Zero_State _infeedTiltZeroState;
	long _infeedTiltZeroStartTime;

    /*****************************************************************************************************
     * This function is run when the robot is first started up.
	 * This is where we initialize the robot hardware configuration.
	 * 
	 * We try and fully configure the Motor controllers each robot startup.
	 *  We are as explicit as possible even when we do not need to be to make it as clear as possible for others
	 * 	This way we do not assume what their current configuration is.
	 * 	The only thing we depend on is that the CAN Bus address is correct
	 * 
	 * FYI:	Additional relevant documentation about each control object is included here
     *****************************************************************************************************/
    public void robotInit() 
    {    	
    	
    	// ===================
    	// Left Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
    	_leftDriveMasterMtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_TALON);
    	_leftDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
    	_leftDriveMasterMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_leftDriveMasterMtr.reverseSensor(false);  							// do not invert encoder feedback
    	_leftDriveMasterMtr.enableLimitSwitch(false, false);
    	//_leftDriveMasterMtr.reverseOutput(true);
    	
    	_leftDriveSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_SLAVE_TALON);	
    	_leftDriveSlaveMtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
    	_leftDriveSlaveMtr.set(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_TALON);
    	_leftDriveSlaveMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_leftDriveSlaveMtr.enableLimitSwitch(false, false);
    	//_leftDriveSlaveMtr.reverseOutput(true);
    	
    	_leftDriveSlave2Mtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_SLAVE_2_TALON);
    	_leftDriveSlave2Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);   // set this mtr ctrlr as a slave
    	_leftDriveSlave2Mtr.set(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_TALON);
    	_leftDriveSlave2Mtr.enableBrakeMode(false);
    	_leftDriveSlave2Mtr.enableLimitSwitch(false, false);
    	//_leftDriveSlave2Mtr.reverseOutput(true);
    	
    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CCW = Drive FWD
    	// ===================
    	_rightDriveMasterMtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_TALON);
    	_rightDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle.
    	_rightDriveMasterMtr.enableBrakeMode(false);						// default to brake mode DISABLED
    	_rightDriveMasterMtr.reverseSensor(true);  							// invert encoder feedback
    	_rightDriveMasterMtr.enableLimitSwitch(false, false);
    	//_rightDriveMasterMtr.reverseOutput(true);
    	
    	_rightDriveSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_SLAVE_TALON);	
    	_rightDriveSlaveMtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
    	_rightDriveSlaveMtr.set(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_TALON);
    	_rightDriveSlaveMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_rightDriveSlaveMtr.enableLimitSwitch(false, false);
    	//_rightDriveSlaveMtr.reverseOutput(true);
    	
    	_rightDriveSlave2Mtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_SLAVE_2_TALON);
    	_rightDriveSlave2Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
    	_rightDriveSlave2Mtr.set(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_TALON);
    	_rightDriveSlave2Mtr.enableBrakeMode(false);
    	_rightDriveSlave2Mtr.enableLimitSwitch(false, false);
    	//_rightDriveSlave2Mtr.reverseOutput(true);

    	
    	// ===================
    	// Infeed Tilt
    	// ===================
    	_infeedTiltMtr = new CANTalon(RobotMap.CAN_ADDR_INFEED_TILT_MTR_TALON);
    	_infeedTiltMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// we start in % vbus mode until we zero then we swap to position mode
    	_infeedTiltMtr.enableBrakeMode(true);
    	_infeedTiltMtr.enableLimitSwitch(true, false);		// we are using the FWD limit switch
    	_infeedTiltMtr.reverseSensor(false);
    	_infeedTiltMtr.ConfigFwdLimitSwitchNormallyOpen(false);
    	
    	// ===================
    	// Infeed Acquisition
    	// ===================
    	_infeedAcqMtr = new VictorSP(RobotMap.INFEED_ACQ_MTR_PWM_PORT);
    	
    	// ===================
    	// Winch Motor
    	// ===================
    	_winchMtr = new VictorSP(RobotMap.SCALING_MTR_PWM_PORT);
    	
    	// ===================
    	// Gamepads
    	// ===================
    	_driverGamepad = new Joystick(RobotMap.DRIVER_GAMEPAD_USB_PORT);				// std Logitech F310 Gamepad  
    	_operatorGamepad = new Joystick(RobotMap.OPERATOR_GAMEPAD_USB_PORT);			// std Logitech F310 Gamepad  
    	
    	// ===================
    	// Arcade Drive
    	//====================
    	// Arcade Drive configured to drive in three motor setup, other two motors follow as slaves 
    	_robotDrive = new RobotDrive(_leftDriveMasterMtr,_rightDriveMasterMtr);
    	   	
    	//===================
    	// Solenoids
    	//===================
    	_pumaFrontSolenoid = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_PUMA_FRONT_SOLENOID_EXTEND, RobotMap.PCM_PORT_PUMA_FRONT_SOLENOID_RETRACT);
    	_pumaBackSolenoid = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_PUMA_BACK_SOLENOID_EXTEND, RobotMap.PCM_PORT_PUMA_BACK_SOLENOID_RETRACT);
    	_shifterSolenoid = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_SHIFTER_SOLENOID_EXTEND, RobotMap.PCM_PORT_SHIFTER_SOLENOID_RETRACT);
    	
    	// ==================
    	// Servo
    	// ==================
    	_cupidServo = new Servo(RobotMap.CUPID_SERVO_PWM_PORT);
    	
    	//===================
    	// Cameras
    	//	the camera name (ex "cam0") can be found through the roborio web interface
    	//===================
        server = DynamicCameraServer.getInstance();
        server.setQuality(25);
        _currentCameraName = RobotMap.SHOOTER_CAMERA_NAME;
        server.startAutomaticCapture(_currentCameraName);
    	
        //===================
        // Smart DashBoard User Input
        //   http://wpilib.screenstepslive.com/s/3120/m/7932/l/81109-choosing-an-autonomous-program-from-smartdashboard
        //===================
        autonModeChooser = new SendableChooser();
        autonModeChooser.addDefault("Do Nothing", RobotData.AutonMode.DO_NOTHING);
        autonModeChooser.addObject("Zero All Axis", RobotData.AutonMode.ZERO_ALL_AXIS);
        autonModeChooser.addObject("Drive Fwd", RobotData.AutonMode.DRIVE_FWD);
        autonModeChooser.addObject("Cross Defense", RobotData.AutonMode.CROSS_DEFENSE);
        SmartDashboard.putData("Auton mode chooser", autonModeChooser);
        
        autonDriveTimeChooser = new SendableChooser();
        autonDriveTimeChooser.addDefault("1 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_1);
        autonDriveTimeChooser.addObject("2 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_2);
        autonDriveTimeChooser.addObject("3 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_3);
        autonDriveTimeChooser.addObject("4 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_4);
        autonDriveTimeChooser.addObject("5 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_5);
        autonDriveTimeChooser.addObject("6 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_6);
        autonDriveTimeChooser.addObject("7 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_7);
        autonDriveTimeChooser.addObject("8 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_8);
        autonDriveTimeChooser.addObject("9 sec", RobotData.Auton_Drive_Time_In_Secs.SECS_9);
        SmartDashboard.putData("Auton Drive Time", autonDriveTimeChooser);
        
    	autonDriveThrottleChooser = new SendableChooser();
    	autonDriveThrottleChooser.addDefault("10 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_10);
    	autonDriveThrottleChooser.addObject("25 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_20);
    	autonDriveThrottleChooser.addObject("30 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_30);
    	autonDriveThrottleChooser.addObject("40 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_40);
    	autonDriveThrottleChooser.addObject("50 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_50);
    	autonDriveThrottleChooser.addObject("60 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_60);
    	autonDriveThrottleChooser.addObject("70 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_70);
    	autonDriveThrottleChooser.addObject("80 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_80);
    	autonDriveThrottleChooser.addObject("90 %", RobotData.Auton_Drive_Throttle_Percent.PERCENT_90);
    	SmartDashboard.putData("Auton Drive", autonDriveThrottleChooser);
    	
    	autonCrossDefenseTypeChooser = new SendableChooser();
    	autonCrossDefenseTypeChooser.addObject("MOAT", RobotData.Auton_Cross_Defense_Type.MOAT);
    	autonCrossDefenseTypeChooser.addObject("RAMPARTS", RobotData.Auton_Cross_Defense_Type.RAMPARTS);
    	autonCrossDefenseTypeChooser.addObject("ROCKWALL", RobotData.Auton_Cross_Defense_Type.ROCKWALL);
    	autonCrossDefenseTypeChooser.addObject("ROUGH_TERRAIN", RobotData.Auton_Cross_Defense_Type.ROUGH_TERRAIN);
    	SmartDashboard.putData("Auton Cross Defense", autonCrossDefenseTypeChooser);
    	
    	autonCrossDefensePositionChooser = new SendableChooser();
    	autonCrossDefensePositionChooser.addDefault("Disabled", RobotData.Auton_Cross_Defense_Position.ZERO);
    	autonCrossDefensePositionChooser.addObject("Position 2", RobotData.Auton_Cross_Defense_Position.TWO);
    	autonCrossDefensePositionChooser.addObject("Position 3", RobotData.Auton_Cross_Defense_Position.THREE);
    	autonCrossDefensePositionChooser.addObject("Position 4", RobotData.Auton_Cross_Defense_Position.FOUR);
    	autonCrossDefensePositionChooser.addObject("Position 5", RobotData.Auton_Cross_Defense_Position.FIVE);
    	SmartDashboard.putData("Auton Auto Shooter Mode", autonCrossDefensePositionChooser);
    	
        //===================
    	// write jar (build) date & time to the dashboard
        //===================
    	try
    	{
    		//DriverStation.reportError("** Team 4028 The Beak Squad **", false);
    		
    		//get the path of the currently executing jar file
			String currentJarFilePath = Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();		
			//DriverStation.reportError(currentJarFilePath , false);
			Path filePath = Paths.get(currentJarFilePath);
			
			//get file system details from current file
			BasicFileAttributes attr = Files.readAttributes(filePath, BasicFileAttributes.class);
			Date utcFileDate = new Date(attr.lastModifiedTime().toMillis());
	
			// convert from UTC to local time zone
			SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
			outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
			String newDateString = outputFormatter.format(utcFileDate);
			
			// write the build date & time to the operator's console log window
			DriverStation.reportError("Build Date and Time: " + newDateString + "|", false);
		} 
    	catch (URISyntaxException e) 
    	{
    		DriverStation.reportError("Error determining filename of current JAR file", true);
			//e.printStackTrace();
		} 
    	catch (IOException e) 
    	{	
    		DriverStation.reportError("General Error trying to determine current JAR file", true);
			//e.printStackTrace();
    	}
    	    	
    	//===================
    	// Vision Server Client
    	//===================
    	// Start the imaging thread
     	_visionLiveData = new VisionData();
     	_visionClient = VisionClient.getInstance();
     	_visionClient.startPolling();
    }
        
    // ========================================================================
    //
    // This method is called once at the start of Antonomous Mode
    //
    // ========================================================================
    public void autonomousInit() 
    {
    	// create a new instance of the RobotData object
    	_robotLiveData = new RobotData();
    	
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	// Set desired initial (default) solenoid positions
    	outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_DOWN_POSITION;
    	outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_DOWN_POSITION;
    	outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_LOW_GEAR_POSITION;
    	
    	// init some working variables
    	_isInfeedPeriodZeroMode = false;
    	
    	// get user input values from the Smart Dashboard
    	inputDataValues.AutonModeRequested = (RobotData.AutonMode) autonModeChooser.getSelected();
    	inputDataValues.AutonDriveTimeInSecsRequested = (RobotData.Auton_Drive_Time_In_Secs) autonDriveTimeChooser.getSelected();
    	inputDataValues.AutonDriveThrottlePercentRequested = (RobotData.Auton_Drive_Throttle_Percent) autonDriveThrottleChooser.getSelected();
    	inputDataValues.AutonCrossDefenseTypeRequested = (RobotData.Auton_Cross_Defense_Type) autonCrossDefenseTypeChooser.getSelected();
    	inputDataValues.AutonCrossDefensePosition = (RobotData.Auton_Cross_Defense_Position) autonCrossDefensePositionChooser.getSelected();
    	
    	// write out the selected Auton Mode
    	DriverStation.reportError("AutonModeRequested: [" + inputDataValues.AutonModeRequested.toString() + "]", false);
   	
    	// decide what to do based on the selected Auton Mode
    	switch(inputDataValues.AutonModeRequested)
    	{
    		case DO_NOTHING:
    			break;
    			
    		case ZERO_ALL_AXIS:
    			// in this auton mode we just sit still and zero all the axis to save time in telop
    	    	
    	    	if (!_isInfeedTiltAxisZeroedYet)
    	    	{
    	    		_infeedTiltZeroStartTime = System.currentTimeMillis();
    	    		_infeedTiltZeroState = Infeed_Tilt_Zero_State.TILT_TO_HOME;
    	    		ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    	    	}
    			break;

    			
    		case DRIVE_FWD:
    	    	DriverStation.reportError("DriveTimeInSecs: [" + inputDataValues.AutonDriveTimeInSecsRequested.toString() + "]", false);
    	    	DriverStation.reportError("DriveThrottlePercent: [" + inputDataValues.AutonDriveThrottlePercentRequested.toString() + "]", false);
    	    	
    	    	// determine the requested drive time
    			switch(inputDataValues.AutonDriveTimeInSecsRequested)
    	    	{
    	    		case SECS_1:
    	    			_autonTargetDriveTimeMSecs = 1 * 1000;
    	    			break;
    	    			
    	    		case SECS_2:
    	    			_autonTargetDriveTimeMSecs = 2 * 1000;
    	    			break;	
    	    			
    	    		case SECS_3:
    	    			_autonTargetDriveTimeMSecs = 3 * 1000;
    	    			break;
    	    			
    	    		case SECS_4:
    	    			_autonTargetDriveTimeMSecs = 4 * 1000;
    	    			break;
    	    			
    	    		case SECS_5:
    	    			_autonTargetDriveTimeMSecs = 5 * 1000;
    	    			break;
    	    			
    	    		case SECS_6:
    	    			_autonTargetDriveTimeMSecs = 6 * 1000;
    	    			break;
    	    			
    	    		case SECS_7:
    	    			_autonTargetDriveTimeMSecs = 7 * 1000;
    	    			break;
    	    			
    	    		case SECS_8:
    	    			_autonTargetDriveTimeMSecs = 8 * 1000;
    	    			break;
    	    			
    	    		case SECS_9:
    	    			_autonTargetDriveTimeMSecs = 9 * 1000;
    	    			break;
    	    	}
    			
    			// determine the requested drive throttle 
    			switch(inputDataValues.AutonDriveThrottlePercentRequested)
    	    	{
    	    		case PERCENT_10:
    	    			_autonTargetDriveThrottlePercent = 0.10;
    	    			break;
    	    			
    	    		case PERCENT_20:
    	    			_autonTargetDriveThrottlePercent = 0.20;
    	    			break;
    	    			
    	    		case PERCENT_30:
    	    			_autonTargetDriveThrottlePercent = 0.30;
    	    			break;
    	    			
    	    		case PERCENT_40:
    	    			_autonTargetDriveThrottlePercent = 0.40;
    	    			break;
    	    			
    	    		case PERCENT_50:
    	    			_autonTargetDriveThrottlePercent = 0.50;
    	    			break;
    	    			
    	    		case PERCENT_60:
    	    			_autonTargetDriveThrottlePercent = 0.60;
    	    			break;
    	    		
    	    		case PERCENT_70:
    	    			_autonTargetDriveThrottlePercent = 0.70;
    	    			break;
    	    			
    	    		case PERCENT_80:
    	    			_autonTargetDriveThrottlePercent = 0.80;
    	    			break;
    	    			
    	    		case PERCENT_90:
    	    			_autonTargetDriveThrottlePercent = 0.90;
    	    			break;
    	    	}
    			    			
    			// puma up to cross defenses
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    			outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_LOW_GEAR_POSITION;
    			// low gear to cross defenses
    			
    			workingDataValues.AutonDriveFwdStartTime = new Date().getTime();
    			
    			
    			break;
    		
    		case CROSS_DEFENSE:
    	    	DriverStation.reportError("CrossDefenseType: [" + inputDataValues.AutonCrossDefenseTypeRequested.toString() + "]", false);
    	    	
    	    	// determine the requested drive throttle 
    			switch(inputDataValues.AutonCrossDefenseTypeRequested)
    	    	{
    				case MOAT:
    					_autonTargetDriveTimeMSecs = 3 * 1000;
    					_autonTargetDriveThrottlePercent = 0.70;
    					break;
    					
    				case RAMPARTS:
    					_autonTargetDriveTimeMSecs = 6 * 1000;
    					_autonTargetDriveThrottlePercent = 0.40;
    					break;
    					
    				case ROCKWALL:
    					_autonTargetDriveTimeMSecs = 6 * 1000;
    					_autonTargetDriveThrottlePercent = 0.40;
    					break;
    					
    				case ROUGH_TERRAIN:
    					_autonTargetDriveTimeMSecs = 6 * 1000;
    					_autonTargetDriveThrottlePercent = 0.40;
    					break;    				
    	    	}
    			
    			DriverStation.reportError("DriveTimeInSecs: [" + _autonTargetDriveTimeMSecs + "]", false);
    	    	DriverStation.reportError("DriveThrottlePercent: [" + _autonTargetDriveThrottlePercent + "]", false);
    	    	DriverStation.reportError("CrossDefensePosition: [" + inputDataValues.AutonCrossDefensePosition.toString() + "]", false);
    			
    			// puma up to cross defenses
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    			outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_LOW_GEAR_POSITION;
    			
    			// perimeter expansion should be out the entire match
    			//outputDataValues.PerimeterSolenoidPosition = RobotMap.PERIMETER_EXPANSION_IN;
    			
    			workingDataValues.AutonDriveFwdStartTime = new Date().getTime();
    			
	    		//_crossDefenseAutonState = Cross_Defense_Auton_State.DRIVE_AND_ZERO;
	    		DriverStation.reportError("Changing Auton State To: ZERO_AXES_AND_DRIVE | ", false);
    	    	break;
    	    	
    		default:
    			break;
    	}
    	
    	// Optionally Setup logging to a usb stick
    	setupLogging("auton");
    }

	// ========================================================================
    // This method is called every loop (about 50 x / sec) or 1 x every 20 mSec
    // ========================================================================
    public void autonomousPeriodic() {	
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	// ===============================
    	// Step 1: Get Inputs
    	// ===============================
    	UpdateInputAndCalcWorkingDataValues(inputDataValues, workingDataValues);
    	
    	// ===============================
    	// Step 2: call the appropriate auton mode method
    	// ===============================
    	switch(inputDataValues.AutonModeRequested)
    	{
    		case DO_NOTHING:
    			autonomousDoNothing();
    			break;
    			
    	    case ZERO_ALL_AXIS:
    	     	autonomousZeroAllAxis();
    	     	break;
    	    	
    	    case DRIVE_FWD:
    	    	autonomousDriveFwd();
    	    	break;
    	    	
    	    case CROSS_DEFENSE:
    	    	autonomousCrossDefense();
    	    	break;
    	    		
			default:
				break;
    	}
    	
    	// ===============================
    	// Step 3: Set outputs
    	//			to avoid unintended consequences... 
    	//				for a giveb auton mode we only set the outputs that we know we should be be using
    	// ===============================
    	
    	if (inputDataValues.AutonModeRequested == RobotData.AutonMode.ZERO_ALL_AXIS)
		{
    		if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_infeedTiltMtr.set(outputDataValues.InfeedTiltTargetPositionInRotationsCmd);
        	}
        	else if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_infeedTiltMtr.set(outputDataValues.InfeedTiltMtrVelocityCmd);
        	}
		}
    	else if (inputDataValues.AutonModeRequested == RobotData.AutonMode.DRIVE_FWD)
		{
    		// set motor commmands
        	_robotDrive.arcadeDrive(-1.0 * outputDataValues.ArcadeDriveThrottleAdjCmd, outputDataValues.ArcadeDriveTurnAdjCmd, false);
        	
        	// set solenoids
        	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
        	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
        	_shifterSolenoid.set(outputDataValues.ShifterSolenoidPosition);
		}
    	else if (inputDataValues.AutonModeRequested == RobotData.AutonMode.CROSS_DEFENSE)
    	{
    		// set motor commmands
        	_robotDrive.arcadeDrive(-1.0 * outputDataValues.ArcadeDriveThrottleAdjCmd, outputDataValues.ArcadeDriveTurnAdjCmd, false);
        	
        	// set solenoids
        	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
        	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
        	_shifterSolenoid.set(outputDataValues.ShifterSolenoidPosition);
    	}
    	
    	// ==============================
    	// Step 4: Update the Dashboard
    	// ==============================
    	UpdateDashboard(_robotLiveData);
    	
    	// set last scan DT
    	workingDataValues.LastScanDT = new Date();
    	
    	// optionally send messages to the driver station
    	if ((outputDataValues.DriversStationMsg != null) && (outputDataValues.DriversStationMsg.length() > 0))
    	{
    		DriverStation.reportError(outputDataValues.DriversStationMsg, false);
    	}
    	
    	// =============================
    	// 5.0 Optional Data Logging
    	// =============================
    	
    	if(workingDataValues.IsLoggingEnabled == true)
    	{
    		_dataLogger.WriteData(_robotLiveData);
    	}
    	
    }
        
    // this auton mode just does nothing
    public void autonomousDoNothing()
    {
    }
    
    // this auton mode sits still and zeros all axes
    public void autonomousZeroAllAxis()
    {
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	if(!_isInfeedTiltAxisZeroedYet)
    	{
    		ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    	}
    }
    
    // This Auton mode supports driving formward usually across the defenses
    public void autonomousDriveFwd()
    {
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	long driveFwdElapsedTime = (new Date().getTime() - workingDataValues.AutonDriveFwdStartTime);
		if (driveFwdElapsedTime  <= _autonTargetDriveTimeMSecs)
    	{
    		outputDataValues.ArcadeDriveThrottleAdjCmd = _autonTargetDriveThrottlePercent;
    		outputDataValues.ArcadeDriveTurnAdjCmd = 0;
    	}
    	else
    	{
        	outputDataValues.ArcadeDriveThrottleAdjCmd = 0;
        	outputDataValues.ArcadeDriveTurnAdjCmd = 0;
    	}
    }
    
    public void autonomousCrossDefense()
    {
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	if(!_isInfeedTiltAxisZeroedYet)
    	{
    		ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    	}
    	
		long driveFwdElapsedTime = (new Date().getTime() - workingDataValues.AutonDriveFwdStartTime);
		    	
		if (driveFwdElapsedTime  <= _autonTargetDriveTimeMSecs)
    	{
    		outputDataValues.ArcadeDriveThrottleAdjCmd = _autonTargetDriveThrottlePercent;
    		outputDataValues.ArcadeDriveTurnAdjCmd = 0;
    	}
    	else
    	{
        	outputDataValues.ArcadeDriveThrottleAdjCmd = 0;
        	outputDataValues.ArcadeDriveTurnAdjCmd = 0;
    	}
    }
    /*
     *****************************************************************************************************
     * This function is called 1x each time the robot enters tele-operated mode
     *  (setup the initial robot state for telop mode)
     *****************************************************************************************************
     */
    public void teleopInit()
    {
    	//server.startAutomaticCapture(_currentCameraName);
    	
    	
    	_robotLiveData = new RobotData();
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	// set motors to 0 position/velocity command 
    	outputDataValues.ArcadeDriveThrottleAdjCmd = 0.0;
    	outputDataValues.ArcadeDriveTurnAdjCmd = 0.0;
    	outputDataValues.InfeedAcqMtrVelocityCmd = 0.0;
    	outputDataValues.InfeedTiltMtrVelocityCmd = 0.0;
    	outputDataValues.CupidServoPositionCmd = 0.0;
    	    	
    	// initialize axis (Encoder) positions (for any talon where we care about position but do not have a home position
    	_leftDriveMasterMtr.setPosition(0);
    	_rightDriveMasterMtr.setPosition(0);
    	
    	// set motors to output velocity command 
    	_leftDriveMasterMtr.set(outputDataValues.ArcadeDriveThrottleAdjCmd);
    	_rightDriveMasterMtr.set(outputDataValues.ArcadeDriveTurnAdjCmd);
    	_infeedAcqMtr.set(outputDataValues.InfeedAcqMtrVelocityCmd);
    	_cupidServo.set(outputDataValues.CupidServoPositionCmd);
    	  	    	
    	// init the drive speed scaling factor to 95%
    	workingDataValues.DriveSpeedScalingFactor = 0.95;

    	// set our desired default state for the puma solenoids
    	outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    	outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    	outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_LOW_GEAR_POSITION;
    	
    	// set initial state of "pressed last scan" working values to be false
    	workingDataValues.IsPumaFrontToggleBtnPressedLastScan = false;
    	workingDataValues.IsPumaBackToggleBtnPressedLastScan = false;
    	workingDataValues.IsShifterBtnPressedLastScan = false;
    	
    	// send the initial states to the solenoids
    	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
    	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
    	_shifterSolenoid.set(outputDataValues.ShifterSolenoidPosition);
    	
    	inputDataValues.IsInfeedAcquireBtnPressed = false;
    	inputDataValues.IsInfeedReleaseBtnPressed = false;
    	
    	_isInfeedPeriodZeroMode = false;
    	
    	if (!_isInfeedTiltAxisZeroedYet)
    	{
    		ZeroInfeedTiltAxis(_robotLiveData);
    	}
    	else 
    	{
    		outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_STORED_POSITION_CMD;
    	}
    	
    	// optionally shut down vision processing
    	if(_visionClient != null)
    	{
    		_visionClient.stopPolling();
    		DriverStation.reportError("Vision Polling Stopped", false);
    	}
    	
    	// start the camera
    	//server.startAutomaticCapture(_currentCameraName);
    	
    	// ===================
    	// optionally setup logging to USB Stick (if it is plugged into one of the RoboRio Host USB ports)
    	// ===================
    	setupLogging("telop");
    	
    }
    	
    /*
     *****************************************************************************************************
     * This function is called periodically during operator control
     * 	(about 50x/sec or about every 20 mSec)
     * 
     * Four (4) main steps
     * 	1.	Get Inputs
     *  2. 	Update Working Values
     *  3.	Calc new motor speeds
     *  4.	Set Outputs
     *  5.	Update Dashboard
     * 
     *****************************************************************************************************
     */
    public void teleopPeriodic() 
    {
    	// =====================================
    	// === Step 0: get local references for objects that are properties of _robotLiveData to
    	//				make variable references shorter
    	// =====================================
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
        	
    	// =====================================
    	// Step 1: Get Inputs  and Update Working Values
    	// =====================================
    	UpdateInputAndCalcWorkingDataValues(inputDataValues, workingDataValues);
    	outputDataValues.DriversStationMsg = "";    
    	
    	// =====================================
    	// === Step 2.1: Calc New Drive Motor Speeds ===
    	// =====================================
    	// set the drive speed scale factor (currently we support 0.7 & 1.0)
    	// 	notes: 	this is a toggle,  the previous value is retained between scans
    	//			need to de-bounce key press since the scan rate is so fast 
    	if(inputDataValues.IsScaleDriveSpeedUpBtnPressed 
    			&& inputDataValues.IsScaleDriveSpeedDownBtnPressed)
    	{
    		// Don't change scale factor if both buttons are pressed
    	}
    	else if(inputDataValues.IsScaleDriveSpeedUpBtnPressed 
    			&& !inputDataValues.IsScaleDriveSpeedDownBtnPressed)
    	{
    		// scale up
    		workingDataValues.DriveSpeedScalingFactor = 1;
    	}
    	else if(!inputDataValues.IsScaleDriveSpeedUpBtnPressed
    			&& inputDataValues.IsScaleDriveSpeedDownBtnPressed)
    	{
    		// scale down
    		workingDataValues.DriveSpeedScalingFactor = 0.8;
    	}
    	else if(!inputDataValues.IsScaleDriveSpeedUpBtnPressed 
    			&& !inputDataValues.IsScaleDriveSpeedDownBtnPressed)
    	{
    		// if neither button is pressed do nothing
    	}
    	
    	outputDataValues.ArcadeDriveThrottleAdjCmd 
    			= inputDataValues.ArcadeDriveThrottleRawCmd * workingDataValues.DriveSpeedScalingFactor * -1.0;  	
    	outputDataValues.ArcadeDriveTurnAdjCmd 
    			= inputDataValues.ArcadeDriveTurnRawCmd * workingDataValues.DriveSpeedScalingFactor * -0.6;

    	// =====================================
    	// Step 2.2:  Infeed Tilt (Tilt the infeed up and down)
    	// =====================================    	
    	if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		if (_isInfeedPeriodZeroMode && !_isInfeedTiltAxisZeroedYet)
    		{
    			ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    		}
    		else if (_isInfeedPeriodZeroMode && _isInfeedTiltAxisZeroedYet)
    		{
    			_infeedTiltMtr.set(RobotMap.INFEED_TILT_HOME_POSITION_IN_ROTATIONS);
    			_isInfeedPeriodZeroMode = false;
    			DriverStation.reportError("Infeed Tilt encoder rezeroed", false);
    		}
    		else if (inputDataValues.IsInfeedTiltDeployBtnPressed && !inputDataValues.IsInfeedTiltStoreBtnPressed)
    		{
    			// rotate down;
    			outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_DEPLOYED_POSITION_CMD;
    		}
    		else if (!inputDataValues.IsInfeedTiltDeployBtnPressed && inputDataValues.IsInfeedTiltStoreBtnPressed && !workingDataValues.IsInfeedTiltStoreBtnPressedLastScan)
    		{
    			// rotate up
    			//outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_STORED_POSITION_CMD;
    			
    			// special zero mode in periodic
    			_isInfeedPeriodZeroMode = true;
    			_isInfeedTiltAxisZeroedYet = false;
    			_infeedTiltZeroState = Infeed_Tilt_Zero_State.TILT_TO_HOME;
    			_infeedTiltZeroStartTime = System.currentTimeMillis();
    			
    			ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    			//outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_HOME_POSITION_IN_ROTATIONS;
    		}
    		else if (inputDataValues.IsInfeedTiltFixedBtnPressed && !workingDataValues.IsInfeedTiltFixedBtnPressedLastScan)
    		{
    			//outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_FIXED_POSITION_CMD;
    			// No longer required with smaller infeed (originally used as a counterweight)
    		}
    		else if (!inputDataValues.IsInfeedTiltDeployBtnPressed && !inputDataValues.IsInfeedTiltStoreBtnPressed)
    		{
    			if ((inputDataValues.InfeedTiltUpCmd > 0.1) && (inputDataValues.InfeedTiltDownCmd < 0.1))		// remember, "up" on the joystick is a - value, (we use .1 as joystick deadband)
    			{
    				outputDataValues.InfeedTiltTargetPositionInRotationsCmd = outputDataValues.InfeedTiltTargetPositionInRotationsCmd + 0.01;
    				
    				// If the position is greater than 90 degrees = 0.25 rotations, prevent infeed from continuing to drive up
    				if (outputDataValues.InfeedTiltTargetPositionInRotationsCmd > RobotMap.INFEED_TILT_STORED_POSITION_CMD)
    				{
    					DriverStation.reportError("Upper Soft Limit Reached", false);
    					outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_STORED_POSITION_CMD;
    				}
    			}
    			else if ((inputDataValues.InfeedTiltUpCmd < 0.1) && (inputDataValues.InfeedTiltDownCmd > 0.1))	// remember, "down" on the joystick is a + value, (we use .1 as joystick deadband)
    			{
    				outputDataValues.InfeedTiltTargetPositionInRotationsCmd = outputDataValues.InfeedTiltTargetPositionInRotationsCmd - 0.01;
    				
    				// If the position is less than 0 degrees = -0.13 rotations, prevent infeed from continuing to drive down
    				if (outputDataValues.InfeedTiltTargetPositionInRotationsCmd < RobotMap.INFEED_TILT_LOWER_LIMIT_PUMA_UP)
    				{
    					DriverStation.reportError("Lower Soft Limit Reached", false);
    					outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_LOWER_LIMIT_PUMA_UP;
    				}
    			}
    			else
    			{
    				// else we are within the joystick deadband, so do nothing
    			}
    		}
    	}
    	else if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{
    		if (_isInfeedPeriodZeroMode && !_isInfeedTiltAxisZeroTimedOut)
    		{
    			ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    		}
    		else if (_isInfeedPeriodZeroMode && _isInfeedTiltAxisZeroTimedOut && inputDataValues.IsInfeedTiltStoreBtnPressed)
    		{
    			// if infeed fails to zero (ex. if ball was in the elevator) restart the zero function 
    			
    			// special zero mode in periodic
    			_isInfeedPeriodZeroMode = true;
    			_isInfeedTiltAxisZeroedYet = false;
    			_isInfeedTiltAxisZeroTimedOut = false;
    			_infeedTiltZeroState = Infeed_Tilt_Zero_State.TILT_TO_HOME;
    			_infeedTiltZeroStartTime = System.currentTimeMillis();
    			
    			ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    			
    		}
    		else
    		{
    			// we determined that 9% will roughly hold the axis at its current position when the axis is near flat
        		outputDataValues.InfeedTiltMtrVelocityCmd = 0.09;
    		}
    	}
    	
    	// =====================================
    	// Step 2.3 Infeed Acquisition 
    	// =====================================
    	// Run infeed motors based on command from acquire and release buttons
    	if(inputDataValues.IsInfeedAcquireBtnPressed && inputDataValues.IsInfeedReleaseBtnPressed)
    	{
    		// do nothing if both buttons are pressed
    	}
    	else if (inputDataValues.IsInfeedAcquireBtnPressed)
    	{
    		outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
    	}
    	else if (inputDataValues.IsInfeedReleaseBtnPressed)
    	{
    		outputDataValues.InfeedAcqMtrVelocityCmd = 11.0;
    	}
    	
    	
    	// ===========================
    	// Step 2.4: Camera
    	// ===========================
    	if (inputDataValues.IsCameraSwitchBtnPressed 
    			&& !workingDataValues.IsCameraSwitchBtnPressedLastScan)
    	{
    		if (_currentCameraName == RobotMap.SHOOTER_CAMERA_NAME)
			{
    			_currentCameraName = RobotMap.INFEED_CAMERA_NAME;
    			DriverStation.reportError("..Switching to Infeed Camera", false);
			}
    		else if (_currentCameraName == RobotMap.INFEED_CAMERA_NAME)
    		{
    			_currentCameraName = RobotMap.SHOOTER_CAMERA_NAME;
    			DriverStation.reportError("..Switching to Shooter Camera", false);
    		}
    		else
    		{
    			_currentCameraName = RobotMap.SHOOTER_CAMERA_NAME;
    			DriverStation.reportError("..Switching to Shooter Camera", false);
    		}
    		
    		server.switchAutomaticCapture(_currentCameraName);
    	}
    	else if (inputDataValues.IsCupidCameraBtnPressed
    			&& !workingDataValues.IsCupidSwitchBtnPressedLastScan)
    	{
    		_currentCameraName = RobotMap.CUPID_CAMERA_NAME;
    		DriverStation.reportError("..Switching to Cupid Camera", false);
    		
    		server.switchAutomaticCapture(_currentCameraName);
    	}
    	  
    	// ===========================
    	// Step 2.5: Winch
    	// ===========================
    	if (inputDataValues.WinchRawCmd > 0.05)
    	{
    		outputDataValues.WinchVelocityCmd = inputDataValues.WinchRawCmd;
    	}
    	else if (inputDataValues.WinchRawCmd < -0.05)
    	{
    		outputDataValues.WinchVelocityCmd = inputDataValues.WinchRawCmd;
    	}
    	else
    	{
    		outputDataValues.WinchVelocityCmd = 0.0;
    	}
    	
    	// ===========================
    	// Step 2.6: Servo
    	// ===========================
    	if (inputDataValues.IsCupidLoadBtnPressed)
    	{
    		_cupidServo.set(1);
    		DriverStation.reportError("Cupid Servo set to 1.0", false);
    	}
    	else if (inputDataValues.IsCupidShootBtnPressed)
    	{
    		_cupidServo.set(0);
    		DriverStation.reportError("Cupid Servo set to 0", false);
    	}
    	
    	// =====================================
    	// Step 3: Push the target Outputs out to the physical devices
    	// =====================================
    	
    	// ==========================
    	// 3.1 Handle Puma Front, Back and Shifter Solenoids
    	//		Solenoids work like a toggle, the current value is retained until it is changed
    	// ==========================
    	if (!workingDataValues.IsPumaBothToggleBtnPressedLastScan && inputDataValues.IsPumaBothToggleBtnPressed)
    	{
    		if ((outputDataValues.PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION) 
    				|| (outputDataValues.PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_UP_POSITION))
    		{
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_DOWN_POSITION;
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_DOWN_POSITION;
    		}
    		else
    		{
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    		}
    	}
    	
    	if (!workingDataValues.IsPumaFrontToggleBtnPressedLastScan && inputDataValues.IsPumaFrontToggleBtnPressed)
    	{
    		if (outputDataValues.PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION)
    		{
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_DOWN_POSITION;
    		}
    		else
    		{
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    		}
    	}
    	
    	if (!workingDataValues.IsPumaBackToggleBtnPressedLastScan && inputDataValues.IsPumaBackToggleBtnPressed)
    	{
    		if (outputDataValues.PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_UP_POSITION)
    		{
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_DOWN_POSITION;
    		}
    		else
    		{
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    		}
    	}
    	
    	if (!workingDataValues.IsShifterBtnPressedLastScan && inputDataValues.IsShifterBtnPressed)
    	{
    		if (outputDataValues.ShifterSolenoidPosition == RobotMap.SHIFTER_HIGH_GEAR_POSITION)
    		{
    			outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_LOW_GEAR_POSITION;
    		}
    		else
    		{
    			outputDataValues.ShifterSolenoidPosition = RobotMap.SHIFTER_HIGH_GEAR_POSITION;
    		}
    	}
    		 
    	
    	// set solenoids state
    	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
    	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
    	_shifterSolenoid.set(outputDataValues.ShifterSolenoidPosition);
    	
    	// set motor commmands
    	_robotDrive.arcadeDrive((-1.0 * outputDataValues.ArcadeDriveThrottleAdjCmd), (-1.0 * outputDataValues.ArcadeDriveTurnAdjCmd), false);
    	
    	_infeedAcqMtr.set(outputDataValues.InfeedAcqMtrVelocityCmd);
    	
    	if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		_infeedTiltMtr.set(outputDataValues.InfeedTiltTargetPositionInRotationsCmd);
    	}
    	else if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{
    		_infeedTiltMtr.set(outputDataValues.InfeedTiltMtrVelocityCmd);
    	}
    	
    	_winchMtr.set(outputDataValues.WinchVelocityCmd);
    	
    	// ==========================
    	// 4.0 publish image from currently selected Camera to the dashboard
    	// ==========================
    	//VisionData visionData = _visionClient.GetVisionData();
    	//visionData.DesiredTurretTurnInDegrees
    	//CameraServer.getInstance().setImage(_cameraFrame);
    	
    	// ==========================
    	// 5.0 Update Dashboard
    	// ==========================
    	UpdateDashboard(_robotLiveData);
    	
    	workingDataValues.LastScanDT = new Date();  	
    	
    	// optionally send message to drivers station
    	if(outputDataValues.DriversStationMsg != null 
    			&& outputDataValues.DriversStationMsg.length() > 0)
    	{
    		DriverStation.reportError(outputDataValues.DriversStationMsg, false);
    	}
    	
    	// ==========================
    	// 6.0 Optional Data logging
    	// ==========================
    	if(workingDataValues.IsLoggingEnabled == true)
    	{
    		_dataLogger.WriteData(_robotLiveData);
    	}
    	// set last scan DT

    	// ==========================
    	// 7.0 Stuff we want to do at the very end (because they operate as toggles)
    	// ==========================
    	workingDataValues.IsPumaFrontToggleBtnPressedLastScan = inputDataValues.IsPumaFrontToggleBtnPressed;
    	workingDataValues.IsPumaBackToggleBtnPressedLastScan = inputDataValues.IsPumaBackToggleBtnPressed;
    	workingDataValues.IsPumaBothToggleBtnPressedLastScan = inputDataValues.IsPumaBothToggleBtnPressed;
    	workingDataValues.IsShifterBtnPressedLastScan = inputDataValues.IsShifterBtnPressed;
    	workingDataValues.IsInfeedTiltStoreBtnPressedLastScan = inputDataValues.IsInfeedTiltStoreBtnPressed;
    	workingDataValues.IsInfeedTiltFixedBtnPressedLastScan = inputDataValues.IsInfeedTiltFixedBtnPressed;
    	workingDataValues.IsInfeedAcquireBtnPressedLastScan = inputDataValues.IsInfeedAcquireBtnPressed;
    	workingDataValues.IsCameraSwitchBtnPressedLastScan =  inputDataValues.IsCameraSwitchBtnPressed;
    	workingDataValues.IsCupidSwitchBtnPressedLastScan = inputDataValues.IsCupidCameraBtnPressed;
    }
    
    public void disabledPeriodic()
    {
    	if(_dataLogger != null)
    	{
    		_dataLogger.close();
    		_dataLogger = null;
    	}
    }
    
    // ==========  Absolute Axis Homing Logic ==============================================
 
    // This method Zeros (ie Homes) the Infeed Tilt Axis
	private void ZeroInfeedTiltAxis(RobotData p_robotLiveData) 
	{
		//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	boolean isTimeout = false; //control whether we want this method to run at startup
    	boolean isOnHomeSwitch = !_infeedTiltMtr.isFwdLimitSwitchClosed();	// switch is normally closed
    	
    	// zero the current encoder reading
    	_infeedTiltMtr.setPosition(0);	
    	
    	// if we are not already on the up limit switch
    	if(!isOnHomeSwitch)
    	{
	    	// start out in %VBUS mode
	    	_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    	
	    	// drive the axis up at 19%
	    	outputDataValues.InfeedTiltMtrVelocityCmd = 0.35;
	    	_infeedTiltMtr.set(outputDataValues.InfeedTiltMtrVelocityCmd);
	    	
	    	long startTime = System.currentTimeMillis();
	    	long elapsedTime = 0L;
	    	long maxTimeInMSec = 10000; // 10 secs
	    	
	    	// if we are not on the limit switch, drive up until we hit it but only wait for 10 secs max
	    	while(!isOnHomeSwitch && !isTimeout)
	    	{	
	        	elapsedTime = (new Date().getTime() - startTime);
	    		
	    		if (elapsedTime  >= maxTimeInMSec)
	    		{
	    			isTimeout = true;
	    			_isInfeedTiltAxisZeroTimedOut = true;
	    		}
	    		isOnHomeSwitch = !_infeedTiltMtr.isFwdLimitSwitchClosed();	// switch is normally closed
	    	}
    	}
    	
    	// we are on the UP Limit Switch (and we did not timeout)
    	if(!isTimeout)
    	{
	    	_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.Position);
	    	_isInfeedTiltAxisZeroTimedOut = false;
	    	// once we hit the home switch, reset the encoder	- this is at approx 106deg
	    	
	    	_infeedTiltMtr.setPosition(RobotMap.INFEED_TILT_HOME_POSITION_IN_ROTATIONS);	
	    	//_infeedTiltMtr.setEncPosition(newPosition);
	    	try {
	    		// sleep a little to let the zero occur
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    
	    	// setup the PID Loop
	    	_infeedTiltMtr.setPID(RobotMap.INFEED_TILT_KP, RobotMap.INFEED_TILT_KI, RobotMap.INFEED_TILT_KD, RobotMap.INFEED_TILT_KF, RobotMap.INFEED_TILT_IZONE, RobotMap.INFEED_TILT_RAMPRATE, RobotMap.INFEED_TILT_PROFILE);
	    	_infeedTiltMtr.setProfile(RobotMap.INFEED_TILT_PROFILE);
	    	_infeedTiltMtr.setCloseLoopRampRate(RobotMap.INFEED_TILT_RAMPRATE);
	    	_infeedTiltMtr.setVoltageRampRate(5);
	    	
	    	// write to the operator's console log window
	    	DriverStation.reportError("..Infeed Tilt Axis Zeroed, Chging to Position Ctrl Mode.", false);
	    	
	    	outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_STORED_POSITION_CMD;
	    	
	    	// finally mark the axis as zeroed
	    	_isInfeedTiltAxisZeroedYet = true;
    	}
    	else
    	{
    		// write to the operator's console log window
	    	DriverStation.reportError("..ERROR: Timeout in Infeed Tilt Axis Zero procedure.", false);
    	}
	}
	
	private void ZeroInfeedTiltAxisReEntrant(RobotData p_robotLiveData) 
	{
		//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	boolean isTimeout = false; //control whether we want this method to run at startup
    	boolean isOnHomeSwitch = !_infeedTiltMtr.isFwdLimitSwitchClosed();	// switch is normally closed
    	
    	// zero the current encoder reading
    	//_infeedTiltMtr.setPosition(0);	
    	
    	switch (_infeedTiltZeroState)
    	{
    		case TILT_TO_HOME:
    			if(!isOnHomeSwitch)
    	    	{
    		    	long elapsedTime = (new Date().getTime() - _infeedTiltZeroStartTime);
    		    	long maxTimeInMSec = 8000; // 10 secs
    		    	
    		    	// if we are not on the limit switch, drive up until we hit it but only wait for 10 secs max
    		    	if (elapsedTime  >= maxTimeInMSec)	   
    		    	{
			    		_infeedTiltZeroState = Infeed_Tilt_Zero_State.TIMEOUT;
    		    	}
    		    	else
    		    	{
    		    		// start out in %VBUS mode
    		    		if (_infeedTiltMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus)
    		    		{
    		    			_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    		    		}
    		    		// drive the axis up at 19%
        		    	outputDataValues.InfeedTiltMtrVelocityCmd = 0.6;
    		    	}
    	    	}
    			else 
    			{
    				_infeedTiltZeroState = Infeed_Tilt_Zero_State.ON_HOME;
    			}
    			break;
    		
    		case ON_HOME:
    			_isInfeedTiltAxisZeroTimedOut = false;
    			outputDataValues.InfeedTiltMtrVelocityCmd = 0.0;
    			_infeedTiltMtr.changeControlMode(CANTalon.TalonControlMode.Position);
    	    	
    	    	// once we hit the home switch, reset the encoder	- this is at approx 106deg
    	    	_infeedTiltMtr.setPosition(RobotMap.INFEED_TILT_HOME_POSITION_IN_ROTATIONS);	
    	    	//_infeedTiltMtr.setEncPosition(newPosition);
    	    	try {
    	    		// sleep a little to let the zero occur
    				Thread.sleep(1);
    			} catch (InterruptedException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    			}
    	    
    	    	// setup the PID Loop
    	    	_infeedTiltMtr.setPID(RobotMap.INFEED_TILT_KP, RobotMap.INFEED_TILT_KI, RobotMap.INFEED_TILT_KD, RobotMap.INFEED_TILT_KF, RobotMap.INFEED_TILT_IZONE, RobotMap.INFEED_TILT_RAMPRATE, RobotMap.INFEED_TILT_PROFILE);
    	    	_infeedTiltMtr.setProfile(RobotMap.INFEED_TILT_PROFILE);
    	    	_infeedTiltMtr.setCloseLoopRampRate(RobotMap.INFEED_TILT_RAMPRATE);
    	    	_infeedTiltMtr.setVoltageRampRate(5);
    	    	
    	    	// write to the operator's console log window
    	    	DriverStation.reportError("..Infeed Tilt Axis Zeroed, Chging to Position Ctrl Mode.", false);
    	    	
    	    	_infeedTiltZeroState = Infeed_Tilt_Zero_State.GO_TO_REQUESTED_POSITION;
    			break;
    			
    		case GO_TO_REQUESTED_POSITION:
    			outputDataValues.InfeedTiltTargetPositionInRotationsCmd = (RobotMap.INFEED_TILT_STORED_POSITION_CMD);
    	    	// finally mark the axis as zeroed
    	    	_isInfeedTiltAxisZeroedYet = true;
    			break;
    			
    		case TIMEOUT:
    			_isInfeedTiltAxisZeroTimedOut = true;
    			DriverStation.reportError("Infeed Tilt Zero procedure timed out", false);
    			outputDataValues.InfeedTiltMtrVelocityCmd = 0.0;
    			break;	
    	}
	}
	
    /**
    / This method updates all of the input values
	**/
    private void UpdateInputAndCalcWorkingDataValues(InputData inputDataValues, WorkingData workingDataValues)
    {	
    	// ==========================
    	// 1.1 get high resolution timer
    	// ==========================
    	inputDataValues.FPGATimeMicroSecs = Utility.getFPGATime();
    	
    	// ==========================
    	// 1.2 get values from the gamepads
    	// ==========================
    	
    	inputDataValues.IsPumaFrontToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN);
    	inputDataValues.IsPumaBackToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN);
    	inputDataValues.IsPumaBothToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_BOTH_TOGGLE_BTN);
    	inputDataValues.IsInfeedTiltStoreBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_STORE_BTN);
    	inputDataValues.IsInfeedTiltDeployBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_DEPLOY_BTN);
    	inputDataValues.IsCupidLoadBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_CUPID_LOAD_BTN);
    	inputDataValues.IsCupidShootBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_CUPID_SHOOT_BTN);
    	inputDataValues.IsShifterBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SHIFTER_BTN);
    	
    	inputDataValues.IsInfeedAcquireBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_ACQUIRE_BTN);
    	inputDataValues.IsInfeedReleaseBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_RELEASE_BTN);
    	inputDataValues.IsCameraSwitchBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_CAMERA_SWITCH_BTN);
    	inputDataValues.IsCupidCameraBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_CUPID_CAMERA_BTN);
    	
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	inputDataValues.ArcadeDriveThrottleRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK);
    	inputDataValues.ArcadeDriveTurnRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK);
    	inputDataValues.InfeedTiltUpCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_UP_AXIS);
    	inputDataValues.InfeedTiltDownCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_DOWN_AXIS);
    	
    	inputDataValues.WinchRawCmd = _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_WINCH_AXIS);
 		
    	// ==========================
    	// 1.3 get values from motor controlllers
    	// ==========================	
    	inputDataValues.InfeedTiltEncoderCurrentCount = _infeedTiltMtr.getPosition();
    	inputDataValues.IsInfeedTiltAxisOnUpLimitSwitch = !_infeedTiltMtr.isFwdLimitSwitchClosed();
    	
    	// ==========================
    	// 1.6 get values from the last available scan on the Vision PC
    	// ==========================
    	VisionData visionData = _visionClient.GetVisionData();
    	if(visionData != null)
    	{
    		inputDataValues.DesiredTurretTurnInDegrees = visionData.DesiredTurretTurnInDegrees;
    	}
    	else
    	{
    		inputDataValues.DesiredTurretTurnInDegrees = 0;
    	}
    }	
    
    /**
    / This method updates the Smart Dashboard
	**/
    private void UpdateDashboard(RobotData robotDataValues)
    {
    	//get local references
    	InputData inputDataValues = robotDataValues.InputDataValues;
    	WorkingData workingDataValues = robotDataValues.WorkingDataValues;
    	OutputData outputDataValues = robotDataValues.OutputDataValues;
    	VisionData visionData = _visionClient.GetVisionData();

		// Axis Home Routines
		SmartDashboard.putBoolean("Is Infeed Zeroed", _isInfeedTiltAxisZeroedYet);
		
		// Kangaroo Charge Level
		//SmartDashboard.putNumber("Kangaroo charge level", visionData.BatteryChargeLevel);

		// Logging
		//SmartDashboard.putBoolean("Log:IsLoggingEnabled", workingDataValues.IsLoggingEnabled);
		//SmartDashboard.putString("Log:LogFilePathName", workingDataValues.LogFilePathName); 
		
        ///SmartDashboard.putString("Misc:LastUpdateDT", ZonedDateTime.now().toString());
    }
    
    /**
    / This method optionally sets up logging
	**/
	private void setupLogging(String mode) 
	{
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(RobotMap.LOG_FILE_PATH);
    	if (Files.exists(path)) 
    	{
    		try 
    		{
				_dataLogger = new DataLogger(RobotMap.LOG_FILE_PATH, mode);
				_dataLogger.WriteData(_robotLiveData.BuildTSVHeader());
				
				_robotLiveData.WorkingDataValues.LogFilePathName = _dataLogger.LogFilePathName;
	    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = true;
	    		_robotLiveData.WorkingDataValues.LoggingStartedDT = new Date();
	    		_robotLiveData.WorkingDataValues.LastScanDT = new Date();
			} 
    		catch (IOException e) 
    		{
				e.printStackTrace();
				
	    		_dataLogger = null;
				_robotLiveData.WorkingDataValues.LogFilePathName = "";
	    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = false;
			}
    	}
    	else
    	{
    		_dataLogger = null;
			_robotLiveData.WorkingDataValues.LogFilePathName = "";
    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = false;
    	}
	}
	
	/*****************************************************************************************************
     * This function is called 1 time at the start of Test Mode
     *****************************************************************************************************/
	public void testInit() 
	{
	}
	
    /*****************************************************************************************************
     * This function is called periodically during test mode
     * 	(about 50x/sec or about every 20 mSec)
     *****************************************************************************************************/
    @SuppressWarnings("deprecation")
	public void testPeriodic()
    {    	
    } 
}