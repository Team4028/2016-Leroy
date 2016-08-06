package org.usfirst.frc.team4028.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
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
import org.usfirst.frc.team4028.robot.RobotData.Auto_Aim_And_Shoot_State;
import org.usfirst.frc.team4028.robot.RobotData.AutonMode;
import org.usfirst.frc.team4028.robot.RobotData.Auton_Drive_Throttle_Percent;
import org.usfirst.frc.team4028.robot.RobotData.Auton_Shoot_Ball_State;
import org.usfirst.frc.team4028.robot.RobotData.Cross_Defense_Auto_Aim_And_Shoot_State;
import org.usfirst.frc.team4028.robot.RobotData.Cross_Defense_Auton_State;
import org.usfirst.frc.team4028.robot.RobotData.Infeed_Tilt_Zero_State;
import org.usfirst.frc.team4028.robot.RobotData.InputData;
import org.usfirst.frc.team4028.robot.RobotData.OutputData;
import org.usfirst.frc.team4028.robot.RobotData.Slider_Zero_State;
import org.usfirst.frc.team4028.robot.RobotData.Teleop_Elevator_State;
import org.usfirst.frc.team4028.robot.RobotData.Turret_Zero_State;
import org.usfirst.frc.team4028.robot.RobotData.WorkingData;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

import com.kauailabs.navx.frc.AHRS;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

/**
 * Date			Rev		Author						Comments
 * -----------	------	-------------------------	----------------------------------
 * 18.Feb.2-16 	0.81	Sebastian Rodriguez			Updated controller to run infeed and drive off of one motor, added all shooter motors
 * 15.Feb.2016	0.8		Sebastian Rodriguez			Added more untested auton base code, refined pid control for turret
 * 8.Feb.2016	0.7		Sebastian Rodriguez			PID control for the turret, untested auton mode
 * 6.Feb.2016	0.6		Sebastian Rodriguez			Added Infeed code
 * 4.Feb.2016	0.5		Sebastian Rodriguez			Added Turret and turret encoder
 * 28.Jan.2016	0.4		Sebastian Rodriguez			Added untested navx code
 * 23.Jan.2016	0.3		Sebastian Rodriguez			Changed to 6 wheel drive train
 * 18.Jan.2016	0.21	Sebastian Rodriguez			Debugging to allow for Solenoid functionality
 * 16.Jan.2016 	0.2		Sebastian Rodriguez			Added Untested Solenoid functionality
 * 													Setup two additional Talons and two Victors
 * 15.Jan.2016	0.1		Sebastian Rodriguez			Initial Version
 * 													-Basic Robot Config
 * 													-Added Arcade Drive functionality
 *
 */

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
	private CANTalon _turretMtr;
	private CANTalon _shooterMasterMtr;
	private CANTalon _shooterSlaveMtr;
	private CANTalon _sliderMtr;
	private CANTalon _infeedTiltMtr;
	
	// CIM DC Motors on Victor SP Speed Controllers (via PWM Ports)
	private VictorSP _infeedAcqMtr;
	private VictorSP _kickerMtr;
	private VictorSP _winchMtr;
	
	// Limit switches for turret
	private DigitalInput _turretHomeLimitSwitch;
	private DigitalInput _turretApproachingHomeLimitSwitch;
	private DigitalInput _isBallInPositionLimitSwitch;
	
	// Arcade Drive with four drive motors
	private RobotDrive _robotDrive;
	
	// Pneumatic Solenoids for Air Cylinders
	private DoubleSolenoid _pumaFrontSolenoid;
	private DoubleSolenoid _pumaBackSolenoid;
	private DoubleSolenoid _shifterSolenoid;
	private DoubleSolenoid _perimeterExpansionSolenoid;
	
	// Camera
	DynamicCameraServer server;
	private String _currentCameraName;
	
	// Vision Server Client
	private VisionClient _visionClient;
	
	// navX
	private AHRS _navXSensor;
	
	// Servo
	private Servo _cupidServo;
	
	// Vision 
	private Socket _visionServer;	
	private DataInputStream _inFromServer;
	private DataOutputStream _outToServer;
	
	private String visionData;
	
	private Long lastCycleTime;
	private Thread _pollingThread;
	
	// ==================
	// PID Controller
	// ==================
	//private PIDController _turretControl;
	//private Encoder _turretEncoder;
	
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
	SendableChooser autonPumaBackPositionChooser;
	SendableChooser autonSliderPositionChooser;
	SendableChooser autonShooterWheelRPMChooser;
	SendableChooser autonDriveTimeChooser;
	SendableChooser autonDriveThrottleChooser;
	SendableChooser autonCrossDefenseTypeChooser;
	SendableChooser autonCrossDefensePositionChooser;
	
	double _sliderAutonPosition = 0.0;
	double _turretAutonPosition = 0.0;
	boolean _isTurretAxisZeroedYet = false;
	boolean _isInfeedTiltAxisZeroedYet = false;
	boolean _isSliderAxisZeroedYet = false;
	int _autonShooterWheelTargetRPM = 0;
	double _autonTargetDriveTimeMSecs = 0;
	double _autonTargetDriveThrottlePercent = 0;
	boolean  _isAutonAutoShooterEnabled = false;
	
	boolean _isInfeedPeriodZeroMode = false;
	boolean _isInfeedTiltAxisZeroTimedOut = false;
	
	boolean _isSliderAxisZeroTimedOut = false;
	
	Slider_Zero_State _sliderZeroState;
	long _sliderZeroStartTime;
	Infeed_Tilt_Zero_State _infeedTiltZeroState;
	long _infeedTiltZeroStartTime;
	Turret_Zero_State _turretZeroState;
	long _turretZeroStartTime;
	Cross_Defense_Auton_State _crossDefenseAutonState;
	long _crossDefenseAutonStartTime;
	Auto_Aim_And_Shoot_State _autoAimAndShootState;
	Cross_Defense_Auto_Aim_And_Shoot_State _crossDefenseAutoAimAndShootState;

	
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
    	//_leftDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
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
    	//_rightDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
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
    	// Turret
    	// ===================
    	_turretMtr = new CANTalon(RobotMap.CAN_ADDR_TURRET_TALON);
    	_turretMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	_turretMtr.reverseSensor(false);
    	_turretMtr.enableBrakeMode(true);
    	_turretMtr.enableLimitSwitch(false, false);
    	_turretMtr.configNominalOutputVoltage(+0.0f, -0.0f);
    	_turretMtr.configPeakOutputVoltage(+5.0f, -5.0f);
    	
    	// ===================
    	// Shooter
    	// ===================
    	_shooterMasterMtr = new CANTalon(RobotMap.CAN_ADDR_MASTER_SHOOTER_TALON);
    	_shooterMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	_shooterMasterMtr.reverseSensor(true);
    	_shooterMasterMtr.enableBrakeMode(false);
    	_shooterMasterMtr.configNominalOutputVoltage(+0.0f, -0.0f);
    	_shooterMasterMtr.configPeakOutputVoltage(+12.0f, 0.0f);
    	_shooterMasterMtr.changeControlMode(CANTalon.TalonControlMode.Speed);
    	//_shooterMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);  // 
    	_shooterMasterMtr.configEncoderCodesPerRev(RobotMap.SHOOTER_ENCODER_COUNTS_PER_REV);	// try to enable Unit Scaling
    	_shooterMasterMtr.enableLimitSwitch(false, false);
    	
    	// setup shooter PID Loop
    	// shooterMaxVelociytInNativeUnitsPer100mSec = CalcShooterVelociytInNativeUnitsPer100mSec(RobotMap.SHOOTER_MAX_MOTOR_RPM);
    	//double shooterFeedFwdGain = CalcShooterFeedFwdGain(shooterMaxVelociytInNativeUnitsPer100mSec);
    	_shooterMasterMtr.setPID(RobotMap.SHOOTER_KP, RobotMap.SHOOTER_KI, RobotMap.SHOOTER_KD, RobotMap.SHOOTER_KF, RobotMap.SHOOTER_IZONE, RobotMap.SHOOTER_RAMPRATE, RobotMap.SHOOTER_PROFILE);
    	_shooterMasterMtr.setProfile(RobotMap.SHOOTER_PROFILE);
    	
    	_shooterSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_SLAVE_SHOOTER_TALON);
    	_shooterSlaveMtr.changeControlMode(CANTalon.TalonControlMode.Follower);
    	_shooterSlaveMtr.set(RobotMap.CAN_ADDR_MASTER_SHOOTER_TALON);
    	_shooterSlaveMtr.enableBrakeMode(false);
    	_shooterSlaveMtr.enableLimitSwitch(false, false);
    	
    	// ===================
    	// Kicker
    	// ===================
    	_kickerMtr = new VictorSP(RobotMap.SHOOTER_KICKER_PWM_PORT);
    	
    	// ===================
    	// Slider
    	// ===================
    	_sliderMtr = new CANTalon(RobotMap.CAN_ADDR_SHOOTER_SLIDER_TALON);
    	_sliderMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);			// default to %VBus on startup, chgs to Position in Axis Zero Function
    	_sliderMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
    	_sliderMtr.enableLimitSwitch(false, true);
    	_sliderMtr.ConfigRevLimitSwitchNormallyOpen(false);
    	_sliderMtr.reverseSensor(true);
    	_sliderMtr.enableBrakeMode(true);
    	_sliderMtr.configNominalOutputVoltage(+0.0f, -0.0f);
    	_sliderMtr.configPeakOutputVoltage(+12.0f, -12.0f);
    	_sliderMtr.configEncoderCodesPerRev(RobotMap.SLIDER_ENCODER_COUNTS_PER_REV);	// try to enable Unit Scaling
    	    	
    	// ===================
    	// Limit Switches
    	// ===================
    	_turretHomeLimitSwitch = new DigitalInput(RobotMap.TURRET_HOME_LIMIT_SWITCH_DIO_PORT);
    	_turretApproachingHomeLimitSwitch = new DigitalInput(RobotMap.TURRET_APPROACHING_HOME_LIMIT_SWITCH_DIO_PORT);
    	_isBallInPositionLimitSwitch = new DigitalInput(RobotMap.IS_BALL_IN_POSITION_LIMIT_SWITCH);
    	
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
    	_perimeterExpansionSolenoid = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_PERIMETER_EXPANSION_EXTEND, RobotMap.PCM_PORT_PERIMETER_EXPANSION_RETRACT);
    	
    	//===================
    	// Default all Absolute Position Axes to NOT ZEROED
    	//===================
    	_isTurretAxisZeroedYet = false;
    	_isInfeedTiltAxisZeroedYet = false;
    	_isSliderAxisZeroedYet = false;
    	
    	// ==================
    	// Servo
    	// ==================
    	_cupidServo = new Servo(RobotMap.CUPID_SERVO_PWM_PORT);
    	
    	//===================
    	// PIDController
    	//===================
    	// Test code for PID loop that runs on the Roborio, haven't figured out how to get PIDSourceType to provide an input values, not sure if it will be necessary though since we can run PID loops on the talons
    	/*
    	_turretEncoder = new Encoder(0,0,1);
    	_turretEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    	_turretControl= new PIDController(1.0, 0.0, 0.0, _turretEncoder, _turret);
    	*/
    	
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
        autonModeChooser.addObject("Do Nothing", RobotData.AutonMode.DO_NOTHING);
        autonModeChooser.addObject("Zero All Axis", RobotData.AutonMode.ZERO_ALL_AXIS);
        autonModeChooser.addDefault("Shoot Ball", RobotData.AutonMode.SHOOT_BALL);
        autonModeChooser.addObject("Drive Fwd", RobotData.AutonMode.DRIVE_FWD);
        autonModeChooser.addObject("Cross Defense", RobotData.AutonMode.CROSS_DEFENSE);
        autonModeChooser.addObject("Aim & Shoot", RobotData.AutonMode.AIM_AND_SHOOT);
        SmartDashboard.putData("Auton mode chooser", autonModeChooser);
        
    	autonPumaBackPositionChooser = new SendableChooser();
    	autonPumaBackPositionChooser.addDefault("Puma Down", RobotData.Auton_Puma_Back_Position.PUMA_BACK_DOWN);
    	autonPumaBackPositionChooser.addObject("Puma Up", RobotData.Auton_Puma_Back_Position.PUMA_BACK_UP);
        SmartDashboard.putData("Auton Puma Back Position", autonPumaBackPositionChooser);
        
    	autonSliderPositionChooser = new SendableChooser();
    	autonSliderPositionChooser.addObject("24 Clicks", RobotData.Auton_Slider_Position.CLICKS_24);
    	autonSliderPositionChooser.addObject("30 Clicks", RobotData.Auton_Slider_Position.CLICKS_30);
    	autonSliderPositionChooser.addDefault("34 Clicks", RobotData.Auton_Slider_Position.CLICKS_34);
        SmartDashboard.putData("Auton Slider Position", autonSliderPositionChooser);
        
        autonShooterWheelRPMChooser = new SendableChooser();
        autonShooterWheelRPMChooser.addDefault("3500 RPM", RobotData.Auton_Shooter_Wheel_RPM.RPM_3500);
        autonShooterWheelRPMChooser.addObject("3250 RPM", RobotData.Auton_Shooter_Wheel_RPM.RPM_3250);
        autonShooterWheelRPMChooser.addObject("3000 RPM", RobotData.Auton_Shooter_Wheel_RPM.RPM_3000);
        autonShooterWheelRPMChooser.addObject("2750 RPM", RobotData.Auton_Shooter_Wheel_RPM.RPM_2750);
        autonShooterWheelRPMChooser.addObject("2500 RPM", RobotData.Auton_Shooter_Wheel_RPM.RPM_2500);
        SmartDashboard.putData("Auton Shooter Wheel Speed", autonShooterWheelRPMChooser);
        
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
    	// try to communicate to the naxX
    	//===================
    	try 
    	{
            /* Communicate w/ navX MXP via one of the following ports                           */
            /*   				I2C.Port.kMXP, 												   	*/
            /* 					SerialPort.Port.kMXP, 										   	*/
            /* 					SerialPort.Port.kUSB										   	*/
            /* 					SPI.Port.kMXP   			plugged into mxp port on RoboRio	*/			
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.  */
            //_navXSensor = new AHRS(SPI.Port.kMXP);
            _navXSensor = new AHRS(SerialPort.Port.kMXP);
            
            DriverStation.reportError("..navX sensor connected" + " |" , false);
        } 
    	catch (RuntimeException ex ) 
    	{
            DriverStation.reportError("Error connecting to navX Sensor: " + ex.getMessage() + " |", true);
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
    	/*
    	try {
			server.wait();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	*/
    	
    	// create a new instance of the RobotData object
    	_robotLiveData = new RobotData();
    	
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	// Set desired initial (default) solenoid positions
    	outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_DOWN_POSITION;
    	outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_DOWN_POSITION;
    	outputDataValues.PerimeterSolenoidPosition = RobotMap.PERIMETER_EXPANSION_IN;
    	
    	// init some working variables
    	_isInfeedPeriodZeroMode = false;
    	
    	// get user input values from the Smart Dashboard
    	inputDataValues.AutonModeRequested = (RobotData.AutonMode) autonModeChooser.getSelected();
    	inputDataValues.AutonPumaBackPositionRequested = (RobotData.Auton_Puma_Back_Position) autonPumaBackPositionChooser.getSelected();
    	inputDataValues.AutonSliderPositionRequested = (RobotData.Auton_Slider_Position) autonSliderPositionChooser.getSelected();
    	inputDataValues.AutonShooterWheelRPMRequested = (RobotData.Auton_Shooter_Wheel_RPM) autonShooterWheelRPMChooser.getSelected();
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
    			if (!_isTurretAxisZeroedYet)
    	    	{
    	    		ZeroTurretAxis(_robotLiveData);
    	    	}
    	    	
    	    	if (!_isInfeedTiltAxisZeroedYet)
    	    	{
    	    		_infeedTiltZeroStartTime = System.currentTimeMillis();
    	    		_infeedTiltZeroState = Infeed_Tilt_Zero_State.TILT_TO_HOME;
    	    		ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    	    	}
    	    	
    	    	if (!_isSliderAxisZeroedYet)
    	    	{
    	    		_sliderZeroStartTime = System.currentTimeMillis();
    	    		_sliderZeroState = Slider_Zero_State.DRIVE_TO_HOME;
    	    		//ZeroSliderAxisReEntrant(_robotLiveData, RobotMap.SLIDER_DEFAULT_TARGET_POSITION);
    	    		ZeroSliderAxis(_robotLiveData, RobotMap.SLIDER_DEFAULT_TARGET_POSITION);
    	    	}
    			break;
    			
    		case SHOOT_BALL:
    			// in this auton mode we are positioned in the front left spybot posiiton and we just shot the ball into the side goal
    	    	DriverStation.reportError("PumaAutonPositionRequested: [" + inputDataValues.AutonPumaBackPositionRequested.toString() + "]", false);
    	    	DriverStation.reportError("SliderAutonPositionRequested: [" + inputDataValues.AutonSliderPositionRequested.toString() + "]", false);
    	    	DriverStation.reportError("ShooterWheelRPMRequested: [" + inputDataValues.AutonShooterWheelRPMRequested.toString() + "]", false);
    	    	
    	    	// position the slider
    	    	switch(inputDataValues.AutonSliderPositionRequested)
    	    	{
    	    	    case CLICKS_24:
    	    	     	 _sliderAutonPosition = 24;
    	    	     	 break;
    	    	     
    	    	    case CLICKS_30:
    	    	    	 _sliderAutonPosition = 30;
    	    	    	 break;
    	    	    	 
    	    	    case CLICKS_34:
    	    	    	 _sliderAutonPosition = 34;
    	    	    	 break;
    	    	    	
    				default:
    					_sliderAutonPosition = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
    					break;
    	    	}
    	    
    	    	// set the desired target shooter speed
    	    	switch(inputDataValues.AutonShooterWheelRPMRequested)
    	    	{
    	    		case RPM_3500:
    	    			_autonShooterWheelTargetRPM = 3500;
    	    			break;
    	    			
    	    		case RPM_3250:
    	    			_autonShooterWheelTargetRPM = 3250;
    	    			break;
    	    			
    	    		case RPM_3000:
    	    			_autonShooterWheelTargetRPM = 3000;
    	    			break;
    	    			
    	    		case RPM_2750:
    	    			_autonShooterWheelTargetRPM = 2750;
    	    			break;
    	    			
    	    		case RPM_2500:
    	    			_autonShooterWheelTargetRPM = 2500;
    	    			break;
    	    	}
    	    	
    	    	// zero the slider and send to the requested position
    	    	if (!_isSliderAxisZeroedYet)
    	    	{
    	    		_sliderZeroState = Slider_Zero_State.DRIVE_TO_HOME;
    	    		ZeroSliderAxisReEntrant(_robotLiveData);
    	    	}
    	    	
    	    	// set the requested position of the back pumas
    	    	Value PumaBackSolenoidPosition;
    	    	switch(inputDataValues.AutonPumaBackPositionRequested)
    	    	{
    	    	    case PUMA_BACK_DOWN:
    	    	    	PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    	    	     	 break;
    	    	     
    	    	    case PUMA_BACK_UP:
    	    	    	PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    	    	    	 break;
    	    	    	
    				default:
    					PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_DOWN_POSITION;
    					break;
    	    	}
    	    	
    	    	_pumaBackSolenoid.set(PumaBackSolenoidPosition);
    	    	
    	    	workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.INFEED_1;
    	    	DriverStation.reportError("ChangingAutonModeTo: INFEED_1", false);
    	    	
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
    			
    	    	// determine the requested auto shooter mode
    			switch(inputDataValues.AutonCrossDefensePosition)
    	    	{
    				case ZERO:
    					_turretAutonPosition = 0.0;
    					break;
    					
    				case TWO:
    					_turretAutonPosition = 0.18;
    					break;
    					
    				case THREE:
    					_turretAutonPosition = -0.13;
    					break;
    					
    				case FOUR:
    					_turretAutonPosition = -0.55;
    					break;
    					
    				case FIVE:
    					_turretAutonPosition = -1.0;
    					break;
    	    	}
    			
    			DriverStation.reportError("DriveTimeInSecs: [" + _autonTargetDriveTimeMSecs + "]", false);
    	    	DriverStation.reportError("DriveThrottlePercent: [" + _autonTargetDriveThrottlePercent + "]", false);
    	    	DriverStation.reportError("CrossDefensePosition: [" + inputDataValues.AutonCrossDefensePosition.toString() + "]", false);
    			
    			// puma up to cross defenses
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    			
    			// perimeter expansion should be out the entire match
    			//outputDataValues.PerimeterSolenoidPosition = RobotMap.PERIMETER_EXPANSION_IN;
    			
    			workingDataValues.AutonDriveFwdStartTime = new Date().getTime();
    			
    			_sliderZeroStartTime = System.currentTimeMillis();
	    		_sliderZeroState = Slider_Zero_State.DRIVE_TO_HOME;
	    		
	    		_turretZeroStartTime = System.currentTimeMillis();
	    		_turretZeroState = Turret_Zero_State.BEFORE_APPROACHING_SWITCH;
	    		
	    		//_crossDefenseAutonState = Cross_Defense_Auton_State.DRIVE_AND_ZERO;
	    		_crossDefenseAutoAimAndShootState = Cross_Defense_Auto_Aim_And_Shoot_State.ZERO_AXES_AND_DRIVE;
	    		DriverStation.reportError("Changing Auton State To: ZERO_AXES_AND_DRIVE | ", false);
    	    	break;
    			
    		case AIM_AND_SHOOT:
    			// puma up to cross defenses
    			outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    			outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    			
    			// perimeter expansion should be out the entire match
    			//outputDataValues.PerimeterSolenoidPosition = RobotMap.PERIMETER_EXPANSION_IN;
    			
    			workingDataValues.AutonDriveFwdStartTime = new Date().getTime();
    			
    			_sliderZeroStartTime = System.currentTimeMillis();
	    		_sliderZeroState = Slider_Zero_State.DRIVE_TO_HOME;
	    		
	    		_turretZeroStartTime = System.currentTimeMillis();
	    		_turretZeroState = Turret_Zero_State.BEFORE_APPROACHING_SWITCH;
    			
    	    	_autoAimAndShootState = RobotData.Auto_Aim_And_Shoot_State.ZERO_AXES;
    	    	DriverStation.reportError("Changing Auton State To: ZERO_AXES | ", false);
    	    	
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
    	     	
    	    case SHOOT_BALL:
    	    	autonomousShootBall();
    	    	break;
    	    	
    	    case DRIVE_FWD:
    	    	autonomousDriveFwd();
    	    	break;
    	    	
    	    case CROSS_DEFENSE:
    	    	autonomousCrossDefense3();
    	    	break;
    	    
    	    case AIM_AND_SHOOT:
    	    	AimAndShoot();
    	    	break;
    	    	
			default:
				break;
    	}
    	
    	// ===============================
    	// Step 3: Set outputs
    	//			to avoid unintended consequences... 
    	//				for a giveb auton mode we only set the outputs that we know we should be be using
    	// ===============================
    	
    	if (inputDataValues.AutonModeRequested == RobotData.AutonMode.SHOOT_BALL)
    	{   
    		// set motor commmands
        	_infeedAcqMtr.set(outputDataValues.InfeedAcqMtrVelocityCmd);        	
        	_kickerMtr.set(outputDataValues.KickerMtrVelocityCmd);
        	_shooterMasterMtr.set(outputDataValues.ShooterMtrCurrentVelocityCmd);
        	
        	// set slider
        	if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_sliderMtr.set(outputDataValues.SliderTargetPositionCmd);
        	}
        	else if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_sliderMtr.set(outputDataValues.SliderVelocityCmd);
        	}
        	
        	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
    	}
    	else if (inputDataValues.AutonModeRequested == RobotData.AutonMode.ZERO_ALL_AXIS)
		{
    		if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_infeedTiltMtr.set(outputDataValues.InfeedTiltTargetPositionInRotationsCmd);
        	}
        	else if (_infeedTiltMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_infeedTiltMtr.set(outputDataValues.InfeedTiltMtrVelocityCmd);
        	}
        	 
        	if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
        	}
        	else if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_turretMtr.set(outputDataValues.TurretVelocityCmd);
        	}
        	
        	if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_sliderMtr.set(outputDataValues.SliderTargetPositionCmd);
        	}
        	else if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_sliderMtr.set(outputDataValues.SliderVelocityCmd);
        	}
		}
    	else if (inputDataValues.AutonModeRequested == RobotData.AutonMode.DRIVE_FWD)
		{
    		// set motor commmands
        	_robotDrive.arcadeDrive(outputDataValues.ArcadeDriveThrottleAdjCmd, outputDataValues.ArcadeDriveTurnAdjCmd, false);
        	
        	// set solenoids
        	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
        	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
		}
    	else if (inputDataValues.AutonModeRequested == RobotData.AutonMode.CROSS_DEFENSE)
		{
    		// set motor commmands
        	_robotDrive.arcadeDrive((-1.0 * outputDataValues.ArcadeDriveThrottleAdjCmd), outputDataValues.ArcadeDriveTurnAdjCmd, false);
        	_infeedAcqMtr.set(outputDataValues.InfeedAcqMtrVelocityCmd);        	
        	_kickerMtr.set(outputDataValues.KickerMtrVelocityCmd);
        	_shooterMasterMtr.set(outputDataValues.ShooterMtrCurrentVelocityCmd);
        	
        	if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
        	}
        	else if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_turretMtr.set(outputDataValues.TurretVelocityCmd);
        	}
        	
        	// set slider
        	if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_sliderMtr.set(outputDataValues.SliderTargetPositionCmd);
        	}
        	else if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_sliderMtr.set(outputDataValues.SliderVelocityCmd);
        	}
        	
        	// set solenoids
        	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
        	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
        	_perimeterExpansionSolenoid.set(outputDataValues.PerimeterSolenoidPosition);
		}
    	else if (inputDataValues.AutonModeRequested == RobotData.AutonMode.AIM_AND_SHOOT)
    	{
        	_infeedAcqMtr.set(outputDataValues.InfeedAcqMtrVelocityCmd);        	
        	_kickerMtr.set(outputDataValues.KickerMtrVelocityCmd);
        	_shooterMasterMtr.set(outputDataValues.ShooterMtrCurrentVelocityCmd);
        	
        	if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
        	}
        	else if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_turretMtr.set(outputDataValues.TurretVelocityCmd);
        	}
        	
        	// set slider
        	if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.Position)
        	{
        		_sliderMtr.set(outputDataValues.SliderTargetPositionCmd);
        	}
        	else if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
        	{
        		_sliderMtr.set(outputDataValues.SliderVelocityCmd);
        	}
        	
        	// set solenoids
        	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
        	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
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
    	
    	if(!_isSliderAxisZeroedYet)
    	{
    		ZeroSliderAxisReEntrant(_robotLiveData);
    	}
    	else
    	{
    		outputDataValues.SliderTargetPositionCmd = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
    	}
    	
    	if(!_isInfeedTiltAxisZeroedYet)
    	{
    		ZeroInfeedTiltAxisReEntrant(_robotLiveData);
    	}
    }
    
    // this auton mode supports shooting the ball usally from the spybot position
    public void autonomousShootBall()
    {
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	if(!_isSliderAxisZeroedYet)
    	{
    		//ZeroSliderAxisReEntrant(_robotLiveData, _sliderAutonPosition);
    		ZeroSliderAxis(_robotLiveData, _sliderAutonPosition);
    	}
    	
    	// Step 1: Infeed Until Limit Switch Is Hit
    	// Step 2: Set Slider Position
    	// Step 3: Set Puma Back Position
    	// Step 4: Start Shooter and wait for 3600 RPM
    	// Step 5: Shoot (drive infeed up)
    	switch (workingDataValues.AutonShootBallState)
    	{
    		case INFEED_1:
    			if (!inputDataValues.IsBallInPosition)
        		{
        			outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
        			DriverStation.reportError("Ball not in position", false);
        		}
        		else if(inputDataValues.IsBallInPosition)
        		{
        			outputDataValues.InfeedAcqMtrVelocityCmd = 0.0;
        			workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.ADJUST_SLIDER_2;
        			DriverStation.reportError("ChangingAutonModeTo: ADJUST_SLIDER_2", false);
        		}
    			
    			outputDataValues.ShooterMtrCurrentVelocityCmd = _autonShooterWheelTargetRPM;
    			outputDataValues.KickerMtrVelocityCmd = RobotMap.KICKER_TARGET_PERCENT_VBUS_CMD;
    			workingDataValues.AutonShooterStartTime = new Date().getTime();
    			break;
    			
    		case ADJUST_SLIDER_2:	
				if(_isSliderAxisZeroedYet && (Math.abs(_sliderAutonPosition - inputDataValues.SliderCurrentPosition) < 2))
				{
					workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.CHANGE_PUMA_3;
					DriverStation.reportError("ChangingAutonModeTo: CHANGE_PUMA_3", false);
				}
				else
				{
					DriverStation.reportError("Slider position not reached", false);
				}
    			break;
    			
    		case CHANGE_PUMA_3:
    			workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.START_SHOOTER_4;
    			DriverStation.reportError("ChangingAutonModeTo: START_SHOOTER_4", false);
    			break;
    			
    		case START_SHOOTER_4:
    			
    			long elapsedTime = (new Date().getTime() - workingDataValues.AutonShooterStartTime);
    			
    			//workingDataValues.AutonShooterStartTime = new Date().getTime();
    			if (inputDataValues.ShooterActualSpeed > (_autonShooterWheelTargetRPM * 0.95))
    			{
    				workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.SHOOT_5;
    				DriverStation.reportError("Shooter reached target speed", false);
    				DriverStation.reportError("ChangingAutonModeTo: SHOOT_5", false);
    				DriverStation.reportError("Shooter motor speed: " + inputDataValues.ShooterActualSpeed, false);
    			}
    			else if (elapsedTime  >= RobotMap.SHOOTER_AUTON_START_MAX_TIME)
	    		{
	    			workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.SHOOT_5;
	    			DriverStation.reportError("Shooter Timeout: waiting for target speed", false);
	    			DriverStation.reportError("ChangingAutonModeTo: SHOOT_5", false);
	    			DriverStation.reportError("Shooter motor speed: " + inputDataValues.ShooterActualSpeed, false);
	    		}
    			break;
    		
    		case SHOOT_5:
    			outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
    			DriverStation.reportError("ChangingAutonModeTo: WAIT_FOR_BALL_TO_SHOOT_6", false);
    			workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.WAIT_FOR_BALL_TO_SHOOT_6;
    			workingDataValues.AutonShooterStartTime = new Date().getTime();
    			break;
    		
    		case WAIT_FOR_BALL_TO_SHOOT_6:
    			long shooterElapsedTime = (new Date().getTime() - workingDataValues.AutonShooterStartTime);
    			if (shooterElapsedTime  >= RobotMap.SHOOTER_AUTON_RUN_MAX_TIME)
	    		{
    				workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.STOP_SHOOTER_7;
    				DriverStation.reportError("ChangingAutonModeTo: STOP_SHOOTER_7", false);
	    		}
    			break;
    		
    		case STOP_SHOOTER_7:
    			outputDataValues.ShooterMtrCurrentVelocityCmd = 0.0;
    			outputDataValues.InfeedAcqMtrVelocityCmd = 0.0;
    			outputDataValues.KickerMtrVelocityCmd = 0.0;
    			workingDataValues.AutonShootBallState = Auton_Shoot_Ball_State.AUTON_FINISHED_10;
    			DriverStation.reportError("ChangingAutonModeTo: AUTON_FINISHED_10", false);
    			break;
    			
    		case AUTON_FINISHED_10:
    			break;
    			
    		default:
    			break;
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
    
    // This Auton mode supports drive across the defenses and just maybe shootng the ball
    public void autonomousCrossDefense()
    {
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	VisionData visionData = _visionClient.GetVisionData();
    	/*
    	if(!_isSliderAxisZeroedYet)
    	{
    		ZeroSliderAxisReEntrant(_robotLiveData);
    		//ZeroSliderAxis(_robotLiveData, _sliderAutonPosition);
    	}
    	else
    	{
    		outputDataValues.SliderTargetPositionCmd = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
    	}
    	
    	if (!_isTurretAxisZeroedYet)
    	{
    		ZeroTurretAxisReEntrant(_robotLiveData);
    	}
    
    	
    	// how long have we been driving
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
        	
    		// drive to default position
	    	outputDataValues.TurretTargetPositionCmd = CalcTurretTargetPosition(_turretAutonPosition);
	    	_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
	    	DriverStation.reportError("Turret target position: " + Double.toString(_turretAutonPosition), false);
	    	
	    	long startTime = System.currentTimeMillis();
	    	long elapsedTime = 0L;
	    	long maxTimeInMSec = 5000; // 5 secs
	    	
	    	// wait until we get close to the target
	    	while((Math.abs(_turretMtr.getClosedLoopError()) > 400) && !isTimeout)
	    	{	
	        	elapsedTime = (new Date().getTime() - startTime);
	    		
	    		if (elapsedTime  >= maxTimeInMSec)
	    		{
	    			isTimeout = true;
	    		}
	    	}
	    	
	    	// now move to regular gains after the big move
	    	if (Math.abs(_turretMtr.getClosedLoopError()) < 400)
	    	{
	    		_turretMtr.setPID(RobotMap.TURRET_FAST_KP, RobotMap.TURRET_FAST_KI, RobotMap.TURRET_FAST_KD, RobotMap.TURRET_FAST_KF, RobotMap.TURRET_FAST_IZONE, RobotMap.TURRET_FAST_RAMPRATE, RobotMap.TURRET_FAST_PROFILE);
	    		_turretMtr.setProfile(RobotMap.TURRET_FAST_PROFILE);
	    		DriverStation.reportError("Fast turret profile set", false);
	    	}
    	}
    	*/
    	long driveFwdElapsedTime = (new Date().getTime() - workingDataValues.AutonDriveFwdStartTime);
    	
		switch (_crossDefenseAutonState)
		{
			case DRIVE_AND_ZERO:
				// Zero the slider and turret axis
				if(!_isSliderAxisZeroedYet)
		    	{
		    		ZeroSliderAxisReEntrant(_robotLiveData);
		    		//ZeroSliderAxis(_robotLiveData, _sliderAutonPosition);
		    	}
		    	else
		    	{
		    		outputDataValues.SliderTargetPositionCmd = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
		    	}
		    	
		    	if (!_isTurretAxisZeroedYet)
		    	{
		    		ZeroTurretAxisReEntrant(_robotLiveData);
		    	}
		    	
		    	// drive forward for the set time and speed selected on the dashboard
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
		    	
		    	// if both axis are zeroed and we've moved for the set time, go to the next state
		    	if (_isSliderAxisZeroedYet && _isTurretAxisZeroedYet && (driveFwdElapsedTime  >= _autonTargetDriveTimeMSecs))
    			{
		    		_crossDefenseAutonState = Cross_Defense_Auton_State.COARSE_TURRET_TO_TARGET;
		    		DriverStation.reportError("Changing Auton State To: COARSE_TURRET_TO_TARGET", false);
    			}
    			
				break;
				
			case COARSE_TURRET_TO_TARGET:
				// Drive the turret to a preset position based on the location of the defense to get us close enough to ensure valid vision data
				outputDataValues.TurretTargetPositionCmd = CalcTurretTargetPosition(_turretAutonPosition);
		    	_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
		    	DriverStation.reportError("Turret target position: " + Double.toString(_turretAutonPosition), false);
		    	
		    	// Make sure ball goes up to the limit switch
    			if (!inputDataValues.IsBallInPosition)
        		{
        			outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
        			DriverStation.reportError("Ball not in position", false);
        		}
        		else if(inputDataValues.IsBallInPosition)
        		{
        			outputDataValues.InfeedAcqMtrVelocityCmd = 0.0;
        			DriverStation.reportError("Ball in position", false);
        		}
    			
    			// if the ball is on the limit switch and the turret is within the deadband, move to the next state
    			if (inputDataValues.IsBallInPosition && (Math.abs(_turretMtr.getClosedLoopError()) < RobotMap.TURRET_AUTON_MAX_CLOSED_LOOP_ERROR))
    			{
    				_turretMtr.setPID(RobotMap.TURRET_MEDIUM_KP, RobotMap.TURRET_MEDIUM_KI, RobotMap.TURRET_MEDIUM_KD, RobotMap.TURRET_MEDIUM_KF, RobotMap.TURRET_MEDIUM_IZONE, RobotMap.TURRET_MEDIUM_RAMPRATE, RobotMap.TURRET_MEDIUM_PROFILE);
		    		_turretMtr.setProfile(RobotMap.TURRET_MEDIUM_PROFILE);
		    		DriverStation.reportError("Fast turret profile set", false);
    				
    				_crossDefenseAutonState = Cross_Defense_Auton_State.FINE_TURRET_TO_TARGET;
    				DriverStation.reportError("Changing Auton State To: FINE_TURRET_TO_TARGET", false);
    			}
				break;
				
			case FINE_TURRET_TO_TARGET:
				if (visionData != null){
					outputDataValues.TurretTargetPositionCmd = CalcTurretTargetPosition(inputDataValues.TurretEncoderCurrentPosition
																+ ((RobotMap.TURRET_GEAR_RATIO * inputDataValues.DesiredTurretTurnInDegrees)/360.0));
					DriverStation.reportError("Desired Turret Turn: " + Double.toString(inputDataValues.DesiredTurretTurnInDegrees), false);
					DriverStation.reportError("Turret Target Position Cmd: " + Double.toString(outputDataValues.TurretTargetPositionCmd), false);
					
					if (Math.abs(visionData.DesiredTurretTurnInDegrees) < RobotMap.TURRET_AUTON_MAX_ADJUSTABLE_ERROR_IN_DEGREES)
					{
						_crossDefenseAutonState = Cross_Defense_Auton_State.SHOOT;
						DriverStation.reportError("Changing Auton State To: SHOOT", false);
					}
				}
				else 
				{
					DriverStation.reportError("Not recieving valid vision data", false);
				}
				break;
				
			case SHOOT:
				outputDataValues.ShooterMtrCurrentVelocityCmd = RobotMap.SHOOTER_TARGET_MOTOR_RPM;
				outputDataValues.KickerMtrVelocityCmd = RobotMap.KICKER_TARGET_PERCENT_VBUS_CMD;
				
				if (inputDataValues.ShooterActualSpeed > (RobotMap.SHOOTER_TARGET_MOTOR_RPM * 0.95))
				{
					outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
				}
				break;
				
			case TIMEOUT:
				DriverStation.reportError("Cross Defense Auton Timed Out", false);
				break;	
		}	
    }
    
    public void autonomousCrossDefense2()
    {
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;

    	
    	if(!_isSliderAxisZeroedYet)
    	{
    		ZeroSliderAxisReEntrant(_robotLiveData);
    		//ZeroSliderAxis(_robotLiveData, _sliderAutonPosition);
    	}
    	else
    	{
    		outputDataValues.SliderTargetPositionCmd = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
    	}
    	
    	if (!_isTurretAxisZeroedYet)
    	{
    		ZeroTurretAxisReEntrant(_robotLiveData);
    	}
    
    	
    	// how long have we been driving
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
        	
    		// drive to default position
	    	outputDataValues.TurretTargetPositionCmd = CalcTurretTargetPosition(_turretAutonPosition);
	    	DriverStation.reportError("Turret target position: " + Double.toString(_turretAutonPosition), false);
	    	
	    	double turretPositionErrorInRotations = outputDataValues.TurretTargetPositionCmd - _turretMtr.getPosition(); 
	    	DriverStation.reportError("Turret Position Error: " + Double.toString(turretPositionErrorInRotations) + "| ", false);
	    	if (Math.abs(turretPositionErrorInRotations) > 0.5)
			{
				if(_turretMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus )
				{
					// switch to % VBUS Mode
    				_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    				DriverStation.reportError("Turret changing to PercentVBus mode", false);
				}
				
				if (turretPositionErrorInRotations > 0)
				{
    				outputDataValues.TurretVelocityCmd = 0.10;
    				//DriverStation.reportError("Turret Speed at 10% | ", false);
				}
				else
				{
    				outputDataValues.TurretVelocityCmd = -0.10;
    				//DriverStation.reportError("Turret Speed at -10% | ", false);
				}    					
			}
			else
			{
				//_autoAimAndShootState = Auto_Aim_And_Shoot_State.FINE_TURRET_TO_TARGET;
				DriverStation.reportError("Switching to coarse adjustmennt", false);
				outputDataValues.TurretVelocityCmd = 0.0;
			}
	    	
	    	if (_visionClient != null)
	    	{
	    		DriverStation.reportError("Desired Turret Turn: " + Double.toString(inputDataValues.DesiredTurretTurnInDegrees), false);
	    	}
    	}
    }
    
    public void autonomousCrossDefense3()
    {
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	VisionData visionData = _visionClient.GetVisionData();
    	    	
    	switch (_crossDefenseAutoAimAndShootState)
    	{
	    	case ZERO_AXES_AND_DRIVE:
	    		
	    		// zero the Slider Axis
    	    	if(!_isSliderAxisZeroedYet)
    	    	{
    	    		ZeroSliderAxisReEntrant(_robotLiveData);
    	    		//ZeroSliderAxis(_robotLiveData, _sliderAutonPosition);
    	    	}
    	    	else
    	    	{
    	    		outputDataValues.SliderTargetPositionCmd = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
    	    	}
    	    	
    	    	// Zero the Turret
    	    	if (!_isTurretAxisZeroedYet)
    	    	{
    	    		ZeroTurretAxisReEntrant(_robotLiveData);
    	    	}
    	    	
    	    	// how long have we been driving
    	    	long driveFwdElapsedTime = (new Date().getTime() - workingDataValues.AutonDriveFwdStartTime);
    	    	
    	    	if (driveFwdElapsedTime  <= _autonTargetDriveTimeMSecs)
    	    	{
    	    		// drive forward
    	    		outputDataValues.ArcadeDriveThrottleAdjCmd = _autonTargetDriveThrottlePercent;
    	    		outputDataValues.ArcadeDriveTurnAdjCmd = 0;
    	    	}
    	    	else
    	    	{
    	    		// stop driving
    	        	outputDataValues.ArcadeDriveThrottleAdjCmd = 0;
    	        	outputDataValues.ArcadeDriveTurnAdjCmd = 0;
    	    	}
    	    	
    	    	// how far is the slider from the desired location?
    	    	double sliderPositionError = Math.abs(outputDataValues.SliderTargetPositionCmd - _sliderMtr.getPosition());
    	    	
    	    	// check the exit conditions for this state
    	    	if(_isSliderAxisZeroedYet 
    	    			&& _isTurretAxisZeroedYet 
    	    			&& (sliderPositionError < 1.0) 
    	    			&& (driveFwdElapsedTime  >= _autonTargetDriveTimeMSecs)
    	    			&& (outputDataValues.SliderTargetPositionCmd == RobotMap.SLIDER_DEFAULT_TARGET_POSITION))
    	    	{
    	    		DriverStation.reportError("Changing Auton State To: GROSS_TURRET_TO_TARGET | ", false);
    	    		_crossDefenseAutoAimAndShootState = Cross_Defense_Auto_Aim_And_Shoot_State.GROSS_TURRET_TO_TARGET;
    	    		
    	    		//DriverStation.reportError("Changing Auton State To: COARSE_TURRET_TO_TARGET | ", false);
    				//_crossDefenseAutoAimAndShootState = Cross_Defense_Auto_Aim_And_Shoot_State.COARSE_TURRET_TO_TARGET;
    	    	}
    			break;
    			
    		case GROSS_TURRET_TO_TARGET:
    			
    			/*
    			     				case ZERO:
    					_turretAutonPosition = 0.0;
    					break;
    					
    				case TWO:
    					_turretAutonPosition = 0.18;
    					break;
    					
    				case THREE:
    					_turretAutonPosition = -0.13;
    					break;
    					
    				case FOUR:
    					_turretAutonPosition = -0.55;
    					break;
    					
    				case FIVE:
    					_turretAutonPosition = -1.0;
    					break;
    			*/
    			
    			// how far is the turrent from the desired location?
    			double turretPositionErrorInRotations = CalcTurretTargetPosition(_turretAutonPosition) - _turretMtr.getPosition(); 
    			
    			if (Math.abs(turretPositionErrorInRotations) > 0.5)
    			{
    				if(_turretMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus )
    				{
    					// switch to % VBUS Mode
        				_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        				DriverStation.reportError("Turret changing to PercentVBus mode", false);
    				}
    				
    				if (turretPositionErrorInRotations > 0)
    				{
        				outputDataValues.TurretVelocityCmd = 0.15;
        				//DriverStation.reportError("Turret Speed at 10% | ", false);
    				}
    				else
    				{
        				outputDataValues.TurretVelocityCmd = -0.15;
        				//DriverStation.reportError("Turret Speed at -10% | ", false);
    				}    					
    			}
    			else
    			{
    				DriverStation.reportError("Changing Auton State To: COARSE_TURRET_TO_TARGET | ", false);
    				_crossDefenseAutoAimAndShootState = Cross_Defense_Auto_Aim_And_Shoot_State.COARSE_TURRET_TO_TARGET;
    			}
    			break;
    			
    		case COARSE_TURRET_TO_TARGET:
    			
    			if (Math.abs(visionData.DesiredTurretTurnInDegrees) > 10.0)
    			{
    				if(_turretMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus )
    				{
    					// switch to % VBUS Mode
        				_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        				DriverStation.reportError("Turret changing to PercentVBus mode", false);
    				}
    				
    				if (visionData.DesiredTurretTurnInDegrees > 0)
    				{
	    				outputDataValues.TurretVelocityCmd = 0.15;
	    				//DriverStation.reportError("Turret Speed at 10% | ", false);
    				}
    				else
    				{
	    				outputDataValues.TurretVelocityCmd = -0.15;
	    				//DriverStation.reportError("Turret Speed at -10% | ", false);
    				}    					
    			}
    			else if (Math.abs(visionData.DesiredTurretTurnInDegrees) > 0.0)
    			{
    				_crossDefenseAutoAimAndShootState = Cross_Defense_Auto_Aim_And_Shoot_State.FINE_TURRET_TO_TARGET;
    				//outputDataValues.TurretVelocityCmd = 0.0;
					DriverStation.reportError("Changing Auton State To: FINE_TURRET_TO_TARGET | ", false);
    			}
    			else
    			{
    				DriverStation.reportError("Coarse Vision Data looks invalid" , false);
    			}
    			//_turretMtr.set(outputDataValues.TurretVelocityCmd);
    			//DriverStation.reportError("Turret Velocity Cmd: " + Double.toString(outputDataValues.TurretVelocityCmd) + "| ", false);
    			break;
    			
    		case FINE_TURRET_TO_TARGET:
    			
    			DriverStation.reportError("TopVisionCmd= " + visionData.DesiredTurretTurnInDegrees + " | ", false);
    			
    			if (Math.abs(visionData.DesiredTurretTurnInDegrees) > 1.25)
    			{
    				if(_turretMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus )
    				{
    					// switch to % VBUS Mode
        				_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        				DriverStation.reportError("Turret changing to PercentVBus mode", false);
    				}
    				
    				// decide what direction turn
    				if (visionData.DesiredTurretTurnInDegrees > 0)
    				{
	    				outputDataValues.TurretVelocityCmd = 0.05;
	    				//DriverStation.reportError("Turret Speed at 5% | ", false);
    				}
    				else
    				{
	    				outputDataValues.TurretVelocityCmd = -0.05;
	    				//DriverStation.reportError("Turret Speed at -5% | ", false);
    				}    					
    			}
    			else if (Math.abs(visionData.DesiredTurretTurnInDegrees) > 0.0)
    			{
    	    		// switch to PID Position Mode
    	    		
    	    		// stop driving the axis
    	    		outputDataValues.TurretVelocityCmd = 0;
    		    	//_turretMtr.set(outputDataValues.TurretVelocityCmd);
    	    		
    		    	// now switch to position loop mode
    		    	//_turretMtr.changeControlMode(CANTalon.TalonControlMode.Position);
    		    	//outputDataValues.TurretTargetPositionCmd = _turretMtr.getPosition();
    		    	//DriverStation.reportError("Turret changing to Position mode | ", false);
    		    	
    		    	//try {
    		    		// sleep a little to let the zero occur
    				//	Thread.sleep(1);
    				//} catch (InterruptedException e) {
    					// TODO Auto-generated catch block
    				//	e.printStackTrace();
    				//}
    				
    		    	_crossDefenseAutoAimAndShootState = Cross_Defense_Auto_Aim_And_Shoot_State.SHOOT;
					DriverStation.reportError("Changing Auton State To: SHOOT | ", false);
					DriverStation.reportError("VisionCmd= " + visionData.DesiredTurretTurnInDegrees + " | ", false);
					DriverStation.reportError("VisionCmd= " + visionData.IsValidData + " | ", false);
    			}
    			else
    			{
    				DriverStation.reportError("Fine Vision Data looks invalid" , false);
    			}
    			DriverStation.reportError("Turret Velocity Cmd: " + Double.toString(outputDataValues.TurretVelocityCmd) + "| ", false);
    			//_turretMtr.set(outputDataValues.TurretVelocityCmd);
				break;
    			
    		case SHOOT:
				// wait until we reach 95% of target wheel speed
    			if (Math.abs(_navXSensor.getRawAccelX()) > 0.0)
    			{
    				DriverStation.reportError("AccelX= " + _navXSensor.getRawAccelX() + " | ", false);
    			}
    			if (Math.abs(_navXSensor.getRawAccelY()) > 0.0)
    			{
    				DriverStation.reportError("AccelY= " + _navXSensor.getRawAccelY() + " | ", false);
    			}
    			
				if (inputDataValues.ShooterActualSpeed > (RobotMap.SHOOTER_TARGET_MOTOR_RPM * 0.95))
				{
					// start the infeed to drive the ball up into the shooter
					outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
				}
				
				// drive both sets of wheels
    			outputDataValues.ShooterMtrCurrentVelocityCmd = RobotMap.SHOOTER_TARGET_MOTOR_RPM;
				outputDataValues.KickerMtrVelocityCmd = RobotMap.KICKER_TARGET_PERCENT_VBUS_CMD;
    			break;
    			
    		case TIMEOUT:
    			DriverStation.reportError("Error... Auto Aim and Shoot sequence timed out | ", false);
    			break;
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
    	    	
    	outputDataValues.KickerMtrVelocityCmd = 0.0;
    	outputDataValues.ShooterMtrCurrentVelocityCmd = 0.0;
    	outputDataValues.ShooterMtrTargetVelocityCmd = RobotMap.SHOOTER_TARGET_MOTOR_RPM;
    	outputDataValues.SliderVelocityCmd = 0.0;
    	    	
    	// initialize axis (Encoder) positions (for any talon where we care about position but do not have a home position
    	_leftDriveMasterMtr.setPosition(0);
    	_rightDriveMasterMtr.setPosition(0);
    	
    	// set motors to output velocity command 
    	_leftDriveMasterMtr.set(outputDataValues.ArcadeDriveThrottleAdjCmd);
    	_rightDriveMasterMtr.set(outputDataValues.ArcadeDriveTurnAdjCmd);
    	_infeedAcqMtr.set(outputDataValues.InfeedAcqMtrVelocityCmd);
    	_shooterMasterMtr.set(outputDataValues.ShooterMtrCurrentVelocityCmd);
    	_kickerMtr.set(outputDataValues.KickerMtrVelocityCmd);
    	//_sliderMtr.set(outputDataValues.SliderVelocityCmd);
    	_cupidServo.set(outputDataValues.CupidServoPositionCmd);
    	  	    	
    	// init the drive speed scaling factor to 95%
    	workingDataValues.DriveSpeedScalingFactor = 0.95;

    	// get initial values from Encoders
    	workingDataValues.LeftDriveEncoderInitialCount = _leftDriveMasterMtr.getPosition();
    	workingDataValues.RightDriveEncoderInitialCount = _rightDriveMasterMtr.getPosition();

    	// set our desired default state for the puma solenoids
    	outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
    	outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    	outputDataValues.PerimeterSolenoidPosition = RobotMap.PERIMETER_EXPANSION_OUT;
    	
    	// set initial state of "pressed last scan" working values to be false
    	workingDataValues.IsPumaFrontToggleBtnPressedLastScan = false;
    	workingDataValues.IsPumaBackToggleBtnPressedLastScan = false;
    	workingDataValues.IsShifterToggleBtnPressedLastScan = false;
    	
    	// send the initial states to the solenoids
    	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
    	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
    	_perimeterExpansionSolenoid.set(outputDataValues.PerimeterSolenoidPosition);
    	
    	inputDataValues.IsInfeedAcquireBtnPressed = false;
    	inputDataValues.IsInfeedReleaseBtnPressed = false;
    	workingDataValues.IsTurretEncoderDegreesTargetYet = false;
    	workingDataValues.TeleopElevatorState = Teleop_Elevator_State.HONOR_INFEED_TRIGGERS;
    	DriverStation.reportError("Elevator switching to HONOR_INFEED_TRIGGERS state", false);
    	
    	_isInfeedPeriodZeroMode = false;
    	
    	if (!_isTurretAxisZeroedYet)
    	{
    		ZeroTurretAxis(_robotLiveData);
    	}
    	else
    	{
    		outputDataValues.TurretTargetPositionCmd = _turretMtr.getPosition();
    	}
    	
    	if (!_isInfeedTiltAxisZeroedYet)
    	{
    		ZeroInfeedTiltAxis(_robotLiveData);
    	}
    	else 
    	{
    		outputDataValues.InfeedTiltTargetPositionInRotationsCmd = RobotMap.INFEED_TILT_STORED_POSITION_CMD;
    	}
    	
    	if (!_isSliderAxisZeroedYet)
    	{
    		ZeroSliderAxis(_robotLiveData, RobotMap.SLIDER_DEFAULT_TARGET_POSITION);
    	}
    	else
    	{
    		outputDataValues.SliderTargetPositionCmd = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
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
    	
    	// =====================================
    	// This code block implements a Robot Chassis "tilt safety" feature
    	//		if the chassis tilt angle as measured by the NavX sensor exceeds a threshhold, override the operator input to prevent tipping 
    	// =====================================
    	double tiltSafetyScalingFactor = 1.0;
    	if(inputDataValues.NavxIsConnected)
    	{
    		// based on direction
    		if (inputDataValues.ArcadeDriveThrottleRawCmd < 0.0)
    		{
    			// "fwd" direction is towards the flyer 
    			if (inputDataValues.NavxRoll >= RobotMap.ROBOT_FWD_DRIVE_MAX_TILT_CUTOFF)
    			{
    				tiltSafetyScalingFactor = -0.4;
    				DriverStation.reportError("Fwd Tilt Angle " + String.format("%.1f", inputDataValues.NavxRoll) 
    											+ " Exceeds Cutoff: " + String.format("%.1f", RobotMap.ROBOT_FWD_DRIVE_MAX_TILT_CUTOFF) 
    											+ " Safety Engaged|", false);
    				workingDataValues.IsFwdDriveTiltSafetyEngagedLastScan = true;
    			}
    			else if (inputDataValues.NavxRoll >= RobotMap.ROBOT_FWD_DRIVE_MAX_TILT_REENABLE && workingDataValues.IsFwdDriveTiltSafetyEngagedLastScan)
    			{
    				tiltSafetyScalingFactor = 0.0;
    				DriverStation.reportError("Fwd Tilt Angle " + String.format("%.1f", inputDataValues.NavxRoll) 
												+ " Exceeds Reenable: " + String.format("%.1f", RobotMap.ROBOT_FWD_DRIVE_MAX_TILT_REENABLE) 
												+ " Safety Still Engaged|", false);
    			}
    			else
    			{
    				workingDataValues.IsFwdDriveTiltSafetyEngagedLastScan = false;
    			}
    		}
    		else if (inputDataValues.ArcadeDriveThrottleRawCmd > 0.0)
    		{
    			// "rev" direction is opposite the flyer 
    			if (inputDataValues.NavxRoll <= RobotMap.ROBOT_REV_DRIVE_MAX_TILT_CUTOFF)
    			{
    				tiltSafetyScalingFactor = -0.4;
    				DriverStation.reportError("Rev Tilt Angle " + String.format("%.1f", inputDataValues.NavxRoll) 
    											+ " Exceeds Cutoff: " + String.format("%.1f", RobotMap.ROBOT_REV_DRIVE_MAX_TILT_CUTOFF) 
    											+ " Safety Engaged|", false);
    				
    				workingDataValues.IsRevDriveTiltSafetyEngagedLastScan = true;
    			}
    			else if (inputDataValues.NavxRoll <= RobotMap.ROBOT_REV_DRIVE_MAX_TILT_REENABLE && workingDataValues.IsRevDriveTiltSafetyEngagedLastScan)
    			{
    				tiltSafetyScalingFactor = 0.0;
    				DriverStation.reportError("Rev Tilt Angle " + String.format("%.1f", inputDataValues.NavxRoll) 
												+ " Exceeds Reenable: " + String.format("%.1f", RobotMap.ROBOT_REV_DRIVE_MAX_TILT_REENABLE) 
												+ " Safety Still Engaged|", false);
    			}
    			else
    			{
    				workingDataValues.IsRevDriveTiltSafetyEngagedLastScan = false;
    			}
    		}
    	}
    	
    	outputDataValues.ArcadeDriveThrottleAdjCmd 
    			= inputDataValues.ArcadeDriveThrottleRawCmd * workingDataValues.DriveSpeedScalingFactor * tiltSafetyScalingFactor;  	
    	outputDataValues.ArcadeDriveTurnAdjCmd 
    			= inputDataValues.ArcadeDriveTurnRawCmd * workingDataValues.DriveSpeedScalingFactor * tiltSafetyScalingFactor * 0.6;

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
    	
    	// implement state machine for the the elevator controls
    	switch (workingDataValues.TeleopElevatorState)
    	{    			
    		case HONOR_INFEED_TRIGGERS:
    			
    			if (inputDataValues.IsInfeedAcquireBtnPressed && !inputDataValues.IsInfeedReleaseBtnPressed)
    			{
    				if(inputDataValues.IsBallInPosition)
    				{
    					// stop infeed
	        			outputDataValues.InfeedAcqMtrVelocityCmd = 0.0;
	        			workingDataValues.TeleopElevatorState = Teleop_Elevator_State.ON_BALL_IN_POSITION_SWITCH;
	        			DriverStation.reportError("Elevator switching to ON_BALL_IN_POSITION_SWITCH state", false);
    				}
    				else
    				{
	    				// drive infeed at full speed fwd
	        			outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
    				}
    			}
    			else if (!inputDataValues.IsInfeedAcquireBtnPressed && inputDataValues.IsInfeedReleaseBtnPressed)
    			{
    				// drive infeed at full speed reverse
        			outputDataValues.InfeedAcqMtrVelocityCmd = -1.0;
    			}
    			else
    			{
    				// stop infeed
    				outputDataValues.InfeedAcqMtrVelocityCmd = 0.0;
    			}
    			break;
    			
    		case ON_BALL_IN_POSITION_SWITCH:
    			workingDataValues.InfeedPauseOnBallInPositionSwitchStartTime = new Date().getTime();
    			workingDataValues.TeleopElevatorState = Teleop_Elevator_State.IN_DELAY_PERIOD;
    			DriverStation.reportError("Elevator switching to IN_DELAY_PERIOD state", false);
    			break;
    				
    		case IN_DELAY_PERIOD:
    			if (!inputDataValues.IsInfeedAcquireBtnPressed && inputDataValues.IsInfeedReleaseBtnPressed)
    			{
    				// drive infeed at full speed reverse
        			workingDataValues.TeleopElevatorState = Teleop_Elevator_State.HONOR_INFEED_TRIGGERS;
        			DriverStation.reportError("Elevator switching to INFEED_TRIGGER_PRESSED state", false);
    			}
    			else
    			{
    				// see how long we have been paused
    				long elapsedTime = (new Date().getTime() - workingDataValues.InfeedPauseOnBallInPositionSwitchStartTime);
    				
    				if(elapsedTime > 1000)	// we always want to pause the infeed for 1 sec
    				{
    					workingDataValues.TeleopElevatorState = Teleop_Elevator_State.POST_DELAY_TRIGGER_RELEASED;
    	    			DriverStation.reportError("Elevator switching to POST_DELAY_TRIGGER_RELEASED state", false);
    				}
    				else if (inputDataValues.IsElevatorTimerOverrideBtnPressed)
    				{
    					workingDataValues.TeleopElevatorState = Teleop_Elevator_State.POST_DELAY_TRIGGER_RELEASED;
    	    			DriverStation.reportError("Elevator switching to POST_DELAY_TRIGGER_RELEASED state", false);
    				}
    			}
    			break;
    			
    		case POST_DELAY_TRIGGER_RELEASED:
    			if (!inputDataValues.IsInfeedAcquireBtnPressed && inputDataValues.IsInfeedReleaseBtnPressed)
    			{
        			workingDataValues.TeleopElevatorState = Teleop_Elevator_State.HONOR_INFEED_TRIGGERS;
        			DriverStation.reportError("Elevator switching to HONOR_INFEED_TRIGGERS state", false);
    			}
    			else if (!inputDataValues.IsInfeedAcquireBtnPressed && !inputDataValues.IsInfeedReleaseBtnPressed)
    			{
    				workingDataValues.TeleopElevatorState = Teleop_Elevator_State.HONOR_INFEED_TRIGGERS_AFTER_SWITCH;
        			DriverStation.reportError("Elevator switching to HONOR_INFEED_TRIGGERS_AFTER_SWITCH state", false);
    			}
    			break;
    			
    		case HONOR_INFEED_TRIGGERS_AFTER_SWITCH:
    			if (!inputDataValues.IsInfeedAcquireBtnPressed && inputDataValues.IsInfeedReleaseBtnPressed)
    			{
        			workingDataValues.TeleopElevatorState = Teleop_Elevator_State.HONOR_INFEED_TRIGGERS;
        			DriverStation.reportError("Elevator switching to HONOR_INFEED_TRIGGERS state", false);
    			}
    			else if (inputDataValues.IsInfeedAcquireBtnPressed && !inputDataValues.IsInfeedReleaseBtnPressed)
    			{
    				if(!inputDataValues.IsBallInPosition)
    				{
    					workingDataValues.TeleopElevatorState = Teleop_Elevator_State.HONOR_INFEED_TRIGGERS;
            			DriverStation.reportError("Elevator switching to HONOR_INFEED_TRIGGERS state", false);
    				}
    				else
    				{
    					outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
    				}
    			}
    			else if (!inputDataValues.IsInfeedAcquireBtnPressed && !inputDataValues.IsInfeedReleaseBtnPressed)
    			{
    				outputDataValues.InfeedAcqMtrVelocityCmd = 0.0;
    			}
    			break;
    	}
    	    	
    	// =====================================
    	// Step 2.4: Turret 
    	// =====================================
    	
    	// Determine what mode we shoudl be in %VBus or PID Position
    	if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.Position
    			&& ((inputDataValues.TurretCCWRawVelocityCmd > 0.1) || (inputDataValues.TurretCWRawVelocityCmd > 0.1)))
    	{
    		// switch to % VBUS Mode
    		_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    		DriverStation.reportError("Turret changing to PercentVBus mode", false);
    	}
    	else if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus 
    			&& ((inputDataValues.TurretCCWRawVelocityCmd < 0.1) && (inputDataValues.TurretCWRawVelocityCmd < 0.1)))
    	{
    		// switch to PID Position Mode
    		
    		// stop driving the axis
    		outputDataValues.TurretVelocityCmd = 0;
	    	_turretMtr.set(outputDataValues.TurretVelocityCmd);
    		
	    	// now switch to position loop mode
	    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.Position);
	    	outputDataValues.TurretTargetPositionCmd = _turretMtr.getPosition();
	    	DriverStation.reportError("Turret changing to Position mode", false);
	    	
	    	try {
	    		// sleep a little to let the zero occur
				Thread.sleep(1);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    	
    	}
    	
    	if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{	
    		// we are using a 5% deadband on the joysticks
    		if ((inputDataValues.TurretCCWRawVelocityCmd > 0.05) && (inputDataValues.TurretCWRawVelocityCmd < 0.05))
    		{
    			//outputDataValues.TurretVelocityCmd = -(Math.pow(inputDataValues.TurretCCWRawVelocityCmd, 3) * RobotMap.TURRET_PERCENTVBUS_SCALING_FACTOR);
    			if (inputDataValues.TurretCCWRawVelocityCmd <= 0.45)
    			{
    				outputDataValues.TurretVelocityCmd = -0.05;
    				DriverStation.reportError("Turret Speed at -5%", false);
    			}
    			else if (inputDataValues.TurretCCWRawVelocityCmd <= 0.90)
    			{
    				outputDataValues.TurretVelocityCmd = -0.10;
    				DriverStation.reportError("Turret Speed at -10%", false);
    			}
    			else
    			{
    				outputDataValues.TurretVelocityCmd = -0.20;
    				DriverStation.reportError("Turret Speed at -20%", false);
    			}
    			
    			// enforce (-) soft limit
    			if (inputDataValues.TurretEncoderCurrentPosition < RobotMap.TURRET_MIN_TRAVEL_IN_ROTATIONS){
    				outputDataValues.TurretVelocityCmd = 0.0;
    			}
    		}
    		else if ((inputDataValues.TurretCCWRawVelocityCmd < 0.05) && (inputDataValues.TurretCWRawVelocityCmd > 0.05))
    		{
    			//outputDataValues.TurretVelocityCmd = (Math.pow(inputDataValues.TurretCWRawVelocityCmd, 3) * RobotMap.TURRET_PERCENTVBUS_SCALING_FACTOR);
    			if (inputDataValues.TurretCWRawVelocityCmd <= 0.45)
    			{
    				outputDataValues.TurretVelocityCmd = 0.05;
    				DriverStation.reportError("Turret Speed at 5%", false);
    			}
    			else if (inputDataValues.TurretCWRawVelocityCmd <= 0.90)
    			{
    				outputDataValues.TurretVelocityCmd = 0.10;
    				DriverStation.reportError("Turret Speed at 10%", false);
    			}
    			else
    			{
    				outputDataValues.TurretVelocityCmd = 0.20;
    				DriverStation.reportError("Turret Speed at 20%", false);
    			}
    			
    			// enforce (+) soft limit
    			if (inputDataValues.TurretEncoderCurrentPosition > RobotMap.TURRET_MAX_TRAVEL_IN_ROTATIONS){
    				outputDataValues.TurretVelocityCmd = 0.0;
    			}
    		}
    	}
    	else if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		/*
			if (inputDataValues.IsShooterSpeedUpBtnPressed 
					&& !workingDataValues.IsTurretCWButtonPressedLastScan
					&& !inputDataValues.IsShooterSpeedDownBtnPressed)		
			{
				double NewTurretTargetPositionCmd = outputDataValues.TurretTargetPositionCmd + 0.06;
				outputDataValues.TurretTargetPositionCmd = CalcTurretTargetPosition(NewTurretTargetPositionCmd);
			}
			else if (!inputDataValues.IsShooterSpeedUpBtnPressed 
					&& !workingDataValues.IsTurretCCWButtonPressedLastScan
					&& inputDataValues.IsShooterSpeedDownBtnPressed)	
			{
				double NewTurretTargetPositionCmd = outputDataValues.TurretTargetPositionCmd - 0.06;
				outputDataValues.TurretTargetPositionCmd = CalcTurretTargetPosition(NewTurretTargetPositionCmd);
			}
			*/
    	}
    	
    	
    	// ============================
    	// 2.4.1 Turret Aiming
    	// ============================
    	
    	//if (inputDataValues.IsTurretAutoAimBtnPressed)
    		
    	    	
    	// ============================
    	// Step 2.5: Slider
    	// ============================
    	if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{
    		if ((inputDataValues.IsSliderFwdBtnPressed == true) && (!inputDataValues.IsSliderRevBtnPressed == false))
    		{
    			outputDataValues.SliderVelocityCmd = 0.1;
    		}
    		else if ((!inputDataValues.IsSliderFwdBtnPressed == false) && (inputDataValues.IsSliderRevBtnPressed == true))
    		{
    			outputDataValues.SliderVelocityCmd = -0.1;
    		}
    		else
    		{
    			outputDataValues.SliderVelocityCmd = 0.0;
    		}
    	}
    	else if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		if ((inputDataValues.IsSliderFwdBtnPressed == true) && (inputDataValues.IsSliderRevBtnPressed == false))
    		{
    			if (!workingDataValues.IsSliderFwdBtnPressedLastScan)	// debounce keypress
    			{
    				double newSliderTargetPosition = inputDataValues.SliderCurrentPosition + 2.0;
    				outputDataValues.SliderTargetPositionCmd = CalcSliderTargetPositionCmd(newSliderTargetPosition);
    				DriverStation.reportError(String.format("..Info: New Forward Target Slider Position: {0}", newSliderTargetPosition), false);
    				
    			}
    		}
    		else if ((inputDataValues.IsSliderFwdBtnPressed == false) && (inputDataValues.IsSliderRevBtnPressed == true))
    		{
    			if (!workingDataValues.IsSliderRevBtnPressedLastScan)	// debounce keypress
    			{
    				double newSliderTargetPosition = inputDataValues.SliderCurrentPosition - 2.0;
    				outputDataValues.SliderTargetPositionCmd = CalcSliderTargetPositionCmd(newSliderTargetPosition);
    				DriverStation.reportError(String.format("..Info: New Reverse Target Slider Position: {0}", newSliderTargetPosition), false);
    			}
    		}
    		else
    		{
    			// else no change
    		}
    	}
    	    	
    	// ============================
    	//  Step 2.6: Shooter 
    	// ============================
    	
    	if (inputDataValues.ShooterRawVelocityCmd < -0.1)
    	{    	
    		if (_shooterMasterMtr.getControlMode() == CANTalon.TalonControlMode.Speed)
    		{
    			outputDataValues.ShooterMtrCurrentVelocityCmd = RobotMap.SHOOTER_TARGET_MOTOR_RPM;
    		}
    		else if (_shooterMasterMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    		{
    			outputDataValues.ShooterMtrCurrentVelocityCmd = workingDataValues.DriveSpeedScalingFactor;
    		}
    	}
    	else
    	{
    		outputDataValues.ShooterMtrCurrentVelocityCmd = 0.0;
    	}
    	
    	// ============================
    	// Step 2.7: Kicker
    	// ============================
    	if (outputDataValues.ShooterMtrCurrentVelocityCmd > 0.1)
    	{
    		outputDataValues.KickerMtrVelocityCmd = RobotMap.KICKER_TARGET_PERCENT_VBUS_CMD;
    	}
    	else if (inputDataValues.IsKickerReverseBtnPressed)
    	{
    		outputDataValues.KickerMtrVelocityCmd = -1.0;
    		DriverStation.reportError("Kicker in reverse mode", false);
    	}
    	else 
    	{
    		outputDataValues.KickerMtrVelocityCmd = 0.0;
    	}
    	
    	
    	// ===========================
    	// Step 2.8: Camera
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
    	// Step 2.9. Winch
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
    	// Step 2.10. Servo
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
    	
    	// set solenoids state
    	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
    	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
    	
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
    	 
    	if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
    	}
    	else if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{
    		_turretMtr.set(outputDataValues.TurretVelocityCmd);
    	}
    	
    	_kickerMtr.set(outputDataValues.KickerMtrVelocityCmd);
    	
    	_shooterMasterMtr.set(outputDataValues.ShooterMtrCurrentVelocityCmd);
    	
    	if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		_sliderMtr.set(outputDataValues.SliderTargetPositionCmd);
    	}
    	else if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{
    		_sliderMtr.set(outputDataValues.SliderVelocityCmd);
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
    	workingDataValues.IsSliderFwdBtnPressedLastScan = inputDataValues.IsSliderFwdBtnPressed;
    	workingDataValues.IsSliderRevBtnPressedLastScan = inputDataValues.IsSliderRevBtnPressed;
    	workingDataValues.IsInfeedTiltStoreBtnPressedLastScan = inputDataValues.IsInfeedTiltStoreBtnPressed;
    	workingDataValues.IsInfeedTiltFixedBtnPressedLastScan = inputDataValues.IsInfeedTiltFixedBtnPressed;
    	workingDataValues.IsInfeedAcquireBtnPressedLastScan = inputDataValues.IsInfeedAcquireBtnPressed;
    	workingDataValues.IsCameraSwitchBtnPressedLastScan =  inputDataValues.IsCameraSwitchBtnPressed;
    	workingDataValues.IsCupidSwitchBtnPressedLastScan = inputDataValues.IsCupidCameraBtnPressed;
    	workingDataValues.IsBallInPositionLastScan = inputDataValues.IsBallInPosition;
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
    
    private double CalcInfeedTiltAngleInRotations(double InfeedAssemblyAngleInDegrees, double HomePositionAngleInDegrees)
    {   
    	double InfeedAssemblyTiltTargetPositionInRotationsCmd = InfeedAssemblyAngleInDegrees/360.0;
    	double HomePositionAngleInRotations = HomePositionAngleInDegrees/360.0;
    	
    	double InfeedMtrTiltTargetPositionInRotationsCmd = (RobotMap.INFEED_TILT_GEAR_RATIO * (InfeedAssemblyTiltTargetPositionInRotationsCmd
    															- HomePositionAngleInRotations)) + HomePositionAngleInRotations;
    	
    	return -0.11;
    }
    
    // caluclate the appropriate # of leadscrew rotations
    private double CalcSliderTargetPositionCmd(double targetPositionFromHomeInRotations)
    {
    	// Notes:
    	//	The encoder is directly coupled to the leadscrew
    	//		leadscrew pitch : 16 rev / inch
    	//		quad encoder	: 1024 pulses / rev x 4 = 4096 counts / rev
    	//	We setup the TALON is use API Unit Scaling by using the ConfigEncoderCodesPerRev in the axis home method
    	
    	// protect the axis by enforcing guard rails on what can be requested
    	if(targetPositionFromHomeInRotations > RobotMap.SLIDER_FWD_MAX_TRAVEL_IN_ROTATIONS)
    	{
    		targetPositionFromHomeInRotations = RobotMap.SLIDER_FWD_MAX_TRAVEL_IN_ROTATIONS;
    		DriverStation.reportError("Slider already at forward limit", false);
    	}
    	else if (targetPositionFromHomeInRotations < RobotMap.SLIDER_REV_MAX_TRAVEL_IN_ROTATIONS)
    	{
    		targetPositionFromHomeInRotations = RobotMap.SLIDER_REV_MAX_TRAVEL_IN_ROTATIONS;
    		DriverStation.reportError("Slider already at reverse limit", false);
    	}
    	
    	double sliderTargetPositionCmd = targetPositionFromHomeInRotations;
    	    	
    	return sliderTargetPositionCmd;
    }
    
    private double CalcTurretTargetPosition(double TurretAngleInRotations)
    {
    	//double TurretPositionInEncoderCounts = (TurretAngleInRotations * 360)/ RobotMap.TURRET_TRAVEL_DEGREES_PER_COUNT;
    	//return TurretPositionInEncoderCounts;
    	if (TurretAngleInRotations > RobotMap.TURRET_MAX_TRAVEL_IN_ROTATIONS)
    	{
    		TurretAngleInRotations = RobotMap.TURRET_MAX_TRAVEL_IN_ROTATIONS;
    		DriverStation.reportError("Turret already at forward limit", false);
    	}
    	else if (TurretAngleInRotations < RobotMap.TURRET_MIN_TRAVEL_IN_ROTATIONS)
    	{
    		TurretAngleInRotations = RobotMap.TURRET_MIN_TRAVEL_IN_ROTATIONS;
    		DriverStation.reportError("Turret already at reverse limit", false);
    	}
    	
    	double turretSetPositionInRotations = TurretAngleInRotations;
    	
    	return turretSetPositionInRotations;
    }
    
    private double CalcTurretAutoAimTargetPosition(double TurretAngleInRotations, double DesiredTurretTurnInDegrees)
    {
    	double DesiredTurretTurnInRotations = DesiredTurretTurnInDegrees / 360;
    	double AutoAimTargetPosition = TurretAngleInRotations + DesiredTurretTurnInRotations;
    	return AutoAimTargetPosition;
    }
    
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

    // This method Zeros (ie Homes) the Slider Axis
    private void ZeroSliderAxis(RobotData p_robotLiveData, double DesiredSliderPositionAfterZero) 
    {
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	boolean isTimeout = false; //control whether we want this method to run at startup
    	boolean isOnHomeSwitch = !_sliderMtr.isRevLimitSwitchClosed();	// switch is normally closed
    	
    	// zero the current encoder reading
    	//_sliderMtr.setPosition(0);	
    	
    	// if we are not already on the up limit switch
    	if(!isOnHomeSwitch)
    	{
	    	// start out in %VBUS mode
	    	_sliderMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    	
	    	// drive the axis down at 5%
	    	outputDataValues.SliderVelocityCmd = -0.60;
	    	_sliderMtr.set(outputDataValues.SliderVelocityCmd);
	    	
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
	    		}
	    		
	    		isOnHomeSwitch = !_sliderMtr.isRevLimitSwitchClosed();	// switch is normally closed
	    	}
    	}
    	
    	if(!isTimeout)
    	{
    		
	    	_sliderMtr.changeControlMode(CANTalon.TalonControlMode.Position);
	    	
	    	// once we hit it reset the encoder	- this is at approx 90deg
	    	double SliderHomePosition = 0;
	    	
	    	_sliderMtr.setPosition(SliderHomePosition);	
	    	//_infeedTiltMtr.setEncPosition(newPosition);
	    	try {
	    		// sleep a little to let the zero occur
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    	
	    	// setup the PID Loop
	    	_sliderMtr.setPID(RobotMap.SLIDER_KP, RobotMap.SLIDER_KI, RobotMap.SLIDER_KD, RobotMap.SLIDER_KF, RobotMap.SLIDER_IZONE, RobotMap.SLIDER_RAMPRATE, RobotMap.SLIDER_PROFILE);
	    	_sliderMtr.setProfile(RobotMap.SLIDER_PROFILE);
	    	// write to the operator's console log window
	    	DriverStation.reportError("..Slider Axis Zeroed, Chging to Position Ctrl Mode.", false);
	    		    	
	    	outputDataValues.SliderTargetPositionCmd = DesiredSliderPositionAfterZero;
	    	_sliderMtr.set(outputDataValues.SliderTargetPositionCmd);
	    	// finally mark the axis as zeroed
	    	_isSliderAxisZeroedYet = true;
    	}
    	else
    	{
    		// write to the operator's console log window
	    	DriverStation.reportError("..ERROR: Timeout in Slider Axis Zero procedure.", false);
    	}
    	
	}
	
 // This method Zeros (ie Homes) the Slider Axis
    private void ZeroSliderAxisReEntrant(RobotData p_robotLiveData) 
    {
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	boolean isOnHomeSwitch = !_sliderMtr.isRevLimitSwitchClosed();	// switch is normally closed
    	
    	// zero the current encoder reading
    	//_sliderMtr.setPosition(0);	
    	
    	// if we are not already on the up limit switch
    	switch (_sliderZeroState)
    	{
    		case DRIVE_TO_HOME:
    			if(!isOnHomeSwitch)
    	    	{
    		    	// start out in %VBUS mode
    		    	
    		    	long elapsedTime = (new Date().getTime() - _sliderZeroStartTime);
    		    	long maxTimeInMSec = 10000; // 10 secs
    		    	
    		    	// if we are not on the limit switch, drive up until we hit it but only wait for 10 secs max	
    		    	if (elapsedTime  >= maxTimeInMSec)
    		    	{
    		    		_sliderZeroState = Slider_Zero_State.TIMEOUT;
    		    	}
    		    	else 
    		    	{
    		    		if (_sliderMtr.getControlMode() != TalonControlMode.PercentVbus)
    		    		{
    		    			_sliderMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    		    		}
    		    		// drive the axis down at 60%
        		    	outputDataValues.SliderVelocityCmd = -0.60;
    		    	}
    	    	}
    			else 
    			{
    				_sliderZeroState = Slider_Zero_State.ON_HOME;
    			}
    			break;
    		
    		case ON_HOME:
    			outputDataValues.SliderVelocityCmd = 0.0;
    			_sliderMtr.changeControlMode(CANTalon.TalonControlMode.Position);
    	    	
    	    	// once we hit it reset the encoder	- this is at approx 90deg
    	    	double SliderHomePosition = 0;
    	    	
    	    	_sliderMtr.setPosition(SliderHomePosition);	
    	    	//_infeedTiltMtr.setEncPosition(newPosition);
    	    	try {
    	    		// sleep a little to let the zero occur
    				Thread.sleep(1);
    			} catch (InterruptedException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    			}
    	    	
    	    	// setup the PID Loop
    	    	_sliderMtr.setPID(RobotMap.SLIDER_KP, RobotMap.SLIDER_KI, RobotMap.SLIDER_KD, RobotMap.SLIDER_KF, RobotMap.SLIDER_IZONE, RobotMap.SLIDER_RAMPRATE, RobotMap.SLIDER_PROFILE);
    	    	_sliderMtr.setProfile(RobotMap.SLIDER_PROFILE);
    	    	// write to the operator's console log window
    	    	
    	    	_isSliderAxisZeroedYet = true;
    	    	
    	    	DriverStation.reportError("..Slider Axis Zeroed, Chging to Position Ctrl Mode.", false);
    			break;
    			
    		case TIMEOUT:
    			_isSliderAxisZeroTimedOut = true;
    			DriverStation.reportError("Slider Axis Zero procedure timed out", false);
    			outputDataValues.SliderVelocityCmd = 0.0;
    			break;	
    			
    	
    	}
	}
    
	// This method Zeros (ie Homes) the Turret Axis
	private void ZeroTurretAxis(RobotData p_robotLiveData) 
	{
		//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	boolean isTimeout = false; //control whether we want this method to run at startup
    	boolean isOnApproachingHomeSwitch = _turretApproachingHomeLimitSwitch.get();
    	
    	// zero the current encoder reading
    	//_turretMtr.setPosition(0);	
    	
    	// if we are not already on the up limit switch
    	if(!isOnApproachingHomeSwitch)
    	{
	    	// start out in %VBUS mode
    		DriverStation.reportError("..Turret Chg to %VBus Mode.", false);
	    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    	
	    	// drive the axis up at 15%
	    	outputDataValues.TurretVelocityCmd = 0.1;
	    	_turretMtr.set(outputDataValues.TurretVelocityCmd);
	    	
	    	long startTime = System.currentTimeMillis();
	    	long elapsedTime = 0L;
	    	long maxTimeInMSec = 5000; // 5 secs
	    	
	    	// if we are not on the limit switch, drive up until we hit it but only wait for 10 secs max
	    	while(!isOnApproachingHomeSwitch && !isTimeout)
	    	{	
	        	elapsedTime = (new Date().getTime() - startTime);
	    		
	    		if (elapsedTime  >= maxTimeInMSec)
	    		{
	    			isTimeout = true;
	    		}
	    		isOnApproachingHomeSwitch = _turretApproachingHomeLimitSwitch.get();	// switch is normally closed
	    	}
    	}
    	
    	boolean isOnHomeSwitch = _turretHomeLimitSwitch.get();	// switches are normally closed
    	
    	if(!isTimeout)
    	{
    		long startTime = System.currentTimeMillis();
	    	long elapsedTime = 0L;
	    	long maxTimeInMSec = 5000; // 5 secs
	    	
	    	// if we are not on the limit switch, drive up until we hit it but only wait for 10 secs max
	    	while(!isOnHomeSwitch && !isTimeout)
	    	{	
	        	elapsedTime = (new Date().getTime() - startTime);
	    		
	    		if (elapsedTime  >= maxTimeInMSec)
	    		{
	    			isTimeout = true;
	    		}
	    		isOnHomeSwitch = _turretHomeLimitSwitch.get();	// switch is normally closed
	    	}
    	}
    	
    	// we are on the ZERO Limit Switch (and we did not timeout)
    	if(!isTimeout)
    	{
    		// stop driving the axis
    		outputDataValues.TurretVelocityCmd = 0;
	    	_turretMtr.set(outputDataValues.TurretVelocityCmd);
    		
	    	// now switch to position loop mode
	    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.Position);
	    	
	    	// once we hit it reset the encoder
	    	double homePosition = 0;
	    	_turretMtr.setPosition(homePosition);	

	    	try {
	    		// sleep a little to let the zero occur
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    	
	    	// setup the PID Loop
	    	_turretMtr.setPID(RobotMap.TURRET_SLOW_KP, RobotMap.TURRET_SLOW_KI, RobotMap.TURRET_SLOW_KD, RobotMap.TURRET_SLOW_KF, RobotMap.TURRET_SLOW_IZONE, RobotMap.TURRET_SLOW_RAMPRATE, RobotMap.TURRET_SLOW_PROFILE);
	    	_turretMtr.setProfile(RobotMap.TURRET_SLOW_PROFILE);
	    	
	    	// write to the operator's console log window
	    	DriverStation.reportError("..Turret Axis Zeroed, Chg to Positon Ctrl Mode.", false);
	    	
	    	// drive to default position
	    	outputDataValues.TurretTargetPositionCmd = CalcTurretTargetPosition(RobotMap.TURRET_DEFAULT_POSITION_IN_ROTATIONS);
	    	_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
	    	
	    	long startTime = System.currentTimeMillis();
	    	long elapsedTime = 0L;
	    	long maxTimeInMSec = 5000; // 5 secs
	    	
	    	// wait until we get close to the target
	    	while((Math.abs(_turretMtr.getClosedLoopError()) > 400) && !isTimeout)
	    	{	
	        	elapsedTime = (new Date().getTime() - startTime);
	    		
	    		if (elapsedTime  >= maxTimeInMSec)
	    		{
	    			isTimeout = true;
	    		}
	    	}
	    	
	    	// now move to regular gains after the big move
	    	_turretMtr.setPID(RobotMap.TURRET_FAST_KP, RobotMap.TURRET_FAST_KI, RobotMap.TURRET_FAST_KD, RobotMap.TURRET_FAST_KF, RobotMap.TURRET_FAST_IZONE, RobotMap.TURRET_FAST_RAMPRATE, RobotMap.TURRET_FAST_PROFILE);
	    	_turretMtr.setProfile(RobotMap.TURRET_FAST_PROFILE);
	    	
	    	// finally mark the axis as zeroed
	    	_isTurretAxisZeroedYet = true;
    	}
    	else
    	{
    		// write to the operator's console log window
	    	DriverStation.reportError("..ERROR: Timeout in Turret Axis Zero procedure.", false);
    	}
	}
	
	private void ZeroTurretAxisReEntrant(RobotData p_robotLiveData)
	{
		//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	boolean isTimeout = false; //control whether we want this method to run at startup
    	boolean isOnApproachingHomeSwitch = _turretApproachingHomeLimitSwitch.get();
    	boolean isOnHomeSwitch = _turretHomeLimitSwitch.get();	// switches are normally closed
    	
    	switch (_turretZeroState)
    	{
    		case BEFORE_APPROACHING_SWITCH:
    	    	// start out in %VBUS mode
	    		DriverStation.reportError("..Turret Chg to %VBus Mode.", false);
		    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		    	
		    	// drive the axis up at 15%
		    	outputDataValues.TurretVelocityCmd = 0.10;
		    	_turretMtr.set(outputDataValues.TurretVelocityCmd);
    			
    			if(!isOnApproachingHomeSwitch)
    	    	{	    	
    		    	long elapsedTime = 0L;
    		    	long maxTimeInMSec = 8000; // 8 secs
    		    	
    		    	// if we are not on the limit switch, drive up until we hit it but only wait for 5 secs max
		        	elapsedTime = (new Date().getTime() - _turretZeroStartTime);
		    		
		    		if (elapsedTime  >= maxTimeInMSec)
		    		{
		    			_turretZeroState = Turret_Zero_State.TIMEOUT;
		    		}
    	    	}
    			else
    			{
    				_turretZeroState = Turret_Zero_State.ON_APPROACHING_SWITCH;
    				DriverStation.reportError("Turret Zero State: ON_APPROACHING_SWITCH", false);
    			}
    			break;
    		
    		case ON_APPROACHING_SWITCH:
    			_turretZeroStartTime = System.currentTimeMillis();
    			_turretZeroState = Turret_Zero_State.BEFORE_HOME_SWITCH;
    			DriverStation.reportError("Turret Approaching Switch hit", false);
    			break;
    			
    		case BEFORE_HOME_SWITCH:
    	    	long elapsedTime = 0L;
    	    	long maxTimeInMSec = 5000; // 5 secs
    	    	
    	    	// if we are not on the limit switch, drive up until we hit it but only wait for 5 secs max
    	    	if(!isOnHomeSwitch)
    	    	{
    	        	elapsedTime = (new Date().getTime() - _turretZeroStartTime);
    	    		if (elapsedTime  >= maxTimeInMSec)
    	    		{
    	    			_turretZeroState = Turret_Zero_State.TIMEOUT;
    	    		}

    	    	}
    	    	else
    	    	{
    	    		_turretZeroState = Turret_Zero_State.ON_HOME_SWITCH;
    	    		DriverStation.reportError("Turret Zero State: ON_HOME_SWITCH", false);
    	    	}
    			break;
    			
    		case ON_HOME_SWITCH:
    			// stop driving the axis
        		outputDataValues.TurretVelocityCmd = 0;
    	    	_turretMtr.set(outputDataValues.TurretVelocityCmd);
        		
    	    	// now switch to position loop mode
    	    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.Position);
    	    	
    	    	// once we hit it reset the encoder
    	    	double homePosition = 0;
    	    	_turretMtr.setPosition(homePosition);	

    	    	try {
    	    		// sleep a little to let the zero occur
    				Thread.sleep(1);
    			} catch (InterruptedException e) {
    				// TODO Auto-generated catch block
    				e.printStackTrace();
    			}
    	    	
    	    	// setup the PID Loop
    	    	_turretMtr.setPID(RobotMap.TURRET_SLOW_KP, RobotMap.TURRET_SLOW_KI, RobotMap.TURRET_SLOW_KD, RobotMap.TURRET_SLOW_KF, RobotMap.TURRET_SLOW_IZONE, RobotMap.TURRET_SLOW_RAMPRATE, RobotMap.TURRET_SLOW_PROFILE);
    	    	_turretMtr.setProfile(RobotMap.TURRET_SLOW_PROFILE);
    	    	DriverStation.reportError("Turret set to Slow Profile", false);
    	    	
    	    	// write to the operator's console log window
    	    	DriverStation.reportError("..Turret Axis Zeroed, Chg to Positon Ctrl Mode.", false);
    	    	
    	    	// finally mark the axis as zeroed
    	    	_isTurretAxisZeroedYet = true;
    			break;
    			
    		case TIMEOUT:
    			DriverStation.reportError("Turret Axis Zero timed out", false);
    			outputDataValues.TurretVelocityCmd = 0.0;
		    	_turretMtr.set(outputDataValues.TurretVelocityCmd);
    			break;
    	}
    	
	}
		
	// For PID velocity control we need to convert Target RPM into encoder counts per 100mSec
    private double CalcShooterVelociytInNativeUnitsPer100mSec(int targetWheelRPM)
    {
    	//  (1 / 60) => 1 min / 60 Sec
    	//  (1 / 10) => 1 sec / 10 (100mSec chunks)
    	double encoderCountsPer100mSec = targetWheelRPM * (1 / 60) * (1 / 10) * (RobotMap.SHOOTER_ENCODER_COUNTS_PER_REV * RobotMap.SHOOTER_ENCODER_QUAD_MULTIPLIER);
    	
    	return encoderCountsPer100mSec;
    }
    
	
    private double CalcShooterFeedFwdGain(double shooterVelocityInNativeUnitsPer100mSec)
    {
    	//  (1 / 60) => 1 min / 60 Sec
    	//  (1 / 10) => 1 sec / 10 (100mSec chunks)
    	double targetShooterFeedFwdGain = 1023 / shooterVelocityInNativeUnitsPer100mSec;
    	
    	return targetShooterFeedFwdGain;
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
    	//inputDataValues.IsScaleDriveSpeedUpBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SCALE_SPEED_UP_BTN);
    	//inputDataValues.IsScaleDriveSpeedDownBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SCALE_SPEED_DOWN_BTN);
    	inputDataValues.IsPumaFrontToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN);
    	inputDataValues.IsPumaBackToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN);
    	inputDataValues.IsPumaBothToggleBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_PUMA_BOTH_TOGGLE_BTN);
    	inputDataValues.IsInfeedTiltStoreBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_STORE_BTN);
    	inputDataValues.IsInfeedTiltDeployBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_DEPLOY_BTN);
    	//inputDataValues.IsInfeedTiltFixedBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_FIXED_BTN);
    	inputDataValues.IsKickerReverseBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_KICKER_REVERSE_BTN);
    	inputDataValues.IsCupidLoadBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_CUPID_LOAD_BTN);
    	inputDataValues.IsCupidShootBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_CUPID_SHOOT_BTN);
    	
    	//inputDataValues.IsShooterSpeedUpBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_SHOOTER_SPEED_UP_BTN);
    	//inputDataValues.IsShooterSpeedDownBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_SHOOTER_SPEED_DOWN_BTN);
    	inputDataValues.IsSliderFwdBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_SLIDER_FWD_BTN);
    	inputDataValues.IsSliderRevBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_SLIDER_REV_BTN);
    	inputDataValues.IsInfeedAcquireBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_ACQUIRE_BTN);
    	inputDataValues.IsInfeedReleaseBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_RELEASE_BTN);
    	inputDataValues.IsCameraSwitchBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_CAMERA_SWITCH_BTN);
    	inputDataValues.IsElevatorTimerOverrideBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_ELEVATOR_TIMER_OVERRIDE_BTN);
    	inputDataValues.IsCupidCameraBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_CUPID_CAMERA_BTN);
    	
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	inputDataValues.ArcadeDriveThrottleRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK);
    	inputDataValues.ArcadeDriveTurnRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK);
    	
    	inputDataValues.ShooterRawVelocityCmd = _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_SHOOTER_AXIS);
    	inputDataValues.TurretCCWRawVelocityCmd = _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_TURRET_ANALOG_CCW_AXIS);
    	inputDataValues.TurretCWRawVelocityCmd = _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_TURRET_ANALOG_CW_AXIS);
    	inputDataValues.WinchRawCmd = _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_WINCH_AXIS);
    	
    	//inputDataValues.InfeedRawTiltCmd = _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_INFEED_TILT_AXIS);
    	inputDataValues.InfeedTiltUpCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_UP_AXIS);
    	inputDataValues.InfeedTiltDownCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_INFEED_TILT_DOWN_AXIS);
 		
    	// ==========================
    	// 1.3 get values from motor controlllers
    	// ==========================
    	//inputDataValues.LeftDriveEncoderCurrentCount = _leftDriveMasterMtr.getPosition();
    	//inputDataValues.RightDriveEncoderCurrentCount = _rightDriveMasterMtr.getPosition();	
    	inputDataValues.InfeedTiltEncoderCurrentCount = _infeedTiltMtr.getPosition();
    	inputDataValues.IsInfeedTiltAxisOnUpLimitSwitch = !_infeedTiltMtr.isFwdLimitSwitchClosed();
    	inputDataValues.TurretEncoderCurrentPosition = _turretMtr.getPosition();
    	inputDataValues.SliderEncoderCurrentCount = _sliderMtr.getPosition();
    	inputDataValues.SliderCurrentPosition = _sliderMtr.getPosition();
    	inputDataValues.ShooterActualSpeed = _shooterMasterMtr.getSpeed();

    	// ==========================
    	// 1.4 get values from Limit Switches on DIO Ports
    	// ==========================
    	inputDataValues.IsTurretHomeLimitSwitchClosed = _turretHomeLimitSwitch.get();
    	inputDataValues.IsTurretApproachingHomeLimitSwitchClosed = _turretApproachingHomeLimitSwitch.get();
    	inputDataValues.IsBallInPosition = _isBallInPositionLimitSwitch.get();
    	
    	// ==========================
    	// 1.5 get values from navX
    	// ==========================
    	inputDataValues.NavxIsConnected = _navXSensor.isConnected();
    	inputDataValues.NavxIsCalibrating = _navXSensor.isCalibrating();
    	inputDataValues.NavxYaw = _navXSensor.getYaw();
    	inputDataValues.NavxPitch = _navXSensor.getPitch();
    	inputDataValues.NavxRoll = _navXSensor.getRoll();
    	inputDataValues.NavxCompassHeading = _navXSensor.getCompassHeading();
    	inputDataValues.NavxFusedHeading = _navXSensor.getFusedHeading();
    	inputDataValues.NavxTotalYaw = _navXSensor.getAngle();
    	inputDataValues.NavxYawRateDPS = _navXSensor.getRate();
    	inputDataValues.NavxAccelX = _navXSensor.getWorldLinearAccelX();
    	inputDataValues.NavxAccelY = _navXSensor.getWorldLinearAccelY();
    	inputDataValues.NavxIsMoving = _navXSensor.isMoving();
    	inputDataValues.NavxIsRotating = _navXSensor.isRotating();
    	
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
    	
    	// =========================
    	// 2.0 Calc Working Values
    	// ==========================
    	
    	// speed units are are sensor's native ticks per 100mSec
    	//  1000 counts => 10 RPS (Rotation per second)
    	
    	// 2.1 Left Axis
    	
		workingDataValues.LeftDriveEncoderLastDeltaCount = (inputDataValues.LeftDriveEncoderCurrentCount 
																- workingDataValues.LeftDriveEncoderLastCount);
		workingDataValues.LeftDriveEncoderTotalDeltaCount = (inputDataValues.LeftDriveEncoderCurrentCount 
																- workingDataValues.LeftDriveEncoderInitialCount);
		workingDataValues.LeftDriveEncoderLastCount = inputDataValues.LeftDriveEncoderCurrentCount;
    	
    	//workingDataValues.LeftDriveEncoderCurrentCPS = _leftDriveMasterMtr.getSpeed() * 10.0;

    	workingDataValues.LeftDriveWheelsCurrentSpeedIPS = workingDataValues.LeftDriveEncoderCurrentCPS
    														* RobotMap.LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT;

    	workingDataValues.LeftDriveGearBoxCurrentRPM = (workingDataValues.LeftDriveEncoderCurrentCPS 
															* 60										// CPS -> CPM
															/ RobotMap.LEFT_DRIVE_ENCODER_COUNTS_PER_REV);		// CPM -> RPM

		workingDataValues.LeftDriveMotorCurrentRPM = workingDataValues.LeftDriveGearBoxCurrentRPM
														* RobotMap.LEFT_DRIVE_GEAR_BOX_RATIO;
    	
		// 2.2 Right Axis
		workingDataValues.RightDriveEncoderLastDeltaCount = (inputDataValues.RightDriveEncoderCurrentCount 
																- workingDataValues.RightDriveEncoderLastCount);
		workingDataValues.RightDriveEncoderTotalDeltaCount = (inputDataValues.RightDriveEncoderCurrentCount 
																- workingDataValues.RightDriveEncoderInitialCount);
		workingDataValues.RightDriveEncoderLastCount = inputDataValues.RightDriveEncoderCurrentCount;
		
    	//workingDataValues.RightDriveEncoderCurrentCPS = _rightDriveMasterMtr.getSpeed() * 10.0;
    	
    	workingDataValues.RightDriveWheelsCurrentSpeedIPS = workingDataValues.RightDriveEncoderCurrentCPS
    															* RobotMap.RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT;
    	
    	workingDataValues.RightDriveGearBoxCurrentRPM = (workingDataValues.RightDriveEncoderCurrentCPS 
															* 60										// CPS -> CPM
															/ RobotMap.RIGHT_DRIVE_ENCODER_COUNTS_PER_REV);		// CPM -> RPM
    	
    	workingDataValues.RightDriveMotorCurrentRPM = workingDataValues.RightDriveGearBoxCurrentRPM
														* RobotMap.RIGHT_DRIVE_GEAR_BOX_RATIO;
    	
    	// 2.3 Turret
    	
    	//workingDataValues.TurretEncoderTotalDeltaCount = (inputDataValues.TurretEncoderCurrentPosition
    	//													- workingDataValues.TurretEncoderInitialCount);
    	
    	//workingDataValues.TurretEncoderDegreesCount = (workingDataValues.TurretEncoderTotalDeltaCount)
    	//												* RobotMap.TURRET_TRAVEL_DEGREES_PER_COUNT;
    	
    	
    	// 2.4 Shooter 
    	inputDataValues.ShooterEncoderCurrentCP100MS = _shooterMasterMtr.getSpeed();
    	
    	// Counts per 100 ms * 60 seconds per minute * 10 (100 ms per second)/ 4096 counts per rev
    	workingDataValues.ShooterWheelCurrentRPM = (inputDataValues.ShooterEncoderCurrentCP100MS * 60 * 10) / 4096  ;
    	
    	
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
    	    	
		// Drive Motors
		//SmartDashboard.putNumber("Drive.Btn:SpeedScaleFactor", workingDataValues.DriveSpeedScalingFactor);
		
		//SmartDashboard.putNumber("Drive.Left:JoyThrottleRawCmd", inputDataValues.ArcadeDriveThrottleRawCmd);
		//SmartDashboard.putNumber("Drive.Right:JoyTurnRawCmd", inputDataValues.ArcadeDriveTurnRawCmd);
				
		//SmartDashboard.putNumber("Drive.Left:ArcadeDriveThrottleCmd", outputDataValues.ArcadeDriveThrottleAdjCmd);
		//SmartDashboard.putNumber("Drive.Right:ArcadeDriveTurnCmd", outputDataValues.ArcadeDriveTurnAdjCmd);
		/*
		SmartDashboard.putNumber("Drive.Left:EncInitCount", workingDataValues.LeftDriveEncoderInitialCount);
		SmartDashboard.putNumber("Drive.Left:EncCurrCount", inputDataValues.LeftDriveEncoderCurrentCount);
		SmartDashboard.putNumber("Drive.Left:EncDeltaCount", workingDataValues.LeftDriveEncoderTotalDeltaCount);
		
		SmartDashboard.putNumber("Drive.Left:MtrCurSpeedRPM", workingDataValues.LeftDriveMotorCurrentRPM);
		SmartDashboard.putNumber("Drive.Left:GBCurSpeedRPM", workingDataValues.LeftDriveGearBoxCurrentRPM);
		SmartDashboard.putNumber("Drive.Left:EncCurSpeedCPS", workingDataValues.LeftDriveEncoderCurrentCPS);
		SmartDashboard.putNumber("Drive.Left:WheelCurSpeedIPS", workingDataValues.LeftDriveWheelsCurrentSpeedIPS);

		SmartDashboard.putNumber("Drive.Right:EncInitCount", workingDataValues.RightDriveEncoderInitialCount);  
		SmartDashboard.putNumber("Drive.Right:EncCurCount", inputDataValues.RightDriveEncoderCurrentCount);
		SmartDashboard.putNumber("Drive.Right:EncDeltaCount", workingDataValues.RightDriveEncoderTotalDeltaCount);
		
		SmartDashboard.putNumber("Drive.Right:MtrCurSpeedRPM", workingDataValues.RightDriveMotorCurrentRPM);
		SmartDashboard.putNumber("Drive.Right:GBCurSpeedRPM", workingDataValues.RightDriveGearBoxCurrentRPM);
		SmartDashboard.putNumber("Drive.Right:EncCurSpeedCPS", workingDataValues.RightDriveEncoderCurrentCPS);
		SmartDashboard.putNumber("Drive.Right:WheelCurSpeedIPS", workingDataValues.RightDriveWheelsCurrentSpeedIPS);
		*/
    	
		// Turret
		//SmartDashboard.putNumber("Turret.EncDeltaCount", workingDataValues.TurretEncoderTotalDeltaCount);
		//SmartDashboard.putNumber("Turret.EncDegreesCount", workingDataValues.TurretEncoderDegreesCount);
		//SmartDashboard.putBoolean("Turret.IsTurretTargetBtnPressed", inputDataValues.IsTurretTargetBtnPressed);
		//SmartDashboard.putBoolean("Turret.IsTurretEncoderDegreesTargetYet", workingDataValues.IsTurretEncoderDegreesTargetYet);
		//SmartDashboard.putNumber("Turret.TurnDegreesCmd", workingDataValues.TurretTurnRotationsCmd);
    	
    	SmartDashboard.putNumber("Turret.ClosedLoopError", _turretMtr.getClosedLoopError());
		// Infeed
		//SmartDashboard.putNumber("Infeed.RawTiltCmd", inputDataValues.InfeedRawTiltCmd);
		//SmartDashboard.putNumber("InfeedAcqMtrVelocityCmd", outputDataValues.InfeedAcqMtrVelocityCmd);
		//SmartDashboard.putNumber("InfeedTiltMtrVelocityCmd", outputDataValues.InfeedTiltMtrVelocityCmd);
		
		// Puma Drive
    	/*
		SmartDashboard.putBoolean("IsInfeedAcquireBtnPressed", inputDataValues.IsInfeedAcquireBtnPressed);
		SmartDashboard.putBoolean("IsInfeedReleaseBtnPressed", inputDataValues.IsInfeedReleaseBtnPressed);
		SmartDashboard.putBoolean("IsPumaFrontToggleBtnPressed", inputDataValues.IsPumaFrontToggleBtnPressed);
		SmartDashboard.putBoolean("IsPumaBackToggleBtnPressed", inputDataValues.IsPumaBackToggleBtnPressed);
		*/
		// Shooter
		//SmartDashboard.putNumber("Shooter.EncoderCurrentCP100MS", inputDataValues.ShooterEncoderCurrentCP100MS);
    	SmartDashboard.putNumber("Shooter.TargetSpeed", outputDataValues.ShooterMtrTargetVelocityCmd);
    	SmartDashboard.putNumber("Shooter.ActualSpeed", inputDataValues.ShooterActualSpeed);
		
		// Slider
		SmartDashboard.putNumber("Slider.NumberOfClicks", inputDataValues.SliderCurrentPosition);
		SmartDashboard.putNumber("Slider.TargetPosition", outputDataValues.SliderTargetPositionCmd);
		
		// Axis Home Routines
		SmartDashboard.putBoolean("Is Infeed Zeroed", _isInfeedTiltAxisZeroedYet);
		SmartDashboard.putBoolean("Is Slider Zeroed", _isSliderAxisZeroedYet);
		SmartDashboard.putBoolean("Is Turret Zeroed", _isTurretAxisZeroedYet);
		
		// Kangaroo Charge Level
		//SmartDashboard.putNumber("Kangaroo charge level", visionData.BatteryChargeLevel);
		
		// Vision Data
		//SmartDashboard.putBoolean("Vision.IsValidData", inputDataValues.IsValidData);
		//SmartDashboard.putNumber("Vision.DistanceToTarget", inputDataValues.DistanceToTarget);
		//SmartDashboard.putNumber("Vision.EffectiveTargetWidth", inputDataValues.EffectiveTargetWidth);
		//SmartDashboard.putNumber("Vision.DesiredSliderPosition", inputDataValues.DesiredSliderPosition);
		//SmartDashboard.putNumber("Vision.DesiredTurretTurnInDegrees", inputDataValues.DesiredTurretTurnInDegrees);
		//SmartDashboard.putBoolean("Vision.IsValidShot", inputDataValues.IsValidShot);
		SmartDashboard.putNumber("Vision.DesiredTurretTurnInDegrees", inputDataValues.DesiredTurretTurnInDegrees);
		//SmartDashboard.putString("Vision.LastVisionDataRecievedDT", (new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(inputDataValues.LastVisionDataRecievedDT)));

		// Logging
		//SmartDashboard.putBoolean("Log:IsLoggingEnabled", workingDataValues.IsLoggingEnabled);
		//SmartDashboard.putString("Log:LogFilePathName", workingDataValues.LogFilePathName); 
		
		// NavX
		SmartDashboard.putBoolean("NavX:IsConnected", inputDataValues.NavxIsConnected);
        //SmartDashboard.putBoolean("NavX_IsCalibrating", inputDataValues.NavxIsCalibrating);
        //SmartDashboard.putNumber("NavX_Yaw", inputDataValues.NavxYaw);
        //SmartDashboard.putNumber("NavX_Pitch", inputDataValues.NavxPitch);
        SmartDashboard.putNumber("NavX:PitchAngle", inputDataValues.NavxRoll);
        //SmartDashboard.putNumber("NavX_CompassHeading", inputDataValues.NavxCompassHeading);
        //SmartDashboard.putNumber("NavX_FusedHeading", inputDataValues.NavxFusedHeading); 
        //SmartDashboard.putNumber("NavX_TotalYaw", inputDataValues.NavxTotalYaw); 
        //SmartDashboard.putNumber("NavX_YawRateDPS", inputDataValues.NavxYawRateDPS); 
        //SmartDashboard.putNumber("NavX_Accel_X", inputDataValues.NavxAccelX); 
        //SmartDashboard.putNumber("NavX_Accel_Y", inputDataValues.NavxAccelY); 
        //SmartDashboard.putBoolean("NavX_IsMoving", inputDataValues.NavxIsMoving); 
        //SmartDashboard.putBoolean("NavX_IsRotating", inputDataValues.NavxIsRotating); 
		
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
		// create a new instance of the RobotData object
    	_robotLiveData = new RobotData();
    	
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	// set defaults for initial motor speeds
    	outputDataValues.KickerMtrVelocityCmd = 0.0;
    	outputDataValues.ShooterMtrCurrentVelocityCmd = 0.0;
    	outputDataValues.SliderVelocityCmd = 0.0;
    	
    	// Set desired initial (default) solenoid positions
    	// puma up to cross defenses
		outputDataValues.PumaFrontSolenoidPosition = RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION;
		outputDataValues.PumaBackSolenoidPosition = RobotMap.PUMA_BACK_SOLENOID_UP_POSITION;
    	
    	// zero turret
    	if (!_isTurretAxisZeroedYet)
    	{
    		ZeroTurretAxis(_robotLiveData);
    	}
    	
    	// zero slider
    	if (!_isSliderAxisZeroedYet)
    	{
    		_sliderZeroStartTime = System.currentTimeMillis();
    		_sliderZeroState = Slider_Zero_State.DRIVE_TO_HOME;
    		//ZeroSliderAxisReEntrant(_robotLiveData, RobotMap.SLIDER_DEFAULT_TARGET_POSITION);
    		ZeroSliderAxis(_robotLiveData, RobotMap.SLIDER_DEFAULT_TARGET_POSITION);
    	}
    	
    	_autoAimAndShootState = RobotData.Auto_Aim_And_Shoot_State.COARSE_TURRET_TO_TARGET;
    	DriverStation.reportError("Changing Auton State To: COARSE_TURRET_TO_TARGET | ", false);
    	
    	_visionClient.startPolling();
	}
	
    /*****************************************************************************************************
     * This function is called periodically during test mode
     * 	(about 50x/sec or about every 20 mSec)
     *****************************************************************************************************/
    @SuppressWarnings("deprecation")
	public void testPeriodic()
    {    	
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	// ===============================
    	// Step 1: Get Inputs
    	// ===============================
    	UpdateInputAndCalcWorkingDataValues(inputDataValues, workingDataValues);
    	
    	// ===============================
    	// Step 2: Do all the real work
    	// ===============================
    	AimAndShoot();
    	
    	// ===================================
    	// Step 3: Below we drive the outputs
    	// ===================================
    	
    	// set solenoids
    	_pumaFrontSolenoid.set(outputDataValues.PumaFrontSolenoidPosition);
    	_pumaBackSolenoid.set(outputDataValues.PumaBackSolenoidPosition);
    	
		// set motor commmands
    	_infeedAcqMtr.set(outputDataValues.InfeedAcqMtrVelocityCmd);        	
    	_kickerMtr.set(outputDataValues.KickerMtrVelocityCmd);
    	_shooterMasterMtr.set(outputDataValues.ShooterMtrCurrentVelocityCmd);
    	
    	// set turret
    	if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		_turretMtr.set(outputDataValues.TurretTargetPositionCmd);
    	}
    	else if (_turretMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{
    		_turretMtr.set(outputDataValues.TurretVelocityCmd);
    	}
    	
    	// set slider
    	if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.Position)
    	{
    		_sliderMtr.set(outputDataValues.SliderTargetPositionCmd);
    	}
    	else if (_sliderMtr.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    	{
    		_sliderMtr.set(outputDataValues.SliderVelocityCmd);
    	}
    	
    	// ==========================
    	// Step 4 Update Dashboard
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
    	// Step 5. Optional Data logging
    	// ==========================
    	if(workingDataValues.IsLoggingEnabled == true)
    	{
    		_dataLogger.WriteData(_robotLiveData);
    	}
    } 
    
    // this is the heart of this function
    //	this method implements a state machine
    private void AimAndShoot()
    {
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	VisionData visionData = _visionClient.GetVisionData();
    	    
    	if (visionData == null || !visionData.IsValidData)
    	{
    		DriverStation.reportError("Error... No valid vision data to use. | ", false); 
    		return;
    	}
    	
    	//	coarse	5 => soft limit		%VBus	.1%
    	//	fine	0 => 5				%VBus	.05%
    	//	shoot						PID Position
    	
    	switch (_autoAimAndShootState)
    	{
    		case ZERO_AXES:
    	    	if(!_isSliderAxisZeroedYet)
    	    	{
    	    		ZeroSliderAxisReEntrant(_robotLiveData);
    	    		//ZeroSliderAxis(_robotLiveData, _sliderAutonPosition);
    	    	}
    	    	else
    	    	{
    	    		outputDataValues.SliderTargetPositionCmd = RobotMap.SLIDER_DEFAULT_TARGET_POSITION;
    	    	}
    	    	
    	    	if (!_isTurretAxisZeroedYet)
    	    	{
    	    		ZeroTurretAxisReEntrant(_robotLiveData);
    	    	}
    	    	
    	    	double sliderPositionError = Math.abs(outputDataValues.SliderTargetPositionCmd - _sliderMtr.getPosition());
    	    	if (_isSliderAxisZeroedYet && _isTurretAxisZeroedYet && (sliderPositionError < 1.0) && (outputDataValues.SliderTargetPositionCmd == RobotMap.SLIDER_DEFAULT_TARGET_POSITION))
    	    	{
    	    		DriverStation.reportError("Changing Auton State To: COARSE_TURRET_TO_TARGET | ", false);
    	    		_autoAimAndShootState = Auto_Aim_And_Shoot_State.COARSE_TURRET_TO_TARGET;
    	    	}
    			break;
    			
    		case COARSE_TURRET_TO_TARGET:

    			if (Math.abs(visionData.DesiredTurretTurnInDegrees) > 10.0)
    			{
    				if(_turretMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus )
    				{
    					// switch to % VBUS Mode
        				_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        				DriverStation.reportError("Turret changing to PercentVBus mode", false);
    				}
    				
    				if (visionData.DesiredTurretTurnInDegrees > 0)
    				{
	    				outputDataValues.TurretVelocityCmd = 0.15;
	    				//DriverStation.reportError("Turret Speed at 10% | ", false);
    				}
    				else
    				{
	    				outputDataValues.TurretVelocityCmd = -0.15;
	    				//DriverStation.reportError("Turret Speed at -10% | ", false);
    				}    					
    			}
    			else if (Math.abs(visionData.DesiredTurretTurnInDegrees) > 0)
    			{
    				_autoAimAndShootState = Auto_Aim_And_Shoot_State.FINE_TURRET_TO_TARGET;
					DriverStation.reportError("Changing Auton State To: FINE_TURRET_TO_TARGET | ", false);
    			}
    			else
    			{
    				DriverStation.reportError("Coarse Vision Data looks invalid" , false);
    			}
    			break;
    			
    		case FINE_TURRET_TO_TARGET:

    			DriverStation.reportError("Vision: " + visionData.DesiredTurretTurnInDegrees + " | ", false);
    			if (Math.abs(visionData.DesiredTurretTurnInDegrees) > 1.25)
    			{
    				if(_turretMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus )
    				{
    					// switch to % VBUS Mode
        				_turretMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        				DriverStation.reportError("Turret changing to PercentVBus mode", false);
    				}
    				
    				if (visionData.DesiredTurretTurnInDegrees > 0)
    				{
	    				outputDataValues.TurretVelocityCmd = 0.05;
	    				DriverStation.reportError("Turret Speed at 10% | ", false);
    				}
    				else
    				{
	    				outputDataValues.TurretVelocityCmd = -0.05;
	    				DriverStation.reportError("Turret Speed at -10% | ", false);
    				}    					
    			}
    			else if(Math.abs(visionData.DesiredTurretTurnInDegrees) > 0)
    			{
    	    		// switch to PID Position Mode
    	    		
    	    		// stop driving the axis
    	    		outputDataValues.TurretVelocityCmd = 0;
    		    	_turretMtr.set(outputDataValues.TurretVelocityCmd);
    	    		
    		    	// now switch to position loop mode
    		    	_turretMtr.changeControlMode(CANTalon.TalonControlMode.Position);
    		    	outputDataValues.TurretTargetPositionCmd = _turretMtr.getPosition();
    		    	DriverStation.reportError("Turret changing to Position mode | ", false);
    		    	
    		    	try {
    		    		// sleep a little to let the zero occur
    					Thread.sleep(1);
    				} catch (InterruptedException e) {
    					// TODO Auto-generated catch block
    					e.printStackTrace();
    				}
    				
    				_autoAimAndShootState = Auto_Aim_And_Shoot_State.SHOOT;
					DriverStation.reportError("Changing Auton State To: SHOOT | ", false);
    			}
    			else
    			{
    				DriverStation.reportError("Fine Vision Data looks invalid" , false);
    			}
    			
				break;
    			
    		case SHOOT:
				// wait until we reach 95% of target wheel speed
				if (inputDataValues.ShooterActualSpeed > (RobotMap.SHOOTER_TARGET_MOTOR_RPM * 0.95))
				{
					// start the infeed to drive the ball up into the shooter
					outputDataValues.InfeedAcqMtrVelocityCmd = 1.0;
					//DriverStation.reportError("Ready to Shoot | ", false);
				}
				
				// drive both sets of wheels
    			outputDataValues.ShooterMtrCurrentVelocityCmd = RobotMap.SHOOTER_TARGET_MOTOR_RPM;
				outputDataValues.KickerMtrVelocityCmd = RobotMap.KICKER_TARGET_PERCENT_VBUS_CMD;
    			break;
    			
    		case TIMEOUT:
    			DriverStation.reportError("Error... Auto Aim and Shoot sequence timed out | ", false);
    			break;
    	}
    }
}