package org.usfirst.frc.team4028.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

//This class contains global constants that define the layout of Team 4028's 2016 Offseason Robot for the Beef Squad

public class RobotMap 
{	
	// ======================================
	// Constants for CAN Bus Addresses
	// ======================================
	
	// define constant for PCM (Pneumatic Control Module)
	public static final int CAN_ADDR_PCM = 0;				
	
	// PDP (Power Distribution Panel) CAN Bus Address
	public static final int CAN_ADDR_PDP = 3; 
	
	// define constants for Talon SRX CAN Bus Addresses
	public static final int CAN_ADDR_LEFT_DRIVE_MASTER_TALON = 14;
	public static final int CAN_ADDR_LEFT_DRIVE_SLAVE_TALON = 15;
	public static final int CAN_ADDR_LEFT_DRIVE_SLAVE_2_TALON = 11;
	public static final int CAN_ADDR_RIGHT_DRIVE_MASTER_TALON = 12;
	public static final int CAN_ADDR_RIGHT_DRIVE_SLAVE_TALON = 13;
	public static final int CAN_ADDR_RIGHT_DRIVE_SLAVE_2_TALON = 10;
	public static final int CAN_ADDR_INFEED_TILT_MTR_TALON = 20;
	
	// ======================================
	// define constants for PWM Ports on RobioRio
	// ======================================
	public static final int CUPID_SERVO_PWM_PORT = 6;
	public static final int SCALING_MTR_PWM_PORT = 7;
	public static final int INFEED_ACQ_MTR_PWM_PORT = 8;
	
	// ======================================
	// Define constants for solenoid ports on Pneumatic Control Module (PCM)
	// ======================================
	public static final int PCM_PORT_PUMA_FRONT_SOLENOID_RETRACT = 2;
	public static final int PCM_PORT_PUMA_FRONT_SOLENOID_EXTEND = 3;
	public static final int PCM_PORT_PUMA_BACK_SOLENOID_RETRACT = 0;   	// 3
	public static final int PCM_PORT_PUMA_BACK_SOLENOID_EXTEND = 1;		// 2
	public static final int PCM_PORT_SHIFTER_SOLENOID_EXTEND = 7;
	public static final int PCM_PORT_SHIFTER_SOLENOID_RETRACT = 6;
	
	// ======================================
	// define constants for air cylinder states / positions
	//	(map the physical air cylinder position to logical state)
	// ======================================
	public static final Value PUMA_FRONT_SOLENOID_UP_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value PUMA_FRONT_SOLENOID_DOWN_POSITION = DoubleSolenoid.Value.kReverse;		
	public static final Value PUMA_BACK_SOLENOID_UP_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value PUMA_BACK_SOLENOID_DOWN_POSITION = DoubleSolenoid.Value.kReverse;
	public static final Value SHIFTER_HIGH_GEAR_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value SHIFTER_LOW_GEAR_POSITION = DoubleSolenoid.Value.kReverse;

	// ======================================
	// Define constants for Infeed Tilt Motor
	// ======================================
	public static final int INFEED_TILT_ENCODER_COUNTS_PER_REV = 4096;                  // 1024 CPR
	public static final int INFEED_TILT_ENCODER_QUAD_MILTIPLIER = 1;                  	//  x 4 (Quad Encoder)
	public static final double INFEED_TILT_GEAR_RATIO = 34.0 / 22.0;					// 1.5455
	public static final double INFEED_TILT_TRAVEL_DISTANCE_DEGREES_PER_REV = 360.0  / INFEED_TILT_GEAR_RATIO;
	
	public static final double INFEED_TILT_KP = 0.37; //0.4;
	public static final double INFEED_TILT_KI = 0.0;
	public static final double INFEED_TILT_KD = 30.0;
	public static final double INFEED_TILT_KF = 0.0;
	public static final int INFEED_TILT_IZONE = 0;
	public static final double INFEED_TILT_RAMPRATE = 1;
	public static final int INFEED_TILT_PROFILE = 0;
	
	public static final double INFEED_TILT_TRAVEL_DEGREES_PER_COUNT 
			= INFEED_TILT_TRAVEL_DISTANCE_DEGREES_PER_REV / (4 * INFEED_TILT_ENCODER_COUNTS_PER_REV);
	
	public static final double PUMA_DOWN_INFEED_TILT_SOFT_LIMIT_DEGREES = 25;
	public static final double PUMA_UP_INFEED_TILT_SOFT_LIMIT_DEGREES = 10;
	
	public static final double INFEED_TILT_HOME_POSITION_IN_ROTATIONS = 0.29444;
	public static final double INFEED_TILT_STORED_POSITION_CMD = 0.18;			// this is approx 90 deg
	public static final double INFEED_TILT_FIXED_POSITION_CMD = 0.18; 	  //0.05;
	public static final double INFEED_TILT_DEPLOYED_POSITION_CMD = -0.1;		// this is approx 0 deg
	public static final double INFEED_TILT_LOWER_LIMIT_PUMA_DOWN = -0.36; //-0.25;
	public static final double INFEED_TILT_LOWER_LIMIT_PUMA_UP = -0.59;
		
	// ======================================
	// define constants for Driver Station Gamepad
	// ======================================
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;	
	public static final int DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK = LogitechF310.LEFT_Y_AXIS;		
	public static final int DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK = LogitechF310.RIGHT_X_AXIS;
	public static final int DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN = LogitechF310.BACK_BUTTON;
	public static final int DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN = LogitechF310.START_BUTTON;
	public static final int DRIVER_GAMEPAD_PUMA_BOTH_TOGGLE_BTN = LogitechF310.GREEN_BUTTON_A;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_STORE_BTN = LogitechF310.BLUE_BUTTON_X;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_DEPLOY_BTN = LogitechF310.RED_BUTTON_B;
	public static final int DRIVER_GAMEPAD_SHIFTER_BTN = LogitechF310.YELLOW_BUTTON_Y;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_UP_AXIS = LogitechF310.LEFT_TRIGGER;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_DOWN_AXIS = LogitechF310.RIGHT_TRIGGER;
	public static final int DRIVER_GAMEPAD_CUPID_LOAD_BTN = LogitechF310.LEFT_BUMPER;
	public static final int DRIVER_GAMEPAD_CUPID_SHOOT_BTN = LogitechF310.RIGHT_BUMPER;
	
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	public static final int OPERATOR_GAMEPAD_CUPID_CAMERA_BTN = LogitechF310.BLUE_BUTTON_X;
	public static final int OPERATOR_GAMEPAD_INFEED_ACQUIRE_BTN = LogitechF310.RIGHT_BUMPER;
	public static final int OPERATOR_GAMEPAD_INFEED_RELEASE_BTN = LogitechF310.LEFT_BUMPER;
	public static final int OPERATOR_GAMEPAD_SHOOTER_AXIS = LogitechF310.LEFT_Y_AXIS;
	public static final int OPERATOR_GAMEPAD_WINCH_AXIS = LogitechF310.RIGHT_Y_AXIS;
	public static final int OPERATOR_GAMEPAD_CAMERA_SWITCH_BTN = LogitechF310.GREEN_BUTTON_A;
	
	// ======================================
	// define constants for logging
	// ======================================
	// this is where the USB stick is mounted on the RoboRIO filesystem.  You can confirm by logging into the RoboRIO using WinSCP
	public static final String LOG_FILE_PATH = "/media/sda1/logging";
	
	// ======================================
	// define constants for usb cameras
	// ======================================
	public static final String CAMERA = "cam0";
}