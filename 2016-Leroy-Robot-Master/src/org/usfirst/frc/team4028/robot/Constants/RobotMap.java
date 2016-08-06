package org.usfirst.frc.team4028.robot.Constants;

import java.net.InetAddress;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * This class contains global constants that define the layout of Team 4028's 2015 Season Recycle Rush Robot
 * 
 * 
 * Date			Rev		Author						Comments
 * -----------	------	-------------------------	---------------------------------- 
 * 23.Aug.2015	0.2		Tom Bruns					Refactored and moved constants to a new class
 * 27.Jun.2015	0.1		Sebastian Rodriguez			Initial Version
 */
public class RobotMap 
{	
	
	// define constants for the socket connecting to the vision PC
	public static final int VISION_PC_PORT = 5806;  // Allowed to use 5800-5810
	public static final int CAMERA_SWAP_PORT = 5807;
	public static final String VISION_PC_IP_ADDRESS = "10.40.28.78";
	
	// define constants for array values of vision data
	public static final int IS_VALID_DATA_ARRAY_POSITION = 0;
	public static final int DISTANCE_TO_TARGET_ARRAY_POSITION = 1;
	public static final int EFFECTIVE_TARGET_WIDTH_ARRAY_POSITION = 2;
	public static final int DESIRED_SLIDER_POSITION_ARRAY_POSITION = 3;
	public static final int BATTERY_CHARGE_LEVEL = 4;
	public static final int DESIRED_TURRET_TURN_IN_DEGREES_ARRAY_POSITION = 5;
	public static final int IS_VALID_SHOT_ARRAY_POSITION = 6;
	
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
	public static final int CAN_ADDR_TURRET_TALON = 16;
	public static final int CAN_ADDR_MASTER_SHOOTER_TALON = 17;
	public static final int CAN_ADDR_SLAVE_SHOOTER_TALON = 18;
	public static final int CAN_ADDR_SHOOTER_SLIDER_TALON = 19;
	public static final int CAN_ADDR_INFEED_TILT_MTR_TALON = 20;
	
	// ======================================
	// define constants for PWM Ports on RobioRio
	// ======================================
	public static final int CUPID_SERVO_PWM_PORT = 6;
	public static final int SCALING_MTR_PWM_PORT = 7;
	public static final int INFEED_ACQ_MTR_PWM_PORT = 8;
	public static final int SHOOTER_KICKER_PWM_PORT = 9;
	
	// ======================================
	// define constantsw for DIO Ports on RoboRio
	// ======================================
	public static final int TURRET_HOME_LIMIT_SWITCH_DIO_PORT = 0;
	public static final int TURRET_APPROACHING_HOME_LIMIT_SWITCH_DIO_PORT = 1;
	public static final int IS_BALL_IN_POSITION_LIMIT_SWITCH = 2;
	
	// ======================================
	// Define constants for solenoid ports on Pneumatic Control Module (PCM)
	// ======================================
	public static final int PCM_PORT_PUMA_FRONT_SOLENOID_RETRACT = 2;
	public static final int PCM_PORT_PUMA_FRONT_SOLENOID_EXTEND = 3;
	public static final int PCM_PORT_PUMA_BACK_SOLENOID_RETRACT = 0;   	// 3
	public static final int PCM_PORT_PUMA_BACK_SOLENOID_EXTEND = 1;		// 2
	public static final int PCM_PORT_SHIFTER_SOLENOID_EXTEND = 7;
	public static final int PCM_PORT_SHIFTER_SOLENOID_RETRACT = 6;
	public static final int PCM_PORT_PERIMETER_EXPANSION_EXTEND = 4;
	public static final int PCM_PORT_PERIMETER_EXPANSION_RETRACT = 5;
	
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
	public static final Value PERIMETER_EXPANSION_IN = DoubleSolenoid.Value.kForward;
	public static final Value PERIMETER_EXPANSION_OUT = DoubleSolenoid.Value.kReverse;
	
	// ======================================
	// define constants for the navx
	// ======================================
	public static final double NAVX_MAX_ALLOWABLE_ANGLE = 0.0;
	
	// ======================================
	// define constants for Left Drive Motor
	// ======================================
	public static final int LEFT_DRIVE_ENCODER_COUNTS_PER_REV = 1000;					// 250 CPR, 4X (Quad Encoder)
	public static final double LEFT_DRIVE_GEAR_BOX_RATIO = 14.88;						// 14:88 : 1 (not relevant since encoder is on output shaft of gearbox)
	public static final double LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV = 18.850;		// 6" Wheel Dia, C = 2*Pi*R
	
	public static final double LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT 
			= LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV / LEFT_DRIVE_ENCODER_COUNTS_PER_REV;
	
	// ======================================
	// Define constants for Right Drive Motor
	// ======================================
	public static final int RIGHT_DRIVE_ENCODER_COUNTS_PER_REV = 1000;					// 250 CPR, 4X (Quad Encoder)
	public static final double RIGHT_DRIVE_GEAR_BOX_RATIO = 14.88;						// 14:88 : 1 (not relevant since encoder is on output shaft of gearbox)
	public static final double RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV = 18.850;		// 6" Wheel Dia, C = 2*Pi*R

	public static final double RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT 
			= RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV / RIGHT_DRIVE_ENCODER_COUNTS_PER_REV;
	
	// ======================================
	// Define constants for Infeed Tilt Motor
	// ======================================
	public static final int INFEED_TILT_ENCODER_COUNTS_PER_REV = 4096;                  // 1024 CPR
	public static final int INFEED_TILT_ENCODER_QUAD_MILTIPLIER = 1;                  	//  x 4 (Quad Encoder)
	public static final double INFEED_TILT_GEAR_RATIO = 34.0 / 22.0;						// 1.5455
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
	public static final double INFEED_TILT_FIXED_POSITION_CMD = 0.18; //0.05;
	public static final double INFEED_TILT_DEPLOYED_POSITION_CMD = -0.1;		// this is approx 0 deg
	public static final double INFEED_TILT_LOWER_LIMIT_PUMA_DOWN = -0.36; //-0.25;
	public static final double INFEED_TILT_LOWER_LIMIT_PUMA_UP = -0.59;
	
	// ======================================
	// Define constants for Turret Motor
	// ======================================
	public static final int TURRET_ENCODER_COUNTS_PER_REV = 1024;                        // 7 Counts per rev, 71:1 reduction  (encoder is after gearbox)
	public static final double TURRET_GEAR_RATIO = 9.5556;								// gear ratio from motor assy output shaft to big gear under the turret
	public static final double TURRET_TRAVEL_DISTANCE_DEGREES_PER_REV = 37.674;			// 360deg / TURRET_GEAR_RATIO
	
	public static final double TURRET_MAX_TRAVEL_IN_ROTATIONS = 0.36;
	public static final double TURRET_MIN_TRAVEL_IN_ROTATIONS = -1.41;
	public static final double TURRET_DEFAULT_POSITION_IN_ROTATIONS = 0.0; //-2.413;
	
	public static final double TURRET_PERCENTVBUS_SCALING_FACTOR = 0.25;
	
	public static final double TURRET_AUTON_MAX_CLOSED_LOOP_ERROR = 200;
	public static final double TURRET_AUTON_MAX_ADJUSTABLE_ERROR_IN_DEGREES = 0.5;
	
	public static final double TURRET_SLOW_KP = 0.06;   // Proportional 
	public static final double TURRET_SLOW_KI = 0.0;   // Integral
	public static final double TURRET_SLOW_KD = 0.0;   // Derivative
	public static final double TURRET_SLOW_KF = 0.0;   // Feed Forward
	public static final int TURRET_SLOW_IZONE = 0;     // Encoder ticks/Analog Units, max value of integral term before it's reset	
	public static final double TURRET_SLOW_RAMPRATE = 64; // Volts/Second
	public static final int TURRET_SLOW_PROFILE = 0;

	public static final double TURRET_MEDIUM_KP = 0.11;   // Proportional 
	public static final double TURRET_MEDIUM_KI = 0.0;   // Integral
	public static final double TURRET_MEDIUM_KD = 0.0;   // Derivative
	public static final double TURRET_MEDIUM_KF = 0.0;   // Feed Forward
	public static final int TURRET_MEDIUM_IZONE = 0;     // Encoder ticks/Analog Units, max value of integral term before it's reset	
	public static final double TURRET_MEDIUM_RAMPRATE = 64; // Volts/Second
	public static final int TURRET_MEDIUM_PROFILE = 1;
	
	public static final double TURRET_FAST_KP = 0.25;   // Proportional 
	public static final double TURRET_FAST_KI = 0.0;   // Integral
	public static final double TURRET_FAST_KD = 0.0;   // Derivative
	public static final double TURRET_FAST_KF = 0.0;   // Feed Forward
	public static final int TURRET_FAST_IZONE = 0;     // Encoder ticks/Analog Units, max value of integral term before it's reset	
	public static final double TURRET_FAST_RAMPRATE = 64; // Volts/Second
	public static final int TURRET_FAST_PROFILE = 1;
	
	
	
	public static final double TURRET_TRAVEL_DEGREES_PER_COUNT 
			= TURRET_TRAVEL_DISTANCE_DEGREES_PER_REV / (4 * TURRET_ENCODER_COUNTS_PER_REV);
	
	// ======================================
	// Define constants for Kicker Motor
	// ======================================
	public static final double KICKER_TARGET_PERCENT_VBUS_CMD = 0.9;
	
	// ======================================
	// Define constants for Shooter Motor
	// ======================================
	public static final int SHOOTER_ENCODER_COUNTS_PER_REV = 1024;						// 1024
	public static final int SHOOTER_ENCODER_QUAD_MULTIPLIER = 4;                  		//  x 4 (Quad Encoder)
	
	public static final int SHOOTER_AUTON_START_MAX_TIME = 6000;
	public static final int SHOOTER_AUTON_RUN_MAX_TIME = 3000;
	
	// _shooterMasterMtr.setPID(.2, 0, 0, .032, 0, 0, RobotMap.SHOOTER_PROFILE);
	public static final double SHOOTER_KP = 0.2;
	public static final double SHOOTER_KI = 0.0;
	public static final double SHOOTER_KD = 0.0;
	public static final double SHOOTER_KF = 0.032;
	public static final int SHOOTER_IZONE = 0;
	public static final double SHOOTER_RAMPRATE = 64;
	public static final int SHOOTER_PROFILE = 1;
	public static final int SHOOTER_TARGET_MOTOR_RPM = 3500;
	public static final int SHOOTER_MAX_MOTOR_RPM = 4600;
	
	// ======================================
	// Define constants for Slider Motor
	// ======================================
	public static final int SLIDER_ENCODER_COUNTS_PER_REV = 250;						// 1024
	public static final int SLIDER_ENCODER_QUAD_MULTIPLIER = 4;                  		//  x 4 (Quad Encoder)
	public static final double SLIDER_ROTATIONS_PER_INCH = 16.0;						// lead screw 
	public static final double SLIDER_FWD_MAX_TRAVEL_IN_ROTATIONS = 60.0;
	public static final double SLIDER_REV_MAX_TRAVEL_IN_ROTATIONS = 0.0;
	public static final double SLIDER_DEFAULT_TARGET_POSITION = 34.0;  //38.0;
	
	public static final double SLIDER_KP = 0.5;
	public static final double SLIDER_KI = 0.0;
	public static final double SLIDER_KD = 0.0;
	public static final double SLIDER_KF = 0.0;
	public static final int SLIDER_IZONE = 0;
	public static final double SLIDER_RAMPRATE = 0;
	public static final int SLIDER_PROFILE = 0;
		
	// ======================================
	// define constants for Driver Station Gamepad
	// ======================================
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	//public static final int DRIVER_GAMEPAD_SCALE_SPEED_UP_BTN = LogitechF310.START_BUTTON;
	//public static final int DRIVER_GAMEPAD_SCALE_SPEED_DOWN_BTN = LogitechF310.BACK_BUTTON;	
	public static final int DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK = LogitechF310.LEFT_Y_AXIS;		
	public static final int DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK = LogitechF310.RIGHT_X_AXIS;
	public static final int DRIVER_GAMEPAD_PUMA_BACK_TOGGLE_BTN = LogitechF310.BACK_BUTTON;
	public static final int DRIVER_GAMEPAD_PUMA_FRONT_TOGGLE_BTN = LogitechF310.START_BUTTON;
	public static final int DRIVER_GAMEPAD_PUMA_BOTH_TOGGLE_BTN = LogitechF310.GREEN_BUTTON_A;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_STORE_BTN = LogitechF310.BLUE_BUTTON_X;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_DEPLOY_BTN = LogitechF310.RED_BUTTON_B;
	//public static final int DRIVER_GAMEPAD_INFEED_TILT_FIXED_BTN = LogitechF310.YELLOW_BUTTON_Y;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_UP_AXIS = LogitechF310.LEFT_TRIGGER;
	public static final int DRIVER_GAMEPAD_INFEED_TILT_DOWN_AXIS = LogitechF310.RIGHT_TRIGGER;
	public static final int DRIVER_GAMEPAD_KICKER_REVERSE_BTN = LogitechF310.YELLOW_BUTTON_Y;
	public static final int DRIVER_GAMEPAD_CUPID_LOAD_BTN = LogitechF310.LEFT_BUMPER;
	public static final int DRIVER_GAMEPAD_CUPID_SHOOT_BTN = LogitechF310.RIGHT_BUMPER;
	
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	//public static final int OPERATOR_GAMEPAD_SHOOTER_SPEED_UP_BTN = LogitechF310.YELLOW_BUTTON_Y;
	//public static final int OPERATOR_GAMEPAD_SHOOTER_SPEED_DOWN_BTN = LogitechF310.BLUE_BUTTON_X;
	public static final int OPERATOR_GAMEPAD_CUPID_CAMERA_BTN = LogitechF310.BLUE_BUTTON_X;
	public static final int OPERATOR_GAMEPAD_INFEED_ACQUIRE_BTN = LogitechF310.RIGHT_BUMPER;
	public static final int OPERATOR_GAMEPAD_INFEED_RELEASE_BTN = LogitechF310.LEFT_BUMPER;
	public static final int OPERATOR_GAMEPAD_WINCH_AXIS = LogitechF310.RIGHT_Y_AXIS;
	public static final int OPERATOR_GAMEPAD_SLIDER_FWD_BTN = LogitechF310.START_BUTTON;
	public static final int OPERATOR_GAMEPAD_SLIDER_REV_BTN = LogitechF310.BACK_BUTTON;
	public static final int OPERATOR_GAMEPAD_SHOOTER_AXIS = LogitechF310.LEFT_Y_AXIS;
	public static final int OPERATOR_GAMEPAD_TURRET_ANALOG_CCW_AXIS = LogitechF310.LEFT_TRIGGER;
	public static final int OPERATOR_GAMEPAD_TURRET_ANALOG_CW_AXIS = LogitechF310.RIGHT_TRIGGER;
	public static final int OPERATOR_GAMEPAD_CAMERA_SWITCH_BTN = LogitechF310.GREEN_BUTTON_A;
	public static final int OPERATOR_GAMEPAD_ELEVATOR_TIMER_OVERRIDE_BTN = LogitechF310.RED_BUTTON_B;
	
	// ======================================
	// define constants for logging
	// ======================================
	// this is where the USB stick is mounted on the RoboRIO filesystem.  You can confirm by logging into the RoboRIO using WinSCP
	public static final String LOG_FILE_PATH = "/media/sda1/logging";
	
	// ======================================
	// define constants for usb cameras
	// ======================================
	public static final String SHOOTER_CAMERA_NAME = "cam0";
	public static final String INFEED_CAMERA_NAME = "cam1";
	public static final String CUPID_CAMERA_NAME = "cam2";
	
	// ======================================
	// Define constants for Robot Tilt Angle Protection
	// ======================================
	public static final float ROBOT_FWD_DRIVE_MAX_TILT_CUTOFF = 30.0f;
	public static final float ROBOT_FWD_DRIVE_MAX_TILT_REENABLE = 28.0f;
		
	public static final float ROBOT_REV_DRIVE_MAX_TILT_CUTOFF = -32.0f;
	public static final float ROBOT_REV_DRIVE_MAX_TILT_REENABLE = -30.0f;
}