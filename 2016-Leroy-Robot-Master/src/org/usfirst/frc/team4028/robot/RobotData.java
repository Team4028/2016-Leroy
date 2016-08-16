package org.usfirst.frc.team4028.robot;

import java.util.Date;

import org.usfirst.frc.team4028.robot.Constants.LogitechF310;
import org.usfirst.frc.team4028.robot.Constants.RobotMap;
import org.usfirst.frc.team4028.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
* This class contains the dynamic data while the robot is running in Auton or Teleop
* 	( and follows the DTO [Data Transfer Object] pattern )
* 
* It is used for:
* 	- collection of working data values used by Telop & Auton to control the robot
* 	- data logging to a text file (if enabled)
* 	- sending data to the ShartDashboard
* 
* It contains three (3) inner classes
* 	InputData		all real time data read from the robot (roboRio) 
* 	WorkingData		values calculated from the input data
* 	OutputData		values determined by code that will be pushed back to the robioRio to control motors & solenoids etc.
*/
public class RobotData 
{
	// ==========================================================
	// define the different auton modes (selected on the Smart Dashboard)
	// ==========================================================
	public enum AutonMode
	{
		UNDEFINED,
		DO_NOTHING,
		ZERO_ALL_AXIS,
		DRIVE_FWD,
		CROSS_DEFENSE,
	}
	
	public enum Auton_Puma_Back_Position
	{
		UNDEFINED,
		PUMA_BACK_DOWN,
		PUMA_BACK_UP
	}
	
	public enum Auton_Drive_Time_In_Secs
	{
		UNDEFINED,
		SECS_1,
		SECS_2,
		SECS_3,
		SECS_4,
		SECS_5,
		SECS_6,
		SECS_7,
		SECS_8,
		SECS_9
	}
	
	public enum Auton_Drive_Throttle_Percent
	{
		UNDEFINED,
		PERCENT_10,
		PERCENT_20,
		PERCENT_30,
		PERCENT_40,
		PERCENT_50,
		PERCENT_60,
		PERCENT_70,
		PERCENT_80,
		PERCENT_90
	}
	
	public enum Auton_Cross_Defense_Type
	{
		UNDEFINED,
		MOAT,
		RAMPARTS,
		ROCKWALL,
		ROUGH_TERRAIN
	}
	
	public enum Auton_Cross_Defense_Position
	{
		UNDEFINED,
		ZERO,
		TWO,
		THREE,
		FOUR,
		FIVE
	}
	
	public enum Camera_Selected
	{
		UNDEFINED,
		SHOOTER_CAMERA,
		INFEED_CAMERA
	}
	
	public enum Infeed_Tilt_Zero_State
	{
		UNDEFINED,
		TILT_TO_HOME,
		ON_HOME,
		GO_TO_REQUESTED_POSITION,
		TIMEOUT
	}
	
	// class constructor
	public RobotData()
	{
		this.InputDataValues = new InputData();
		this.WorkingDataValues = new WorkingData();
		this.OutputDataValues = new OutputData();
	}
	
	// properties
	public InputData InputDataValues;
	public WorkingData WorkingDataValues;
	public OutputData OutputDataValues;
	
	// build a TSV (Tab Separated Value) string for the header
	public String BuildTSVHeader()
	{
		StringBuilder sb = new StringBuilder();
		
		sb.append(InputDataValues.BuildTSVHeader() + "\t");
		sb.append(WorkingDataValues.BuildTSVHeader() + "\t");	
		sb.append(OutputDataValues.BuildTSVHeader() + "\n");
		
		return sb.toString();
	}

	// build a TSV for the data values
	public String BuildTSVData()
	{
		StringBuilder sb = new StringBuilder();
		
		sb.append(InputDataValues.BuildTSVData() + "\t");
		sb.append(WorkingDataValues.BuildTSVData() + "\t");	
		sb.append(OutputDataValues.BuildTSVData() + "\n");
		
		return sb.toString();
	}
	
	// internal class representing all of the Input data (sensors, driver station) used to control the robot
	public class InputData
	{
		public long FPGATimeMicroSecs;

		public boolean IsScaleDriveSpeedUpBtnPressed;			// logic will latch output value (so this acts like a single shot)
		public boolean IsScaleDriveSpeedDownBtnPressed;			// logic will latch output value (so this acts like a single shot)
		public boolean IsPumaFrontToggleBtnPressed;
		public boolean IsPumaBackToggleBtnPressed;
		public boolean IsPumaBothToggleBtnPressed;
		public boolean IsShifterBtnPressed;
		public boolean IsInfeedAcquireBtnPressed;
		public boolean IsInfeedReleaseBtnPressed;
		public boolean IsCameraSwitchBtnPressed;
		public boolean IsCupidLoadBtnPressed;
		public boolean IsCupidShootBtnPressed;
		public boolean IsCupidCameraBtnPressed;
		public boolean IsInfeedTiltStoreBtnPressed;
		public boolean IsInfeedTiltDeployBtnPressed;
		public boolean IsInfeedTiltFixedBtnPressed;
		public boolean IsInfeedTiltAxisOnUpLimitSwitch;
		public double ArcadeDriveThrottleRawCmd;
		public double ArcadeDriveTurnRawCmd;    	
		public double InfeedTiltEncoderCurrentCount;
		public double InfeedTiltCurrentOutputVoltage;
		public double InfeedRawTiltCmd;
		public double InfeedTiltUpCmd;
		public double InfeedTiltDownCmd;
		public double WinchRawCmd;
		public boolean IsValidData;
		public double DistanceToTarget;
		public double EffectiveTargetWidth;
		public double DesiredSliderPosition;
		public double DesiredTurretTurnInDegrees;
		public boolean IsValidShot;
		
		public Date LastVisionDataRecievedDT;
		
		AutonMode AutonModeRequested;
		Auton_Puma_Back_Position AutonPumaBackPositionRequested;
		Auton_Drive_Time_In_Secs AutonDriveTimeInSecsRequested;
		Auton_Drive_Throttle_Percent AutonDriveThrottlePercentRequested;
		Auton_Cross_Defense_Type AutonCrossDefenseTypeRequested;
		Auton_Cross_Defense_Position AutonCrossDefensePosition;
		
		Camera_Selected CameraSelected;
		
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("InputData:FPGATimeMicroSecs" + "\t");
			sb.append("InputData:IsScaleDriveSpeedUpBtnPressed" + "\t");
			sb.append("InputData:IsScaleDriveSpeedDownBtnPressed" + "\t");
			sb.append("InputData:IsPumaFrontToggleBtnPressed" + "\t");
			sb.append("InputData:IsPumaBackToggleBtnPressed" + "\t");
			sb.append("InputData:IsPumaBothToggleBtnPressed" + "\t");
			sb.append("InputData:IsInfeedAcquireBtnPressed" + "\t");
			sb.append("InputData:IsInfeedReleaseBtnPressed" + "\t");
			sb.append("InputData:IsShifterBtnPressed" + "\t");
			sb.append("InputData:IsCameraSwitchBtnPressed" + "\t");
			sb.append("InputData:IsCupidLoadBtnPressed" + "\t");
			sb.append("InputData:IsCupidShootBtnPressed" + "\t");
			sb.append("InputData:IsCupidCameraBtnPressed" + "\t");
			sb.append("InputData:IsInfeedTiltStoreBtnPressed" + "\t");
			sb.append("InputData:IsInfeedTiltDeployBtnPressed" + "\t");
			sb.append("InputData:IsInfeedTiltFixedBtnPressed" + "\t");
			sb.append("InputData:IsInfeedTiltAxisOnUpLimitSwitch" + "/t");
			sb.append("InputData:ArcadeDriveThrottleRawCmd" + "\t");
			sb.append("InputData:ArcadeDriveTurnRawCmd" + "\t");
			sb.append("InputData:InfeedTiltEncoderCurrentCount" + "\t");
			sb.append("InputData:InfeedTiltCurrentOutputVoltage" + "\t");
			sb.append("InputData:InfeedRawTiltCmd" + "\t");
			sb.append("InputData:InfeedTiltUpCmd" + "\t");
			sb.append("InputData:InfeedTiltDownCmd" + "\t");
			sb.append("InputData:WinchRawCmd" + "\t");
			sb.append("InputData:IsValidData" + "\t");
			sb.append("InputData:DistanceToTarget" + "\t");
			sb.append("InputData:EffectiveTargetWidth" + "\t");
			sb.append("InputData:DesiredSliderPosition" + "\t");
			sb.append("InputData:DesiredTurretTurnInDegrees" + "\t");
			sb.append("InputData:IsValidShot" + "\t");
			sb.append("InputData:LastVisionDataRecievedDT" + "\t");
			sb.append("InputData:AutonModeRequested" + "\t");
			sb.append("InputData:AutonPumaBackPositionRequested" + "\t");
			sb.append("InputData:AutonDriveTimeInSecsRequested" + "\t");
			sb.append("InputData:AutonDriveThrottlePercentRequested" + "\t");
			sb.append("InputData:AutonCrossDefenseTypeRequested" + "\t");
			sb.append("InputData:AutonCrossDefensePosition" + "\t");
			sb.append("InputData:CameraSelected" + "\t");
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(FPGATimeMicroSecs + "\t");
			sb.append(IsScaleDriveSpeedUpBtnPressed + "\t");
			sb.append(IsScaleDriveSpeedDownBtnPressed + "\t");
			sb.append(IsPumaFrontToggleBtnPressed + "\t");
			sb.append(IsPumaBackToggleBtnPressed + "\t");
			sb.append(IsPumaBothToggleBtnPressed + "\t");
			sb.append(IsInfeedAcquireBtnPressed + "\t");
			sb.append(IsInfeedReleaseBtnPressed + "\t");
			sb.append(IsShifterBtnPressed + "\t");
			sb.append(IsCameraSwitchBtnPressed + "\t"); 
			sb.append(IsCupidLoadBtnPressed + "\t");
			sb.append(IsCupidShootBtnPressed + "\t");
			sb.append(IsCupidCameraBtnPressed + "\t");
			sb.append(IsInfeedTiltStoreBtnPressed + "\t");
			sb.append(IsInfeedTiltDeployBtnPressed + "\t");
			sb.append(IsInfeedTiltFixedBtnPressed + "\t");
			sb.append(IsInfeedTiltAxisOnUpLimitSwitch + "\t");
			sb.append(ArcadeDriveThrottleRawCmd + "\t");
			sb.append(ArcadeDriveTurnRawCmd + "\t");
			sb.append(InfeedTiltEncoderCurrentCount + "\t");
			sb.append(InfeedTiltCurrentOutputVoltage + "\t");
			sb.append(InfeedRawTiltCmd + "\t");
			sb.append(InfeedTiltUpCmd + "\t");
			sb.append(InfeedTiltDownCmd + "\t");
			sb.append(WinchRawCmd + "\t");
			sb.append(IsValidData + "\t");
			sb.append(DistanceToTarget + "\t");
			sb.append(EffectiveTargetWidth + "\t");
			sb.append(DesiredSliderPosition + "\t");
			sb.append(DesiredTurretTurnInDegrees + "\t");
			sb.append(IsValidShot + "\t");
			sb.append(LastVisionDataRecievedDT + "\t");
			sb.append(AutonModeRequested + "\t");
			sb.append(AutonPumaBackPositionRequested + "\t");
			sb.append(AutonDriveTimeInSecsRequested + "\t");
			sb.append(AutonDriveThrottlePercentRequested + "\t");
			sb.append(AutonCrossDefenseTypeRequested + "\t");
			sb.append(AutonCrossDefensePosition + "\t");
			sb.append(CameraSelected + "\t");

			return sb.toString();
		}
	}
	
	// internal class representing all of the working data
	public class WorkingData
	{
		public boolean IsLoggingEnabled;
		public String LogFilePathName;
		public Date LoggingStartedDT;
		public Date LastScanDT;
		public boolean IsDriveSpeedScalingButtonPressedLastScan;
		public double DriveSpeedScalingFactor;			// min = 0.0, max = 1.0, 1.0 = 100%, 
		public boolean IsPumaFrontToggleBtnPressedLastScan;
		public boolean IsPumaBackToggleBtnPressedLastScan;
		public boolean IsPumaBothToggleBtnPressedLastScan;
		public boolean IsPrintDataBtnPressedLastScan;
		public boolean IsInfeedTiltStoreBtnPressedLastScan;
		public boolean IsInfeedTiltFixedBtnPressedLastScan;
		public boolean IsInfeedAcquireBtnPressedLastScan;
		public boolean IsShifterBtnPressedLastScan;
		public boolean IsCameraSwitchBtnPressedLastScan;
		public boolean IsCupidSwitchBtnPressedLastScan;
    	public double InfeedTiltEncoderInitialCount;
    	public double InfeedTiltEncoderTotalDeltaCount;
    	public double InfeedTiltEncoderDegreesCount;
    	public double InfeedTiltTargetDegreesCount;
    	public double InfeedTiltTurnDegreesCmd;    	
    	public long AutonShooterStartTime;
    	public long AutonDriveFwdStartTime;
    	public long InfeedPauseOnBallInPositionSwitchStartTime;
    	
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("WorkingData:IsDriveSpeedScalingButtonPressedLastScan" + "\t");
			sb.append("WorkingData:DriveSpeedScalingFactor" + "\t");
			sb.append("WorkingData:IsPumaFrontToggleBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsPumaBackToggleBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsPumaBothToggleBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsPrintDataBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsInfeedTiltStoreBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsInfeedTiltFixedBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsInfeedAcquireBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsShifterBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsCameraSwitchBtnPressedLastScan" + "\t");
			sb.append("WorkingData:IsCupidSwitchBtnPressedLastScan" + "\t");
			sb.append("WorkingData:InfeedTiltEncoderInitialCount" + "\t");
			sb.append("WorkingData:InfeedTiltEncoderTotalDeltaCount" + "\t");
			sb.append("WorkingData:InfeedTiltEncoderDegreesCount" + "\t");
			sb.append("WorkingData:InfeedTiltTargetDegreesCount" + "\t");
			sb.append("WorkingData:InfeedTiltTurnDegreesCmd" + "\t");
			sb.append("WorkingData:AutonShooterStartTime" + "\t");
			sb.append("WorkingData:AutonDriveFwdStartTime" + "\t");
			sb.append("WorkingData:InfeedPauseOnBallInPositionSwitchStartTime");
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(IsDriveSpeedScalingButtonPressedLastScan + "\t");
			sb.append(DriveSpeedScalingFactor + "\t");
			sb.append(IsPumaFrontToggleBtnPressedLastScan + "\t");
			sb.append(IsPumaBackToggleBtnPressedLastScan + "\t");
			sb.append(IsPumaBothToggleBtnPressedLastScan + "\t");
			sb.append(IsPrintDataBtnPressedLastScan + "\t");
			sb.append(IsInfeedTiltStoreBtnPressedLastScan + "\t");
			sb.append(IsInfeedTiltFixedBtnPressedLastScan + "\t");
			sb.append(IsInfeedAcquireBtnPressedLastScan + "\t");
			sb.append(IsShifterBtnPressedLastScan + "\t");
			sb.append(IsCameraSwitchBtnPressedLastScan + "\t");
			sb.append(IsCupidSwitchBtnPressedLastScan + "\t");
			sb.append(InfeedTiltEncoderInitialCount + "\t");
			sb.append(InfeedTiltEncoderTotalDeltaCount + "\t");
			sb.append(InfeedTiltEncoderDegreesCount + "\t");
			sb.append(InfeedTiltTargetDegreesCount + "\t");
			sb.append(InfeedTiltTurnDegreesCmd + "\t");
			sb.append(AutonShooterStartTime + "\t");
			sb.append(AutonDriveFwdStartTime + "\t");
			sb.append(InfeedPauseOnBallInPositionSwitchStartTime);
					
			return sb.toString();
		}
	}
	
	// internal class representing all of the Motor Output Data used to control the robot
	public class OutputData
	{
		public double ArcadeDriveThrottleAdjCmd;
		public double ArcadeDriveTurnAdjCmd;
		public double InfeedAcqMtrVelocityCmd;
		public double InfeedTiltMtrVelocityCmd;
		public double InfeedTiltTargetPositionInRotationsCmd;
		public double WinchVelocityCmd;
		public double CupidServoPositionCmd;
		public Value PumaFrontSolenoidPosition;
		public Value PumaBackSolenoidPosition;
		public Value ShifterSolenoidPosition;
		public String DriversStationMsg;
		
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("OutputData:ArcadeDriveAdjThrottleCmd" + "\t");
			sb.append("OutputData:ArcadeDriveAdjTurnCmd" + "\t");
			sb.append("OutputData:InfeedAcqMtrVelocityCmd" + "\t");
			sb.append("OutputData:InfeedTiltMtrVelocityCmd" + "\t");
			sb.append("OutputData:InfeedTiltTargetPositionCmd" + "\t");
			sb.append("OutputData:WinchVelocityCmd" + "\t");
			sb.append("OutputData:CupidServoPositionCmd" + "\t");
			sb.append("OutputData:PumaFrontSolenloidPosition" + "\t");
			sb.append("OutputData:PumaBackSolenoidPosition" + "\t");
			sb.append("OutputData:ShifterSolenoidPosition" + "\t");
			sb.append("OutputData:DriversStationMsg");
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(ArcadeDriveThrottleAdjCmd + "\t");
			sb.append(ArcadeDriveTurnAdjCmd + "\t");
			sb.append(InfeedAcqMtrVelocityCmd + "\t");
			sb.append(InfeedTiltMtrVelocityCmd + "\t");
			sb.append(InfeedTiltTargetPositionInRotationsCmd + "\t");
			sb.append(WinchVelocityCmd + "\t");
			sb.append(CupidServoPositionCmd + "\t");
			
			String PumaFrontSolenoidPositionDesc = "";
			if (PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_UP_POSITION)
			{
				PumaFrontSolenoidPositionDesc = "PUMA_FRONT_SOLENOID_OPEN";
			}
			else if (PumaFrontSolenoidPosition == RobotMap.PUMA_FRONT_SOLENOID_DOWN_POSITION)
			{
				PumaFrontSolenoidPositionDesc = "PUMA_FRONT_SOLENOID_CLOSED";
			}
			else
			{
				PumaFrontSolenoidPositionDesc = "UNKNOWN";
			}
			
			String PumaBackSolenoidPositionDesc = "";
			if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_UP_POSITION)
			{
				PumaBackSolenoidPositionDesc = "PUMA_BACK_SOLENOID_OPEN";
			}
			else if (PumaBackSolenoidPosition == RobotMap.PUMA_BACK_SOLENOID_DOWN_POSITION)
			{
				PumaBackSolenoidPositionDesc = "PUMA_BACK_SOLENOID_CLOSED";
			}
			else
			{
				PumaBackSolenoidPositionDesc = "UNKNOWN";
			}
			
			String ShifterSolenoidPositionDesc ="";
			if (PumaBackSolenoidPosition == RobotMap.SHIFTER_HIGH_GEAR_POSITION)
			{
				ShifterSolenoidPositionDesc = "SHIFTER_SOLENOID_OPEN";
			}
			else if (PumaBackSolenoidPosition == RobotMap.SHIFTER_LOW_GEAR_POSITION)
			{
				ShifterSolenoidPositionDesc = "SHIFTER_SOLENOID_CLOSED";
			}
			else
			{
				ShifterSolenoidPositionDesc = "UNKNOWN";
			}
			
			sb.append(PumaFrontSolenoidPositionDesc + "\t");
			sb.append(PumaBackSolenoidPositionDesc + "\t");
			sb.append(ShifterSolenoidPositionDesc + "\t");
			sb.append(DriversStationMsg);
					
			return sb.toString();
		}
	}
}