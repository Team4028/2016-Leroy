package org.usfirst.frc.team4028.robot;

import java.util.Date;

public class VisionData 
{
	public boolean IsValidData;
	public double DistanceToTarget;
	public double EffectiveTargetWidth;
	public double DesiredSliderPosition;
	public double DesiredTurretTurnInDegrees;
	public int BatteryChargeLevel;
	public boolean IsValidShot;
	public Date LastVisionDataRecievedDT;
	public String StatusMsg;
}