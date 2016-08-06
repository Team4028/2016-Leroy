package org.usfirst.frc.team4028.robot;

public class Utilities {

    /**
     * **************************************************************************************************
     * This function calculates the actual distance moved based on encoder counts
     * 
     ***************************************************************************************************
     **/
	public static double CalcDistanceMoved(double travelDistanceInchesPerCount, double fromEncoderCount, double toEndcoderCount)
	{
		// calc how far the axis actually moved in encoder counts
		double deltaEncoderCount = toEndcoderCount - fromEncoderCount;
		
		// calc how far the axis actually moved in inches
		double travelDistanceInches = deltaEncoderCount * travelDistanceInchesPerCount;
		
		// return the result
		return travelDistanceInches;
	}
}