package org.usfirst.frc.team4028.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;
import java.util.concurrent.TimeUnit;

/**
 * This class contains implements logic to log data to a text file
 * 
 * Date			Rev		Author						Comments
 * -----------	------	-------------------------	---------------------------------- 
 * 26.Jul.2015	0.1		Tom Bruns					initial version
 */
public class DataLogger 
{
    private PrintWriter _writer;

    // constructor, open a new timestamped log file in the target directory
    public DataLogger(String parentFolder, String fileSuffix) throws IOException 
    {
    	SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
		outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
		String newDateString = outputFormatter.format(new Date());
    	
    	// build the new filename
    	String fileName = newDateString + "_" + fileSuffix + ".tsv";
    	// build the full file path name
    	this.LogFilePathName = parentFolder + File.separator + fileName;
    	
        _writer = new PrintWriter(new BufferedWriter(new FileWriter(this.LogFilePathName, true)));
    }

    // Property containing Full PathName to log file
    public String LogFilePathName; 
    
    // Write a string to the file
    public void WriteData(String textToLog) 
    {        
        _writer.print("StartDetaMS" + "\t" + "LastScanDeltaMS" + "\t" +textToLog);
        _writer.flush();
    }

    // Write a structured data object to the log file
    public void WriteData(RobotData dataToLog) 
    {
    	TimeUnit timeUnit = TimeUnit.MILLISECONDS;
       	Date now = new Date();
       	
    	long startDeltaDiffInMillies = now.getTime() - dataToLog.WorkingDataValues.LoggingStartedDT.getTime();
        long startDeltaTimestamp = timeUnit.convert(startDeltaDiffInMillies,TimeUnit.MILLISECONDS);
        
    	long lastScanDeltaDiffInMillies = now.getTime() - dataToLog.WorkingDataValues.LastScanDT.getTime();
        long lastScanDeltaTimestamp = timeUnit.convert(lastScanDeltaDiffInMillies,TimeUnit.MILLISECONDS);
        
        _writer.print(startDeltaTimestamp + "\t" + lastScanDeltaTimestamp + "\t" + dataToLog.BuildTSVData());
        _writer.flush();
    }
    
    // close the file
    public void close() 
    {
    	_writer.close();
    }
}