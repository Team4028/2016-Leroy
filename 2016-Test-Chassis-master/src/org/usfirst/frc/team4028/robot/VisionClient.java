package org.usfirst.frc.team4028.robot;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.channels.AsynchronousSocketChannel;
import java.util.ArrayDeque;
import java.util.Date;
import java.util.Deque;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.vision.USBCamera;

import org.usfirst.frc.team4028.robot.Constants.RobotMap;

public class VisionClient 
{
	// define some local working variables
	private static VisionClient _visionClient;
	private Thread pollingThread;
	private VisionData _visionData;
	private Socket _visionServer;	
	private DataInputStream _inFromServer;
	private DataOutputStream _outToServer;
	private boolean _isVisionServerPollingStarted;
	private boolean _isVisionServerPollingStopRequested = false;
	  
	private final ReentrantReadWriteLock _readWriteLock = new ReentrantReadWriteLock();
	private final Lock _readLock = _readWriteLock.readLock();
	private final Lock _writeLock = _readWriteLock.writeLock();
	  
	// private constructor used in a singleton pattern
	private VisionClient() 
	{
		IsVisionServerAvailable = false;
		_visionData = null;

		// open the connection to the remote vision server
		try
		{
			_visionServer = new Socket(RobotMap.VISION_PC_IP_ADDRESS, RobotMap.VISION_PC_PORT);
			  
			if(_visionServer != null)
			{
				DriverStation.reportError("Connection to Vision server successful | ", false);
				IsVisionServerAvailable = true; 
			}
			else
			{
				DriverStation.reportError("Connection to Vision server failed | ", false);
			}
		 } 
		catch (UnknownHostException e) 
		{
			DriverStation.reportError("Connection to Vision server has failed! | ", false);
			e.printStackTrace();
		}
		catch (IOException e)
		{
			DriverStation.reportError("Connection to Vision server has failed!! | ", false);
			e.printStackTrace();
		}
		catch (Exception e)
		{
			DriverStation.reportError("Connection to Vision server has failed!!! | ", false);
			e.printStackTrace();
		}
	}
	
	// implement singleton
	public static VisionClient getInstance() 
	{
		if (_visionClient == null) 
		{
			_visionClient = new VisionClient();
		}
    
		return _visionClient;
	}
	  
	// Public Property
	public static boolean IsVisionServerAvailable;

	// Public Thread Safe Property Getter
	public synchronized VisionData GetVisionData() 
	{
		_readLock.lock();
		try
		{
			return _visionData;
		}
		finally
		{
			_readLock.unlock();
		}  
	}
	  
	// Private Thread Safe Property Setter
	private synchronized void SetVisionData(VisionData visionData) 
	{
		_writeLock.lock();
		try
		{
			_visionData = visionData;
		}
		finally
		{
			_writeLock.unlock();
		}
	}
	  
	  // this method starts a thread continuously polling the vision server
	  public synchronized void startPolling() 
	  {
		  // make sure we are not already polling.
		  if (_isVisionServerPollingStarted)
		  {
			  return;
		  }
		  
		 // make sure the remote server is available
		  if (!IsVisionServerAvailable)
		  {
			  DriverStation.reportError("Vision server is NOT available | ", false);
			  return;
		  }
		  		  
		  // reset this variable that help us exit the polling loop
		  _isVisionServerPollingStopRequested = false;
		  
		  // open the connection to the remote vision server
		  try
		  {
			  // create a new background thread to read data from a remote socket
			  pollingThread = new Thread(new Runnable() 
									  {    
										  public void run() 
										  {
											  try 
											  {
												  pollVisionServer();
												  //pollVisionServerAsync();
											  } 
											  catch (IOException e) 
											  {
												  // do stuff here
											  } 
											  catch (InterruptedException e) 
											  {
												  DriverStation.reportError("Vision server polling thread interrupted | ", false);
											  }
									      }
									    });
				  
			  // start the polling thread
			  pollingThread.setName("Poll Remote Socket Thread");
			  pollingThread.start();
			  
			  // set this status flag
			  _isVisionServerPollingStarted = true;
			  
			  DriverStation.reportError("Vision Server Polling Thread started | ", false);
		  } 
		  catch (Exception e)
		  {
			  // TODO Auto-generated catch block
			  DriverStation.reportError("Vision Server Pollng Thread CANNOT BE started | ", false);
			  e.printStackTrace();
		  }
	  }
	
	  // this method stops the vision polling thread
	  public synchronized void stopPolling()
	  {
		  if (!_isVisionServerPollingStarted)
			  return;
		  
		  _isVisionServerPollingStopRequested = true;
		  		  
		  DriverStation.reportError("Vision Server Polling Thread stopped | ", false);
	  }
	  
	  // =====================================================================================================
	  // synchronous version
	  // this method is the heart of the functionality, it continuously polls the vision server
	  // =====================================================================================================
	  protected void pollVisionServer() throws IOException, InterruptedException 
	  {	    
		  String rawVisionData;
		  VisionData _visionLiveData;
		  long loopCounter = 0;
		  
			_outToServer = new DataOutputStream(_visionServer.getOutputStream());
			  
			if(_outToServer != null)
			{
				DriverStation.reportError("Creation of output stream to Vision Server successful | ", false); 
			}
			  
			_inFromServer = new DataInputStream( _visionServer.getInputStream());
			if(_inFromServer != null)
			{
				DriverStation.reportError("Creation of input stream from Vision Server successful | ", false); 
			}
		  
		  while (IsVisionServerAvailable  && !_isVisionServerPollingStopRequested) 
		  {
			  try 
			  {
				  if ((_visionServer != null) && (_outToServer != null) && (_inFromServer != null))
				  {
					  
					  loopCounter++;
					  
					  // ask the server for data
					  char cr = 0x013;
					  _outToServer.writeChar(cr);
						
					  // call returns a delimited string
					  rawVisionData = "";
						
					  // ==========================
					  // get values from Vision PC
					  // ==========================
					  BufferedReader in = new BufferedReader(new InputStreamReader(_visionServer.getInputStream()));
					  rawVisionData = in.readLine();
				
					  // parse the delimited string
					  String delims = "[|]+";
					  String[] splitRawVisionData = rawVisionData.split(delims);
		    			
					  // init object to hold vision data
					  _visionLiveData = new VisionData();
	    				
					  // see if got valid data, if so parse the delimited string into its individual data elements
					  if (splitRawVisionData.length == 7)
					  {		    				
						  _visionLiveData.IsValidData = Boolean.parseBoolean(splitRawVisionData[RobotMap.IS_VALID_DATA_ARRAY_POSITION]);
						  if (_visionLiveData.IsValidData)
						  {
							  _visionLiveData.DistanceToTarget = Double.parseDouble(splitRawVisionData[RobotMap.DISTANCE_TO_TARGET_ARRAY_POSITION]);
							  _visionLiveData.EffectiveTargetWidth = Double.parseDouble(splitRawVisionData[RobotMap.EFFECTIVE_TARGET_WIDTH_ARRAY_POSITION]);
							  _visionLiveData.DesiredSliderPosition = Double.parseDouble(splitRawVisionData[RobotMap.DESIRED_SLIDER_POSITION_ARRAY_POSITION]);
							  _visionLiveData.BatteryChargeLevel = Integer.parseInt(splitRawVisionData[RobotMap.BATTERY_CHARGE_LEVEL]);
							  _visionLiveData.DesiredTurretTurnInDegrees = Double.parseDouble(splitRawVisionData[RobotMap.DESIRED_TURRET_TURN_IN_DEGREES_ARRAY_POSITION]);
							  _visionLiveData.IsValidShot = Boolean.parseBoolean(splitRawVisionData[RobotMap.IS_VALID_SHOT_ARRAY_POSITION]);
			    				
							  _visionLiveData.LastVisionDataRecievedDT = new Date();
			    				
							  SetVisionData(_visionLiveData);
						  }
						  
						  if (loopCounter % 100 == 0)
						  {
							  DriverStation.reportError("Vision Data= " + rawVisionData + " | ", false);
						  }
					  }
					  else 
					  {
						  _visionLiveData.StatusMsg = "Did not receive correct vision data: " + rawVisionData;
						  if (loopCounter % 50 == 0)
						  {
							  DriverStation.reportError("Did not not get correct Vision Data= " + rawVisionData + " | ", false);
						  }
					  }
				  }
				  else
				  {
					  DriverStation.reportError("Error.. Unexpected Null object in Vision Polling | ", false);
				  }
			  }
			  catch (IOException ex) 
			  {
				  DriverStation.reportError(ex.getMessage(), true);
				  continue;
			  }
			  
			  //Thread.sleep(100);
		  	}
		
		  _isVisionServerPollingStarted = false;
	  }
	
	  // =====================================================================================================
	  // async version
	  // this method is the heart of the functionality, it continuously polls the vision server
	  // =====================================================================================================
	  private void pollVisionServerAsync() throws IOException, InterruptedException 
	  {
		  try 
		  {
			  AsynchronousSocketChannel asyncChannel = AsynchronousSocketChannel.open();
			  InetSocketAddress remoteIP = new InetSocketAddress(RobotMap.VISION_PC_IP_ADDRESS, RobotMap.VISION_PC_PORT);
				
			  // async connection, wait for 5 secs
			  Future<Void> future = asyncChannel.connect(remoteIP);
			  future.wait(5000);
			  DriverStation.reportError("Socket Opened Successfully | ", false);
				
			  // setup to write async to remote server is waiting for a <cr>
			  char carriageReturn = 0x013;
			  ByteBuffer writeBuffer = ByteBuffer.allocateDirect(2);
			  Future<Integer> writeFuture;
			  long bytesWritten;
			  
			  // setup to read async from remote server
			  ByteBuffer readBuffer = ByteBuffer.allocateDirect(1000);
			  Future<Integer> readFuture;
			  long bytesRead;
			  String delims = "[|]+";
			  String[] splitRawVisionData;
			  String rawVisionData;
			  VisionData visionLiveData;
			  long loopCounter = 0;
				    
			  while(true)
			  {
				  loopCounter++;
					  	
				  // async write, wait for 5 secs
				  writeBuffer.putChar(carriageReturn);
					
				  writeFuture = asyncChannel.write(writeBuffer);
				  bytesWritten = writeFuture.get(5, TimeUnit.SECONDS);
					
				  // async read, wait for 5 secs
				  readFuture = asyncChannel.read(readBuffer);
				  bytesRead = readFuture.get(5, TimeUnit.SECONDS);
				  rawVisionData = new String(readBuffer.array());
					
				  // split the delimited data
				  splitRawVisionData = rawVisionData.split(delims);
					
				  // make a new empty object
				  visionLiveData = new VisionData();
					
				  // see if got valid data, if so parse the delimited string into its individual data elements
				  if (splitRawVisionData.length == 6)
				  {		    				
					  visionLiveData.IsValidData = Boolean.parseBoolean(splitRawVisionData[RobotMap.IS_VALID_DATA_ARRAY_POSITION]);
					  visionLiveData.DistanceToTarget = Double.parseDouble(splitRawVisionData[RobotMap.DISTANCE_TO_TARGET_ARRAY_POSITION]);
					  visionLiveData.EffectiveTargetWidth = Double.parseDouble(splitRawVisionData[RobotMap.EFFECTIVE_TARGET_WIDTH_ARRAY_POSITION]);
					  visionLiveData.DesiredSliderPosition = Double.parseDouble(splitRawVisionData[RobotMap.DESIRED_SLIDER_POSITION_ARRAY_POSITION]);
					  visionLiveData.DesiredTurretTurnInDegrees = Double.parseDouble(splitRawVisionData[RobotMap.DESIRED_TURRET_TURN_IN_DEGREES_ARRAY_POSITION]);
					  visionLiveData.IsValidShot = Boolean.parseBoolean(splitRawVisionData[RobotMap.IS_VALID_SHOT_ARRAY_POSITION]);
		    				
					  visionLiveData.LastVisionDataRecievedDT = new Date();
		    				
					  SetVisionData(visionLiveData);
						  
					  if (loopCounter % 100 == 0)
					  {
						  DriverStation.reportError("Vision Data= " + rawVisionData + " | ", false);
					  }
				  }
				  else 
				  {
					  visionLiveData.StatusMsg = "Did not receive correct vision data: " + rawVisionData + " | ";
						
					  //if (loopCounter % 100 == 0)
					  //{
						  DriverStation.reportError("Did not not receive correct Vision Data= " + rawVisionData + " | ", false);
					  //}
				  }
			  }

		  } 
		  catch (IOException e) 
		  {
			  // TODO Auto-generated catch block
			  e.printStackTrace();
			  DriverStation.reportError("IOException" + " | ", true);
		  } 
		  catch (InterruptedException e) 
		  {
			  // TODO Auto-generated catch block
			  e.printStackTrace();
				DriverStation.reportError("InterruptedException" + " | ", true);
		  } 
		  catch (ExecutionException e) 
		  {
			  // TODO Auto-generated catch block
			  e.printStackTrace();
			  DriverStation.reportError("ExecutionException" + " | ", true);
		  } 
		  catch (TimeoutException e) 
		  {
			  // TODO Auto-generated catch block
			  e.printStackTrace();
			  DriverStation.reportError("TimeoutException" + " | ", true);
		  }
	  }
}	