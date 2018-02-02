/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5057.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	//ports
	final int leftDrivePort=0;
	final int rightDrivePort=1;
	
	//driveTrain
	DifferentialDrive chassis;
	Spark leftMotor = new Spark(leftDrivePort);
	Spark rightMotor = new Spark(rightDrivePort);

	//Joystick
	JoystickLocations porting = new JoystickLocations();
	Joystick xbox = new Joystick(porting.joystickPort);
	
	//Gyro
	private static final double kVoltsPerDegreePerSecond = 0.0128;
	private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	//Accelerometer. Ignore for now
	//Accelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
	
	//Sonar Distance Sensor
	private static final int kUltrasonicPort = 0;
	private AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
	private static final double kValueToInches = 0.049;
	
	
	//values When driving
	double dampen;
	double[] accVal = new double[3];
	double currentDistance;
	
	//joystick axes
	double leftY;
	double leftX;
	double rightY;
	double rightX;
	
	//vision init. This is code we will use for the vision programming. Ignore for now
	/*VisionThread visionT;
	UsbCamera cam;
	double centerX=0.0;
	double centerX1;
	double centerX2;
	final Object imgLock = new Object();*/
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//choose which autonomous to use
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		//setup drivetrain
		chassis = new DifferentialDrive(leftMotor, rightMotor);
		chassis.setExpiration(.1);
		chassis.setSafetyEnabled(false);
		
		//setup gyro
		gyro.calibrate();
		gyro.reset();
		
		/*This is code we will use for the vision programming. Ignore for now
		//vision code init
		cam= CameraServer.getInstance().startAutomaticCapture(0);//create new camera instance
		cam.setResolution(320,240);//control pixel size
		visionT=new VisionThread(cam, new GripPipeline(), pipeline -> {
			if (!pipeline.filterContoursOutput().isEmpty()) {//check to see if image is there. If image:
				Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));//left contour
				Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));//right contour
				synchronized(imgLock) {
            		centerX1 = r1.x + (r1.width / 2);//find x of left
            		centerX2 = r2.x + (r2.width/2);//find x or right
            		centerX = (centerX1 + centerX2)/2;//average them together
            	}SmartDashboard.putNumber("centerX", centerX);//publish to dashboard
			}else {//if image doesnt exist
				synchronized (imgLock) {
	    		centerX = -1;//center isn't available
	    		}
	    		SmartDashboard.putNumber("centerX", centerX);//publish
	    	}
			
			if(isAutonomous()){//if autonomous, allow camera to take lot of CPU
	        	Timer.delay(.01);
	        }
	        else{//if not auto, don't let camera run often
	            Timer.delay(0.1);
	        }
			
		});
		visionT.start();*/
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. 
	 * 
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		chassis.setSafetyEnabled(true);
		//This is default code we use to choose an autonomous
	}

	/**
	 * This function is called periodically during autonomous.
	 **/
	@Override
	public void autonomousPeriodic() { 
		switch (m_autoSelected) {
			case kCustomAuto://testing autonomous
				chassis.setSafetyEnabled(false);//allow auto to happen
				getHeading();
				getDistance();
				//getAccel();
				driveDistance();
				break;
			case kDefaultAuto://nothing right now
				break;
			default:
				// Put default auto code here
				break;
				
			
		}
	}
	//temporary autonomous to drive with sonar
	public void driveDistance() {
		if(currentDistance>18) {//if the robot is more than 18 inches from the wall
			SmartDashboard.putNumber("AutoDriveOn",0);//say robot  moving
			leftMotor.set(-.2);//left forward .2 speed
			rightMotor.set(.2);//right forward .2 speed
		}else {
			SmartDashboard.putNumber("AutoDriveOn", 1);//say robot not moving
			leftMotor.set(0);//left stop
			rightMotor.set(0);//right stop
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		chassis.setSafetyEnabled(true);
		updateAxes();
		//changeDrive();
		driveArcade();
		//chassis.arcadeDrive(leftX, .75*xbox.getRawAxis(porting.rXAxis));
		//driveTank();
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {}
	
	//makes sure all values of joystick are up to date
	public void updateAxes() {
		leftY=-xbox.getRawAxis(porting.lYAxis);
		leftX=xbox.getRawAxis(porting.lXAxis);
		rightX=xbox.getRawAxis(porting.rXAxis);
		rightY=xbox.getRawAxis(porting.rYAxis);
		dampen=1-3/4*xbox.getRawAxis(porting.rTrigger);
		SmartDashboard.putNumber("leftY", leftY);
		SmartDashboard.putNumber("leftX", leftX);
		SmartDashboard.putNumber("rightY", rightY);
		SmartDashboard.putNumber("rightX", rightX);
		SmartDashboard.putNumber("dampen", dampen);
	}
	
	//This changes which kind of drive system to use. Ignore for now
	public void changeDrive() {
		int mode=1;
		if(xbox.getRawButton(porting.butA))mode=1;
		else if(xbox.getRawButton(porting.butB))mode=2;
		else if(xbox.getRawButton(porting.butX))mode=3;
		
		switch (mode) {
		case 1://button A does field oriented arcade
			driveArcade();
			break;
		case 2://button B does just arcade
			chassis.arcadeDrive(-leftY, leftX);
			break;
		case 3://button X does tank drive
			driveTank();
			getHeading();
			getDistance();
			getAccel();
			break;
		default: break;
		}
	}
	
	//drive with arcade drive with respect to the field. Left joystick only
	public void driveArcade() {		
		double radians = getHeading()*Math.PI/180;//convert angle to radian value
		
		getHeading();//update gyro
		getDistance();//update sonar
		getAccel();//update accelerometer
		
		//math to find how to turn robot to make field oriented work
		double temp = Math.cos(radians)*leftX + Math.sin(radians)*leftY;
		leftY = -leftX*Math.sin(radians) + leftY*Math.cos(radians);
		leftX=temp;
						
		chassis.arcadeDrive(this.dampen*leftX, this.dampen*leftY);//drive
		
		if(Math.abs(xbox.getRawAxis(porting.rXAxis)) > .25) {
			chassis.arcadeDrive(0, xbox.getRawAxis(porting.rXAxis));
		}
		
	}
	
	//drive with tank drive. left and right joysticks
	public void driveTank() {		
		getHeading();//update gyr0
		getDistance();//update sonar
		getAccel();//update accelrometer
		
		//drive
		chassis.tankDrive(leftY*dampen, rightY*dampen);
	}
	
	//updates the value of the sonar sensor
	public void getDistance() {
		this.currentDistance = m_ultrasonic.getValue()*this.kValueToInches;
		SmartDashboard.putNumber("distance", currentDistance);
	}
	
	//updates the value of the gyro
	public double getHeading() {
		SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
		return gyro.getAngle();
	}
	
	//updates the value of the accelerometer Ignore for now
	public double[] getAccel() {
		//accVal[0] = accel.getX()*9.81;
	//	accVal[1] = accel.getY()*9.81;
		//accVal[2] = accel.getZ()*9.81;
		//SmartDashboard.putNumberArray("Acceleration m/s^2 x,y,z", accVal);
		return accVal;
	}
}
