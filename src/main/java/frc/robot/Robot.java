/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Digital Input Imports
import edu.wpi.first.wpilibj.DigitalInput;

//Analog Potentiometer Imports
import edu.wpi.first.wpilibj.AnalogPotentiometer;

//Encoder Imports
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

//Accelerometer Imports
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import java.lang.Math;

//Gryo Imports
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

//Ultrasonic Imports
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Digital IO - Mapped to RoboRIO
  private int switchPort = 0;
  private int encoderChannelA = 1;
  private int encoderChannelB = 2;  
  private int absoluteEncoderChannel = 3;
  private int vexUltrasonicInput = 4;
  private int vexUltrasonicOutput = 5;

  //Analog - Mapped to RoboRIO
  private int potentiometerPort = 0;
  private int ultraPort = 1;
  
  //Initialize Limit Switch
  private DigitalInput armSwitch = new DigitalInput(switchPort);

  private DutyCycleEncoder absEncoder = new DutyCycleEncoder(absoluteEncoderChannel);
 
  //Initialize Potentiometer
  double fullRange = 360; //Convert into degrees, full range of Pot = 360 degrees   
  double offset = 15;    //Starting position at 0 V
  private AnalogPotentiometer armPot = new AnalogPotentiometer(potentiometerPort, fullRange, offset);
  
  //Initialize Encoder 
  boolean reverse = false; //Set which direction is positive
  private Encoder eArm = new Encoder(encoderChannelA, encoderChannelB, reverse, Encoder.EncodingType.k4X);

  //Initialize Accel 
  private BuiltInAccelerometer accel = new BuiltInAccelerometer(Accelerometer.Range.k8G); 
  
  //Initialize gyro 
  private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro(kGyroPort);
  
  //Initialize Vex Ultrasonic
  private Ultrasonic vex_sonic = new Ultrasonic(vexUltrasonicInput, vexUltrasonicOutput);

  //Initialize MaxBotix Ultrasonic, plugged into analog port 0
  private AnalogInput ultrasonic = new AnalogInput(ultraPort);

  //Initialize Joystick
  private Joystick operator = new Joystick(0);
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

     //Encoder setup - set measurement to degrees – 360 degrees per 1024 encoder ticks
     eArm.setDistancePerPulse(360.0/1024.0);

     //Reset Gyro
     gyro.reset(); 

     //Ultrasonic - Turn automatic ping mode
      vex_sonic.setAutomaticMode(true); 

      //Absolute Encoder set distance pre pulse
      absEncoder.setDistancePerRotation(360);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      /******************************/
      /* Limit Switch Example Code  */
      /******************************/
      boolean atLimit = armSwitch.get();
      SmartDashboard.putBoolean("Limit Switch", atLimit);
      
      /******************************/
      /* Potentiometer Example Code */
      /******************************/
      double arm_position = armPot.get();
      SmartDashboard.putNumber("Potentiometer", arm_position);

      /******************************/
      /* Encoder Example Code */
      /******************************/
      //Read sensor value
      //Get current encoder count scaled by Encoding Type (k4X)
      int eCounts = eArm.get();

      //Get real world distance, using value set in setDistancePerPulse
      double eDistance = eArm.getDistance();

      //Get the distance per second. Scaled by the value from setDistancePerPulse 
      double eRate = eArm.getRate();

      //Press B on Joystick 0 to reset encoder distance
      if(operator.getRawButton(2))
         eArm.reset();

      //Display these values on smart dashboard
      SmartDashboard.putNumber("ArmEncoder Count", eCounts);
      SmartDashboard.putNumber("ArmEncoder Distance", eDistance);
      SmartDashboard.putNumber("ArmEncoder Rate", eRate);

      /******************************/
      /* Accelerometer Example Code */
      /******************************/
      double y = accel.getY();
      double x = accel.getX();
      double z = accel.getZ();

      double rot_x_rad = Math.atan2(z, y); // Rotation about x-axis in radians
      double rot_y_rad = Math.atan2(z, x); // Rotation about y-axis in radians
      double rot_x_deg = Math.toDegrees(rot_x_rad); // Rotation x-axis degrees
      double rot_y_deg = Math.toDegrees(rot_y_rad); // Rotation y-axis degrees
    
      SmartDashboard.putNumber("Accel Y", y);
      SmartDashboard.putNumber("Accel X", x);  
      SmartDashboard.putNumber("Accel Z", z);
      SmartDashboard.putNumber("Rotation X", rot_x_deg);  
      SmartDashboard.putNumber("Rotation Y", rot_y_deg);
      
      /*********************/
      /* Gyro Example Code */
      /*********************/
      //Angle measurements are relative to angle when reset was last called
      double gyro_angle = gyro.getAngle(); 
      double gyro_rate = gyro.getRate(); 

      //Press A on Joystick 0 to reset angle
      if(operator.getRawButton(1))
          gyro.reset();

      SmartDashboard.putNumber("Gryo Angle", gyro_angle);  
      SmartDashboard.putNumber("Gryo Rate", gyro_rate);

      /************************************/
      /* MaxBotix Ultrasonic Example Code */
      /************************************/
      int ultrasonic_value = ultrasonic.getValue();
      double ultrasonic_voltage = ultrasonic.getVoltage();

      SmartDashboard.putNumber("Ultra Value", ultrasonic_value);  
      SmartDashboard.putNumber("Ultra Voltage", ultrasonic_voltage);

      /************************************/
      /* Vex Ultrasonic Example Code */
      /************************************/
      double vex_range = vex_sonic.getRangeInches(); 
      SmartDashboard.putNumber("Vex Ultra Inches", vex_range);

      /******************************** */
      double encoder_distance =  absEncoder.getDistance();
      SmartDashboard.putNumber("Absolute Encoder", encoder_distance);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
