package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;
//import edu.wpi.first.wpilibj.SensorBase;
//import edu.wpi.first.wpilibj.PWM;
//import edu.wpi.first.wpilibj.SafePWM;
//import edu.wpi.first.wpilibj.PWMSpeedController;
//import edu.wpi.first.wpilibj.PWMTalonSRX;
//import edu.wpi.first.wpilibj.TalonSRX;
//import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port; 
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.cscore.*;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.VideoSource;

public class Robot extends IterativeRobot {

  
  MecanumDrive robotDrive;
  WPI_TalonSRX frontLeft;
  WPI_TalonSRX rearLeft;
  WPI_TalonSRX frontRight;
  WPI_TalonSRX rearRight;
  WPI_TalonSRX elevator;
  // Old Robot: 0
  // New Robot: 1
  int robotMode = 1;

  Encoder elevatorEncoder;
  double motorSpeed = 0.7;

  //Driver Controller Setup
  double deadzone = 0.15;
  double driverJoyY = 0;
  double driverJoyX = 0;
  double driverJoyZ = 0;
  double driverRightTrigger = 0;
  boolean driverButtonA;
  boolean driverButtonB;
  boolean driverButtonX;
  boolean driverButtonY;
  boolean driverButtonRight;
  boolean driverButtonLeft;

  //Manipulator Controller Setup
  double manipulatorJoyY = 0;

  boolean switchValue;
  double target = 0.5;
  double correctSpeed = 0.2;
  XboxController driver = new XboxController(RobotMap.xBoxDriverChannel);
  XboxController manipulator = new XboxController(RobotMap.xBoxManipulatorChannel);
  int frames = 30;
  double currentData;
  public double preTime = 0.0; 
  int ledCode = 1;
  DoubleSolenoid doubleSolenoid = new DoubleSolenoid(RobotMap.doubleSolenoidForwardChannel, RobotMap.doubleSolenoidReverseChannel);
  Compressor compressor = new Compressor();
  DigitalInput limitSwitch = new DigitalInput(RobotMap.limitSwitchChannel);
  double elevatorMax = 10;//set later

 
  //instantiate output of PIDout
  PIDout output = new PIDout(this); 
  PIDoutX strafeOutput = new PIDoutX(this);
  PIDoutY forwardOutput = new PIDoutY(this);
  EncoderPID encoderPID = new EncoderPID(this);
  //Ints. Class 
  CameraSource limelight = new CameraSource(this); 
  CameraSourceX limelightX = new CameraSourceX(this);
  CameraSourceY limelightY = new CameraSourceY(this);
  EncoderSource encoder = new EncoderSource(this);

  //Set PIDs
  PIDController visionLoop = new PIDController(0.03, 0.0, 0.0, limelight, output);
  PIDController strafeLoop = new PIDController(0.1, 0.0, 0.0, limelightX, strafeOutput);
  PIDController forwardLoop = new PIDController(0.04, 0.0, 0.0, limelightY, forwardOutput);
  PIDController encoderLoop = new PIDController(0.02, 0.0, 0.0, encoder, encoderPID);
  //Global Varable for CameraValues
  public double CameraValue = 0;
  public double StrafeValue = 0;
  public double ForwardValue = 0;
  public double encoderValue = 0;
  public double rotations = 0;
  public double circumfrence = 0;
  public double distance = 0;
  double actualSkew;

  LED Leds = new LED();


  @Override
  public void robotInit() {
    //Setup Drive Train

    if (robotMode == 0){
      RobotMap.talonFrontRightChannel = 4;
      RobotMap.talonFrontRightReverse = false;
      RobotMap.talonRearRightChannel = 3;
      RobotMap.talonRearRightReverse = false;
      RobotMap.talonFrontLeftChannel = 2;
      RobotMap.talonFrontLeftReverse = false;
      RobotMap.talonRearLeftChannel = 1;
      RobotMap.talonRearLeftReverse = false;
      RobotMap.talonElevatorChannel = 5;
      RobotMap.talonElevatorReverse = false;
    }else{
      RobotMap.talonFrontRightChannel = 4;
      RobotMap.talonFrontRightReverse = true;
      RobotMap.talonRearRightChannel = 3;
      RobotMap.talonRearRightReverse = false;
      RobotMap.talonFrontLeftChannel = 2;
      RobotMap.talonFrontLeftReverse = true;
      RobotMap.talonRearLeftChannel = 1;
      RobotMap.talonRearLeftReverse = true;
      RobotMap.talonElevatorChannel = 5;
      RobotMap.talonElevatorReverse = false;
    }
   
    frontLeft = new WPI_TalonSRX(RobotMap.talonFrontLeftChannel);
    rearLeft = new WPI_TalonSRX(RobotMap.talonRearLeftChannel);
    frontRight = new WPI_TalonSRX(RobotMap.talonFrontRightChannel);
    rearRight = new WPI_TalonSRX(RobotMap.talonRearRightChannel);
    elevator = new WPI_TalonSRX(RobotMap.talonElevatorChannel);
    frontLeft.setInverted(RobotMap.talonFrontLeftReverse);
    rearLeft.setInverted(RobotMap.talonRearLeftReverse);
    frontRight.setInverted(RobotMap.talonFrontRightReverse);
    rearRight.setInverted(RobotMap.talonRearRightReverse);
    
    robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    
    //Start Compressor
    //compressor.start();

    //Setup Encoders
    elevatorEncoder = new Encoder(RobotMap.encoderAChannel, RobotMap.encoderBChannel, false, Encoder.EncodingType.k4X);
    elevatorEncoder.setDistancePerPulse((Math.PI * 8) / 360);     

//    rearRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    //Seting Camera value Ranges and Setpoints
    strafeLoop.setSetpoint(0.0);
    strafeLoop.setOutputRange(-1,1);
    strafeLoop.setInputRange(-25.0, 25.0);
    
    visionLoop.setSetpoint(0.0);
    visionLoop.setOutputRange(-1,1);
    visionLoop.setInputRange(-25.0, 25.0);
    
    forwardLoop.setSetpoint(6.0);
    forwardLoop.setOutputRange(-1,1);
    forwardLoop.setInputRange(-25.0, 25.0); 

    encoderLoop.setSetpoint(24.0); //distance
    encoderLoop.setOutputRange(-0.5,0.5); //max speed
    encoderLoop.setInputRange(-25.0, 25.0); //the minimum or maximum percentage to write to the output

    /*new Thread(() -> {
        
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      
      camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 360, 320, frames);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 480, 320);

      Mat source = new Mat();
      Mat output = new Mat();
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(source);
          currentData = camera.getActualDataRate();
          if (currentData > 3.5 && frames > 8){
            frames = frames - 2;
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 360, 320, frames);
          }else if (currentData < 1.5 && frames < 30){
            frames = frames + 2;
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 360, 320, frames);
          }
          Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }).start();*/

      /*new Thread(() -> {
        Timer frameTimer = new Timer();
        frameTimer.start();
        preTime = frameTimer.get();
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setVideoMode(VideoMode.PixelFormat.kMJPEG , 320, 240, 30);
        //camera.setResolution(320, 240);
        //camera.setFPS(30);
        
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 200, 200);
        
        Mat source = new Mat();
        Mat output = new Mat();
        
        while(!Thread.interrupted()) {
            double currentTime = frameTimer.get();
            double diffTime = currentTime - preTime;
            SmartDashboard.putNumber("FPSTimer", (1/diffTime));
            preTime = currentTime;
            cvSink.grabFrame(source);
            Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
            outputStream.putFrame(output);
        }
    }).start();*/

    }
  @Override
  public void teleopPeriodic() {
    //Gather encoder position, post to smartDashboard. Chech to see if B is pressed to reset encoder.
    encoderValue = -rearRight.getSelectedSensorPosition(0);
    rotations = encoderValue/4000;
    circumfrence = Math.PI*8;
    distance = circumfrence * rotations;
    SmartDashboard.putNumber("Rotations", rotations);
    SmartDashboard.putNumber("Encoder Value", encoderValue);
    SmartDashboard.putNumber("Distance", distance);
    if (driverButtonX) {
        rearRight.setSelectedSensorPosition(0, 0, 0);
    }

    SmartDashboard.putNumber("Encoder Distance:" , elevatorEncoder.getDistance());   
    SmartDashboard.putNumber("Encoder Value:" , elevatorEncoder.get());  
    
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double skew = ts.getDouble(0.0);
    CameraValue = x;
    ForwardValue = area;
    
    if (skew > -45){
      actualSkew = Math.abs(skew);
    } else {
      actualSkew = Math.abs(skew) - 90;
    }
    StrafeValue = actualSkew;

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightSkew", actualSkew);

    //Driver Controller
    driverRightTrigger = driver.getRawAxis(RobotMap.xBoxRightTriggerChannel);
    driverButtonA = driver.getRawButton(RobotMap.xBoxButtonAChannel);
    driverButtonB = driver.getRawButton(RobotMap.xBoxButtonBChannel);
    driverButtonX = driver.getRawButton(RobotMap.xBoxButtonXChannel);
    driverButtonRight = driver.getRawButton(RobotMap.xBoxButtonRightChannel);
    driverJoyY = driver.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    driverJoyX = driver.getRawAxis(RobotMap.xBoxLeftStickXChannel);
    driverJoyZ = driver.getRawAxis(RobotMap.xBoxRightStickXChannel);

    //Manipulator Controller
    manipulatorJoyY = manipulator.getRawAxis(RobotMap.xBoxLeftStickYChannel);


    //Read Values on Smartdashboard
    SmartDashboard.putNumber("JoyX", driverJoyX);
    SmartDashboard.putNumber("JoyY x number", driverJoyY*motorSpeed);
    SmartDashboard.putNumber("JoyY", driverJoyY);
    SmartDashboard.putNumber("JoyZ", driverJoyZ);
    SmartDashboard.putNumber("timer", ledCode);
    SmartDashboard.putBoolean("Limit Switch", switchValue);
    
    //Deadzone
    if (Math.abs(driverJoyY) < (deadzone)) {
      driverJoyY = 0;
    }
    if (Math.abs(driverJoyX) < (deadzone)) {
      driverJoyX = 0;
    }
    if (Math.abs(driverJoyZ) < (deadzone)) {
      driverJoyZ = 0;
    }
    

    //Controlling PID Loops 
    if (driverButtonA){
      visionLoop.enable();
      strafeLoop.enable();
      forwardLoop.enable();
      robotDrive.driveCartesian(-strafeOutput.outputX, -forwardOutput.outputY, output.outputSkew, 0.0);
    } else if (driverRightTrigger > 0.6){
      encoderLoop.enable();
      robotDrive.driveCartesian(-0.0, encoderPID.outputEncoder, 0.0, 0.0);
    }else{
      visionLoop.disable();
      strafeLoop.disable();
      forwardLoop.disable();
      encoderLoop.disable();
      robotDrive.driveCartesian(-driverJoyX * motorSpeed, driverJoyY * motorSpeed, -driverJoyZ * motorSpeed, 0.0);
    }

    //Checking if reflective tape area is less and change LED lights
    if(y < 4)
    {
      //Lower Hatches
      if(area >= 4.0 && x > -4 && x < 4)
      {
        Leds.sendCode(1);
      }  
      else
      {
        Leds.sendCode(2);
      }
    }
    else if(y > 4)
    {
      //Ball Hatches
      if(area >= 2.5 && x > -3.5 && x < 3.5)
      {
        Leds.sendCode(1);
      }
      else
      {
        Leds.sendCode(2);
      }
    }
    else 
    {
      Leds.sendCode(2);
    }


    if (driverButtonB){
      Leds.sendCode(9);
    }

    if (driverButtonRight){
      doubleSolenoid.set(Value.kForward);
    }  else{
      doubleSolenoid.set(Value.kReverse);
    }
    if (limitSwitch.get()){
      elevatorEncoder.reset();
    }
    if (distance < elevatorMax){
      elevator.set(manipulatorJoyY);
    }  else if(distance <= 0){
      if(manipulatorJoyY < 0){
        elevator.set(0);
      }  else{
        elevator.set(manipulatorJoyY);
      }
    }
    else{
      if(manipulatorJoyY > 0){
        elevator.set(0);
      }  else{
        elevator.set(manipulatorJoyY);
      }
    }

  }  
  
  public void testPeriodic(){
  /*Proper Code On Robot
  Ping Talons(Drive Base, Arms)
  Limelight Sees Vision Targets
  Vision Camera Sends Feed
  LEDs Work
  Compressor Runs
  Indicator Light Works
  Encoders Returning Values
  Motors Run
  Joysticks Return Value*/
  }
}
