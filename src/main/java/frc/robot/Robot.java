package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.IMotorController;
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
import edu.wpi.first.wpilibj.AnalogInput;

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
  WPI_TalonSRX ballIntake;
  WPI_TalonSRX elevator;
  // Old Robot: 0
  // New Robot: 1
  int robotMode = 1;

  Encoder testCoder;

  double deadzone = 0.15;
  double joyY = 0;
  double m_JoyY = 0;
  double joyX = 0;
  double joyZ = 0;
  double rightTrigger = 0;
  boolean buttonA;
  boolean m_ButtonA;
  boolean buttonB;
  boolean m_ButtonB;
  boolean buttonX;
  boolean m_ButtonX;
  boolean buttonY;
  boolean buttonRight;
  boolean m_ButtonRight;
  boolean buttonLeft;
  boolean m_ButtonLeft;
  double target = 0.5;
  double correctSpeed = 0.2;
  XboxController controller = new XboxController(RobotMap.xBoxControllerChannel);
  XboxController manipulator = new XboxController(RobotMap.xBoxManipulatorControllerChannel);
  int frames = 30;
  double currentData;
  public double preTime = 0.0; 
  int ledCode = 1;
  DoubleSolenoid hingeSolenoid = new DoubleSolenoid(RobotMap.hingeSolenoidForwardChannel, RobotMap.hingeSolenoidReverseChannel);
  DoubleSolenoid hatchSolenoid = new DoubleSolenoid(RobotMap.hatchSolenoidForwardChannel, RobotMap.hatchSolenoidReverseChannel);
  Compressor compressor = new Compressor();
  
  //Pressure Sensor 
  AnalogInput PressureSensor = new AnalogInput(1);
  double PSVolt;  

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
    }else{
      RobotMap.talonFrontRightChannel = 2;
      RobotMap.talonFrontRightReverse = false;
      RobotMap.talonRearRightChannel = 4;
      RobotMap.talonRearRightReverse = true;
      RobotMap.talonFrontLeftChannel = 1;
      RobotMap.talonFrontLeftReverse = true;
      RobotMap.talonRearLeftChannel = 3;
      RobotMap.talonRearLeftReverse = true;
    }
   
    frontLeft = new WPI_TalonSRX(RobotMap.talonFrontLeftChannel);
    rearLeft = new WPI_TalonSRX(RobotMap.talonRearLeftChannel);
    frontRight = new WPI_TalonSRX(RobotMap.talonFrontRightChannel);
    rearRight = new WPI_TalonSRX(RobotMap.talonRearRightChannel);
    ballIntake = new WPI_TalonSRX(RobotMap.talonBallIntakeChannel);
    elevator = new WPI_TalonSRX(RobotMap.talonElevatorChannel);
    frontLeft.setInverted(RobotMap.talonFrontLeftReverse);
    rearLeft.setInverted(RobotMap.talonRearLeftReverse);
    frontRight.setInverted(RobotMap.talonFrontRightReverse);
    rearRight.setInverted(RobotMap.talonRearRightReverse);
    
    robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    
    //Start Compressor
    //compressor.start();

    //Setup Encoders
    testCoder = new Encoder(RobotMap.encoderAChannel, RobotMap.encoderBChannel, false, Encoder.EncodingType.k4X);
    testCoder.setDistancePerPulse((Math.PI * 8) / 360);     

    //rearRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
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
    if (buttonX) {
        rearRight.setSelectedSensorPosition(0, 0, 0);
    }

    SmartDashboard.putNumber("Encoder Distance:" , testCoder.getDistance());   
    SmartDashboard.putNumber("Encoder Value:" , testCoder.get());  
    
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

    //Pressure Sensor Voltage to Pressure converstion
    PSVolt = PressureSensor.getVoltage();
    double PSPress = ((250*(PSVolt/5)) - 25);

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
    SmartDashboard.putNumber("PressureSensPress", PSPress);


    rightTrigger = controller.getRawAxis(RobotMap.xBoxRightTriggerChannel);
    buttonA = controller.getRawButton(RobotMap.xBoxButtonAChannel);
    m_ButtonA = manipulator.getRawButton(RobotMap.xBoxButtonAChannel);
    buttonB = controller.getRawButton(RobotMap.xBoxButtonBChannel);
    m_ButtonB = manipulator.getRawButton(RobotMap.xBoxButtonBChannel);
    buttonX = controller.getRawButton(RobotMap.xBoxButtonXChannel);
    m_ButtonX = manipulator.getRawButton(RobotMap.xBoxButtonXChannel);
    buttonRight = controller.getRawButton(RobotMap.xBoxButtonRightChannel);
    m_ButtonRight = manipulator.getRawButton(RobotMap.xBoxButtonRightChannel);
    m_ButtonLeft = manipulator.getRawButton(RobotMap.xBoxButtonLeftChannel);
    joyY = controller.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    m_JoyY = manipulator.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    joyX = controller.getRawAxis(RobotMap.xBoxLeftStickXChannel);
    joyZ = controller.getRawAxis(RobotMap.xBoxRightStickXChannel);
    

    //Read Values on Smartdashboard
    SmartDashboard.putNumber("JoyX", joyX);
    SmartDashboard.putNumber("JoyY", joyY);
    SmartDashboard.putNumber("JoyZ", joyZ);
    SmartDashboard.putNumber("timer", ledCode);
    
    //Deadzone
    if (Math.abs(joyY) < (deadzone)) {
      joyY = 0;
    }
    if (Math.abs(joyX) < (deadzone)) {
      joyX = 0;
    }
    if (Math.abs(joyZ) < (deadzone)) {
      joyZ = 0;
    }
    

    //Controlling PID Loops 
    if (buttonA){
      visionLoop.enable();
      strafeLoop.enable();
      forwardLoop.enable();
      robotDrive.driveCartesian(-strafeOutput.outputX, -forwardOutput.outputY, output.outputSkew, 0.0);
    } else if (rightTrigger > 0.6){
      encoderLoop.enable();
      robotDrive.driveCartesian(-0.0, encoderPID.outputEncoder, 0.0, 0.0);
    }else{
      visionLoop.disable();
      strafeLoop.disable();
      forwardLoop.disable();
      encoderLoop.disable();
      robotDrive.driveCartesian(-joyX , joyY , -joyZ , 0.0);
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


    if (buttonB){
      Leds.sendCode(8);
      System.out.println("B press");
    }


    //Ball Intake State Machine
    if (m_ButtonA){
    hingeSolenoid.set(Value.kReverse);
    ballIntake.set(0.4);
      if(ballIntake.getSelectedSensorPosition() >= 10){
      ballIntake.set(0);
      hingeSolenoid.set(Value.kReverse);
      }
    } else{
    ballIntake.set(0);
    hingeSolenoid.set(Value.kForward);
    }
    if (buttonB){
    hatchSolenoid.set(Value.kReverse);
  }else{
    hatchSolenoid.set(Value.kForward);
  }
 elevator.set(m_JoyY);
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
