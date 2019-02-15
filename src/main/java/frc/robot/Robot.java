package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.DemandType;
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
  WPI_TalonSRX m_Climb;
  WPI_TalonSRX s_Climb;
  WPI_TalonSRX d_Climb;
  WPI_TalonSRX drive_Climb;
  // Old Robot: 0
  // New Robot: 1
  int robotMode = 0;

  //Encoder testCoder;

  double db_joyDeadzone = 0.15;
  double db_cntlDriverJoyLeftY = 0;
  double db_cntlManipJoyLeftY = 0;
  double db_cntlDriverJoyLeftX = 0;
  double db_cntlDriverJoyRightX = 0;
  double db_cntlManipJoyRightX = 0;
  double db_cntlManipJoyRightY = 0;
  double db_cntlDriverTriggerRight = 0;
  double db_cntlManipTriggerRight = 0;
  boolean b_cntlDriverButtonA;
  boolean b_cntlManipButtonA;
  boolean b_cntlDriverButtonB;
  boolean b_cntlManipButtonB;
  boolean b_cntlDriverButtonX;
  boolean b_cntlManipButtonX;
  boolean b_cntlDriverButtonY;
  boolean b_cntlManipButtonY;
  boolean b_cntlDriverButtonRight;
  boolean b_cntlManipButtonRight;
  boolean b_cntlDriverButtonLeft;
  boolean b_cntlManipButtonLeft;
  //double target = 0.5;
  //double correctSpeed = 0.2;
  XboxController cntlDriver = new XboxController(RobotMap.xBoxControllerChannel);
  XboxController cntlManipulator = new XboxController(RobotMap.xBoxManipulatorControllerChannel);
  //int frames = 30;
  //double currentData;
  int n_ledColorCode = 1;
  DoubleSolenoid hingeSolenoid = new DoubleSolenoid(RobotMap.hingeSolenoidForwardChannel, RobotMap.hingeSolenoidReverseChannel);
  DoubleSolenoid hatchSolenoid = new DoubleSolenoid(RobotMap.hatchSolenoidForwardChannel, RobotMap.hatchSolenoidReverseChannel);
  DoubleSolenoid elevatorSolenoid = new DoubleSolenoid(RobotMap.elevatorSolenoidForwardChannel, RobotMap.elevatorSolenoidReverseChannel);
  Compressor compressor = new Compressor();
  DigitalInput topLimitSwitchFrontRight = new DigitalInput(0);
  boolean limitTopFrontRight = false;
  DigitalInput bottomLimitSwitchFrontRight = new DigitalInput(1);
  boolean limitBottomFrontRight = false;
  DigitalInput topLimitSwitchFrontLeft = new DigitalInput(2);
  boolean limitTopFrontLeft = false;
  DigitalInput bottomLimitSwitchFrontLeft = new DigitalInput(3);
  boolean limitBottomFrontLeft = false;
  DigitalInput topLimitSwitchRear = new DigitalInput(4);
  boolean limitTopRear = false;
  DigitalInput bottomLimitSwitchRear = new DigitalInput(5);
  boolean limitBottomRear = false;
  //AnalogInput infrared = new AnalogInput(0);
  double voltage = 0;
  //double encoderValue = 0;
  double powerFR = 0;
  double powerFL = 0;
  double powerR = 0;

  //Timers
  Timer timer = new Timer();
  Timer timerSystem = new Timer();
  double db_prevTime = 0.0;
  double db_endTime = 0.0;
  boolean b_ballIntake = false;
  SampleSmoother timerSmoother = new SampleSmoother(200);
  //double irDistance = 0;
  
  //New EndGame Class
  EndGame endGame = new EndGame();

  //Pressure Sensor 
  AnalogInput PressureSensor = new AnalogInput(1);
  double PSVolt;  

  //instantiate output of PIDout
  PIDout outputSkew = new PIDout(this); 
  PIDoutX strafeOutput = new PIDoutX(this);
  PIDoutY forwardOutput = new PIDoutY(this);
  EncoderPID encoderPID = new EncoderPID(this);
  //Ints. Class 
  CameraSource limelight = new CameraSource(this); 
  CameraSourceX limelightX = new CameraSourceX(this);
  CameraSourceY limelightY = new CameraSourceY(this);
  EncoderSource encoder = new EncoderSource(this);

  //Set PIDs
  PIDController visionLoop = new PIDController(0.03, 0.0, 0.0, limelight, outputSkew);
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
    // Setup timers
    timer.reset();
    timerSystem.reset();
    timerSystem.start();
    db_prevTime = timerSystem.get();


    //Setup Drive Train

    if (robotMode == 0){
      RobotMap.talonFrontRightChannel = 4;
      RobotMap.talonFrontRightReverse = false;
      RobotMap.talonRearRightChannel = 3;
      RobotMap.talonRearRightReverse = true;
      RobotMap.talonFrontLeftChannel = 2;
      RobotMap.talonFrontLeftReverse = false;
      RobotMap.talonRearLeftChannel = 1;
      RobotMap.talonRearLeftReverse = true;
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
    //Drive Train
    frontLeft = new WPI_TalonSRX(RobotMap.talonFrontLeftChannel);
    rearLeft = new WPI_TalonSRX(RobotMap.talonRearLeftChannel);
    frontRight = new WPI_TalonSRX(RobotMap.talonFrontRightChannel);
    rearRight = new WPI_TalonSRX(RobotMap.talonRearRightChannel);
    frontLeft.setInverted(RobotMap.talonFrontLeftReverse);
    rearLeft.setInverted(RobotMap.talonRearLeftReverse);
    frontRight.setInverted(RobotMap.talonFrontRightReverse);
    rearRight.setInverted(RobotMap.talonRearRightReverse);
    
    //Ball and Elevator
    ballIntake = new WPI_TalonSRX(RobotMap.talonBallIntakeChannel);
    
    //Endgame
    m_Climb = new WPI_TalonSRX(RobotMap.talonClimbAChannel);
    s_Climb = new WPI_TalonSRX(RobotMap.talonClimbBChannel);
    d_Climb = new WPI_TalonSRX(RobotMap.talonClimbCChannel);
    drive_Climb = new WPI_TalonSRX(RobotMap.talonClimbDriveChannel);
    s_Climb.setInverted(true);
    d_Climb.setInverted(true);
    s_Climb.follow(m_Climb);
    
    
   robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    
    //Initialize EndGame Parameters
    endGame.driveTrainFrontRight = frontRight;
    endGame.driveTrainFrontLeft = frontLeft;
    endGame.driveTrainRearRight = rearRight;
    endGame.driveTrainRearLeft = rearLeft;
    endGame.climberFrontMaster = m_Climb;
    endGame.climberRear = d_Climb;
    endGame.climberDrive = drive_Climb;

    endGame.init();
    //Start Compressor
    //compressor.start();

    //Setup Encoders
    //testCoder = new Encoder(RobotMap.encoderAChannel, RobotMap.encoderBChannel, false, Encoder.EncodingType.k4X);
    //testCoder.setDistancePerPulse((Math.PI * 8) / 360);     

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
    // display system speed
    double db_currentTime = timerSystem.get();
    double db_deltaTime = db_currentTime - db_prevTime;
    db_prevTime = db_currentTime;
    timerSmoother.addSample(db_deltaTime);




    //Gather encoder position, post to smartDashboard. Chech to see if B is pressed to reset encoder.
    //voltage = infrared.getAverageVoltage();
    //irDistance = 4800/(voltage - 20);
    encoderValue = -rearRight.getSelectedSensorPosition(0);
    //rotations = encoderValue/4000;
    //circumfrence = Math.PI*8;
    //distance = circumfrence * rotations;
    //SmartDashboard.putNumber("Rotations", rotations);
    SmartDashboard.putNumber("Encoder Value", encoderValue);
    //SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("Voltage", voltage);
    if (b_cntlDriverButtonX) {
        rearRight.setSelectedSensorPosition(0, 0, 0);
    }

    //SmartDashboard.putNumber("Encoder Distance:" , testCoder.getDistance());   
    //SmartDashboard.putNumber("Encoder Value:" , testCoder.get());  
    
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
    limitTopFrontRight = topLimitSwitchFrontRight.get();
    powerFR = m_Climb.get();
    powerFL = s_Climb.get();
    powerR = d_Climb.get();
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightSkew", actualSkew);
    SmartDashboard.putNumber("PressureSensPress", PSPress);
    SmartDashboard.putBoolean("Limit", limitTopFrontRight);
    SmartDashboard.putNumber("Amps", powerFR);


    db_cntlDriverTriggerRight = cntlDriver.getRawAxis(RobotMap.xBoxdb_cntlDriverTriggerRightChannel);
    db_cntlManipTriggerRight = cntlManipulator.getRawAxis(RobotMap.xBoxdb_cntlDriverTriggerRightChannel);
    b_cntlDriverButtonA = cntlDriver.getRawButton(RobotMap.xBoxButtonAChannel);
    b_cntlManipButtonA = cntlManipulator.getRawButton(RobotMap.xBoxButtonAChannel);
    b_cntlDriverButtonB = cntlDriver.getRawButton(RobotMap.xBoxButtonBChannel);
    b_cntlManipButtonB = cntlManipulator.getRawButton(RobotMap.xBoxButtonBChannel);
    b_cntlDriverButtonX = cntlDriver.getRawButton(RobotMap.xBoxButtonXChannel);
    b_cntlManipButtonX = cntlManipulator.getRawButton(RobotMap.xBoxButtonXChannel);
    b_cntlDriverButtonRight = cntlDriver.getRawButton(RobotMap.xBoxButtonRightChannel);
    b_cntlManipButtonRight = cntlManipulator.getRawButton(RobotMap.xBoxButtonRightChannel);
    b_cntlManipButtonLeft = cntlManipulator.getRawButton(RobotMap.xBoxButtonLeftChannel);
    db_cntlDriverJoyLeftY = cntlDriver.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    db_cntlManipJoyLeftY = cntlManipulator.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    db_cntlDriverJoyLeftX = cntlDriver.getRawAxis(RobotMap.xBoxLeftStickXChannel);
    db_cntlDriverJoyRightX = cntlDriver.getRawAxis(RobotMap.xBoxRightStickXChannel);
    db_cntlManipJoyRightX = cntlManipulator.getRawAxis(RobotMap.xBoxRightStickXChannel);
    db_cntlManipJoyRightY = cntlManipulator.getRawAxis(RobotMap.xBoxRightStickYChannel);
    

    //Read Values on Smartdashboard
    SmartDashboard.putNumber("db_cntlDriverJoyLeftX", db_cntlDriverJoyLeftX);
    SmartDashboard.putNumber("db_cntlDriverJoyLeftY", db_cntlDriverJoyLeftY);
    SmartDashboard.putNumber("db_cntlDriverJoyRightX", db_cntlDriverJoyRightX);
    SmartDashboard.putNumber("timer", n_ledColorCode);
    SmartDashboard.putNumber("Delta Time",1/db_deltaTime);
    SmartDashboard.putNumber("Smooth Time", 1/timerSmoother.getAverage());
    
    //db_joyDeadzone
    if (Math.abs(db_cntlDriverJoyLeftY) < (db_joyDeadzone)) {
      db_cntlDriverJoyLeftY = 0;
    }
    if (Math.abs(db_cntlDriverJoyLeftX) < (db_joyDeadzone)) {
      db_cntlDriverJoyLeftX = 0;
    }
    if (Math.abs(db_cntlDriverJoyRightX) < (db_joyDeadzone)) {
      db_cntlDriverJoyRightX = 0;
    }
    

    //Controlling PID Loops
  
    if (b_cntlDriverButtonA){
      visionLoop.enable();
      strafeLoop.enable();
      forwardLoop.enable();
      robotDrive.driveCartesian(-strafeOutput.outputX, -forwardOutput.outputY, outputSkew.output, 0.0);
    } else if (db_cntlDriverTriggerRight > 0.6){
      encoderLoop.enable();
      robotDrive.driveCartesian(-0.0, encoderPID.outputEncoder, 0.0, 0.0);
    }else{
      visionLoop.disable();
      strafeLoop.disable();
      forwardLoop.disable();
      encoderLoop.disable();
      robotDrive.driveCartesian(-db_cntlDriverJoyLeftX ,db_cntlDriverJoyLeftY,-db_cntlDriverJoyRightX , 0.0);
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


    if (b_cntlDriverButtonB){
      Leds.sendCode(8);
      System.out.println("B press");
    }


    //Ball Intake State Machine
    if (b_cntlManipButtonRight){
      Leds.sendCode(10);
      hingeSolenoid.set(Value.kReverse);
      ballIntake.set(0.7);
      db_currentTime = timer.get();
      if(ballIntake.getSelectedSensorPosition() >= 3 && b_ballIntake == false){
        db_endTime = db_currentTime + 1;
        b_ballIntake = true;
      }
      if (db_currentTime >= db_endTime){
        ballIntake.set(0);
        Leds.sendCode(2);
        hingeSolenoid.set(Value.kReverse);
        b_ballIntake = false;
      }
    } else if (b_cntlManipButtonLeft){
      hingeSolenoid.set(Value.kForward);
      ballIntake.set(0.5);
    } else{
      ballIntake.set(0);
      Leds.sendCode(2); 
      hingeSolenoid.set(Value.kForward);
      b_ballIntake = false;
    }
    
    if (b_cntlManipButtonB){
      hatchSolenoid.set(Value.kForward);
    }  else{
      hatchSolenoid.set(Value.kReverse);
    }
  if(topLimitSwitchFrontRight.get() && powerFR > 0 ){
    m_Climb.set(0);
  } else if(bottomLimitSwitchFrontRight.get() && powerFR < 0){
    m_Climb.set(0);
  }
  
  if(topLimitSwitchFrontLeft.get() && powerFL > 0 ){
    m_Climb.set(0);
  } else if(bottomLimitSwitchFrontLeft.get() && powerFL < 0){
    m_Climb.set(0);
  }
  
  if(topLimitSwitchRear.get() && powerR > 0 ){
    d_Climb.set(0);
  } else if(bottomLimitSwitchRear.get() && powerR < 0){
    d_Climb.set(0);
  }
  
  if (b_cntlManipButtonX){
    elevatorSolenoid.set(Value.kForward);
  } else{
    elevatorSolenoid.set(Value.kReverse);
  }
  m_Climb.set(db_cntlManipJoyRightY *.25);
  d_Climb.set(db_cntlManipJoyLeftY * .25);
  drive_Climb.set(db_cntlManipJoyRightX *.25);
}

  public void testPeriodic(){
  /*
  Things to do before match:
  Proper Code On Robot
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
