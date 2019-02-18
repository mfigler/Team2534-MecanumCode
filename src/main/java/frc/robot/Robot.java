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
import edu.wpi.first.wpilibj.SpeedControllerGroup;

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
  LimitSwitchTalon m_Climb;
  LimitSwitchTalon s_Climb;
  LimitSwitchTalon d_Climb;
  WPI_TalonSRX drive_Climb;
  SpeedControllerGroup driveTrain;
  // Old Robot: 0
  // New Robot: 1
  int robotMode = 1;

  //Encoder testCoder;

  double db_joyDeadzone = 0.15;
  double db_cntlDriverJoyLeftY = 0;
  double db_cntlManipJoyLeftY = 0;
  double db_cntlEndJoyLeftY = 0;
  double db_cntlDriverJoyLeftX = 0;
  double db_cntlDriverJoyRightX = 0;
  double db_cntlManipJoyRightX = 0;
  double db_cntlEndJoyRightX = 0;
  double db_cntlDriverJoyRightY = 0;
  double db_cntlManipJoyRightY = 0;
  double db_cntlEndJoyRightY = 0;
  double db_cntlDriverTriggerRight = 0;
  double db_cntlManipTriggerRight = 0;
  double db_cntlManipTriggerLeft = 0;
  double db_cntlDriverdb_cntlManipJoyRightY = 0;
  double db_cntlDriverTriggerLeft = 0;
  boolean b_cntlDriverButtonA;
  boolean b_cntlManipButtonA;
  boolean b_cntlDriverButtonB;
  boolean b_cntlManipButtonB;
  boolean b_cntlDriverButtonX;
  boolean b_cntlManipButtonX;
  boolean b_cntlDriverButtonY;
  boolean b_cntlManipButtonY;
  boolean b_cntlDriverButtonRight = false;
  boolean b_cntlManipButtonRight;
  boolean b_cntlDriverButtonLeft;
  boolean b_cntlManipButtonLeft;
  boolean b_cntlManipButtonStart;
  boolean b_cntlDriverBackButton;


  //double target = 0.5;
  //double correctSpeed = 0.2;
  XboxController cntlDriver = new XboxController(RobotMap.xBoxControllerChannel);
  XboxController cntlManipulator = new XboxController(RobotMap.xBoxManipulatorControllerChannel);
  XboxController cntlEndGame = new XboxController(2);
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
  AnalogInput frontIR = new AnalogInput(2);
  AnalogInput rearIR = new AnalogInput(3);
  double frontVoltage = 0;
  double rearVoltage = 0;
  double powerFR = 0;
  double powerFL = 0;
  double powerR = 0;
  //Intake StateMachine
  int intake_default = 0;
  int intake_downTake = 1;
  int intake_upShoot = 2;
  int intake_upTake = 3;
  int intake_downShoot = 4;
  int intake_downShootB = 5;
  int intake_downTakeX = 6;
  int intake_shoot = 7;
  int intake_cargo = 8;
  int intake_cargoShoot = 9;
  int intake_elevatorUp = 10;
  int intakeMachine = intake_default;
  //Timers
  Timer timer = new Timer();
  double db_shootTimer = 0.0;
  double db_cargoTimer = 0.0;
  Timer timerSystem = new Timer();
  double db_prevTime = 0.0;
  double db_endTime = 0.0;
  boolean b_ballIntake = false;
  SampleSmoother timerSmoother = new SampleSmoother(200);
  //double irDistance = 0;
  
  //End Game State Machine
  int endGameState = 0;
  //End Game Climb Values
  double liftSpeed = 0.8;
  double frontRetractSpeed = -0.8;
  double backRetractSpeedSlow = -0.3;
  double backRetractSpeedFast = -0.5;

  //Pressure Sensor 
  AnalogInput PressureSensor = new AnalogInput(1);
  double PSVolt;  

  //instantiate output of PIDout
  PIDout outputSkew = new PIDout(this); 
  PIDoutX strafeOutput = new PIDoutX(this);
  PIDoutY forwardOutput = new PIDoutY(this);
  EncoderPID encoderPID = new EncoderPID(this);
  PIDout mLiftPID = new PIDout(this);
  PIDout sLiftPID = new PIDout(this);

  //Ints. Class 
  CameraSource limelight = new CameraSource(this); 
  CameraSourceX limelightX = new CameraSourceX(this);
  CameraSourceY limelightY = new CameraSourceY(this);
  EncoderSource encoder = new EncoderSource(this);
  LiftSource mLift = new LiftSource(this, true);
  LiftSource sLift = new LiftSource(this, false);

  //Set PIDs
  PIDController visionLoop = new PIDController(0.03, 0.0, 0.0, limelight, outputSkew);
  PIDController strafeLoop = new PIDController(0.08, 0.0, 0.0, limelightX, strafeOutput);
  PIDController forwardLoop = new PIDController(0.05, 0.0, 0.0, limelightY, forwardOutput);
  PIDController encoderLoop = new PIDController(0.02, 0.0, 0.0, encoder, encoderPID);
  PIDController mLiftLoop = new PIDController(0.01, 0.0, 0.02, mLift , mLiftPID);
  PIDController sLiftLoop = new PIDController(0.01, 0.0, 0.02, sLift , sLiftPID);

  //Global Varable for CameraValues
  public double CameraValue = 0;
  public double StrafeValue = 0;
  public double ForwardValue = 0;
  public double encoderValue = 0;
  public double mLiftEncoder = 0;
  public double sLiftEncoder = 0;
  public double dLiftEncoder = 0;
  public double rotations = 0;
  public double circumfrence = 0;
  public double distance = 0;
  double actualSkew;

  LED Leds = new LED();
  public boolean disableInt;


  @Override
  public void robotInit() {
    //Speed Controller Group
    //driveTrain = new SpeedControllerGroup(frontRight, frontLeft, rearRight, rearLeft);

    // Setup timers
    timer.reset();
    timerSystem.reset();
    timerSystem.start();
    db_prevTime = timerSystem.get();

    //Leds.sendCode(9);

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
      RobotMap.talonFrontRightReverse = true;
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
    frontLeft.set(0);
    rearLeft.set(0);
    frontRight.set(0);
    rearRight.set(0);
    
    //Ball and Elevator
    ballIntake = new WPI_TalonSRX(RobotMap.talonBallIntakeChannel);
    
    //Endgame
    m_Climb = new LimitSwitchTalon(RobotMap.talonClimbAChannel, topLimitSwitchFrontLeft, bottomLimitSwitchFrontLeft);
    s_Climb = new LimitSwitchTalon(RobotMap.talonClimbBChannel, bottomLimitSwitchFrontRight, topLimitSwitchFrontRight);
    d_Climb = new LimitSwitchTalon(RobotMap.talonClimbCChannel, topLimitSwitchRear, bottomLimitSwitchRear);
    drive_Climb = new WPI_TalonSRX(RobotMap.talonClimbDriveChannel);
    s_Climb.setInverted(true);
    d_Climb.setInverted(true);
    
    
   robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    
    //Initialize EndGame Parameters
   
    /*
    endGame.frontRight = frontRight;
    endGame.frontLeft = frontLeft;
    endGame.rearRight = rearRight;
    endGame.rearLeft = rearLeft;
    endGame.m_Climb = m_Climb;
    endGame.s_Climb = s_Climb;
    endGame.d_Climb = d_Climb;
    endGame.drive_Climb = drive_Climb;
    endGame.db_cntlManipJoyRightX = db_cntlManipJoyRightX;
    endGame.db_cntlManipJoyRightY = db_cntlManipJoyRightY;
    endGame.db_cntlManipJoyLeftY = db_cntlManipJoyLeftY;
    endGame.limitTopFrontLeft = limitTopFrontLeft;
    endGame.limitTopFrontRight = limitTopFrontRight;
    endGame.limitTopRear = limitTopRear;
    endGame.limitBottomFrontLeft = limitBottomFrontLeft;
    endGame.limitBottomFrontRight = limitBottomFrontRight;
    endGame.limitBottomRear = limitBottomRear;
    endGame.mLiftPID = mLiftPID;
    endGame.sLiftPID = sLiftPID;
    endGame.mLiftLoop = mLiftLoop;
    endGame.sLiftLoop = sLiftLoop;
    endGame.init();
    */
    
    //Start Compressor
    //compressor.start();

    //Setup Encoders
    //testCoder = new Encoder(RobotMap.encoderAChannel, RobotMap.encoderBChannel, false, Encoder.EncodingType.k4X);
    //testCoder.setDistancePerPulse((Math.PI * 8) / 360);     

    //rearRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    //Seting Camera value Ranges and Setpoints
    

    encoderLoop.setSetpoint(24.0); //distance
    encoderLoop.setOutputRange(-0.5,0.5); //max speed
    encoderLoop.setInputRange(-25.0, 25.0); //the minimum or maximum percentage to write to the output

    mLiftLoop.setOutputRange(-liftSpeed, 0.05);
    mLiftLoop.setInputRange(-900000,0);

    sLiftLoop.setOutputRange(-liftSpeed, 0.05);
    sLiftLoop.setInputRange(-900000,0);

    }
  
  @Override
  public void teleopInit()
  {
    Leds.sendCode(1);
  }

  @Override
  public void teleopPeriodic() {
    /*
    endGame.frontRight = frontRight;
    endGame.frontLeft = frontLeft;
    endGame.rearRight = rearRight;
    endGame.rearLeft = rearLeft;
    endGame.m_Climb = m_Climb;
    endGame.s_Climb = s_Climb;
    endGame.d_Climb = d_Climb;
    endGame.drive_Climb = drive_Climb;
    endGame.db_cntlManipJoyRightX = db_cntlManipJoyRightX;
    endGame.db_cntlManipJoyRightY = db_cntlManipJoyRightY;
    endGame.db_cntlManipJoyLeftY = db_cntlManipJoyLeftY;
    endGame.limitTopFrontLeft = limitTopFrontLeft;
    endGame.limitTopFrontRight = limitTopFrontRight;
    endGame.limitTopRear = limitTopRear;
    endGame.limitBottomFrontLeft = limitTopFrontLeft;
    endGame.limitBottomFrontRight = limitTopFrontRight;
    endGame.limitBottomRear = limitTopRear;
    endGame.mLiftPID = mLiftPID;
    endGame.sLiftPID = sLiftPID;
    endGame.mLiftLoop = mLiftLoop;
    endGame.sLiftLoop = sLiftLoop;
    */

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

    //LimeLight Setpoint and else
    if(y <= -5)
    {
      //Leds.sendCode(8);  //Yellow
      //Lower Hatch
      strafeLoop.setSetpoint(1.7);
      strafeLoop.setOutputRange(-1,1);
      strafeLoop.setInputRange(-25.0, 25.0);
    
      visionLoop.setSetpoint(3.0);
      visionLoop.setOutputRange(-1,1);
      visionLoop.setInputRange(-25.0, 25.0);
    
      forwardLoop.setSetpoint(8.0);
      forwardLoop.setOutputRange(-1,1);
      forwardLoop.setInputRange(-25.0, 25.0);
    }
    else 
    {  
      //Leds.sendCode(5); //Purple
      //Upper Ball
      strafeLoop.setSetpoint(0.0);
      strafeLoop.setOutputRange(-1,1);
      strafeLoop.setInputRange(-25.0, 25.0);
    
      visionLoop.setSetpoint(0.0);
      visionLoop.setOutputRange(-1,1);
      visionLoop.setInputRange(-25.0, 25.0);
    
      forwardLoop.setSetpoint(11);
      forwardLoop.setOutputRange(-1,1);
      forwardLoop.setInputRange(-25.0, 25.0);  
    }

    Leds.sendCode(2);
    // display system speed
    double db_currentTime = timerSystem.get();
    double db_deltaTime = db_currentTime - db_prevTime;
    db_prevTime = db_currentTime;
    timerSmoother.addSample(db_deltaTime);


    mLiftEncoder = m_Climb.getSelectedSensorPosition(0);
    sLiftEncoder = s_Climb.getSelectedSensorPosition(0);
    dLiftEncoder = d_Climb.getSelectedSensorPosition(0);

    mLiftLoop.setSetpoint(dLiftEncoder);
    sLiftLoop.setSetpoint(dLiftEncoder);
  
    if (b_cntlManipButtonStart) {
      m_Climb.setSelectedSensorPosition(0);
      s_Climb.setSelectedSensorPosition(0);
      d_Climb.setSelectedSensorPosition(0);
    }
    /*
    if (db_cntlManipTriggerLeft > 0.3 && b_cntlManipButtonLeft) {
      mLiftLoop.enable();
      sLiftLoop.enable();
      if (d_Climb.getSelectedSensorPosition(0) > -852500){
        d_Climb.set(0.3); 
      } else{
        d_Climb.set(0.1125);
      }
      m_Climb.set(-mLiftPID.output);
      s_Climb.set(-sLiftPID.output);

    } else if (db_cntlManipTriggerLeft > 0.3) {
      d_Climb.set(0.1125);
      m_Climb.set(db_cntlManipJoyRightY *.25);
      s_Climb.set(db_cntlManipJoyRightY *.25);
    } else {
      mLiftLoop.disable();
      sLiftLoop.disable();
      m_Climb.set(db_cntlManipJoyRightY *.25);
      s_Climb.set(db_cntlManipJoyRightY *.25);
      d_Climb.set(db_cntlManipJoyLeftY * .25);
      
    } */

    if(db_cntlDriverTriggerRight > 0.3){
      drive_Climb.set(db_cntlDriverTriggerRight * 0.25);
    } else if (db_cntlDriverTriggerLeft > 0.3){
      drive_Climb.set(db_cntlDriverTriggerLeft * -0.25);
    } else {
      drive_Climb.set(0);
    }

    SmartDashboard.putNumber("Driver Encoder Value", dLiftEncoder);
    SmartDashboard.putNumber("Master Encoder Value", mLiftEncoder);
    SmartDashboard.putNumber("Slave Encoder Value", sLiftEncoder);



    //Gather encoder position, post to smartDashboard. Chech to see if B is pressed to reset encoder.
    frontVoltage = frontIR.getAverageVoltage();
    SmartDashboard.putNumber("Front Voltage", frontVoltage);

    rearVoltage = rearIR.getAverageVoltage();
    SmartDashboard.putNumber("Rear Voltage", rearVoltage);

    encoderValue = -rearRight.getSelectedSensorPosition(0);
 
    //rotations = encoderValue/4000;
    //circumfrence = Math.PI*8;
    //distance = circumfrence * rotations;
    //SmartDashboard.putNumber("Rotations", rotations);
    //SmartDashboard.putNumber("Encoder Value", encoderValue);
    //SmartDashboard.putNumber("Distance", distance);
    //SmartDashboard.putNumber("Voltage", voltage);
    if (b_cntlDriverButtonX) {
        rearRight.setSelectedSensorPosition(0, 0, 0);
    }

    //SmartDashboard.putNumber("Encoder Distance:" , testCoder.getDistance());   
    //SmartDashboard.putNumber("Encoder Value:" , testCoder.get());  
    
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
   

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
    limitTopFrontLeft = topLimitSwitchFrontLeft.get();
    limitTopRear = topLimitSwitchRear.get();
    limitBottomFrontLeft = bottomLimitSwitchFrontLeft.get();
    limitBottomFrontRight = bottomLimitSwitchFrontRight.get();
    limitBottomRear = bottomLimitSwitchRear.get();
    powerFR = m_Climb.get();
    powerFL = s_Climb.get();
    powerR = d_Climb.get();
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightSkew", actualSkew);
    SmartDashboard.putNumber("PressureSensPress", PSPress);
    SmartDashboard.putBoolean("LimitTFR", limitTopFrontRight);
    SmartDashboard.putBoolean("LimitTFL", limitTopFrontLeft);
    SmartDashboard.putBoolean("LimitTR", limitTopRear);
    SmartDashboard.putBoolean("LimitBFR", limitBottomFrontRight);
    SmartDashboard.putBoolean("LimitBFL", limitBottomFrontLeft);
    SmartDashboard.putBoolean("LimitBR", limitBottomRear);
    //SmartDashboard.putNumber("Amps", powerFR);
    SmartDashboard.putNumber("endGameState", endGameState);

    //Button Mapping 
    db_cntlDriverTriggerRight = cntlDriver.getRawAxis(RobotMap.xBoxTriggerRightChannel);
    db_cntlManipTriggerRight = cntlManipulator.getRawAxis(RobotMap.xBoxTriggerRightChannel);
    db_cntlManipTriggerLeft = cntlManipulator.getRawAxis(RobotMap.xBoxTriggerLeftChannel);
    db_cntlDriverTriggerLeft = cntlDriver.getRawAxis(RobotMap.xBoxTriggerLeftChannel);
    b_cntlDriverButtonA = cntlDriver.getRawButton(RobotMap.xBoxButtonAChannel);
    b_cntlManipButtonA = cntlManipulator.getRawButton(RobotMap.xBoxButtonAChannel);
    b_cntlDriverButtonB = cntlDriver.getRawButton(RobotMap.xBoxButtonBChannel);
    b_cntlManipButtonB = cntlManipulator.getRawButton(RobotMap.xBoxButtonBChannel);
    b_cntlDriverButtonX = cntlDriver.getRawButton(RobotMap.xBoxButtonXChannel);
    b_cntlManipButtonX = cntlManipulator.getRawButton(RobotMap.xBoxButtonXChannel);
    b_cntlManipButtonY = cntlManipulator.getRawButton(RobotMap.xBoxButtonYChannel);
    b_cntlDriverButtonRight = cntlDriver.getRawButton(RobotMap.xBoxButtonRightChannel);
    b_cntlManipButtonRight = cntlManipulator.getRawButton(RobotMap.xBoxButtonRightChannel);
    b_cntlManipButtonLeft = cntlManipulator.getRawButton(RobotMap.xBoxButtonLeftChannel);
    b_cntlDriverButtonLeft = cntlDriver.getRawButton(RobotMap.xBoxButtonLeftChannel);
    db_cntlDriverJoyLeftY = cntlDriver.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    db_cntlManipJoyLeftY = cntlManipulator.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    db_cntlEndJoyLeftY = cntlEndGame.getRawAxis(RobotMap.xBoxLeftStickYChannel);
    db_cntlDriverJoyLeftX = cntlDriver.getRawAxis(RobotMap.xBoxLeftStickXChannel);
    db_cntlDriverJoyRightX = cntlDriver.getRawAxis(RobotMap.xBoxRightStickXChannel);
    db_cntlManipJoyRightX = cntlManipulator.getRawAxis(RobotMap.xBoxRightStickXChannel);
    db_cntlEndJoyRightX = cntlEndGame.getRawAxis(RobotMap.xBoxRightStickXChannel);
    db_cntlManipJoyRightY = cntlManipulator.getRawAxis(RobotMap.xBoxRightStickYChannel);
    db_cntlEndJoyRightY = cntlEndGame.getRawAxis(RobotMap.xBoxRightStickYChannel);
    db_cntlDriverJoyRightY = cntlDriver.getRawAxis(RobotMap.xBoxRightStickYChannel);
    b_cntlManipButtonStart = cntlDriver.getRawButton(RobotMap.xBoxButtonStartChannel);
    b_cntlDriverBackButton = cntlDriver.getRawButton(RobotMap.xBoxBackButtonChannel);
    b_cntlDriverButtonY = cntlDriver.getRawButton(RobotMap.xBoxButtonYChannel);
    //Read Values on Smartdashboard
    //SmartDashboard.putNumber("db_cntlDriverJoyLeftX", db_cntlDriverJoyLeftX);
    //SmartDashboard.putNumber("db_cntlDriverJoyLeftY", db_cntlDriverJoyLeftY);
    //SmartDashboard.putNumber("db_cntlDriverJoyRightX", db_cntlDriverJoyRightX);
    //SmartDashboard.putNumber("timer", n_ledColorCode);
    //SmartDashboard.putNumber("Delta Time",1/db_deltaTime);
    //SmartDashboard.putNumber("Smooth Time", 1/timerSmoother.getAverage());
    
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
    

    //Controlling PID Loop
    if (b_cntlDriverButtonA){
      visionLoop.enable();
      strafeLoop.enable();
      forwardLoop.enable();
      robotDrive.driveCartesian(strafeOutput.outputX, -forwardOutput.outputY, outputSkew.output, 0.0);
    }else{
      visionLoop.disable();
      strafeLoop.disable();
      forwardLoop.disable();
      encoderLoop.disable();
      robotDrive.driveCartesian(-db_cntlDriverJoyLeftX ,db_cntlDriverJoyLeftY,-db_cntlDriverJoyRightX , 0.0);
    }
  

    //Checking if reflective tape area is less and change LED lights
    if(y <= -5)
    {
      //Lower Hatches
      if(area >= 4.0 && x > -4 && x < 4)
      {
        Leds.sendCode(1);
      }  
      else
      {
        Leds.sendCode(11);
      }
    }
    else if(y >= -3)
    {
      //Ball Hatches
      if(area >= 2.5 && x > -3.5 && x < 3.5)
      {
        Leds.sendCode(1);
      }
      else
      {
        Leds.sendCode(11);
      }
    }
    else 
    {
      Leds.sendCode(11);
    }

    //SmartDashboard.putNumber("IntakeSM", intakeMachine);
    //Ball Intake State Machine
    if(intakeMachine == intake_default)
    {
      //State = 0(default)
      hingeSolenoid.set(Value.kReverse);
      ballIntake.set(0);
      elevatorSolenoid.set(Value.kForward);
      //Leds.sendCode(10);
      if(b_cntlManipButtonX)
      {
        intakeMachine = intake_downTake;
      }
      else if(b_cntlManipButtonB)
      {
        db_shootTimer = timerSystem.get() + 0.85;
        intakeMachine = intake_downShoot;
      }
      else if(b_cntlManipButtonY)
      {
        intakeMachine = intake_upShoot;
      }
      else if(b_cntlManipButtonA)
      {
        intakeMachine = intake_upTake;
      }
      else if (b_cntlManipButtonRight && PSPress >= 40)
      {
        db_cargoTimer = timerSystem.get() + 1.2;
        intakeMachine = intake_cargo;
      }
      else if(b_cntlManipButtonLeft)
      {
        intakeMachine = intake_elevatorUp;
      }
    } 
    else if(intakeMachine == intake_downTake)
    {
      //State = 1
      hingeSolenoid.set(Value.kForward);
      ballIntake.set(-0.4);
      if(b_cntlManipButtonB)
      {
          intakeMachine = intake_downShootB;
      }
      if(!b_cntlManipButtonX)
      {
        intakeMachine = intake_default;
      }
    }
    else if(intakeMachine == intake_upShoot)
    {
      //State = 2
      hingeSolenoid.set(Value.kReverse);
      ballIntake.set(0.4);
     
      if(!b_cntlManipButtonY)
      {
        intakeMachine = intake_default;
      }
    }
    else if(intakeMachine == intake_upTake)
    {
      //State =  3
      hingeSolenoid.set(Value.kReverse);
      ballIntake.set(-0.4);
      if(!b_cntlManipButtonA)
      {
        intakeMachine = intake_default;
      }
    }
    else if(intakeMachine == intake_downShoot)
    {
      //State = 4
      hingeSolenoid.set(Value.kForward);
        
      if(db_shootTimer <= timerSystem.get())
      {
        intakeMachine = intake_shoot;
      }
    
      if(b_cntlManipButtonX)
      {
        intakeMachine = intake_downTakeX;
      }
      if(!b_cntlManipButtonB)
      {
        intakeMachine = intake_default;
      }
    }
    else if(intakeMachine == intake_downTakeX)
    {
      //State = 6
      hingeSolenoid.set(Value.kForward);
      ballIntake.set(-0.4);
      if(!b_cntlManipButtonB)
      {
          intakeMachine = intake_downShoot;
      }
      if(!b_cntlManipButtonX)
      {
        intakeMachine = intake_downTake;
      }
    }
    else if(intakeMachine == intake_downShootB)
    {
      //State = 5
      hingeSolenoid.set(Value.kForward);
      ballIntake.set(0.54);
      if(!b_cntlManipButtonX)
      {
        intakeMachine = intake_downShoot;
      }
      if(!b_cntlManipButtonB)
      {
        intakeMachine = intake_downTake;
      }
    }
    else if(intakeMachine == intake_shoot)
    {
      ballIntake.set(0.54);
      if(b_cntlManipButtonX)
      {
        intakeMachine = intake_downTakeX;
      }
      if(!b_cntlManipButtonB)
      {
        intakeMachine = intake_default;
      }
     
    }
    else if (intakeMachine == intake_cargo)
    {
      hingeSolenoid.set(Value.kReverse);
      elevatorSolenoid.set(Value.kReverse);
      if(db_cargoTimer <= timerSystem.get())
      {
        intakeMachine = intake_cargoShoot;
      }
    }
    else if(intakeMachine == intake_cargoShoot)
    {
      ballIntake.set(0.54);
      if(!b_cntlManipButtonRight)
      {
        intakeMachine = intake_default;
      }
    }
    else if(intakeMachine == intake_elevatorUp)
    {
      elevatorSolenoid.set(Value.kReverse);
      if(!b_cntlManipButtonLeft)
      {
        intakeMachine = intake_default;
      }
    }
   
    if (db_cntlManipTriggerRight >= 0.3){
      hatchSolenoid.set(Value.kForward);
    }  else{
      hatchSolenoid.set(Value.kReverse);
    }

  //Leds Test
  if(b_cntlDriverButtonY)
  {
    Leds.sendCode(5);
  }

  
  boolean buttonY = b_cntlDriverButtonRight;
  //endgame state machine
  if (buttonY && endGameState == 0){
    frontRight.setSelectedSensorPosition(0);
    m_Climb.setSelectedSensorPosition(0);
    s_Climb.setSelectedSensorPosition(0);
    d_Climb.setSelectedSensorPosition(0);
    drive_Climb.setSelectedSensorPosition(0);
    endGameState = 2;
  } else if(buttonY && endGameState == 1){ //Use to back away from Hab
    //driveTrain.set(-0.5);
    Leds.sendCode(10);
    if (frontRight.getSelectedSensorPosition() <= -4000){
    endGameState = 2;
    }
  } else if(buttonY && endGameState == 2){
    //driveTrain.set(0.0);
    //frontRight.setSelectedSensorPosition(0);
    mLiftLoop.setSetpoint(dLiftEncoder);
    sLiftLoop.setSetpoint(dLiftEncoder);

    mLiftLoop.enable();
    sLiftLoop.enable();
    
    d_Climb.set(liftSpeed);

    m_Climb.set(-mLiftPID.output);
    s_Climb.set(-sLiftPID.output);
    if (b_cntlDriverButtonLeft){
      if (d_Climb.getSelectedSensorPosition(0) <= -300000)
      endGameState = 3;
    } else{
    if (d_Climb.getSelectedSensorPosition(0) <= -882500){ 
        endGameState = 3;
    }
  }
  } else if(buttonY && endGameState == 3){
    mLiftLoop.disable();
    sLiftLoop.disable();
    m_Climb.set(0.1125);
    s_Climb.set(0.1125);
    d_Climb.set(0.1125);
    drive_Climb.set(0.5);
    if(frontIR.getVoltage() >= 2.3){
        endGameState = 4;
      
    }
  } else if(buttonY && endGameState == 4){
      d_Climb.set(0.1125);
      drive_Climb.set(0.0);
      m_Climb.set(frontRetractSpeed);
      s_Climb.set(frontRetractSpeed);
      if(limitBottomFrontLeft && limitBottomFrontRight){
          endGameState = 5;
      }
  } else if(buttonY && endGameState == 5){
    m_Climb.set(0.0);
    s_Climb.set(0.0);
    d_Climb.set(0.1);
    m_Climb.setSelectedSensorPosition(0); 
    //driveTrain.set(0.5);
    drive_Climb.set(0.5);
    if(rearIR.getVoltage() >= 1.9){
        endGameState = 6;
    } 
  } else if(buttonY && endGameState == 6){
    //driveTrain.set(0.0);
    drive_Climb.set(0.0);
    d_Climb.set(backRetractSpeedSlow);
    if (limitBottomRear){
    d_Climb.set(0.0);
    endGameState = 2000;
    } 
  } else if (endGameState == 7){
      mLiftLoop.setSetpoint(dLiftEncoder);
      sLiftLoop.setSetpoint(dLiftEncoder);
      mLiftLoop.enable();
      sLiftLoop.enable();
      m_Climb.set(-mLiftPID.output);
      s_Climb.set(-sLiftPID.output);
      d_Climb.set(-0.05);
      if(m_Climb.getSelectedSensorPosition() >= 0 && d_Climb.getSelectedSensorPosition() >= 0){
          endGameState = 9;
      }
  } else if(endGameState == 8){
      drive_Climb.set(0.0);
      m_Climb.set(0.0);
      s_Climb.set(0.0);
      d_Climb.set(0.0);
      //driveTrain.set(0.0);
  } else if(endGameState == 9){
      mLiftLoop.disable();
      sLiftLoop.disable();
      endGameState = 0; 
  } else if(!buttonY && endGameState == 1){
      endGameState = 0;
  } else if(!buttonY && endGameState == 2){
      endGameState = 7;
  } else if(!buttonY && endGameState == 3){
      endGameState = 8;
  } else if(!buttonY && endGameState == 4){
      endGameState = 8;
  } else if(!buttonY && endGameState == 5){
      endGameState = 8;
  } else if(!buttonY && endGameState == 6){
      endGameState = 8;
  } else{
      m_Climb.set(db_cntlManipJoyRightY *.25);
      s_Climb.set(db_cntlManipJoyRightY *.25);
      d_Climb.set(db_cntlManipJoyLeftY * .25);
  }
}

  @Override
  public void disabledInit()
  {
    Leds.sendCode(9);
    //SmartDashboard.putNumber("Im Disabled, Like You", 2);
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
