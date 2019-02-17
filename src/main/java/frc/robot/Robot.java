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
  LimitSwitchTalon m_Climb;
  LimitSwitchTalon s_Climb;
  LimitSwitchTalon d_Climb;
  WPI_TalonSRX drive_Climb;
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
  double db_cntlManipJoyRightY = 0;
  double db_cntlEndJoyRightY = 0;
  double db_cntlDriverTriggerRight = 0;
  double db_cntlManipTriggerRight = 0;
  double db_cntlManipTriggerLeft = 0;
  double db_cntlDriverJoyRightY = 0;
  double db_cntlDriverTriggerLeft = 0;
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
  int intakeMachine = intake_default;
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
  PIDController mLiftLoop = new PIDController(0.02, 0.0, 0.02, mLift , mLiftPID);
  PIDController sLiftLoop = new PIDController(0.02, 0.0, 0.0, sLift , sLiftPID);

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
   
    endGame.driveTrainFrontRight = frontRight;
    endGame.driveTrainFrontLeft = frontLeft;
    endGame.driveTrainRearRight = rearRight;
    endGame.driveTrainRearLeft = rearLeft;
    endGame.climberFrontMaster = m_Climb;
    endGame.climberFrontSlave = s_Climb;
    endGame.climberRear = d_Climb;
    endGame.climberDrive = drive_Climb;
    endGame.joyRightX = db_cntlEndJoyRightX;
    endGame.joyRightY = db_cntlEndJoyRightY;
    endGame.joyLeftY = db_cntlEndJoyLeftY;
    endGame.switchTopFL = limitTopFrontLeft;
    endGame.switchTopFR = limitTopFrontRight;
    endGame.switchTopR = limitTopRear;
    endGame.switchBotFL = limitTopFrontLeft;
    endGame.switchBotFR = limitTopFrontRight;
    endGame.switchBotR = limitTopRear;
    endGame.liftFrontMaster = mLiftPID;
    endGame.liftFrontSlave = sLiftPID;
    endGame.liftFrontMasterLoop = mLiftLoop;
    endGame.liftFrontSlaveLoop = sLiftLoop;

    endGame.init();
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

    mLiftLoop.setOutputRange(-0.3, -0.08);
    mLiftLoop.setInputRange(-900000,0);

    sLiftLoop.setOutputRange(-0.3, -0.08);
    sLiftLoop.setInputRange(-900000,0);

    }
  @Override
  public void teleopPeriodic() {

    endGame.driveTrainFrontRight = frontRight;
    endGame.driveTrainFrontLeft = frontLeft;
    endGame.driveTrainRearRight = rearRight;
    endGame.driveTrainRearLeft = rearLeft;
    endGame.climberFrontMaster = m_Climb;
    endGame.climberFrontSlave = s_Climb;
    endGame.climberRear = d_Climb;
    endGame.climberDrive = drive_Climb;
    endGame.joyRightX = db_cntlEndJoyRightX;
    endGame.joyRightY = db_cntlEndJoyRightY;
    endGame.joyLeftY = db_cntlEndJoyLeftY;
    endGame.switchTopFL = limitTopFrontLeft;
    endGame.switchTopFR = limitTopFrontRight;
    endGame.switchTopR = limitTopRear;
    endGame.switchBotFL = limitTopFrontLeft;
    endGame.switchBotFR = limitTopFrontRight;
    endGame.switchBotR = limitTopRear;
    endGame.liftFrontMaster = mLiftPID;
    endGame.liftFrontSlave = sLiftPID;
    endGame.liftFrontMasterLoop = mLiftLoop;
    endGame.liftFrontSlaveLoop = sLiftLoop;

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
      //Leds.sendCode()  
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
      
    }

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
    SmartDashboard.putNumber("State", endGame.state);

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
      robotDrive.driveCartesian(-strafeOutput.outputX, /*-forwardOutput.outputY*/0, /*outputSkew.output*/0, 0.0);
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

    SmartDashboard.putNumber("IntakeSM", intakeMachine);
    //Ball Intake State Machine
    if(intakeMachine == intake_default)
    {
      //State = 0(default)
      hingeSolenoid.set(Value.kReverse);
      ballIntake.set(0);
      Leds.sendCode(10);
      if(b_cntlManipButtonX)
      {
        intakeMachine = intake_downTake;
      }
      else if(b_cntlManipButtonB)
      {
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
      ballIntake.set(0.54);
     
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
   
    if (db_cntlManipTriggerRight >= 0.3){
      hatchSolenoid.set(Value.kForward);
    }  else{
      hatchSolenoid.set(Value.kReverse);
    }
  

  
  //Elevator 
  if (b_cntlManipButtonRight){
    elevatorSolenoid.set(Value.kReverse);
  } else{
    elevatorSolenoid.set(Value.kForward);
  }

  //Leds Test
  if(b_cntlDriverBackButton)
  {
    Leds.sendCode(5);
  }
  endGame.go(b_cntlDriverButtonRight);
}

  public void disablePeriodic()
  {
    Leds.sendCode(9);
    SmartDashboard.putNumber("Im Disabled, Like You", 2);
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
