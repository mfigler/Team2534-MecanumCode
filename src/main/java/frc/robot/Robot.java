/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PIDController;


/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends IterativeRobot {
  private static final int kFrontLeftChannel = 3;
  private static final int kRearLeftChannel = 4;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 2;

  private static final int kJoystickChannel = 0;

  MecanumDrive m_robotDrive;
  WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
  WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
  WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
  WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);
  int deadzone = 20;
  double JoyY = 0;
  double JoyX = 0;
  double JoyZ = 0;
  boolean JoyA;
  double Target = 0.5;
  double CorrectSpeed = 0.2;
  XboxController controller = new XboxController(0);
  PIDout output = new PIDout(this); //instantiate output of PIDout
  PIDoutX strafeOutput = new PIDoutX(this);
  CameraSource limelight = new CameraSource(this); 
  CameraSourceX limelightX = new CameraSourceX(this);
  PIDController visionLoop = new PIDController(0.03, 0.0, 0.0, limelight, output);
  PIDController strafeLoop = new PIDController(0.1, 0.0, 0.0, limelightX, strafeOutput);
  public double CameraValue = 0;
  public double StrafeValue = 0;
  double actualSkew;
  

  @Override
  public void robotInit() {
    frontLeft.setInverted(true);
    rearLeft.setInverted(false);
    frontRight.setInverted(true);
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    strafeLoop.setSetpoint(0.0);
    strafeLoop.setOutputRange(-1,1);
    strafeLoop.setInputRange(-25.0, 25.0);
    visionLoop.setSetpoint(0.0);
    visionLoop.setOutputRange(-1,1);
    visionLoop.setInputRange(-25.0, 25.0);
  


    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
   
  }

  @Override
  public void teleopPeriodic() {
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
    if (skew > -45){
      actualSkew = Math.abs(skew);
    }else {
      actualSkew = Math.abs(skew) - 90;
    }
    StrafeValue = actualSkew;
   

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightSkew", actualSkew);

    JoyA = controller.getRawButton(1);
    
    JoyY = controller.getRawAxis(1);
    JoyX = controller.getRawAxis(0);
    JoyZ = controller.getRawAxis(4);
    if (Math.abs(JoyY) < (deadzone/100)) {
      JoyY = 0;
    }
    if (Math.abs(JoyX) < (deadzone/100)) {
      JoyX = 0;
    }
    if (Math.abs(JoyZ) < (deadzone/100)) {
      JoyZ = 0;
    }
    if (JoyA){
      visionLoop.enable();
      strafeLoop.enable();
      m_robotDrive.driveCartesian(strafeOutput.outputX, JoyY, output.outputSkew, 0.0);
    } else {
      visionLoop.disable();
      strafeLoop.disable();
      m_robotDrive.driveCartesian(JoyX, JoyY, JoyZ, 0.0);
    }

  }
}
