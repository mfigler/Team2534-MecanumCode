package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PIDOutput;



public class PIDoutY implements PIDOutput{
    Robot robot;
    public double outputY;

    public PIDoutY(Robot r){
        robot = r;
    }
    public void pidWrite(double output){
    outputY = output; 
    //robot.m_robotDrive.driveCartesian(output, robot.db_cntlDriverJoyLeftY, robot.db_cntlDriverJoyRightX, 0.0);  
      
    }
}