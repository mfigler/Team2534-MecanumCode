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

public class CameraSourceY implements PIDSource {
    Robot robot;

    public CameraSourceY(Robot r){
        robot = r;
    }

    public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }

    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    public double pidGet(){
        return robot.ForwardValue;
    }
}