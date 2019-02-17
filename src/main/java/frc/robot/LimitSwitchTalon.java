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
import com.ctre.phoenix.motorcontrol.can.*;

public class LimitSwitchTalon extends WPI_TalonSRX{
    DigitalInput topLimitSwitch;
    DigitalInput bottomLimitSwitch;

    public LimitSwitchTalon(int talonID, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch){
        super(talonID);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }

    public void set(double speed){
        if (topLimitSwitch.get()){
            if (speed > 0 && !this.getInverted()){
                speed = 0;
            } else if(speed < 0 && this.getInverted()){
                speed = 0;
            }
        } 
        
        if(bottomLimitSwitch.get()){
            if (speed < 0 && !this.getInverted()){
                speed = 0;
            } else if(speed > 0 && this.getInverted()){
                speed = 0;
            }
        }
        super.set(speed);
    }
}