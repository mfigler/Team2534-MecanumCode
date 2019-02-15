package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.IMotorController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.DemandType;

public class EndGame {
    public WPI_TalonSRX driveTrainFrontRight;
    public WPI_TalonSRX driveTrainFrontLeft;
    public WPI_TalonSRX driveTrainRearRight;
    public WPI_TalonSRX driveTrainRearLeft;
    public WPI_TalonSRX climberFrontMaster;
    public WPI_TalonSRX climberRear;
    public WPI_TalonSRX climberDrive;
    protected int state = 0;
    SpeedControllerGroup driveTrain;

    public EndGame(){
     state = 0; 
    }

    public void init(){
        driveTrain = new SpeedControllerGroup(driveTrainFrontRight, driveTrainRearRight, driveTrainFrontLeft, driveTrainRearLeft);
    }

    public void go(boolean buttonA){
      if (buttonA && state == 0){
        driveTrainFrontRight.setSelectedSensorPosition(0);
        climberFrontMaster.setSelectedSensorPosition(0);
        climberRear.setSelectedSensorPosition(0);
        climberDrive.setSelectedSensorPosition(0);
        state = 1;
      } else if(buttonA && state == 1){
        driveTrain.set(-0.5);
        if (driveTrainFrontRight.getSelectedSensorPosition() <= -4000){
        state = 2;
        }
      } else if(buttonA && state == 2){
        driveTrain.set(0.0);
        driveTrainFrontRight.setSelectedSensorPosition(0);
        climberFrontMaster.set(0.3);
        climberRear.set(0.3);
        if (climberFrontMaster.getSelectedSensorPosition() >= 1000 && climberRear.getSelectedSensorPosition() >= 1000){
            state = 3;
        }
      } else if(buttonA && state == 3){
        climberFrontMaster.set(0.0);
        climberRear.set(0.0);
        climberDrive.set(0.5);
        if(driveTrainFrontLeft.getSelectedSensorPosition() >= 3){
            state = 4;
        }
      } else if(buttonA && state == 4){
          climberDrive.set(0.0);
          climberFrontMaster.set(-0.3);
          if(climberFrontMaster.getSelectedSensorPosition() <= 0){
              state = 5;
          }
      } else if(buttonA && state == 5){
        climberFrontMaster.set(0.0);
        climberFrontMaster.setSelectedSensorPosition(0); 
        driveTrain.set(0.5);
        climberDrive.set(0.5);
        if(driveTrainRearRight.getSelectedSensorPosition() >= 3){
            state = 6;
        } 
      } else if(buttonA && state == 6){
        driveTrain.set(0.0);
        climberDrive.set(0.0);
        climberRear.set(-0.3);
        if (climberRear.getSelectedSensorPosition() <= 0){
        climberRear.set(0.0);
        state = 2000;
        }  
      } else if (state == 7){
          climberFrontMaster.set(-0.3);
          climberRear.set(-0.3);
          if(climberFrontMaster.getSelectedSensorPosition() <= 0 && climberRear.getSelectedSensorPosition() <= 0){
              state = 0;
          }
      } else if(state == 8){
          climberDrive.set(0.0);
          climberFrontMaster.set(0.0);
          climberRear.set(0.0);
          driveTrain.set(0.0);
      } else if(!buttonA && state == 1){
          state = 0;
      } else if(!buttonA && state == 2){
          state = 7;
      } else if(!buttonA && state == 3){
          state = 8;
      } else if(!buttonA && state == 4){
          state = 8;
      } else if(!buttonA && state == 5){
          state = 8;
      } else if(!buttonA && state == 6){
          state = 8;
      }
    }
}