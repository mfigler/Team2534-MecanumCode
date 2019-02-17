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
    public WPI_TalonSRX climberFrontSlave;
    public WPI_TalonSRX climberRear;
    public WPI_TalonSRX climberDrive;
    public double joyLeftY;
    public double joyRightX;
    public double joyRightY;
    public boolean switchTopFR;
    public boolean switchTopFL;
    public boolean switchTopR;
    public boolean switchBotFR;
    public boolean switchBotFL;
    public boolean switchBotR;
    public PIDout liftFrontMaster;
    public PIDout liftFrontSlave;
    public PIDController liftFrontMasterLoop;
    public PIDController liftFrontSlaveLoop;
    protected int state = 0;
    SpeedControllerGroup driveTrain;


    public EndGame(){
     state = 0; 
    }

    public void init(){
        driveTrain = new SpeedControllerGroup(driveTrainFrontRight, driveTrainRearRight, driveTrainFrontLeft, driveTrainRearLeft);

    }

    //State 0 = idle
    //State 1 = back away from HAB
    //State 2 = Robot elevates upward(All three motors)
    //State 3 = drive until contact with HAB
    //State 4 = Stop drive; raise front legs
    //State 5 = Drive forward untill back leg against HAB
    //State 6 = raise back leg
    //State 7 = raise all legs 
    //State 8 = idle(can not go through other states)(cannot press A) 

    public void go(boolean buttonY){
      if (buttonY && state == 0){
        driveTrainFrontRight.setSelectedSensorPosition(0);
        climberFrontMaster.setSelectedSensorPosition(0);
        climberFrontSlave.setSelectedSensorPosition(0);
        climberRear.setSelectedSensorPosition(0);
        climberDrive.setSelectedSensorPosition(0);
        state = 2;
      } else if(buttonY && state == 1){ //Use to back away from Hab
        driveTrain.set(-0.5);
        if (driveTrainFrontRight.getSelectedSensorPosition() <= -4000){
        state = 2;
        }
      } else if(buttonY && state == 2){
        driveTrain.set(0.0);
        //driveTrainFrontRight.setSelectedSensorPosition(0);
        liftFrontMasterLoop.setSetpoint(climberRear.getSelectedSensorPosition(0));
        liftFrontSlaveLoop.setSetpoint(climberRear.getSelectedSensorPosition(0));

        liftFrontMasterLoop.enable();
        liftFrontSlaveLoop.enable();
        
        climberRear.set(0.3);

        climberFrontMaster.set(-liftFrontMaster.output);
        climberFrontSlave.set(-liftFrontSlave.output);

        
      

        if (climberRear.getSelectedSensorPosition(0) <= -852500){ 
            //state = 3;
            state = 7;
        } else if(switchTopFL && switchTopR){
            //state = 3;
            state = 7;
        } else if(switchTopFR && switchTopR){
            //state = 3;
            state = 7;
        }
      } else if(buttonY && state == 3){
        climberFrontMaster.set(0.1125);
        climberFrontSlave.set(0.1125);
        climberRear.set(0.1125);
        climberDrive.set(0.5);
        if(driveTrainFrontLeft.getSelectedSensorPosition() >= 3){
            state = 4;
        }
      } else if(buttonY && state == 4){
          climberDrive.set(0.0);
          climberFrontMaster.set(-0.3);
          if(climberFrontMaster.getSelectedSensorPosition() <= 0){
              state = 5;
          } else if(switchBotFL){
              state = 5;
          } else if(switchBotFR){
              state = 5;
          }
      } else if(buttonY && state == 5){
        climberFrontMaster.set(0.0);
        climberFrontMaster.setSelectedSensorPosition(0); 
        driveTrain.set(0.5);
        climberDrive.set(0.5);
        if(driveTrainRearRight.getSelectedSensorPosition() >= 3){
            state = 6;
        } 
      } else if(buttonY && state == 6){
        driveTrain.set(0.0);
        climberDrive.set(0.0);
        climberRear.set(-0.3);
        if (climberRear.getSelectedSensorPosition() <= 0){
        climberRear.set(0.0);
        state = 2000;
        } else if(switchBotR){
        climberRear.set(0.0);
        state = 2000;
        }
      } else if (state == 7){
          climberFrontMaster.set(0.05);
          climberRear.set(0.05);
          if(climberFrontMaster.getSelectedSensorPosition() <= 0 && climberRear.getSelectedSensorPosition() <= 0){
              state = 0;
          }
      } else if(state == 8){
          climberDrive.set(0.0);
          climberFrontMaster.set(0.0);
          climberRear.set(0.0);
          driveTrain.set(0.0);
      } else if(!buttonY && state == 1){
          state = 0;
      } else if(!buttonY && state == 2){
          state = 7;
      } else if(!buttonY && state == 3){
          state = 8;
      } else if(!buttonY && state == 4){
          state = 8;
      } else if(!buttonY && state == 5){
          state = 8;
      } else if(!buttonY && state == 6){
          state = 8;
      } else {
          /*climberFrontMaster.set(joyRightY);
          climberFrontSlave.set(joyRightX);
          climberRear.set(joyLeftY);*/
      }
    } 
}