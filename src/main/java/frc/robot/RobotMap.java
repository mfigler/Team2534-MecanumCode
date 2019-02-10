package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

public class RobotMap extends IterativeRobot {

//drive train
public static int talonFrontRightChannel;
public static boolean talonFrontRightReverse;
public static int talonRearRightChannel;
public static boolean talonRearRightReverse;
public static int talonFrontLeftChannel;
public static boolean talonFrontLeftReverse;
public static int talonRearLeftChannel;
public static boolean talonRearLeftReverse;





//Controller
public static int xBoxControllerChannel = 0;
public static int xBoxManipulatorControllerChannel = 1;
public static int xBoxButtonAChannel = 1;
public static int xBoxButtonBChannel = 2;
public static int xBoxButtonXChannel = 3;
public static int xBoxButtonYChannel = 4;
public static int xBoxButtonLeftChannel = 5;
public static int xBoxButtonRightChannel = 6;
public static int xBoxLeftStickXChannel = 0;
public static int xBoxLeftStickYChannel = 1;
public static int xBoxLeftTriggerChannel = 2;
public static int xBoxdb_cntlDriverTriggerRightChannel = 3;
public static int xBoxRightStickXChannel = 4;
public static int xBoxRightStickYChannel = 5;

//Encoder
public static int encoderAChannel = 0;
public static int encoderBChannel = 1;

//Solenoid
public static int hingeSolenoidForwardChannel = 0;
public static int hingeSolenoidReverseChannel = 1;
public static int hatchSolenoidForwardChannel = 2;
public static int hatchSolenoidReverseChannel = 3;
public static int elevatorSolenoidForwardChannel = 4;
public static int elevatorSolenoidReverseChannel = 5;

//Ball and Elevator
public static int talonBallIntakeChannel = 5;

//Endgame
public static int talonClimbAChannel = 6;
public static int talonClimbBChannel = 7;
public static int talonClimbCChannel = 8;
public static int talonClimbDriveChannel = 9;
}
