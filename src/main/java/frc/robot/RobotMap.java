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
public static int xBoxDriverChannel = 0;
public static int xBoxManipulatorChannel = 1;
public static int xBoxButtonAChannel = 1;
public static int xBoxButtonBChannel = 2;
public static int xBoxButtonXChannel = 3;
public static int xBoxButtonYChannel = 4;
public static int xBoxButtonLeftChannel = 5;
public static int xBoxButtonRightChannel = 6;
public static int xBoxLeftStickXChannel = 0;
public static int xBoxLeftStickYChannel = 1;
public static int xBoxLeftTriggerChannel = 2;
public static int xBoxRightTriggerChannel = 3;
public static int xBoxRightStickXChannel = 4;
public static int xBoxRightStickYChannel = 5;

//Encoder
public static int encoderAChannel = 0;
public static int encoderBChannel = 1;

//Solenoid
public static int doubleSolenoidForwardChannel = 0;
public static int doubleSolenoidReverseChannel = 1;

//Elevator
public static int talonElevatorChannel = 5;
public static boolean talonElevatorReverse;
public static int limitSwitchChannel = 0;

//infrared
public static int infraredSensorChannel = 0;
}