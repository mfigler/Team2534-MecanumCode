package frc.robot;
// import edu.wpi.first.wpilibj.IterativeRobot;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.XboxController;
// import com.ctre.phoenix.motorcontrol.can.*;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.PIDController;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.I2C.Port; 
// import edu.wpi.first.wpilibj.command.Subsystem;

// /*public class PixyI2C {
//     PixyPacket values;
//     I2C pixy;
//     Port port = Port.kOnboard;
//     PixyException pExc;
//     String print;

//     public PixyI2C
//     {
//         pixy = new I2C(port, 0x54);
//         packets = new PixyPacket[7];
//         pExc = new PixyException(print);
//         values = new PixyPacket();
//     }

//     public PixyI2C(I2C argPixy, PixyPacket[] argPixyPacket, PixyException argPixyException, PixyPacket argValues)
//     {
//         pixy = argPixy;
//         packets = argPixyPacket;
//         pExc = argPixyException;
//         values = argValues; 
//     }

//     public int cvt(byte upper, byte lower)
//     {
//         return (((int)upper & 0xff) << 8) | ((int)lower & 0xff);
//     }

//     public PixyPacket readPacket(int Signature) throws PixyException
//     {
//         int Checksum;
//         int Sig;
//         byte[] rawData = new byte[32];
//         SmartDashboard.putString("rawData", rawData[0]+ " " + rawData[1] + " " + rawData[15] + " " + rawData[31]);
//         try
//         {
//             pixy.readOnly(rawData, 32);
//         }
//         catch(RuntimeException e)
//         {
//             SmartDashboard.putString("Pixy RuntimeException", "Error");
//         }
//         if(rawData.length < 32)
//         {
//             System.out.println("byte array length is broken");
//             return null;
//         }
    
//     }


// }