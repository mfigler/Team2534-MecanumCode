package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PIDSourceType;

public class EncoderSource implements PIDSource {
    Robot robot;

    public EncoderSource(Robot r){
        robot = r;
    }

    public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }

    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    public double pidGet(){
        return robot.distance;
    }
}