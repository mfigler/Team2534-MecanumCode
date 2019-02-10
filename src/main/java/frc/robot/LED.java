package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class LED
{
    private I2C arduinoBus;
    
    public LED(){
        this.arduinoBus = new I2C(I2C.Port.kOnboard, 8);
    }
    
    public boolean sendCode(int n_ledColorCode){
        this.arduinoBus.write(0, n_ledColorCode);
        return true;
    }
    
}
