package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class LED
{
    private I2C arduinoBus;
    
    public LED(I2C.Port port, int deviceAddress){
        this.arduinoBus = new I2C(I2C.Port.kOnboard, 2);
    }
    
    public boolean sendCode(int LEDCode){
        this.arduinoBus.write(0, LEDCode);
        return true;
    }
    
}
