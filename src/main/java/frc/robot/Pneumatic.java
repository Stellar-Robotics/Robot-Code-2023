package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

public class Pneumatic {

    static PneumaticHub phub = new PneumaticHub(35);

    static Compressor compressor = new Compressor(35, PneumaticsModuleType.REVPH);

    static Solenoid Solenoid = phub.makeSolenoid(1);

    public void corePneumatic(boolean controllerbind) {
        
    }
    
}
