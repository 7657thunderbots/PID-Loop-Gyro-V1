package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Pneumatics {
    public static int forwardChannel1 =0;
    public static int reverseChannel1 =1;
    public static int forwardChannel2 =15;
    public static int reverseChannel2 =14;

    private static final int PH_CAN_ID = 1;
    PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
    
    DoubleSolenoid m_doubleSolenoid = m_pH.makeDoubleSolenoid(forwardChannel1,reverseChannel1);
    DoubleSolenoid doubleSolenoid = m_pH.makeDoubleSolenoid(forwardChannel2,reverseChannel2);
    public Pneumatics(){  
    
    }

    public void Run_Pneumatics(){
        switch (m_doubleSolenoid.get()) {
            case kOff:
              SmartDashboard.putString("Get Solenoid", "kOff");
              break;
            case kForward:
              SmartDashboard.putString("Get Solenoid", "kForward");
              break;
            case kReverse:
              SmartDashboard.putString("Get Solenoid", "kReverse");
              break;
            default:
              SmartDashboard.putString("Get Solenoid", "N/A");
              break;
            }
            
            if (SmartDashboard.getBoolean("Enable Compressor Digital", false)) {
                SmartDashboard.putBoolean("Enable Compressor Digital", false);

            m_pH.enableCompressorDigital();
            }
            
            if (SmartDashboard.getBoolean("Disable Compressor", false)) {
                SmartDashboard.putBoolean("Disable Compressor", false);
          
              m_pH.disableCompressor();
              }
    

                switch (doubleSolenoid.get()) {
                case kOff:
                  SmartDashboard.putString("Get Solenoid", "kOff");
                  break;
                case kForward:
                  SmartDashboard.putString("Get Solenoid", "kForward");
                  break;
                case kReverse:
                  SmartDashboard.putString("Get Solenoid", "kReverse");
                  break;
                default:
                  SmartDashboard.putString("Get Solenoid", "N/A");
                  break;
                } 
    }
}
