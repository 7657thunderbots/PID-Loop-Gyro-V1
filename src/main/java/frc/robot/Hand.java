package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Hand {
    private final double hkP = 0.05;
    private final double hkI = 0.05;
   private final double hkD = 0.01;
    private final double hiLimit = 1;
    private double herrorSum = 0;
   private double hlastError=0;
    public CANSparkMax handmax;
    public RelativeEncoder handencoder;
    private double lastTimestamp = 0;
    public double hsetpoint =0;
public Hand() {
    
    handmax = new CANSparkMax(9, MotorType.kBrushless);
    
}
public void HandRun(){
    
    double herror = hsetpoint-handencoder.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(herror) < hiLimit) {
      herrorSum += herror * dt;
    }

    double herrorRate = (herror - hlastError) / dt;

    double houtput = hkP * herror + hkI * herrorSum + hkD * herrorRate;

    // output to motors
  
 handmax.set(houtput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    hlastError = herror; 
}
}