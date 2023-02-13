package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Wrist {
    private final double wkP = 0.05;
    private final double wkI = 0.05;
   private final double wkD = 0.01;
    private final double wiLimit = 1;
    private double werrorSum = 0;
   private double wlastError=0;
    public CANSparkMax Wristmax;
    public RelativeEncoder Wristencoder;
    private double lastTimestamp = 0;
    public double wsetpoint =0;
public Wrist() {
    
    Wristmax = new CANSparkMax(9, MotorType.kBrushless);
    
}
public void WristRun(){
    
    double werror = wsetpoint-Wristencoder.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(werror) < wiLimit) {
      werrorSum += werror * dt;
    }

    double werrorRate = (werror - wlastError) / dt;

    double woutput = wkP * werror + wkI * werrorSum + wkD * werrorRate;

    // output to motors
  
 Wristmax.set(woutput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    wlastError = werror; 
}
}
