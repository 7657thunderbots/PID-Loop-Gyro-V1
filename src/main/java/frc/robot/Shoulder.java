package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Shoulder {
    private final double SkP = 0.05;
    private final double SkI = 0.05;
   private final double SkD = 0.01;
    private final double SiLimit = 1;
    private double SerrorSum = 0;
   private double SlastError=0;
    public CANSparkMax Shouldermax;
    public RelativeEncoder Shoulderencoder;
    private double lastTimestamp = 0;
    public double Ssetpoint =0;
public Shoulder() {
    
    Shouldermax = new CANSparkMax(9, MotorType.kBrushless);
    
}
public void ShoulderRun(){
    
    double Serror = Ssetpoint-Shoulderencoder.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(Serror) < SiLimit) {
      SerrorSum += Serror * dt;
    }

    double SerrorRate = (Serror - SlastError) / dt;

    double Soutput = SkP * Serror + SkI * SerrorSum + SkD * SerrorRate;

    // output to motors
  
    Shouldermax.set(Soutput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    SlastError = Serror; 
}
}
