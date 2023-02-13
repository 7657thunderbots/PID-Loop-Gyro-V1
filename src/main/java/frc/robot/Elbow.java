package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Elbow {
    private final double EkP = 0.05;
    private final double EkI = 0.05;
   private final double EkD = 0.01;
    private final double EiLimit = 1;
    private double EerrorSum = 0;
   private double ElastError=0;
    public CANSparkMax Elbowmax;
    public RelativeEncoder Elbowencoder;
    private double lastTimestamp = 0;
    public double Esetpoint =0;
    private Pneumatics pneumatics;
public Elbow() {
    pneumatics = new Pneumatics();
    Elbowmax = new CANSparkMax(9, MotorType.kBrushless);
    
}
public void ElbowRun(){
   if(Math.abs(Esetpoint-Elbowencoder.getPosition())<1) {
    Elbowmax.set(0);
   }
   else if (Math.abs(Esetpoint-Elbowencoder.getPosition())<1){

    double Eerror = Esetpoint-Elbowencoder.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(Eerror) < EiLimit) {
      EerrorSum += Eerror * dt;
    }

    double EerrorRate = (Eerror - ElastError) / dt;

    double houtput = EkP * Eerror + EkI * EerrorSum + EkD * EerrorRate;

    // output to motors
  if (houtput >= 0){
    pneumatics.m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  else{
    pneumatics.m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
    Elbowmax.set(houtput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    ElastError = Eerror; 
}
}
}
