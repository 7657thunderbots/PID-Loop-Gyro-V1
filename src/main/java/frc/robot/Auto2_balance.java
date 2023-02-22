 package frc.robot;
 import edu.wpi.first.wpilibj.Timer;
public class Auto2_balance{
    final double akP = 0.5;
     final double akI = 0.5;
  final double akD = 0.1;
  final double aiLimit = 5;
    public double setpoint=0;
    private double errorSum = 0;
    private double lastError=0;
     private double lastTimestamp = 0;
    public boolean onchargestation=false;
     private DriveTrain drivetrain;
     private Balancing balancing;
     private final Timer timer = new Timer();
    //private turnadjust turnadjust;
    private boolean placed = false;
  
     public Auto2_balance(){
    timer.reset();
    timer.start();
    double dsensorPosition=0;
     }
     public void Run_Auto2_balance(){
     //turnadjust.turnadjust_run();
         // if (timer.get()<2){
         // balancing.Speedvar=.2;
         if (placed==false){
         // get sensor position
        double dsensorPosition = drivetrain.getAverageEncoderDistance() ;
        // calculations
        double error = setpoint - dsensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < aiLimit) {
          errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;

        double outputSpeed = akP * error + akI * errorSum + akD * errorRate;

        // output to motors
        balancing.Speedvar = (-1*outputSpeed);


        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
        if (dsensorPosition<5){
          placed=true;
          errorSum = 0;
          lastTimestamp = 0;
          lastError = 0;

         }
      }
     
    else {
    balancing.BalancingRun();
   }
}
}
