package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Balancing {
    private DriveTrain drivetrain; 
    
    public double Speedvar=0.0;
    
    public boolean chargestationbalance= false;
   
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    double berror=0;
    double errorRate=0;
   
    final double bkP = -0.008;
    final double bkI = -0.005;
    final double bkD = -0.001;
    final double biLimit = 3;

    public Balancing() {   
    
    }   


    public void BalancingRun(){
        if (drivetrain.m_gyro.getAngle()<3 && drivetrain.m_gyro.getAngle()>-3){
            chargestationbalance=true;
            drivetrain.setbrake(true);
            Speedvar=0;}
        
            else {
                setpoint = 0;
        
                // get sensor position
                Double sensorPosition = drivetrain.m_gyro.getAngle();
        
                // calculations
                berror = setpoint - sensorPosition;
                double dt = Timer.getFPGATimestamp() - lastTimestamp;
        
                if (Math.abs(berror) < biLimit) {
                errorSum += berror * dt;
                }
        
                errorRate = (berror - lastError) / dt;
        
                Double outputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
        
                // output to motors
                Speedvar=(-1*outputSpeed);
        
                // update last- variables
                lastTimestamp = Timer.getFPGATimestamp();
                lastError = berror;
        }
                if (Speedvar>.2){
                    Speedvar=.2;
                }
                
                if (Speedvar<-.2){
                        Speedvar=-.2;
                }

        

    }
}
