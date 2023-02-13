package frc.robot;

public class Auto2_balance{
    final double akP = 0.5;
    final double akI = 0.5;
    final double akD = 0.1;
    final double aiLimit = 5;
    public double setpoint=0;
    public Auto2_balance(){

    }
    public void Run_Auto2_balance(){
         if (onchargestation==false){
          setpoint = 5;
    
        

        // get sensor position
        double sensorPosition = leftParent.getSelectedSensorPosition() * kDriveTick2Feet;

        // calculations
        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < aiLimit) {
          errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;

        double outputSpeed = akP * error + akI * errorSum + akD * errorRate;

        // output to motors
        Speedvar=outputSpeed;


        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
        if (sensorPosition==5){
          onchargestation=true;
          errorSum = 0;
          lastTimestamp = 0;
          lastError = 0;

        }
    }
    }
}