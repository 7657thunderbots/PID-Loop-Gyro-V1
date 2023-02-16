package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.PowerDistribution;

public class SmartDashboards {
   
   private final PowerDistribution m_pdp = new PowerDistribution();
    DoubleLogEntry myDoubleLog;

    private DriveTrain drivetrain;
    
    private Hand Hand;

    private Wrist wrist;

    private Elbow elbow;

    private Shoulder shoulder;

    private Balancing balancing;

    private turnadjust turn;
    
    private Robot robot;

    private Pneumatics pneumatics;

    private color_sensor color;

   public SmartDashboards(){
      
   }

   public void SmartDashboard_run(){

    SmartDashboard.putBoolean("On charge Station", robot.onchargestation);
    SmartDashboard.putBoolean("charge station balance",balancing.chargestationbalance);
    SmartDashboard.putData("PDP", m_pdp);
    SmartDashboard.putNumber("tilt angle", drivetrain.m_gyro.getYComplementaryAngle());
    SmartDashboard.putNumber("voltage",m_pdp.getVoltage());
    SmartDashboard.putNumber("PDP current", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total energy", m_pdp.getTotalEnergy());
    SmartDashboard.putNumber("Total power", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(9));
    SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(8));
    SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(10));
    SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(11));
    SmartDashboard.putNumber("turn angle",drivetrain.m_gyro.getAngle());
    SmartDashboard.putNumber("Balancing Speed output",balancing.Speedvar);
    SmartDashboard.putNumber("error adjustment direction", turn.turnerror);
    SmartDashboard.putNumber("X acceleration",drivetrain.m_gyro.getAccelX());
    SmartDashboard.putNumber("Y acceleration",drivetrain.m_gyro.getAccelY());
    SmartDashboard.putNumber("Z acceleration",drivetrain.m_gyro.getAccelZ());
    SmartDashboard.putNumber("Gyro Rate",drivetrain.m_gyro.getRate());
    SmartDashboard.putNumber("filtered x acceleration Angle",drivetrain.m_gyro.getXFilteredAccelAngle() );
    SmartDashboard.putNumber("filtered y acceleration Angle",drivetrain.m_gyro.getYFilteredAccelAngle() );
    SmartDashboard.putNumber("X Complimentary angle",drivetrain.m_gyro.getXComplementaryAngle() );
    SmartDashboard.putNumber("setpoint", balancing.setpoint);
    SmartDashboard.putNumber("error sum", balancing.errorSum);
    SmartDashboard.putNumber("last error", balancing.lastError);
    SmartDashboard.putNumber("error", balancing.berror);
    SmartDashboard.putNumber("error rate",balancing.errorRate);
    SmartDashboard.putNumber("turn error", turn.turnerror);
    SmartDashboard.putBoolean("Digital Pressure Switch", pneumatics.m_pH.getPressureSwitch());
    SmartDashboard.putBoolean("Compressor Running", pneumatics.m_pH.getCompressor());
    SmartDashboard.putBoolean("cube", color.cube);
    SmartDashboard.putBoolean("cone", color.cone);
    SmartDashboard.putNumber("Red", color.red);
    SmartDashboard.putNumber("Green", color.green);
    SmartDashboard.putNumber("Blue", color.blue);
    SmartDashboard.putNumber("IR", color.IR);
    SmartDashboard.putNumber("Proximity",color.proximity);

    
   }
}
