package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    PWMVictorSPX m_left, m_right;
    Joystick m_controller;
    ADIS16470_IMU m_gyro;
    int timer;
    //tank drive vars
    private DifferentialDrive tankDrive;
    private WPI_TalonFX leftParent;
    private WPI_TalonFX leftChild;
    private WPI_TalonFX rightParent;
    private WPI_TalonFX rightChild;

    private double speedMult;

    private final Timer m_timer = new Timer();

    private Joystick left;
    private Joystick right;
    private XboxController controller2;

    private boolean onchargestation= false;
    private boolean chargestationbalance= false;
    
    private Joystick joy1 = new Joystick(0);

    //private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    private double Speedvar=0.0;
    private double turnerror =0.0;
    private double directionL =0.0;
    private double directionR =0.0;
    private double anglelast=0.0;
    private double newangle=0.0;
    private final PowerDistribution m_pdp = new PowerDistribution();
  @Override
  public void robotInit() {
    m_controller = new Joystick(0);
    
    m_gyro = new ADIS16470_IMU();
   
    m_right.setInverted(true);
    
    speedMult = .5;
   
    //tankdrive
    leftParent = new WPI_TalonFX(4);
		leftChild = new WPI_TalonFX(5);
		leftParent.setInverted(true);
		leftChild.follow(leftParent);
		leftChild.setInverted(true);
		rightParent = new WPI_TalonFX(3);
		rightChild = new WPI_TalonFX(2);
		rightChild.follow(rightParent);
		tankDrive = new DifferentialDrive(rightParent, leftParent);
    
    //driveencoders
		rightParent.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		rightParent.setSelectedSensorPosition(0);
		leftParent.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		leftParent.setSelectedSensorPosition(0);
    
    // MAKE SURE GREEN CONTROLLER IS 0 IN DRIVER STATION!!!!!!!!!
		left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
  }

  @Override
  public void autonomousInit() {
    //encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    m_gyro.reset();
    m_timer.reset();
		m_timer.start();
  }

    final double akP = 0.5;
    final double akI = 0.5;
    final double akD = 0.1;
    final double aiLimit = 1;
 
    final double bkP = 0.5;
    final double bkI = 0.5;
    final double bkD = 0.1;
    final double biLimit = 1;
  
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;

  @Override
  public void autonomousPeriodic() {
    // get joystick command
   if (onchargestation=false){
        if (joy1.getRawButton(1)) {
          setpoint = 10;
        } else if (joy1.getRawButton(2)) {
          setpoint = 0;
        }

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
        if (leftParent.getSelectedSensorPosition()==10){
          onchargestation=true;
          setpoint = 0;
          errorSum = 0;
          lastTimestamp = 0;
          lastError = 0;

        }
    }
      if (onchargestation==true ){
        if (joy1.getRawButton(1)) {
          setpoint = 10;
        } 
        else if (joy1.getRawButton(2)) {
          setpoint = 0;
        }

        // get sensor position
        double sensorPosition = m_gyro.getYComplementaryAngle();

        // calculations
        double error = setpoint - sensorPosition;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < biLimit) {
          errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;

        double outputSpeed = bkP * error + bkI * errorSum + bkD * errorRate;

        // output to motors
        Speedvar=outputSpeed;


        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
        if (m_gyro.getYComplementaryAngle()==0){
          chargestationbalance=true;
        }
      }
        if (m_gyro.getAngle()>3){
          turnerror = .1;
          }
          else if (m_gyro.getAngle()<3 && m_gyro.getAngle() >-3){
          turnerror =0;
          }
          else if (m_gyro.getAngle()<-3){
            turnerror =-.1;
        }
          
            directionL= Speedvar;
            directionR= Speedvar;

          tankDrive.tankDrive (turnerror+directionL,-turnerror+directionR);


        SmartDashboard.putBoolean("On charge Station", onchargestation);
        SmartDashboard.putBoolean("charge station balance", chargestationbalance);
        SmartDashboard.putData("PDP", m_pdp);
        SmartDashboard.putNumber("tilt angle", m_gyro.getYComplementaryAngle());
        SmartDashboard.putNumber("voltage",m_pdp.getVoltage());
        SmartDashboard.putNumber("PDP current", m_pdp.getTotalCurrent());
        SmartDashboard.putNumber("Total energy", m_pdp.getTotalEnergy());
        SmartDashboard.putNumber("Total power", m_pdp.getTotalPower());
        SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(9));
        SmartDashboard.putNumber("Current used by: drivemotor right", m_pdp.getCurrent(8));
        SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(10));
        SmartDashboard.putNumber("Current used by: drivemotor left", m_pdp.getCurrent(11));
        SmartDashboard.putNumber("turn angle",m_gyro.getAngle());
        SmartDashboard.putNumber("Constant speed left",directionL);
        SmartDashboard.putNumber("constant speed right", directionR);
        SmartDashboard.putNumber("error adjustment direction", turnerror);
        SmartDashboard.putNumber("X acceleration",m_gyro.getAccelX());
        SmartDashboard.putNumber("Y acceleration",m_gyro.getAccelY());
        SmartDashboard.putNumber("Z acceleration",m_gyro.getAccelZ());
        SmartDashboard.putNumber("Gyro Rate",m_gyro.getRate());
        SmartDashboard.putNumber("filtered x acceleration Angle",m_gyro.getXFilteredAccelAngle() );
        SmartDashboard.putNumber("filtered y acceleration Angle",m_gyro.getYFilteredAccelAngle() );
        SmartDashboard.putNumber("X Complimentary angle",m_gyro.getXComplementaryAngle() );
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("encoder value", leftParent.getSelectedSensorPosition() * kDriveTick2Feet);
      }

       
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", leftParent.getSelectedSensorPosition() * kDriveTick2Feet);
  }
}