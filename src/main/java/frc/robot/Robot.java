package frc.robot;
 
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
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
    //non drive motors
    private CANSparkMax Joint1;
    private RelativeEncoder J1encoder;
    
    private CANSparkMax Joint2;
    private RelativeEncoder J2encoder;
    
    private CANSparkMax Joint3;
    private RelativeEncoder J3encoder;
    
    private CANSparkMax squezer;
    private RelativeEncoder SQencoder;

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

    
    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    private double Speedvar=0.0;
    private double turnerror =0.0;
    private double directionL =0.0;
    private double directionR =0.0;
    
    private final PowerDistribution m_pdp = new PowerDistribution();
 
 
 
    @Override
  public void robotInit() {
    m_controller = new Joystick(0);
    
    m_gyro = new ADIS16470_IMU();
   
    m_right.setInverted(true);
    
    speedMult = .5;
   
    //motors that aren't drive
      squezer = new CANSparkMax(1, MotorType.kBrushless);
      Joint1 = new CANSparkMax(6, MotorType.kBrushless);
      Joint2 = new CANSparkMax(7, MotorType.kBrushless);
      Joint3 = new CANSparkMax(8, MotorType.kBrushless);
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
      
    J1encoder = Joint1.getEncoder();
    J2encoder = Joint2.getEncoder();
    J3encoder = Joint3.getEncoder();
    SQencoder = squezer.getEncoder();
    
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
    
    //Joint 1 variables
    final double J1kP = 0.5;
    final double J1kI = 0.5;
    final double J1kD = 0.1;
    final double J1iLimit = 1;
    double J1setpoint = 0;
    double J1errorSum = 0;
    double J1lastError=0;
    
    //Joint 2 variables
    final double J2kP = 0.5;
    final double J2kI = 0.5;
    final double J2kD = 0.1;
    final double J2iLimit = 1;
    double J2setpoint = 0;
    double J2errorSum = 0;
    double J2lastError=0;

    //Joint 3 variables
    final double J3kP = 0.5;
    final double J3kI = 0.5;
    final double J3kD = 0.1;
    final double J3iLimit = 1;
    double J3setpoint = 0;
    double J3errorSum = 0;
    double J3lastError=0;

    //squezer variables
    final double SQkP = 0.5;
    final double SQkI = 0.5;
    final double SQkD = 0.1;
    final double SQiLimit = 1;
    double SQsetpoint = 0;
    double SQerrorSum = 0;
    double SQlastError=0;
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
public void teleopInit(){
  setpoint = 0;
  errorSum = 0;
  lastTimestamp = 0;
  lastError = 0;
} 
  @Override
  public void teleopPeriodic() {
   
    tankDrive.tankDrive(right.getY() * speedMult, left.getY() * speedMult);
			tankDrive.feedWatchdog(); 
      
      // Joint 1 controlled by left and right triggers
      if (controller2.getLeftTriggerAxis()>.1) {
        SQsetpoint = 10;
        } else if (controller2.getRightTriggerAxis()>.1) {
        SQsetpoint = 0;
      }
      if (controller2.getAButton()) {
        J1setpoint = 10;
       } else if (controller2.getBButton()) {
        J1setpoint = 0;
      }
      if (controller2.getYButton()) {
        J2setpoint = 10;
        } else if (controller2.getXButton()) {
        J2setpoint = 0;
      }
      if(controller2.getPOV()==0){
        J3setpoint=10;
        }else if (controller2.getPOV()==180) {
        J3setpoint=0;
      }
      
      // get sensor position
      double SQsensorPosition = SQencoder.getPosition();
      double J1sensorPosition = J1encoder.getPosition();
      double J2sensorPosition = J2encoder.getPosition();
      double J3sensorPosition = J3encoder.getPosition();

      // calculations for squezer
      double SQerror = SQsetpoint - SQsensorPosition;
      double dt = Timer.getFPGATimestamp() - lastTimestamp;

      if (Math.abs(SQerror) < SQiLimit) {
        SQerrorSum += SQerror * dt;
      }

      double SQerrorRate = (SQerror - SQlastError) / dt;
      double SQoutputSpeed = SQkP * SQerror + SQkI * SQerrorSum + SQkD * SQerrorRate;

      // output to squezer motor
      squezer.set(SQoutputSpeed);

      // calculations for Joint 1
      double J1error = J1setpoint - J1sensorPosition;

      if (Math.abs(J1error) < J1iLimit) {
        J1errorSum += J1error * dt;
      }

      double J1errorRate = (J1error - J1lastError) / dt;
      double J1outputSpeed = J1kP * J1error + J1kI * J1errorSum + J1kD * J1errorRate;

      // output to Joint 1 motor
      Joint1.set(J1outputSpeed);

      // calculations for Joint 2
      double J2error = J2setpoint - J2sensorPosition;

      if (Math.abs(J2error) < J2iLimit) {
        J2errorSum += J2error * dt;
      }

      double J2errorRate = (J2error - J2lastError) / dt;
      double J2outputSpeed = J2kP * J2error + J2kI * J2errorSum + J2kD * J2errorRate;

      // output to Joint 2 motor
      Joint2.set(J2outputSpeed);

      // calculations for Joint 3
      double J3error = J3setpoint - J3sensorPosition;

      if (Math.abs(J3error) < J3iLimit) {
        J3errorSum += J3error * dt;
      }

      double J3errorRate = (J3error - J3lastError) / dt;
      double J3outputSpeed = J3kP * J3error + J3kI * J3errorSum + J3kD * J3errorRate;

      // output to Joint 3 motor
      Joint3.set(J3outputSpeed);
    
      SmartDashboard.putNumber("encoder value", leftParent.getSelectedSensorPosition() * kDriveTick2Feet);
  
  }
}