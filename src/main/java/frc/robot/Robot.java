package frc.robot;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
   
    int timer;
    
    public double speedMult;

    private final Timer m_timer = new Timer();

    private Joystick left;
    private Joystick right;
    private XboxController controller2;

    public boolean onchargestation= false;
    
    private DriveTrain drivetrain;
    
    private Hand Hand;

    private Wrist wrist;

    private Elbow elbow;

    private Shoulder shoulder;

    private Balancing balancing;

   private turnadjust turn;
    
   
    
    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    @Override
  public void robotInit() {
    
    speedMult = .5;
    
    
     // This creates our drivetrain subsystem that contains all the motors and motor control code
     drivetrain = new DriveTrain();
      
     elbow = new Elbow();

     wrist = new Wrist();

     shoulder = new Shoulder();

     Hand = new Hand();
     
     balancing = new Balancing();

     turn = new turnadjust();
  
    // MAKE SURE GREEN CONTROLLER IS 0 IN DRIVER STATION!!!!!!!!!
		left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
  }
  

  @Override
  public void robotPeriodic() {   
    drivetrain.run_drive();
}


  
  @Override
  public void autonomousInit() {
    drivetrain.m_gyro.reset();
    m_timer.reset();
		m_timer.start();
  }

    final double akP = 0.5;
    final double akI = 0.5;
    final double akD = 0.1;
    final double aiLimit = 5;
  @Override
  public void autonomousPeriodic() {
    DataLogManager.start();
    if (m_timer.get()<3){
      balancing.Speedvar=3;
    }
    else{
      balancing.BalancingRun();
    }
    // get joystick command
  //  if (onchargestation==false){
  //         setpoint = 5;
    
        

  //       // get sensor position
  //       double sensorPosition = leftParent.getSelectedSensorPosition() * kDriveTick2Feet;

  //       // calculations
  //       double error = setpoint - sensorPosition;
  //       double dt = Timer.getFPGATimestamp() - lastTimestamp;

  //       if (Math.abs(error) < aiLimit) {
  //         errorSum += error * dt;
  //       }

  //       double errorRate = (error - lastError) / dt;

  //       double outputSpeed = akP * error + akI * errorSum + akD * errorRate;

  //       // output to motors
  //       Speedvar=outputSpeed;


  //       // update last- variables
  //       lastTimestamp = Timer.getFPGATimestamp();
  //       lastError = error;
  //       if (sensorPosition==5){
  //         onchargestation=true;
  //         errorSum = 0;
  //         lastTimestamp = 0;
  //         lastError = 0;

  //       }
  //   }
          turn.turnadjust_run();
           drivetrain.tankDrive ( -turn.turnerror + balancing.Speedvar, turn.turnerror+ balancing.Speedvar, false);

  }
      @Override
public void teleopInit(){
} 

  @Override
  public void teleopPeriodic() {
    drivetrain.tankDrive(right.getY() * speedMult, left.getY() * speedMult, false);

      // Hand controlled by left and right triggers
       if (controller2.getLeftTriggerAxis()>.1) {
         Hand.hsetpoint = 10;
       } 
         else if (controller2.getRightTriggerAxis()>.1) {
         Hand.hsetpoint=0;
       }
      
      if (controller2.getAButton()) {
        shoulder.Ssetpoint = 10;
        } 
       else if (controller2.getBButton()) {
        shoulder.Ssetpoint = 0;
       }
       
      if (controller2.getYButton()) {
         wrist.wsetpoint = 10;
         } else if (controller2.getXButton()) {
         wrist.wsetpoint = 0;
       }
       
       if(controller2.getPOV()==0){
         elbow.Esetpoint=10;
          }else if (controller2.getPOV()==180) {
         elbow.Esetpoint=0;
        }
    }
  
}