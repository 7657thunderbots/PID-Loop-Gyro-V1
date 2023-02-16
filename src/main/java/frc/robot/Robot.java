package frc.robot;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
  private static final String Balancingauto = "balancing";
  private static final String Auto2 = "Auto2";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

    int timer;
    
    public double speedMult;

    private final Timer m_timer = new Timer();
  private final Timer wait = new Timer();
    private Joystick left;
    private Joystick right;
    private XboxController controller2;
    private boolean down= false;
    public boolean onchargestation= false;
    
    private DriveTrain drivetrain;
    
    private Hand Hand;

    private Wrist wrist;

    private Elbow elbow;

    private Shoulder shoulder;

    private Balancing balancing;

   private turnadjust turn;
    
   private Pneumatics pneumatics;

   private color_sensor color_sensor;

   private Auto1 auto1;

   private Auto2_balance auto2_balance;

   private Auto3 auto3;
   

   private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

    @Override
  public void robotInit() {

    speedMult = .5;
    
    m_chooser.setDefaultOption("balancing", Balancingauto);
    m_chooser.addOption("My Auto", Auto2);
    SmartDashboard.putData("Auto choices", m_chooser);
     // This creates our drivetrain subsystem that contains all the motors and motor control code
     drivetrain = new DriveTrain();
      
     elbow = new Elbow();

     wrist = new Wrist();

      shoulder = new Shoulder();

      Hand = new Hand();
     
    //  balancing = new Balancing();

    //  turn = new turnadjust();
  
     pneumatics = new Pneumatics();

    //  color_sensor = new color_sensor();

    //  auto1 = new Auto1();
    
     auto2_balance = new Auto2_balance();
    
    //  auto3 = new Auto3();

    left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
    drivetrain.m_gyro.reset();
  }

  

  @Override
  public void robotPeriodic() { 
   shoulder.Shoulder_Run();
    elbow.ElbowRun();  
  SmartDashboard.getNumber("elbow", elbow.Elbowencoder.getPosition());
  SmartDashboard.getNumber("Shoulder", shoulder.shouldere.getPosition());
     drivetrain.run_drive();
   Hand.Hand_Run();
   wrist.Wrist_Run();
  pneumatics.Run_Pneumatics();
  SmartDashboard.putNumber("tilt angle",drivetrain.m_gyro.getAngle());
  SmartDashboard.putNumber("turn angle",drivetrain.m_gyro.getXComplementaryAngle());
}


  
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    drivetrain.m_gyro.reset();
    m_timer.reset();
		m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    DataLogManager.start();
    switch (m_autoSelected) {
      case Auto2:
        
        break;
      case Balancingauto:
      default:
      auto2_balance.Run_Auto2_balance();
        break;
    }
    // if (m_timer.get()<3){
    // }
    //  else{
    //    balancing.BalancingRun();
    //  }

     turn.turnadjust_run();
     drivetrain.tankDrive ( -turn.turnerror + balancing.Speedvar, turn.turnerror+ balancing.Speedvar, false);

   }

@Override
public void teleopInit(){
} 

@Override
  public void teleopPeriodic() {
    DataLogManager.start();
    //elbow.ElbowRun();
    drivetrain.tankDrive(right.getY() * speedMult, left.getY() * speedMult, false);
    
      // // Hand controlled by left and right triggers
       if (controller2.getPOV()==90) {
          Hand.hsetpoint = 0;
        pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        } 
          else if (controller2.getPOV()==270) {
         Hand.wait.reset();
         Hand.wait.start();
            Hand.hsetpoint=-20;
      pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
      
      if (controller2.getAButton()) {
        //pneumatics.doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
        // shoulder.Ssetpoint=5;
        // wrist.Wsetpoint=10;
        elbow.Esetpoint = -35;
        elbow.EkP=0.05;
        } 
       else if (controller2.getBButton()) {
        elbow.EkP=0.005;
        elbow.Esetpoint = 0;
        //pneumatics.doubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
        // wrist.Wsetpoint=0;
        // shoulder.Ssetpoint=0;
      }
      
     
      
      // if (elbow.Esetpoint==elbow.Elbowencoder.getPosition()){
      // Hand.hsetpoint=0;
      // }

      
    
        
      
       
       if (controller2.getYButton()) {
          shoulder.Ssetpoint = 120;
         } 
         
         else if (controller2.getXButton()) {
          shoulder.Ssetpoint = 0;
        }
       
       if(controller2.getPOV()==0){
         wrist.Wsetpoint=0;
          }else if (controller2.getPOV()==180) {
         wrist.Wsetpoint=-20;
        }
    }
  
}