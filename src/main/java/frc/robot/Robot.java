/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
  ______  _____    _____   _____  _____  ___    ___  
 |  ____||  __ \  / ____| | ____|| ____|/ _ \  / _ \ 
 | |__   | |__) || |      | |__  | |__ | (_) || (_) |
 |  __|  |  _  / | |      |___ \ |___ \ \__, | \__, |
 | |     | | \ \ | |____   ___) | ___) |  / /    / / 
 |_|     |_|  \_\ \_____| |____/ |____/  /_/    /_/  
   _____               _    _               _        
  / ____|             | |  (_)             | |       
 | (___    ___  _ __  | |_  _  _ __    ___ | | ___   
  \___ \  / _ \| '_ \ | __|| || '_ \  / _ \| |/ __|  
  ____) ||  __/| | | || |_ | || | | ||  __/| |\__ \  
 |_____/  \___||_| |_| \__||_||_| |_| \___||_||___/  
  ____   _   _   _____  _    _   _____               
 |  _ \ | \ | | / ____|| |  | | / ____|              
 | |_) ||  \| || |     | |__| || (___                
 |  _ < | . ` || |     |  __  | \___ \               
 | |_) || |\  || |____ | |  | | ____) |              
 |____/ |_| \_| \_____||_|  |_||_____/               

MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWNXK0OkxxdoooooooooooooodxxkO0KXNWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWX0Oxol:;,,',:llllllllllol,'''''''',,;cloxOKNWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWNKkdl:;,'.''',,;cxd:;;;;;,,;lxocccc:::;;,,',;:::cldkKNWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMNKkoc;:cllooc::clooddko'':odc,''ckOOOOOOkkxdooodolllllc::cokKNMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMWN0dc::clllc:,,oxxkO0KKXKOo,'cxkd,.':xKWMMWWWNNXKOko;',;;:cloo:,,cd0NMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMN0dc;:lddl:,;cc;',okKWMMMMN0o,.:xkd;..;x0NNWMMMMMMN0x;',oxdlccoxo:,'',cd0NMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMWKxl:;lollxkc''ckkd;',oOXWMMMWKd;.:dkd:..;oxdk0XWMMMWKx:',oOKNXKK0Oxdollolc:lxKWMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMW0o:codxkd;';dkc',lxkd;';oOXWMMWKxc;cxkxo::clc:okXWMMWKkc''ck0XWWMWWX0kdl:;:odc,;o0WMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMN0l;:oo:,:okx:',ox:',oxxo;':x0NMMWNK0O00KKKKKKK000KNMMMWKxc,';cldk0KK0xo:,''''cxo;'';o0NMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMW0l;:oo:,;;,':dkl,'co;',oxxdodOKNMMMMWWWWMMMMMMMMMMMMMMMMMNK0kdlc:;lkko:,';lo:',dkxolllc:l0WMMMMMMMMMMMMMM
MMMMMMMMMMMMW0o;:ooc,'cxxo:'':ol,';cccdO0KXNWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWNXK0kOkl;;ldkOd,'ck0Okdlcooc;o0WMMMMMMMMMMMM
MMMMMMMMMMMXd::ooc,,,';ldxxo:,';cc:llodkOKNWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWNKOxolcodc:c;';dOkxc,'',:oo::dXMMMMMMMMMMM
MMMMMMMMMWOc:ooc,,lxdc,';ldxxoccllodxxxxxxxxxOOOOOOOOO000KXXXXXKK00OOOOOOOOOkxxxxxxxxdl;'..,oko:,';ldo;,cdl;cOWMMMMMMMMM
MMMMMMMMXd,':do;',cdxxdc,';lollodOKKK000000OxdlcodxkkkkkkkkkkkkxxxkkkkxxolloxO000000KKKOxl,,c:',:l::oo;''cxl;;dXMMMMMMMM
MMMMMMW0c''.,lxdl;',cdxkdc'':dOKK00OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO000KOdc,,oxc'','';oxxddl;lKWMMMMMM
MMMMMWO:,;::ldxkOxl;',cooclx0K00OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO000kol;',oxddxdc;,:dl;c0WMMMMM
MMMMWk;,ldlllooxkxkxo;';okKK0OOOOOOOOOOOxdodkOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOkxooxkOOOOOOOOOOO0KOo;cddl:,',::,;oo;:OWMMMM
MMMWk;,ld:'.',,,,;;::',dKXX0OOOOOOO00Oxoc;;cdkOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOxl;;codO000OOOOOOO0XXO:'.';cldkkc';oo;:OWMMM
MMWO:''cdl;'':odol,',,':ok0KK00000KKkollodxkOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOkdoollx0KK00000K0Odc,..;dkxdlc,''cxc':0WMM
MM0c''';oxdl,':oOx::dOkdclxO00KXXKOollodkOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOkxdollxKXXK00Oxocldo:':c;,',:ldkxo:,cKMM
MXo';llodxOOxc,'cdxk0NMWKOkkxxxkxollodxkOOkxxxxkkOOOOkdoxkOOOOOkkdoxOOOOOOOkkkkOOOxdolcldxkxxxkO0NNKxc'';coddolc:ldc,oNM
Wk,,odcclodxkkxc,ck0NMMMMMWXo,;cclodxkOkdoollcc:coxOOOxlc:::::::cldkOOOkdolloooddxOkxdollc:,c0WMMMMWKOo:::;,'',,,'ld;;kW
Kc':xo:;,'',;:lllx0NMMWWWNXk;,loodkOOOOkkkOOOOkdl::okOOOOxolccoxkOOOOkdlldkOOOOOkkOOOOkxdol:,dKNNWWWN0x:.,:llodko,;do,lX
x,',cdxxxdolc;'':odkkkxdol:,,:odkOOOOOO0000000OOOkoclxOOOOOOOOOOOOOOxooxOOOO000000OOOOOOkxol,':cloxxkxo:';dkdoll:''cd:,k
l';:ldxk0K0OOl';oxdl::ccc:,.;ldkOOOOO000OOO00KK0OOOkoldkOOOOOOOOOOkdoxOOO0KKK0OOO000OOOOOOxo:'':lllccloo:',;,'',;:cod:'l
;,odlclloodddlcdOXWXOo:,;;'':okOOOOOO0Oxdlccox0K00OOOkxxkOOOOOOOOkxkOOOO0KKkolccoxO0OOOOOOOdc,';:;;lkKX0dc:ccldxkkkxxo;;
':dc'',,''''''cxKWMMMWKkl,.,cdkOOOOOOOOkxoxOo:lxO000OOOOOO0OOO00OOOOO000Oko:ckOodkkOOOOOOOOxo;.'cd0NMMWKOoldxkOK0Od:cdc'
'cd:'lxddooolldkXWMMMMMMNo',lxOOOOOOOOOOoxN0;.':lxkOO00KK0xxOkxOKK00OOOxo:'.,kNOlkOOOOOOOOOko:.:KMMMMMWXOl'',;cloxo,;dl'
'ld;':llooodddxONMMMMMMMNo.;lxOOOOOOOkdxooOkc;ldc;;:cloll;;okx:;clolc::;:oo;;x0doddxOOOOOOOkd:':KMMMMMMXOo';ll:;,,,',oo,
,od:,,,''''',;oOXMMMMMMMNo.;oxOOxddxOOxl::;:clddl:;;,'....,;::;''''',;;:codol:;;:cokOkxdxkOkd:.:0MMMMMMN0o,;dO0Oxl:,;od;
'lxxxddddddoclx0NMMMMMMWKc.,oxdlcoxOOkol::::codxkOOxl;.............:ldkOkxdoc::::coxOOkdlcoxd:.;OWMMMMMW0xloxO00koodxxo,
,lxooxkxlokkc;oOXMMMWKOdl;.,c:;cdkOOkdxkkkxkOkxkOOOOxdl;.........,coxkOOOkxkOOkxkkxdxkOOko;;c;.,cdk0NWMN0d:;;;:c:;:lodc'
'ld:,oOo,;xkc'ckKX0kdlcll:..''cdkOOOOkxxOOOOkooO0OOOOxxkdllllllloxxxkOOOO0dlxOOOOkxxkOOOOxl,'..;lolldk00Ol',;,;::;,',do,
':dc'lkd;,lxc':xOOdlc:;,''...;lxOOOOkdokOOOxdclO0OOkxxOxdxOkxkOkdxOkxkOO00ocoxkOOOdoxOOOOkd:....',;:cldddc';lllodxc';dl'
';dl',;;'''''';dKNWNXKOkdo:..:odOOOOdld0Okdlc,,dKK0xoxOkkO0xoxOOkkOkddO0Kk;,:loxO0klokOOOkoc'.;ldxO0XNNKd;.',;:ll:,'cd:'
,,odc:ccllolc::dOXMMMMMMMMXl.,coxOOxloO0Odc:..:lxO0KK0000Okl;cxO000KKKK0koc'.;cok00xcokOkdl;.:0WMMMMMWXxc;;ccccc:;,;od;;
c';okkxdool;,'';d0NMMMMMMMMK:.,loddclk0kxl::;:odxdodkxlc:;;;',;;::cxOdodxxl;';;cdOK0ocoxdo:.,kWMMMMMMXxcd0KKKKK0Oxddo:'c
x,'ldc,'',',coc,:xKWMMMMMMMWk'.,ccc:lxxxkkkkkkdoodolkdllddxkdkxddolokolddd:,,,,,;coxxc:ll:..oNMMMMMMWOllOKKWMMMWKxdl;',x
Kc'cdc';od:;dxo;'lOXMMMMMMMKc,;:ldO0KXKK0Okxolc;:xx::dxkOxolllodOOxdc;oklc;';llc;,,,:;,::..,ckNNXK0kdlloooOWMMMNOdoc''cK
Wx,,ld;,co:',,'',cx0KKOkkkx:;ldxxxO0Okdoollloc;..lOx:;,;:,'....,::,,:oOd,.,cxkxxko:,,,'''',;,;looooool:;cokKKKX0xdl,.,kW
MXl';dl,'',;ccoddoooooodddo:;ldxxdooloodddldOko:,:oxxooooddddddddooodkdc;;cxOxldOOOxol:,'''',;;cdxxdoc,,;:::ldkkdo;''oXM
MM0:':doclodkkxolooddddddddc';coocokkkOOOklcxxox0KK0000000000000000000KK0kooxlcdOOOO0Kk:;c:;,,,,',,:loddc''',;coxo,'c0MM
MMWO;';cllllol,,oddddddooddoco0X0clKK000Okd::lokKX00OOOOOOOOOOOOOOOOOO0KKOdlc:lxO0000KxckNXK0Okdccc;;coo:,coc:,:dl,:OWMM
MMMWk;'clclo:;:lddoool:loloddoxKNk:d000K0Oko:;:oxOO0OOOOOOOOOOOOOOOOOOOOkdc;:lxO0KK0KkcdNMMMMWN0xl:::,''';okkxddc,;kWMMM
MMMMWkloo:;::lddodkkxo;;lollddolkX0lcx0KX0Okdc;,:ldxkOOOOOOOOOOOOOkkkxdoc;;:oxO0KX0klcxNMMMMWXOxc,';cooc;'';cxd:';kWMMMM
MMMMMWKd;,clloolxXWWX0d:,;cclddldXMNOolodkkxoc;;;,,,;:clooooooooolcc;,,,;,;:lxkkxolokXWMMMNXOdc;;cc;',coxdl:ldc,:OWMMMMM
MMMMMMW0c,;clolcdKWMWXOx:'';ldldKWMMMWXOxddoddo:;;;;'''''''....''''',,;;;;lxdodddkKWMMMMWXOdc,;;',cddc,',lxkd:,c0WMMMMMM
MMMMMMMMXo,';ldlcokK0xl;'',:dlcd0XNWMMMMMMWWMMXo',:;,;oxdoollloodxdc:lc,.;OMMWWMMMMMMWNKOdc,'ckko:',cxxl:cdl,,dXMMMMMMMM
MMMMMMMMMNk:',:lccodc,',loc:c;,;cdOKNWMMMMMMMMWk;;:;cOWMMMWWWWWWMMWKdlo:.lXMMMMMMMWNX0kl:::,';ccoko;',ckko:,:kNMMMMMMMMM
MMMMMMMMMMWKo,',:ol,';lxx:''cddc'':dkOKXNWWMMMM0l::dXMMMMMMMMMMMMMMMNOoc;xWMMMMWXKOkkkd:',ldc,,cdkx:';ldl,;oKWMMMMMMMMMM
MMMMMMMMMMMMWOl,,coooxkl,';lxkd:'';c:;cldk0XNNWXdlOWMMMMMMMMMMMMMMMMMWKdl0WMMMWKOd:;lxkx:''cxxdkxc,;lol;,l0WMMMMMMMMMMMM
MMMMMMMMMMMMMMNOc,;clxo,.,okxl,';oo;.,::,;:oxkkOkOXXNNWWWWWWWWWWWWNNXXKOxOKNMMMN0xc'':lc;,'':dxko:lol;,cONMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMNkc,':ooc;,:;'':xx;.,okxc'';c;,okkxooxOkxkkkkkkOOkdolcc::dOXMMMMWKkl,'';oxo;';okxdl;,lONMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMNOl;,:lol:,;okd;.,lkkl,.:dc',okxl''lxc;;;;;,,;l:'';clloxOXWMMMMWXOo;',oxxdlloo:,;oONMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMW0d:,,:lloxx:.':dko,.;xd,.,col;';dOkkkkxdc''lc''cddolldOXWMMWWX0kd:';dxolc;,cxKWMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMWXOo:''':ooc:;;;,.;dkc'':c:,''ckKNWWKkkc''dx;',;:;'.;d0XK0Okxddddool:,':oONMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMWXOo:,,;cllllccdkd;.:xkx:.,oOKXXX0kkc',okdddxxd:.'lkxddoolc:;;;,,:oOXWMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMWX0xl;,,;:lllddlcdxkd,'cxkxolllll;.,okxo::;;,;:oxl::;,''',:lx0NWMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWX0xoc;'',:cclclolldxkdccccc:ccldxxdlllllllc;''',;cox0XWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMNX0kdoc:;,,,,;;;:cccccccccc:;:c:;,,,,;:codk0XNMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWWNK0OkxddolllccccccccllloddxkO0KNWWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMWWWNNNNNNNNWWWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.networktables.*;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;


  //cameraHandler camera;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
  }

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }


  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        break;
    }
  }

  boolean gear;
  boolean isYPressed;

  XBoxController controller = new XBoxController(1);

  Spark right_front = new Spark(1);
  Spark right_center = new Spark(2);
  Spark right_rear = new Spark(3);

  Spark left_front = new Spark(4);
  Spark left_center = new Spark(5);
  Spark left_rear = new Spark(6);

    Spark elevator_1 = new Spark(7);
    Spark elevator_2 = new Spark(8);

    Spark intakeRotator1 = new Spark(9); ////Re-Enter Actual Value Because This Is A Place Holder Value
    //Spark intakeRotator2 = new Spark(8); ////Re-Enter Actual Value Because This Is A Place Holder Value

    Spark intake = new Spark(0); ////Re-Enter Actual Value Because This Is A Place Holder Value

    SpeedControllerGroup right = new SpeedControllerGroup(right_front, right_center, right_rear);
    SpeedControllerGroup left = new SpeedControllerGroup(left_front, left_center, left_rear);
    SpeedControllerGroup elevator = new SpeedControllerGroup(elevator_1, elevator_2);
    
    DoubleSolenoid gearController = new DoubleSolenoid(0, 1);
    DoubleSolenoid hatchSolenoid = new DoubleSolenoid(2, 3);

    DifferentialDrive myRobot = new DifferentialDrive(left, right);

    

  @Override
  public void teleopPeriodic() {
   /*
   Fix this area of code for driveTrain
   */ 
    double stickLeftY = controller.getLeftThumbstickY();
    double stickRightY = controller.getRightThumbstickY();

  /*
  double steer = controller.getX(Hand.kRight);
  double drive = controller.getY(Hand.kLeft);
  boolean auto = controller.getAButton();
  
  steer *= 0.70;
  drive *= 0.70;
  */   
  
      // stickLeftY = (stickLeftY)*1.0;
      //stickRightY = (stickRightY)*1.0;
    
    
    
    myRobot.tankDrive(stickLeftY, stickRightY);
    
    //Elevator code - Press "A" to extend, "X" to retract.
    if (controller.getRightTrigger() == true) {
      System.out.println("Should go up");
      elevator.set(0.9);
    } else if (controller.getLeftTrigger()) {
      System.out.println("Should go down");
      elevator.set(-0.2);
    } else {
      elevator.set(0.0);
    }
  /*for (int a = 0, a <= 60, a++) {
      if (a <= 30) {

      }
  }
*/
    //intake
    if (controller.getBButton() == true) {
      intake.set(-0.8);
    } else if (controller.getAButton() == true) {
      intake.set(0.8);
    } else {
      intake.set(0.0);
    }

    //intake rotator
    if (controller.getRightBumper() == true) {
      intakeRotator1.set(-0.5);
      //intakeRotator2.set(-0.5);
    } else if (controller.getLeftBumper() == true) {
      intakeRotator1.set(0.5);
      //intakeRotator2.set(0.5);
    } else {
      intakeRotator1.set(0.0);
      //intakeRotator2.set(0.0);
    }

    //High and low gear controllers.
    if (controller.getDPadDown() == true) {

      gearController.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Tanzina Zahan");

    } else if (controller.getDPadUp() == true) {

      gearController.set(DoubleSolenoid.Value.kForward);
      System.out.println("Nazifa 5599 Prapti");

    }
    
    else {
      gearController.set(DoubleSolenoid.Value.kOff);
    }

    if (controller.getXButton()){
      hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Hatch Should be In");
    } else if (controller.getYButton()){
      hatchSolenoid.set(DoubleSolenoid.Value.kForward);
      System.out.println("Hatch Should be Out");
    } else {
      hatchSolenoid.set(DoubleSolenoid.Value.kOff);
    }
  }
    //test

   // public void testPeriodic() {
      


  //  }
}


  
  /*class cameraHandler {
    SerialPort visionPort;
    String visionLocation;

    String x1Temp;
    String y1Temp;
    String x2Temp;
    String y2Temp;

    int x1;
    int y1;
    int x2;
    int y2;

    public int[] getConsole() {
      try {
        visionPort = new SerialPort(115200, SerialPort.Port.kMXP);
        visionLocation = visionPort.readString();

        x1Temp = visionLocation.substring(8, 10);
        y1Temp = visionLocation.substring(12, 14);
        x2Temp = visionLocation.substring(16, 18);
        y2Temp = visionLocation.substring(20, 22);

        int x1 = Integer.parseInt(x1Temp);
        int y1 = Integer.parseInt(y1Temp);
        int x2 = Integer.parseInt(x2Temp);
        int y2 = Integer.parseInt(y2Temp);

        int[] location = {x1, y1, x2, y2};

        return location;
      } catch (Exception e) {
        System.out.println(e);
        return null;
      }
    }

    public String getRawConsole() {
      try {
        visionPort = new SerialPort(115200, SerialPort.Port.kUSB2);
        return visionLocation = visionPort.readString();
      } catch (Exception e) {
        return "We messed up.";
      }
    }

    public double getTargetCenter() {
      try {
        visionPort = new SerialPort(115200, SerialPort.Port.kMXP);
        visionLocation = visionPort.readString();

        x1Temp = visionLocation.substring(8, 10);
        x2Temp = visionLocation.substring(16, 18);

        int x1 = Integer.parseInt(x1Temp);
        int x2 = Integer.parseInt(x2Temp);

        double targetCenter = (x1 + x2)/2;

        return targetCenter;
      } catch (Exception e) {
        System.out.println(e);
        return 0.0;
      }
    }

    public int getx1() {
      visionPort = new SerialPort(115200, SerialPort.Port.kMXP);
      visionLocation = visionPort.readString();

      x1Temp = visionLocation.substring(8, 10);
      int x1 = Integer.parseInt(x1Temp);

      return x1;
    }

    public int gety1() {
      visionPort = new SerialPort(115200, SerialPort.Port.kMXP);
      visionLocation = visionPort.readString();

      y1Temp = visionLocation.substring(12, 14);
      int y1 = Integer.parseInt(x1Temp);

      return y1;
    }

    public int getx2() {
      visionPort = new SerialPort(115200, SerialPort.Port.kMXP);
      visionLocation = visionPort.readString();

      x2Temp = visionLocation.substring(16, 18);
      int x1 = Integer.parseInt(x1Temp);

      return x1;
    }

    public int gety2() {
      visionPort = new SerialPort(115200, SerialPort.Port.kMXP);
      visionLocation = visionPort.readString();

      y2Temp = visionLocation.substring(20, 22);
      int y1 = Integer.parseInt(x1Temp);

      return y1;
    }
  }*/
