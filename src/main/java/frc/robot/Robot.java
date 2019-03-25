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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SolenoidBase;
import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;


public class Robot extends TimedRobot {
  
  
  
/*
  public Robot DoubleSolenoid(int moduleNumber, int forwardChannel, int reverseChannel)
  {
    moduleNumber = 0;
    forwardChannel = 6;
    reverseChannel = 7;
  }
  */
  

  //cameraHandler camera;

  @Override
  public void robotInit() {
    
    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
  }

  

//pcm 1   2 3 high gear 6 7 low gear 
//pcm 0   0 1 hatch grab 6 7 hatch extender 4 5 intake
  

  boolean gear;
  boolean isYPressed;

  XBoxController controller = new XBoxController(0);
  LogitechExtreme3DPro operatorController = new LogitechExtreme3DPro(1);
  

  Spark right_front = new Spark(0);
  Spark right_rear = new Spark(1);
  Spark left_front = new Spark(2);
  Spark left_rear = new Spark(3);

  Spark elevator_1 = new Spark(4);
  Spark elevator_2 = new Spark(5);
  Spark elevator_3 = new Spark(6);
  Spark elevator_4 = new Spark(7);

  //Spark intakeRotator1 = new Spark(9); ////Re-Enter Actual Value Because This Is A Place Holder Value
  //Spark intakeRotator2 = new Spark(8); ////Re-Enter Actual Value Because This Is A Place Holder Value

  Spark intake = new Spark(8);// Re-Enter Actual Value Because This Is A Place Holder Value
  Spark fly_wheel = new Spark(9);

 // Victor left_lift = new Victor(0);
 // Victor right_lift = new Victor(1);
  Compressor compressor = new Compressor(0);
  DigitalInput bottom_limit_switch = new DigitalInput(4);
  AnalogInput sensor = new AnalogInput(0);
  Encoder encoder_left = new Encoder(1, 0, true);
  Encoder encoder_right = new Encoder(3, 2, true);

  SpeedControllerGroup right = new SpeedControllerGroup(right_front, right_rear);
  SpeedControllerGroup left = new SpeedControllerGroup(left_front, left_rear);
  SpeedControllerGroup elevator = new SpeedControllerGroup(elevator_1, elevator_2, elevator_3, elevator_4);
  DoubleSolenoid gearController = new DoubleSolenoid(1, 2, 3);
  DoubleSolenoid elevatorController = new DoubleSolenoid(1, 6, 7);
  DoubleSolenoid hatchGrab = new DoubleSolenoid(0, 0, 1);
  DoubleSolenoid hatchExtender = new DoubleSolenoid(0, 6, 7);
  DoubleSolenoid cargoArm = new DoubleSolenoid(0, 4, 5);


  DifferentialDrive myRobot = new DifferentialDrive(left, right);


  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {

    teleopPeriodic();
  }
    

  @Override
  public void teleopPeriodic() {
    compressor.setClosedLoopControl(true);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
   /*
   pathfinder should replace the current tank drive
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
   
  
    stickLeftY = (stickLeftY)*1.0;
    stickRightY = (stickRightY)*1.0;
    

    

    //System.out.println("Left/Right: " + stickLeftY + ", " + stickRightY);
    myRobot.tankDrive(stickLeftY, stickRightY);
    //elevator.set(controller.getLeftThumbstickY());
   // SmartDashboard.putNumber("eLEVATORtHROTTLE", controller.getLeftThumbstickY());
    if (controller.getAButton() == true) {
      System.out.println("Left encoder value " + encoder_left.get());
      System.out.println("Right encoder value " + encoder_right.get());
    }
   
   //Elevator code 
    
    double joystickY = operatorController.getJoystickY();
    if (Math.abs(joystickY) > 0.1){
      if (joystickY < -0.1) {
        //System.out.println("Should go up");
        elevatorController.set(DoubleSolenoid.Value.kReverse);
        
        elevator.set(joystickY);
      } else if (joystickY > 0.1) {
        //System.out.println("Should go down");
        elevatorController.set(DoubleSolenoid.Value.kReverse);
        elevator.set(joystickY);
      } 
      }
      else {
        elevator.set(0.0);
        elevatorController.set(DoubleSolenoid.Value.kForward);
     /*if (elevatorController.get() == DoubleSolenoid.Value.kReverse){
        System.out.println("UNLOCKING ELEVATOR");
        elevatorController.set(DoubleSolenoid.Value.kForward);
      }
        if (elevatorController.get() == DoubleSolenoid.Value.kForward){
        System.out.println("LOCKING ELEVATOR SOLENOID");
        elevatorController.set(DoubleSolenoid.Value.kReverse);
      }*/
      
    }

    if (operatorController.getButtonThree()){
      System.out.println("Elevator Piston Status: " + elevatorController.get());
    }

    if (operatorController.getButtonSix()){
      System.out.println("Manually locking elevator");
      elevatorController.set(DoubleSolenoid.Value.kReverse);
    } else if (operatorController.getButtonFour()){
      System.out.println("Manually un-locking elevator");
      elevatorController.set(DoubleSolenoid.Value.kForward);
    }

    if (operatorController.getButtonTwo() == true && bottom_limit_switch.get() == false){
      elevator.set(0.5);   //not sure if positive is going up or down, needs testing
      } else if (operatorController.getButtonTwo() == true && bottom_limit_switch.get() == true){
        elevator.set(0.0);
        }

    
    //vision code
    if (controller.getXButton() == true){
    
    

      double vision_target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
      double horizontalOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);	//positive means target is on the right side of the fov, negative is left side
  
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  
  int positiveTolerance = 3;
  int negativeTolerance = -3;
  
  double turn_speed = 0.55;
  double move_speed = 0.5;
  double test_turn_speed = 0.65;
  double test_move_speed = 0.7;
  double min_distance_from_wall = 5.0; // Minimum distance from the wall, set actual value here
  double min_distance_from_wall_meters = min_distance_from_wall / 39.37; //min distance but in meters
  //double min_distance_from_wall_meters = 0;       direct input
  
  
  
      if (vision_target == 1){
  
          System.out.println("Target found at " + horizontalOffset + " degrees away from the center crosshair");
  
      
          //test section, turning while moving forward, sensor using meters
          
              while (controller.getStartButton() == true){
                  
                  
                  double sensor_distance = sensor.getVoltage() * 1.024;
                  
          if (sensor_distance > min_distance_from_wall_meters){
                      if (horizontalOffset < positiveTolerance){
  
                          System.out.println("Turning left to compensate");
                          myRobot.tankDrive(test_turn_speed, test_move_speed);
              
  
                      } else if (horizontalOffset > negativeTolerance) {
  
                          System.out.println("Turning right to compensate");
                          myRobot.tankDrive(test_move_speed, test_turn_speed);
  
                      } else {
  
                          System.out.println("Aligned with target");
                          System.out.println("Autonomously moving to target " + sensor_distance + " meters away. . . RELEASE BUTTON to REGAIN CONTROL");
                          myRobot.tankDrive(test_move_speed, test_move_speed);
                      }    
  
                     
  
                  } else {
                      System.out.println("Should be at goal position - stopping autonomous control");
                      System.out.println("Distance: " + sensor_distance);
                      break;
                  }
          
                  Timer.delay(0.02);
                }
              }
            }
          

    

    //intake/cargo
    if (operatorController.getButtonEleven() == true) {
      intake.set(-0.8);
      
 }  else if (operatorController.getButtonTwelve() == true) {
      intake.set(0.8);

    } else {
      intake.set(0.0);
    }
    
    
    
    
    if (operatorController.getButtonSeven() == true) {
      hatchExtender.set(DoubleSolenoid.Value.kReverse);
      Timer.delay(0.5);
      hatchGrab.set(DoubleSolenoid.Value.kForward);
      Timer.delay(0.25);
      hatchExtender.set(DoubleSolenoid.Value.kReverse);
      
    } 
    else if (operatorController.getButtonEight() == true) {
      hatchExtender.set(DoubleSolenoid.Value.kReverse);
      cargoArm.set(DoubleSolenoid.Value.kForward);
      Timer.delay(0.5);
      hatchGrab.set(DoubleSolenoid.Value.kReverse);
      Timer.delay(0.25);
      hatchExtender.set(DoubleSolenoid.Value.kReverse);
      Timer.delay(0.125);
      cargoArm.set(DoubleSolenoid.Value.kReverse);
    } 
    else {
      hatchExtender.set(DoubleSolenoid.Value.kForward);
      
      
    }
        
    //High and low gear controllers.
    if (controller.getRightTrigger() == true) {

      gearController.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Tanzina Zahan Piston out");

    } else if (controller.getLeftTrigger() == true) {

      gearController.set(DoubleSolenoid.Value.kForward);
      System.out.println("Nazifa 5599 Prapti Piston In");

    }

    
    if (operatorController.getButtonNine() == true && cargoArm.get() == Value.kForward) {
      cargoArm.set(DoubleSolenoid.Value.kReverse);
      
    } else if (operatorController.getButtonTen() == true && cargoArm.get() == Value.kReverse) {
      cargoArm.set(DoubleSolenoid.Value.kForward);
     
    } 
  } 
  }
  
  
   



  
  
