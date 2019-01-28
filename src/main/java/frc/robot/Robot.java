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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  XBoxController controller;

  Spark right_front;
  Spark right_center;
  Spark right_rear;

  Spark left_front;
  Spark left_center;
  Spark left_rear;

  Spark elevator_1;
  Spark elevator_2;

  SpeedControllerGroup right;
  SpeedControllerGroup left;
  SpeedControllerGroup elevator;
    
  DifferentialDrive myRobot;

  double stickLeftY;
  double stickRightY;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
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
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopPeriodic() {
    controller = new XBoxController(0);

    right_front = new Spark(1);
    right_center = new Spark(2);
    right_rear = new Spark(3);

    left_front = new Spark(4);
    left_center = new Spark(5);
    left_rear = new Spark(6);

    elevator_1 = new Spark(7);
    elevator_2 = new Spark(8);

    right = new SpeedControllerGroup(right_front, right_center, right_rear);
    left = new SpeedControllerGroup(left_front, left_center, left_rear);
    elevator = new SpeedControllerGroup(elevator_1, elevator_2);
    
    myRobot = new DifferentialDrive(left, right);

    stickLeftY = controller.getLeftThumbstickY();
    stickRightY = controller.getRightThumbstickY();

    myRobot.tankDrive(stickLeftY, stickRightY);

    //Elevator code - Press "B" to extend, "A" to retract.
    if (controller.getAButton()) {
      System.out.println("Elevator should be retracting");
      elevator.set(-0.7);
    } else if (controller.getBButton()) {
      System.out.println("Lets make a hole in the ceiling!");
      elevator.set(0.7);
    } else {
      elevator.set(0.0);
    }
  }

  @Override
  public void testPeriodic() {
  }
}