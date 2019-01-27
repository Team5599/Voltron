/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
  ___  ___   ___   ___  ___  ___  ___                        
 | __|| _ \ / __| | __|| __|/ _ \/ _ \                       
 | _| |   /| (__  |__ \|__ \\_, /\_, /                       
 |_|  |_|_\ \___| |___/|___/ /_/  /_/                        
  _____  _           ___            _    _             _     
 |_   _|| |_   ___  / __| ___  _ _ | |_ (_) _ _   ___ | | ___
   | |  | ' \ / -_) \__ \/ -_)| ' \|  _|| || ' \ / -_)| |(_-<
   |_|  |_||_|\___| |___/\___||_||_|\__||_||_||_|\___||_|/__/
  ___  _  _   ___  _  _  ___                                 
 | _ )| \| | / __|| || |/ __|                                
 | _ \| .` || (__ | __ |\__ \                                
 |___/|_|\_| \___||_||_||___/                                
                                                             
*/

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

    if (controller.getAButton()) {
      elevator.set(-0.7);
    } else {
      elevator.set(0.0);
    }

    if (controller.getBButton()) {
      elevator.set(0.7);
    } else {
      elevator.set(0.0);
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();

    left.set(0.0);
    right.set(0.0);
    elevator.set(0.0);
  }
}