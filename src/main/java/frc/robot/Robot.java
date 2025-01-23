// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;



/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 * adaptations made and used for the 2024 "Crecendo" competition in Belleville and Livonia
 * At the end there was these capabilities:
 * Driving: fast and slow modes, also options for brake and coast drive
 * Climbing: on chain (stage) with dual climbers
 * Intake-lift: up until limit switch hit and reposition on limit switch if bounces off, driver can stop intake
 *   part way for scoring in Amp
 * Intake: in and out, driver can eject at 50% speed for scoring in Amp
 * Shoot: shooting wheel runs for approx 1 second, then intake, load and feed activate at same time.
 * Autons:
 *   Disabled: does nothing
 *   Auton one: not renamed from default. will cross the line
 *   Auton two: will wait 5 seconds and then cross the line
 * 
 * Robot simulation environment saved on github as it's own repository
 * 
 * Intention of baseline, competition and experimental code sections was to quikly change between different but working code
 *   on the fly. This feature was never used and only continued development on "experimental" after very early code point was
 *   saved to baseline
 */


public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_driver;

  //Drive assignments to reference drives according to function

  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(4);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(51);
  private final WPI_VictorSPX m_rightRearMotor = new WPI_VictorSPX(3);

  // code for arduino management
  private SerialPort arduino;
  private Timer timer;



  // roboInit runs on robot boot, once and then exits
  @Override

  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.setInverted(true);
    m_rightRearMotor.setInverted(true);

    // Make the rears follow the fronts...
    m_leftRearMotor.follow((m_leftFrontMotor));
    m_rightRearMotor.follow((m_rightFrontMotor));

    // Setup Drive and control systems
    m_robotDrive = new DifferentialDrive(m_leftFrontMotor::set, m_rightFrontMotor::set);
    m_driver = new Joystick(0);

    // Add USB discovery for Arduino
    try {
      arduino = new SerialPort (9500, SerialPort.Port.kUSB);
      System.out.println("connected on USB!");
    } catch (Exception e) {
      System.out.println("Failed to connect on kUSB, trying kUSB1");
    try {
      arduino = new SerialPort(0, SerialPort.Port.kUSB1);
      System.out.println("Connected on USB1!");
    } catch (Exception e1) {
      System.out.println("Failed to connect on kUSB1, trying kUSB2");
    try {
      arduino = new SerialPort (9600, SerialPort.Port.kUSB2);
      System.out.println("Connected on USB2!");
   } catch (Exception e2) {
      System.out.println("failed to connect on kUSB2, all connection attempts failed");
    }
  }
}


  // arduino.write(new byte[] (x2), 1);
  try{
  arduino.write(new byte[] {'1'}, 1);
  System.out.println("wrote 1 to Arduino");
  } catch (Exception e3){
    System.out.println("Failed to write to Arduino");
  }
  
  
timer = new Timer();
timer.start();
}







  @Override
  public void teleopPeriodic() {

    m_robotDrive.tankDrive(-m_driver.getRawAxis(1)*0.6,-m_driver.getRawAxis(5)*0.6);

    if(timer.get() > 5) {
      System.out.println("wrote 2 to Arduino");
      arduino.write(new byte[] {'2'}, 1);
      timer.reset();
    }
      if(arduino.getBytesReceived() > 0) {
        System.out.print(arduino.readString());
      }
    }


 }
