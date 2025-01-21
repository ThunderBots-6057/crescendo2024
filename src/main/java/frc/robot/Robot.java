// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;;


public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_driver;
  private Joystick m_operator;


  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(4);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(51);
  private final WPI_VictorSPX m_rightRearMotor = new WPI_VictorSPX(3);

 



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
    m_operator = new Joystick(1);

}
    // Set the neutral mode to brake
    @Override
  public void teleopPeriodic() {
        m_robotDrive.tankDrive(-m_driver.getRawAxis(1)*0.6,-m_driver.getRawAxis(5)*0.6);
  
  }