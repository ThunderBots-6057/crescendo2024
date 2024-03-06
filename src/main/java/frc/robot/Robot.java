// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;;;;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_driver;
  private Joystick m_operator;
  private Long shootTime=0L; // the time for shooting mechanism

    // Define a constant to switch rapidly between code releases
    // Constant to set running code: ("baseline","competition","experimental","") just uncomment the release that you want active
    // then comment the line with blank string("")
  //String runCode = "";
  //String runCode = "baseline";
  //String runCode = "competition";
  String runCode = "experimental";


  // Smooth driving 0 = off 1 = on
//  int smoothDriving = 0;
  int smoothDriving = 1;


  // Constants to set timing for the shooting operation, load will position note at the feed wheel, feed will move the note to the
  // shooting wheel that will already be up to speed, reset will zero out the shoot timer and return to normal operation
  // the other operations will only happen until the free movement time is reached
  private final long m_shooter_load_triggerTime = 200;  // start the loader motor at this time on the trigger to position note to fire
  private final long m_shooter_feed_triggerTime = 450;  // start feed motor to shoot the note
  private final long m_shooter_feemovement_triggerTime = 350;  // start feed motor to shoot the note

  private final long m_shooter_reset_triggerTime = 700;  // reset shooting  

  // Constants for climber multipliers since they are geared different and will have different speeds of climb
  private final double m_climber_left_multiplier = 1.0;
  private final double m_climber_right_multiplier = 0.8;

  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(4);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(51);
  private final WPI_VictorSPX m_rightRearMotor = new WPI_VictorSPX(3);

  private final WPI_VictorSPX m_shooter_wheel = new WPI_VictorSPX(2);
  private final WPI_VictorSPX m_shooter_feed = new WPI_VictorSPX(11);
  private final WPI_VictorSPX m_shooter_load = new WPI_VictorSPX(20);

  // Climber motors defined and driven separately but at the same time because the gearboxes
  // are built different so they go at different speeds for the same distance
  private final WPI_VictorSPX m_climber_left = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_climber_right = new WPI_VictorSPX(8);

  // the intake to pick up the note from the ground and feed to the launcher
  private final WPI_VictorSPX m_floor_intake = new WPI_VictorSPX(10);
  private final WPI_VictorSPX m_intake_lift = new WPI_VictorSPX(9);

  // private final WPI_VictorSPX m_intake_drive = new WPI_VictorSPX(9);
  // private final WPI_VictorSPX m_intake_fold = new WPI_VictorSPX(10);
  // private final WPI_VictorSPX m_leftRearMotor = new WPI_VictorSPX(1); we dont know the number

 
  @Override

  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightFrontMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.setInverted(true);
    m_shooter_load.setInverted(true);

    // Make the rears follow the fronts...
    m_leftRearMotor.follow((m_leftFrontMotor));
    m_rightRearMotor.follow((m_rightFrontMotor));

    m_robotDrive = new DifferentialDrive(m_leftFrontMotor::set, m_rightFrontMotor::set);
    m_driver = new Joystick(0);
    m_operator = new Joystick(1);
  }
   //overrides the main code for 2 seconds so you cant move for 2 seconds


  @Override
  public void teleopPeriodic() {



    // ----------------------------------------------------------------------------------------------------------------
    // Start of waterfall code (to rapidly switch between code releases being deployed)
    // ----------------------------------------------------------------------------------------------------------------

    if (runCode == "baseline"){
     // ----------------------------------------------------------------------------------------------------------------
     // Start of baseline code
     // ----------------------------------------------------------------------------------------------------------------

    
      
      // Right bumper moves note from feeder to shooter wheel
      var feed_speed = m_operator.getRawButton(6) ? 1.0 : 0.0;
      m_shooter_feed.set(feed_speed);

      // turning on the shooter wheel then shooter feed and shooter load in correct order at the correct time then turning it off
      // if the button is not being pressed
      if ((m_operator.getRawAxis(3) >= 0) && (shootTime == 0L)) {
        shootTime=System.currentTimeMillis();
      }
      //starts the actual procces of shooting the note
      if ((m_operator.getRawAxis(3) >= 0) && (shootTime + 1500 <= System.currentTimeMillis())) {
        //m_shooter_load.set(1.0);
        Timer.delay(0.5);
        m_shooter_feed.set(1.0);
        Timer.delay(1.0);
      }
      // right trigger controls the launcher
      if (!(m_operator.getRawAxis(3) != 0)) {
        shootTime = 0L;
      }
    
      // Left bumper controls the loader
//      var loader_speed = m_operator.getRawButton(5) ? 1.0 : 0.0;
//      m_shooter_load.set(loader_speed);
     // ----------------------------------------------------------------------------------------------------------------
     // End of baseline code
     // ----------------------------------------------------------------------------------------------------------------

    } else if (runCode == "competition"){

     // ----------------------------------------------------------------------------------------------------------------
     // Start of competition code
     // ----------------------------------------------------------------------------------------------------------------

      // Right bumper moves note from feeder to shooter wheel
      var feed_speed = m_operator.getRawButton(6) ? 1.0 : 0.0;
      m_shooter_feed.set(feed_speed);

      // turning on the shooter wheel then shooter feed and shooter load in correct order at the correct time then turning it off
      // if the button is not being pressed
      if ((m_operator.getRawAxis(3) >= 0) && (shootTime == 0L)) {
        shootTime=System.currentTimeMillis();
      }
      //starts the actual procces of shooting the note
      if ((m_operator.getRawAxis(3) >= 0) && (shootTime + 1500 <= System.currentTimeMillis())) {
        //m_shooter_load.set(1.0);
        Timer.delay(0.5);
        m_shooter_feed.set(1.0);
        Timer.delay(1.0);
      }
      // right trigger controls the launcher
      if (!(m_operator.getRawAxis(3) != 0)) {
        shootTime = 0L;
      }
    
      // Left bumper controls the loader
      //var loader_speed = m_operator.getRawButton(5) ? 1.0 : 0.0;
      //m_shooter_load.set(loader_speed);

     // ----------------------------------------------------------------------------------------------------------------
     // End of competition code
     // ----------------------------------------------------------------------------------------------------------------


    } else if (runCode == "experimental"){

     // ----------------------------------------------------------------------------------------------------------------
     // Start of experimental code
     // ----------------------------------------------------------------------------------------------------------------

      if (m_driver.getRawButtonReleased(6)){
        if (smoothDriving == 0){
          smoothDriving = 1;
        } else {
          smoothDriving = 0;
        }
      }
      // Make driver controller operate tank drive
      if (smoothDriving == 1) {
        m_robotDrive.tankDrive(-Math.pow(m_driver.getRawAxis(1),3),-Math.pow(m_driver.getRawAxis(5),3));
      } else {
        m_robotDrive.tankDrive(-m_driver.getRawAxis(1),-m_driver.getRawAxis(5));
      }
      

    // Set button 1 to extend on both climbers at the right multiplier

      // turning on the shooter wheel then shooter feed and shooter load in correct order at the correct time then turning it off

    
      // Start shooting timer
      if ((m_operator.getRawAxis(3) > 0) && (shootTime == 0L)) {
        shootTime=System.currentTimeMillis();
      }


      // if shoot  Time ( shooting timer ) is counting up
      if (shootTime > 0) {
        m_shooter_wheel.set(1.0);

        if ((shootTime + m_shooter_load_triggerTime) < System.currentTimeMillis()){
          m_shooter_load.set(1.0);

        }
        if ((shootTime + m_shooter_feed_triggerTime) < System.currentTimeMillis()){
          m_shooter_feed.set(1.0);
        }
        if ((shootTime + m_shooter_reset_triggerTime) >= System.currentTimeMillis()){
         m_shooter_feed.set(0.0);
         m_shooter_load.set(0.0);
        }

        if (shootTime + m_shooter_feemovement_triggerTime <= System.currentTimeMillis()){
          
          // Activate Left bumper control of the loader
          //var loader_speed = m_operator.getRawButton(5) ? 1.0 : 0.0;
          //m_shooter_load.set(loader_speed);

        }



      } else { // This is when shooting is not happening

        m_shooter_wheel.set(0.0);
        m_shooter_load.set(0.0);
        m_shooter_feed.set(0.0);

        // Activate Left bumper control of the loader
        var loader_speed = m_operator.getRawButton(5) ? 1.0 : 0.0;
        m_shooter_load.set(loader_speed);

      }


      // Set button 0 to retract on both climbers at the right multiplier
      if (m_operator.getRawButton(2)) {
        m_climber_left.set(1 * m_climber_left_multiplier);
        m_climber_right.set(1 * m_climber_right_multiplier);
      } else if (m_operator.getRawButton(1)){
        m_climber_left.set(-1 * m_climber_left_multiplier);
        m_climber_right.set(-1 * m_climber_right_multiplier);
      } else {
        m_climber_left.set(0);
        m_climber_right.set(0);

      }

    // this is to move the intake up and down to load it into the launcher
      if ((m_operator.getPOV(0) == 225) || (m_operator.getPOV(0) == 180) || (m_operator.getPOV(0) == 135)) {
        m_intake_lift.set(0.35);
      } else if ((m_operator.getPOV(0) == 315) || (m_operator.getPOV(0) == 0) || (m_operator.getPOV(0) == 45)) {
        m_intake_lift.set(-0.35);
      } else {
        m_intake_lift.set(0);
      } 


      // this is to pick it up from the ground
      if (m_operator.getRawButton(3)) {
        m_floor_intake.set(1);
      } else {
        m_floor_intake.set(0.0);
      }
      if (m_operator.getRawButton(4)) {
        m_floor_intake.set(-1);
      } else if (!m_operator.getRawButton(3)){
        m_floor_intake.set(0.0);
      }


      //starts the actual procces of shooting the note
      //if ((m_operator.getRawAxis(3) >= 0) && (shootTime + 1500 <= System.currentTimeMillis())) {
      //  m_shooter_load.set(1.0);
      //  Timer.delay(0.5);
      //  m_shooter_feed.set(1.0);
      //  Timer.delay(1.0);
      //}
      // right trigger not pressed
      if ((m_operator.getRawAxis(3) == 0)) {
        shootTime = 0L;
      }
    
// ----------------------------------------------------------------------------------------------------------------
// End of experimental code
// ----------------------------------------------------------------------------------------------------------------


    } else {
      System.out.println("runCode constant set to invalid value=" + runCode);
    }

  }
}