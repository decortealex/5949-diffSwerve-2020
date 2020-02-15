/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

// import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.subsystems.DiffSwerveMod;
import frc.subsystems.Drivetrain;
import frc.subsystems.REVEncoder;
import frc.commands.SwerveControlCommand;
import frc.controllers.XOneController;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {  
  public static Drivetrain m_dt;
  public static XOneController m_joystick;
  public static REVEncoder encoder;
  public static AHRS m_gyro;
  // public static DiffSwerveMod mod;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
  */
  @Override
  public void robotInit() {
    m_dt = new Drivetrain();
    m_joystick = new XOneController(0);
    // m_gyro = new AHRS(SerialPort.Port.kUSB);
    // mod = new DiffSwerveMod(DiffSwerveMod.ModuleID.FR);
    CommandScheduler.getInstance().setDefaultCommand(m_dt, new SwerveControlCommand());    
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }



  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // mod.print();

  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println(m_gyro.getAngle());
    // mod.test();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

}
