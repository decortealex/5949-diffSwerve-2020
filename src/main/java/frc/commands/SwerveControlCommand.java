/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.mathutil.MathUtil;
import frc.robot.Robot;

public class SwerveControlCommand extends CommandBase {

  /**
   * Creates a new SwerveControlCommand.
   */
  public SwerveControlCommand() {
    addRequirements(Robot.m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LiveWindow.disableAllTelemetry();
    Robot.m_dt.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if(Robot.m_joystick.xButton.get()) {
      Robot.m_gyro.reset();
      System.out.printf("Gyro Reset%n");
    }
    */

    double xVel = MathUtil.map(-Robot.m_joystick.getLeftYAxis(), -1, 1, -20, 20);
    double yVel = MathUtil.map(Robot.m_joystick.getLeftXAxis(), -1, 1, -20, 20);
    double radVel = MathUtil.map(-Robot.m_joystick.getRightXAxis(), -1, 1, -Math.PI * 5, Math.PI * 5);
    // double robotAngle = Robot.m_gyro.getAngle();

    // Robot.m_dt.swerve(xVel, yVel, radVel /*, robotAngle*/);
    Robot.m_dt.drive(Robot.m_joystick.getRightAxisAngle(), Robot.m_joystick.getLeftYAxis());

    // Robot.m_dt.test();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_dt.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
