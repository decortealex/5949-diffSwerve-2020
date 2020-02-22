/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import frc.robot.RobotConstants;
import frc.subsystems.DiffSwerveMod.ModuleID;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Combines Swerve modules into single drivetrain
 */

public class Drivetrain extends SubsystemBase {
  private DiffSwerveMod m_modFL;
  private DiffSwerveMod m_modFR;
  private DiffSwerveMod m_modBL;
  private DiffSwerveMod m_modBR;

  private Translation2d m_FLPos = new Translation2d(RobotConstants.x_dist_front, RobotConstants.y_dist);
  private Translation2d m_FRPos = new Translation2d(RobotConstants.x_dist_front, -RobotConstants.y_dist);
  private Translation2d m_BLPos = new Translation2d(-RobotConstants.x_dist_back, RobotConstants.y_dist);
  private Translation2d m_BRPos = new Translation2d(-RobotConstants.x_dist_back, -RobotConstants.y_dist);

  private final DiffSwerveMod[] swerveMods;
  private SwerveDriveKinematics m_kinematics;

  public Drivetrain() {
    this.m_modFL = new DiffSwerveMod(ModuleID.FL);
    this.m_modFR = new DiffSwerveMod(ModuleID.FR);
    this.m_modBL = new DiffSwerveMod(ModuleID.BL);
    this.m_modBR = new DiffSwerveMod(ModuleID.BR);

    final DiffSwerveMod[] swervemods = {this.m_modFL, this.m_modFR, this.m_modBL, this.m_modBR};
    this.swerveMods = swervemods;

    this.m_kinematics = new SwerveDriveKinematics(
      this.m_FLPos, this.m_FRPos, this.m_BLPos, this.m_BRPos);
  }

  /**
   * Enables internal PID controllers for each swerve module
   */
  public void enable() {
    for(int i = 0; i < this.swerveMods.length; i++) {
      swerveMods[i].enable();
    }
  }

  /**
   * Disables the internal PID controllers for each swerve module
   */
  public void disable() {
    for(int i = 0; i < this.swerveMods.length; i++) {
      swerveMods[i].stop();
    }
  }

  /**
   * Uses basic module angle + speed driving scheme
   * @param angle angle to set modules to
   * @param power speed at which to drive the modules
   */
  public void drive(double angle, double power) {
    for(int i = 0; i < this.swerveMods.length; i++) {
      if(i < 2)
        this.swerveMods[i].moveModSmart(angle, power);
      else
        this.swerveMods[i].moveModSmart(angle, -power);
    }
  }

  /**
   * Translates {@code x} velocity, {@code y} velocity and {@code rad} rotational velocity (all measured from chassis)
   * into individual module states and then applies those to each module.
   * @param x desired x velocity
   * @param y desired y velocity
   * @param rad desired rotational velocity
   */
  public void swerve(double x, double y, double rad) {
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, rad);
    SwerveModuleState[] modStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(modStates, 20);

    for(int i = 0; i < this.swerveMods.length; i++) {
      this.swerveMods[i].moveModSmart(modStates[i].angle.getDegrees(), modStates[i].speedMetersPerSecond);
    }
  }

  /**
   * Translates chassis velocities into individual module states and then applies those to each module.
   * Takes into account gyro for field-oriented drive.
   * @param x desired x velocity
   * @param y desired y velocity
   * @param rad desired rotational velocity
   * @param robotRotation current gyro angle
   */
  public void swerve(double x, double y, double rad, double robotRotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rad, Rotation2d.fromDegrees(robotRotation));
    SwerveModuleState[] modStates = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(modStates, 20);

    for(int i = 0; i < this.swerveMods.length; i++) {
      this.swerveMods[i].moveModSmart(modStates[i].angle.getDegrees(), modStates[i].speedMetersPerSecond);
    }
  }

  /**
   * tester method for each motor
   */
  public void test() {
    for(int i = 0; i < this.swerveMods.length; i++) {
      this.swerveMods[i].moveModSmart(90, 0.3);
    }
  }
}
