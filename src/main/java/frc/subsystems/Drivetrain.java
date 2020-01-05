/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import frc.subsystems.DiffSwerveMod;
import frc.subsystems.DiffSwerveMod.ModuleID;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Combines Swerve modules into single drivetrain
 */

public class Drivetrain extends SubsystemBase {
  private DiffSwerveMod m_modFR;
  private DiffSwerveMod m_modFL;
  // private DiffSwerveMod m_modBR;
  // private DiffSwerveMod m_modBL;

  private Translation2d m_FLPos;
  private Translation2d m_FRPos;
  // private Translation2d m_BLPos;
  // private Translation2d m_BRPos;

  private SwerveDriveKinematics m_kinematics;

  public Drivetrain() {
    this.m_modFL = new DiffSwerveMod(ModuleID.FL);
    this.m_modFR = new DiffSwerveMod(ModuleID.FR);
    // this.m_modBL = new DiffySwervePID(ModuleID.BL);
    // this.m_modBR = new DiffySwervePID(ModuleID.BR)

    this.m_FLPos = new Translation2d(0.346, 0.248);
    this.m_FRPos = new Translation2d(0.346, -0.248);
    // this.m_BLPos = new Translation2d(-0.346, 0.248);
    // this.m_BRPos = new Translation2d(-0.346, -0.248);

    this.m_kinematics = new SwerveDriveKinematics(
      this.m_FLPos, this.m_FRPos);
  }

  /**
   * Enables internal PID controllers for each swerve module
   */
  public void enable() {
    this.m_modFR.enable();
    this.m_modFL.enable();
    // this.m_modBR.enable();
    // this.m_modBL.enable();
  }

  /**
   * Disables the internal PID controllers for each swerve module
   */
  public void disable() {
    this.m_modFR.stop();
    this.m_modFL.stop();
    // this.m_modBR.stop();
    // this.m_modBL.stop();
  }

  /**
   * Uses basic module angle + speed driving scheme
   * @param angle angle to set modules to
   * @param power speed at which to drive the modules
   */
  public void drive(double angle, double power) {
    this.m_modFR.moveMod(angle, power);
    this.m_modFL.moveMod(angle, power);
    // this.m_modBR.moveMod(angle, power);
    // this.m_modBL.moveMod(angle, power);
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

    this.m_modFL.moveMod(modStates[0].angle.getDegrees(), modStates[0].speedMetersPerSecond);
    this.m_modFR.moveMod(modStates[1].angle.getDegrees(), modStates[1].speedMetersPerSecond);
  }
}
