/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.mathutil.MathUtil;
import frc.robot.RobotConstants;

public class DiffSwerveModule extends PIDSubsystem {
  public enum ModuleID {
    FL, FR, BL, BR;
  }

  private static double kP = 2.5;
  private static double kI = 5.5e-2;
  private static double kD = 8.5;
  // private static double kF = 9e-3;
  private static double period = .025;

  private NEOMotor motor0;
  private NEOMotor motor1;

  private double output;

  /**
   * Creates a new DiffSwerveModule.
   */
  public DiffSwerveModule(ModuleID id) {
    super(
        // The PIDController used by the subsystem
        new PIDController(kP, kI, kD, period));
    
    switch(id) {
      case FL:
        motor0 = new NEOMotor(RobotConstants.FL_motor1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(RobotConstants.FL_motor2, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case FR:
        motor0 = new NEOMotor(RobotConstants.FR_motor1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(RobotConstants.FR_motor2, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case BL:
        motor0 = new NEOMotor(RobotConstants.BL_motor1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(RobotConstants.BL_motor2, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case BR:
        motor0 = new NEOMotor(RobotConstants.BR_motor1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(RobotConstants.BR_motor2, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      default:
        System.out.println("id is invalid");
    }
  }

  @Override
  public void enable() {
    super.enable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    this.output = output;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return this.getModAngle();
  }

  public double getPosAvg() {
    return (motor0.getPos() + motor1.getPos()) / 2;
  }

  public double getModAngle() {
    return Math.floor(this.getPosAvg() * RobotConstants.SWERVE_RATIO);
  }

  public double getModPosWrap() {
    return MathUtil.boundHalfAngleRad((double)(this.getModAngle()));
  }
  
  public void moveModSmart(double angle, double power) {
    double target = MathUtil.boundHalfAngleDeg(angle);
    boolean isReversed = MathUtil.isReversed(angle);

    if(isReversed) {
      setSetpoint(-target);
    } else {
      setSetpoint(target);
    }

    motor0.set(this.output + (MathUtil.msToRpm(power)));
    motor1.set(this.output - (MathUtil.msToRpm(power)));
  }

  public void moveModDumb(double angle, double power) {
    setSetpoint(angle);

    motor0.set(this.output + power);
    motor1.set(this.output - power);
  }

  public void moveModRad(double rad, double power) {
    double target = (MathUtil.wrapAngleRad(rad)) * (180 / Math.PI);
    this.moveModSmart(target, power);
  }

  public void stop() {
    motor0.stop();
    motor1.stop();
  }

  public void troubleshoot() {
    motor0.set(200);
    motor1.set(200);
  }
}
