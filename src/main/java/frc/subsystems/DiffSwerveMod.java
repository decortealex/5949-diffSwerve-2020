/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.mathutil.MathUtil;
import frc.robot.RobotConstants;

import com.revrobotics.CANSparkMaxLowLevel;

/**
 * Differential Swerve Module Class used to control individual swerve modules.
*/
public class DiffSwerveMod extends PIDSubsystem {
  public enum ModuleID {
    FR, FL, BL, BR;
  }

  private static double kP = 2.5;
  private static double kI = 0.055;
  private static double kD = 8.5;
  private static double kF = 0.009;
  private static double period = .025;

  private NEOMotor motor0;
  private NEOMotor motor1;

  // private REVEncoder abs_encoder;

  private double output;


  public DiffSwerveMod(ModuleID id) {
    super("DiffSwerve", kP, kI, kD, kF, period);
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

    // abs_encoder = new REVEncoder(RobotConstants.abs_encoder);

    setOutputRange(-5300, 5300);
    setAbsoluteTolerance(1.5);
  }

  @Override
  protected double returnPIDInput() {
    return this.getModAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    this.output = output;
    // System.out.println("output: " + this.output);
  }

  @Override
  protected void initDefaultCommand() {

  }

  /**
   * Enables internal PID Controller
   */
  @Override
  public void enable() {
    super.enable();
  }
  
  /**
   * 
   * @return The average of the encoder values on each built-in NEO motor encoder
   */
  public double getPosAvg() {
    return (motor0.getPos() + motor1.getPos()) / 2;
  }

  /**
   * 
   * @return Module Angle by taking average of the two motors and multiplying by gear ratio
   */
  public double getModAngle() {
    return (this.getPosAvg() * RobotConstants.SWERVE_RATIO);
  }

  /**
   * 
   * @return Module Angle wrapped around
   */
  public double getModPosWrap() {
    return MathUtil.boundHalfAngleRad((double)(this.getModAngle()));
  }

  public void moveModSmart(double angle, double power) {
    double target = MathUtil.boundHalfAngleDeg(angle);
    boolean isReversed = MathUtil.isReversed(angle);

    if(isReversed) {
      setSetpoint(-target);
      System.out.println("Reversed Module");
    } else {
      setSetpoint(target);
    }

    motor0.set(this.output + MathUtil.msToRpm(power));
    motor1.set(this.output - MathUtil.msToRpm(power));
    System.out.println("motor power set");
  }

  public void moveModDumb(double angle, double power) {
    setSetpoint(angle);

    motor0.set(this.output + power);
    motor1.set(this.output - power);
    System.out.println(this.output + power);
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
