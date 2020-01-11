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
  private static double kI = 5.5-2;
  private static double kD = 8.5;
  private static double kF = 9e-3;
  private static double period = .025;

  private NEOMotor motor0;
  private NEOMotor motor1;

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

    setOutputRange(-5700, 5700);
    setAbsoluteTolerance(1.5);
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
    return Math.floor(this.getPosAvg() * RobotConstants.SWERVE_RATIO);
  }

  /**
   * 
   * @return Module Angle wrapped around
   */
  public double getModPosWrap() {
    return MathUtil.boundHalfAngleRad((double)(this.getModAngle()));
  }

  /**
   * Moves Swerve module to desired angle and runs at desired power in closed loop control
   * @param angle angle to hold the module at
   * @param power speed to run the module at (in meters per second)
   */
  public void moveMod(double angle, double power) {
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

  public void moveModRad(double rad, double power) {
    double target = (MathUtil.wrapAngleRad(rad)) * (180 / Math.PI);
    this.moveMod(target, power);
  }

  /**
   * both motors stop and internal PID controllers are disabled
   */
  public void stop() {
    motor0.stop();
    motor1.stop();
  }

  // @Override
  // protected double getMeasurement() {
  //   return this.getModAngle();
  // }

  @Override
  protected double returnPIDInput() {
    return this.getModAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    this.output = output;
  }

  @Override
  protected void initDefaultCommand() {

  }
}
