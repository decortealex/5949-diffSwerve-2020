/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.mathutil.MathUtil;

import com.revrobotics.CANSparkMaxLowLevel;

/**
 * Differential Swerve Module Class used to control individual swerve modules.
*/
public class DiffSwerveMod extends PIDSubsystem {
  public enum ModuleID {
    FR, FL, BL, BR;
  }

  /*
  two motor rotation: 
    kP = 0.0014;
    kI = 0.0;
    kD = .0021;
    kF = .007;

    
  private static double kP = 0.00067;
  private static double kI = 0.0;
  private static double kD = 0.0015;
  private static double kF = 0.007;

  */

  private static double kP = 2.5;
  private static double kI = 5.5e-2;
  private static double kD = 8.5;
  private static double kF = 9e-3;

  private static double period = .025;

  private static double setpoint = 0;

  private static final double SWERVE_RATIO = 60;
  private static final double MAXRPM = 5700;

  // private CANSparkMax motor0;
  // private CANSparkMax motor1;

  // private CANEncoder motor0Enc;
  // private CANEncoder motor1Enc;

  private NEOMotor motor0;
  private NEOMotor motor1;

  private ModuleID modID;

  private double output;


  public DiffSwerveMod(ModuleID id) {
    super("Differential", kP, kI, kD, kF, period);

    switch(id) {
      case FR:
        motor0 = new NEOMotor(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(4, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case FL:
        motor0 = new NEOMotor(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case BR:
        motor0 = new NEOMotor(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      case BL:
        motor0 = new NEOMotor(7, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor1 = new NEOMotor(8, CANSparkMaxLowLevel.MotorType.kBrushless);
        break;
      default:
        System.out.println("id is invalid");
    }

    setOutputRange(-5700, 5700);
    setAbsoluteTolerance(1.5);
  }

  /**
   * Sets desired setpoint for PID loop to approach
   * @param double setpoint to approach
   */
  @Override
  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
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
    return Math.floor(this.getPosAvg() * SWERVE_RATIO);
  }

  public double getModPosWrap() {
    return MathUtil.boundHalfAngleRad((double)(this.getModAngle()));
  }

  /**
   * Moves Swerve module to desired angle and runs at desired power in closed loop control
   * @param angle angle to hold the module at
   * @param power speed to run the module at
   */
  public void moveMod(double angle, double power) {
    double target = MathUtil.boundHalfAngleDeg(angle);
    this.setSetpoint(angle);
    // System.out.printf("%nPV: %4.2f%n", this.getModAngle());
    motor0.set(this.output + (power * MAXRPM));
    motor1.set(this.output - (power * MAXRPM));
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return this.getModAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    this.output = output;
  }
}
