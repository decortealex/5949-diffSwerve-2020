/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Custom NEO motor class that wraps existing class and adds closed-loop velocity control using PID control on the
 * Spark MAX motor controller
*/
public class NEOMotor extends Subsystem {
  private CANSparkMax m_motor;
  private CANPIDController m_pidControl;
  private CANEncoder m_encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint;

  public NEOMotor(int deviceID, CANSparkMaxLowLevel.MotorType type) {
    m_motor = new CANSparkMax(deviceID, type);
    m_pidControl = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID values for NEO motor. These were experimentally obtained, and may have to be modified for
    // other configurations. MAKE SURE YOU KNOW WHAT YOU ARE DOING WHEN YOU MODIFY THESE! 
    kP = 4.5e-5;
    kI = 1.7e-6;
    kD = 1e-7;
    kIz = 0;
    kFF = 0;

    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5300;
    setPoint = 0;

    m_pidControl.setP(kP);
    m_pidControl.setI(kI);
    m_pidControl.setD(kD);
    m_pidControl.setIZone(kIz);
    m_pidControl.setFF(kFF);
    m_pidControl.setOutputRange(kMinOutput, kMaxOutput);
  }

  /**
   * @return Current position of NEO built-in encoder
   */
  public double getPos() {

    return Math.floor(m_encoder.getPosition());
  }

  /**
   * Sets NEO to run at set rpm using closed-loop control
   * @param rpm desired rpm
   */
  public void set(double rpm) {
    m_pidControl.setReference((rpm), ControlType.kVelocity);
  }

  public void setRaw(double decimal) {
    m_motor.set(decimal);
  }

  /**
   * sets NEO motor to 0, and PID setpoint to 0 (there is no method to 'disable' PID controller on Spark MAX)
   */
  public void stop() {
    m_motor.set(0);
    setPoint = 0;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Tester method to tune PID values through SmartDashBoard
   * @param decimal speed to run NEO motor
   */
   public void run(double decimal) {
    
    setPoint = decimal * maxRPM;
    m_pidControl.setReference(setPoint, ControlType.kVelocity);
  }
}
