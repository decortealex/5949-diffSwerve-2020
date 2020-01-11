/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class REVEncoder extends SubsystemBase {
  private DutyCycleEncoder encoder;
  private int channel;
  /**
   * Creates a new REV Encoder port.
   * @param port port that the encoder is attached to.
   */
  public REVEncoder(int port) {
    encoder = new DutyCycleEncoder(port);
    this.channel = port;
  }

  public double getVal() {
    if(encoder.isConnected()) {
      return encoder.get();
    } else {
      System.out.println("encoder " + this.channel + "not connected");
      return 0.0;
    }
  }

  /**
   * @return Current angle in radians
   */
  public double getAngleRad() {
    if(encoder.isConnected()) {
      return (encoder.get() % 1) * (2 * Math.PI);
    } else {
      System.out.println("encoder " + this.channel + "not connected");
      return 0.0;
    }
  }

  /**
   * @return Current angle in degrees
   */
  public double getAngleDeg() {
    if(encoder.isConnected()) {
      return (encoder.get() % 1) * 360;
    } else {
      System.out.println("encoder " + this.channel + "not connected");
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
