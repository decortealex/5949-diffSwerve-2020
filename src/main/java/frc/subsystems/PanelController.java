/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import java.util.ArrayList;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotConstants;

public class PanelController extends PIDSubsystem {
  private static final double ROTPERSECTOR = 0.95492965855;
  
  private final ColorSensorV3 m_sensor;
  private final ArrayList<Color> sequence;

  /**
   * Creates a new PanelController.
   */
  public PanelController(boolean turnsClockwise) {
    super(new PIDController(0, 0, 0));
    sequence = new ArrayList<Color>();

    if(turnsClockwise) {
      sequence.add(RobotConstants.red);
      sequence.add(RobotConstants.green);
      sequence.add(RobotConstants.blue);
      sequence.add(RobotConstants.yellow);
    } else {
      sequence.add(RobotConstants.red);
      sequence.add(RobotConstants.yellow);
      sequence.add(RobotConstants.blue);
      sequence.add(RobotConstants.green);

    }

    m_sensor = new ColorSensorV3(RobotConstants.i2cPort);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  /**
   * 
   * @return Color that the robot is detecting
   */
  public Color getColorRobot() {
    return m_sensor.getColor();
  }

  /**
   * 
   * @return Color that the FMS is detecting
   */
  public Color getColorField() {
    Color detectedColor = this.getColorRobot();
    int i = sequence.indexOf(detectedColor);

    return sequence.get((i+2) % sequence.size());
  }

  public double rotUntilColor(Color desiredColor) {
    Color fieldColor = this.getColorField();
    int fieldIndex = sequence.indexOf(fieldColor);
    int desiredIndex = sequence.indexOf(desiredColor);

    return (ROTPERSECTOR * (Math.abs(fieldIndex - desiredIndex)));
  }

}
