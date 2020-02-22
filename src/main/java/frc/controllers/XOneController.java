/**
 * Controller class for the driver
 * Instantiates buttons to facilitate writing code for attaching buttons to commands
 * 
 * Built for Official Xbox One Controller
*/
package frc.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class XOneController {
    private Joystick m_stick;
    private final static double DEADBAND = .1;

    private double prevAngle;

    public JoystickButton aButton;
    public JoystickButton bButton;
    public JoystickButton xButton;
    public JoystickButton yButton;
    public JoystickButton leftBumperButton;
    public JoystickButton rightBumperButton;
    public JoystickButton backButton;
    public JoystickButton startButton;
    public JoystickButton leftJoystickButton;
    public JoystickButton rightJoystickButton;


    public XOneController(int port) {
        m_stick = new Joystick(port);

        aButton = new JoystickButton(m_stick, 1);
        bButton = new JoystickButton(m_stick, 2);
        xButton = new JoystickButton(m_stick, 3);
        yButton = new JoystickButton(m_stick, 4);
        leftBumperButton = new JoystickButton(m_stick, 5);
        rightBumperButton = new JoystickButton(m_stick, 6);
        backButton = new JoystickButton(m_stick, 7);
        startButton = new JoystickButton(m_stick, 8);
        leftJoystickButton = new JoystickButton(m_stick, 9);
        rightJoystickButton = new JoystickButton(m_stick, 10);

        prevAngle = 0;
    }

    public double getLeftXAxis() {
        double val = m_stick.getRawAxis(0);
        return handleDeadband(val);
    }

    public double getLeftYAxis() {
        double val = m_stick.getRawAxis(1);
        return handleDeadband(-val);
    }

    public double getLeftAxisHypot() {
        double val = Math.hypot(this.getLeftXAxis(), this.getLeftYAxis());
        return handleDeadband(val);
    }

    public double getLeftAxisAngle() {
        double x = this.getLeftXAxis();
        double y = this.getLeftYAxis();
        double angleRad = Math.atan2(x, y);
        
        if(Math.abs(x) < 0.6 && Math.abs(y) < 0.6) {
            return -(prevAngle);
        } else {
            prevAngle = (Math.toDegrees(angleRad) + 180);
            if(prevAngle > 180) {
                return -(prevAngle - 180);
            } else {
                return -(prevAngle);
            }
        }
    }

    public double getRightXAxis() {
        double val = m_stick.getRawAxis(4);
        return handleDeadband(val);
    }

    public double getRightYAxis() {
        double val = m_stick.getRawAxis(5);
        return handleDeadband(val);
    }

    public double getRightAxisHypot() {
        double val = Math.hypot(this.getRightXAxis(), this.getRightYAxis());
        return handleDeadband(val);
    }

    public double getRightAxisAngle() {
        double x = this.getRightXAxis();
        double y = this.getRightYAxis();
        double angleRad = Math.atan2(x, y);
        
        if(Math.abs(x) < 0.6 && Math.abs(y) < 0.6) {
            // System.out.print("Joystick Angle: " + -prevAngle);
            return -(prevAngle);
        } else {
            // System.out.print("Joystick Angle: " + -prevAngle);
            prevAngle = (Math.toDegrees(angleRad) + 180);
            if(prevAngle > 180) {
                return -(prevAngle - 180);
            } else {
                return -(prevAngle);
            }
        }
    }

    public double getRightTrigger() {
        double val = m_stick.getRawAxis(3);
        return handleDeadband(val);
    }

    public double getLeftTrigger() {
        double val = m_stick.getRawAxis(2);
        return handleDeadband(val);
    }

    private double handleDeadband(double num) {
        if(Math.abs(num) < DEADBAND) {
            return 0;
        }

        return num;
    }
}