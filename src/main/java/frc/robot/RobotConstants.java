package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class RobotConstants {
    // Swerve Motors
    public static final int FL_motor1 = 1;
    public static final int FL_motor2 = 2;
    public static final int FR_motor1 = 4;
    public static final int FR_motor2 = 3;
    public static final int BL_motor1 = 7;
    public static final int BL_motor2 = 8;
    public static final int BR_motor1 = 5;
    public static final int BR_motor2 = 6;

    // Swerve module absolute encoders
    public static final int FL_enc = 0;
    public static final int FR_enc = 1;
    public static final int BL_enc = 2;
    public static final int BR_enc = 3;


    // Control Panel Motor
    public static final int CP_Motor = 9;

    // Gyro Port
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;

    // Robot Dimensions
    public static final double x_dist_front = 0.312;
    public static final double x_dist_back = 0.320;
    public static final double y_dist = 0.276;

    // Important Constants
    public static final double coulsonRad = .0381;

    public static final double SWERVE_RATIO = 60;
    public static final double MAXRPM = 5700;
    
    // Color Constants for Control Panel
    public static Color red = new Color(255, 0, 0);
    public static Color blue = new Color(0, 0, 255);
    public static Color green = new Color(0, 255, 0);
    public static Color yellow = new Color(255, 255, 0);

}