package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class MotorConstants {
        // Constants for the motor go here
        public static final double CruiseVelocity = 100;
        public static final double ACceleration = 160;
        public static final double Jerk = 1600;


        public static final double kS = 0.17255;
        public static final double kV = 0.11311;
        public static final double kA = 0.0050611;
        public static final double kp = 0.10589;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        /** The CAN ID for the motor */



    }
}
