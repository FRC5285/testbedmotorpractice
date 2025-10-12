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
        public static final double kp = 0.1;
        public static final double ki = 0.1;
        public static final double kd = 0.1;
        public static final double maxV = 3.5;
        public static final double maxA = 2.5;
        /** The CAN ID for the motor */
        public static final int motorCanId = 0;
        public static final double tolerance = 0.025;
    }
}
