package frc.robot;

public final class Constants {

    public static class MotorConstants {
        public static final int motorCanId = 1; // TalonFX CAN ID

        // PID constants
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0; // Xbox controller port
    }
}

