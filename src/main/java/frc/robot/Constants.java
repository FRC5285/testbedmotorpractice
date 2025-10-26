package frc.robot;

public final class Constants {

    // Operator/controller constants
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0; // USB port of your joystick/gamepad
    }

    // Motor constants
    public static final class MotorConstants {
        public static final int kMotorCanId = 1; // CAN ID of your Talon FX
        public static final double kP = 0.1;    // PID constants
        public static final double kI = 0.4;
        public static final double kD = 0.0;
    }
}




