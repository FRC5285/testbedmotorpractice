package frc.robot; // This file is in the main robot package

/**
 * The Constants class holds robot-wide constant values.
 * All constants should be declared as public static final
 * so they can be accessed anywhere without creating objects.
 */
public final class Constants {

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    // =========================== MOTOR CONSTANTS ===========================
    public static final class MotorConstants {

        // CAN ID for the Kraken X44 motor controller
        // (Change this to match the actual CAN ID of your testbed motor)
        public static final int motorCanId = 0;

        // PID controller tuning constants — start with these and tune as needed
        // kP: how strongly the motor reacts to position error
        // kI: fixes small steady-state errors (usually 0 unless needed)
        // kD: dampens oscillation (helps avoid overshoot)
        public static final double kP = 4.0;  // Proportional gain — adjust for your motor
        public static final double kI = 0.1;    // Integral gain — usually 0 for position control
        public static final double kD = 0.1;    // Derivative gain — helps smooth motion

        // Maximum velocity (rotations per second) the PID can command
        // This affects how quickly it moves toward its goal
        public static final double maxVelocity = 0.5; 

        // Maximum acceleration (rotations per second squared)
        // Higher = snappier motion, lower = smoother motion
        public static final double maxAccel = 1;

        
        
    }
}

