package frc.robot;

import com.ctre.phoenix6.CANBus;

import org.wpilib.hardware.bus.CANPort;

import edu.wpi.first.wpilibj.Encoder;
//IDK how to do this its not letting me import encoder help pls


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

        /** The CAN ID for the motor */
        public static final CANBus motorCanId = new CANBus(CANPort.CAN_S4);
        public static final int deviceid = 0; 

        public static final double  fF_k_S = 0.0;
        public static final double  fF_k_V = 0.0;

        public static final double p = 0.0;
        public static final double i = 0.0;
        public static final double d = 0.0;

        public static final double m_V = 0.0;
        public static final double m_A = 0.0;

        public static final double motorGoal = 1.0;

        public static final Encoder REVThroughBoreEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);


    }
}
