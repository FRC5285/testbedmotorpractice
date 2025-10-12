package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax; // WPILib motor

import frc.robot.Constants.MotorConstants;

public class MotorSubsystem extends SubsystemBase {

    // Motor object using WPILib
    private final PWMSparkMax m_motor;

    // Target rotations (simulated)
    private double targetPositionRotations = 0;

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {
        m_motor = new PWMSparkMax(MotorConstants.motorCanId); // CAN ID or PWM channel
    }

    /** Turn motor 360° clockwise (simulated). */
    public Command turnClockwise360() {
        return runOnce(() -> {
            targetPositionRotations += 1.0;
            m_motor.set(1.0); // full speed
        });
    }

    /** Turn motor 360° counterclockwise (simulated). */
    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            targetPositionRotations -= 1.0;
            m_motor.set(-1.0); // full speed
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Rotations", targetPositionRotations);
    }
}


