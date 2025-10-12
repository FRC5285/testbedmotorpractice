package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.WPI_TalonFX;
import com.ctre.phoenix6.controls.PositionVelocityControl;
import frc.robot.Constants.MotorConstants;

public class MotorSubsystem extends SubsystemBase {

    private final WPI_TalonFX m_motor;
    private double targetPositionRotations = 0;

    public MotorSubsystem() {
        m_motor = new WPI_TalonFX(MotorConstants.motorCanId);

        // Apply PID constants
        m_motor.getPIDController().setP(MotorConstants.kP);
        m_motor.getPIDController().setI(MotorConstants.kI);
        m_motor.getPIDController().setD(MotorConstants.kD);
        m_motor.getPIDController().setFF(MotorConstants.kF);
    }

    // Clockwise 360° rotation
    public Command turnClockwise360() {
        return runOnce(() -> {
            targetPositionRotations += 1.0; // 1 rotation clockwise
            m_motor.setControl(new PositionVelocityControl(targetPositionRotations, 0));
        });
    }

    // Counterclockwise 360° rotation
    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            targetPositionRotations -= 1.0; // 1 rotation counterclockwise
            m_motor.setControl(new PositionVelocityControl(targetPositionRotations, 0));
        });
    }

    @Override
    public void periodic() {
        // Show telemetry on SmartDashboard
        SmartDashboard.putNumber("Motor Rotations", m_motor.getPosition().getValue());
        SmartDashboard.putNumber("Target Rotations", targetPositionRotations);
    }
}


