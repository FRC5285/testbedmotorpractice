package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(0); 
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private double targetPosition = 0;
    private final double tolerance = 0.01; // rotations, tweak as needed
    private double pendingTurns = 0;

    public MotorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = 80;   
        mm.MotionMagicAcceleration = 160;    
        mm.MotionMagicJerk = 1600;
        configs.MotionMagic = mm;

        Slot0Configs slot0 =configs.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 4.8;
        slot0.kI = 0.0;
        slot0.kD = 0.1;
        configs.Slot0 = slot0;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(configs);
        }

    public Command turnClockwise360() {
        return runOnce(() -> {
            pendingTurns++;

        });
    }
    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            pendingTurns--;
        });
    }

    public Command stopMotor() {
        return runOnce(() -> motor.stopMotor());
    }

    @Override
    public void periodic() {
        double currentPos = motor.getPosition().getValueAsDouble();

        if (Math.abs(currentPos - targetPosition) < tolerance) {
            if (pendingTurns > 0) {
                pendingTurns--;
                targetPosition += 1.0;
            } else if (pendingTurns < 0) {
                pendingTurns++;
                targetPosition -= 1.0;
            }
        }

        motor.setControl(motionMagicRequest.withPosition(targetPosition).withSlot(0));
        // check if motor reached the target within tolerance
        SmartDashboard.putNumber("rotations", currentPos);
        SmartDashboard.putNumber("traget", targetPosition);
        }
}