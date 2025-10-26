package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

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

    public MotorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = 5.0;   
        mm.MotionMagicAcceleration = 10.0;    
        mm.MotionMagicJerk = 15.0;            
        configs.MotionMagic = mm;

        Slot0Configs slot0 = configs.Slot0;
        slot0.kS = 0.08;
        slot0.kV = 0.11;
        slot0.kA = 0.01;
        slot0.kP = 9;
        slot0.kI = 0.3;
        slot0.kD = 0.4;
        configs.Slot0 = slot0;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(configs);
        motor.setPosition(0);
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            targetPosition = motor.getPosition().getValueAsDouble() + 1.0;
            motor.setControl(motionMagicRequest.withPosition(targetPosition));
        });
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            targetPosition = motor.getPosition().getValueAsDouble() - 1.0;
            motor.setControl(motionMagicRequest.withPosition(targetPosition));
        });
    }

    public Command stopMotor() {
        return runOnce(() -> motor.stopMotor());
    }

    @Override
    public void periodic() {
        // check if motor reached the target within tolerance
        double currentPos = motor.getPosition().getValueAsDouble();
        if (Math.abs(targetPosition - currentPos) <= tolerance) {
            motor.stopMotor();               // stop the motor
            motor.setPosition(0);            // reset PID / encoder
            targetPosition = 0;              // reset target
        }
    }
}
