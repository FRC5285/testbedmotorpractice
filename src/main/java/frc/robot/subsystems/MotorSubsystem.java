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

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(0); 
    public MotorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        MotionMagicConfigs mm = configs.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)).withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)).withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(15));

        Slot0Configs slot0 = configs.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.5;

        motor.getConfigurator().apply(configs);
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            motor.setControl(new MotionMagicVoltage(Rotations.of(1)));
        });
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            motor.setControl(new MotionMagicVoltage(Rotations.of(-1)));
        });
    }

    @Override
    public void periodic() {
    }
}