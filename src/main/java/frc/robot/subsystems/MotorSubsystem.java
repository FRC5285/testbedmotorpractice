package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(16); 
    private final TalonFX motor_1 = new TalonFX(17);
    private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

    public MotorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicAcceleration = MotorConstants.ACceleration;    
        mm.MotionMagicJerk = MotorConstants.Jerk;
        configs.MotionMagic = mm;

        Slot0Configs slot0 =configs.Slot0;
        slot0.kS = MotorConstants.kS;
        slot0.kV = MotorConstants.kV;
        slot0.kA = MotorConstants.kA;
        slot0.kP = MotorConstants.kp;
        slot0.kI = MotorConstants.ki;
        slot0.kD = MotorConstants.kd;
        configs.Slot0 = slot0;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor.setPosition(0);
        motor_1.setPosition(0);
        motor_1.getConfigurator().apply(configs);
        motor.getConfigurator().apply(configs);

        SendableRegistry.add(this, "Turret");
        SmartDashboard.putData(this);
        }

    public Command stopMotor() {
        return runOnce(() -> motor.stopMotor());
    }
    public Command runmotor() {
        return run(() -> {
            motor.setControl(motionMagicRequest.withVelocity(-100).withSlot(0));
            motor_1.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));
        });

    }

    @Override
    public void periodic() {
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("error",() -> (motor.getVelocity().getValueAsDouble() + 100), null);
        builder.addDoubleProperty("current speed",() -> (motor.getVelocity().getValueAsDouble()), null);
    }
}