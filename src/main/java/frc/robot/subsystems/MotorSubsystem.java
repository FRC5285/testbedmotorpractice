package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    private final TalonFX motor = new TalonFX(MotorConstants.motorCanId); 
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private double targetPosition = 0;
    private double pendingTurns = 0;
    DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);

    public MotorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = MotorConstants.CruiseVelocity;   
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

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.setPosition(0);
        motor.getConfigurator().apply(configs);
        }

    public Command stopMotor() {
        return runOnce(() -> motor.stopMotor());
    }

    @Override
    public void periodic() {
        double currentPos = motor.getPosition().getValueAsDouble();
        double encoderPos = m_encoder.get();
        targetPosition = encoderPos;
        

        motor.setControl(motionMagicRequest.withPosition(targetPosition).withSlot(0));
        // check if motor reached the target within tolerance
        SmartDashboard.putNumber("rotations", currentPos);
        SmartDashboard.putNumber("traget", targetPosition);
        SmartDashboard.putNumber("error", (currentPos-targetPosition));
        }
}