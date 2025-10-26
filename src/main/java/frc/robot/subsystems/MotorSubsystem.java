package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVelocity;
import com.ctre.phoenix6.signals.SoftwareSensor;
import com.ctre.phoenix6.signals.NeutralMode;

import frc.robot.Constants.MotorConstants;

public class MotorSubsystem extends SubsystemBase {

    private final TalonFX motor;

    public MotorSubsystem() {
        motor = new TalonFX(MotorConstants.kMotorCanId);

        // Apply the configuration
        motor.getConfigurator().apply(MotorConstants.motorConfig);

        // Set neutral mode
        motor.setNeutralMode(NeutralMode.Brake);
    }

    // Simple position-velocity control
    public void setMotor(double velocity) {
        PositionVelocity control = new PositionVelocity();
        control.velocity = velocity;
        motor.getControl().set(control);
    }

    public void stopMotor() {
        motor.getControl().set(new PositionVelocity());
    }
}






