package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import com.ctre.phoenix6.hardware.TalonFX;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(MotorConstants.motorCanId);
    private final TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(MotorConstants.maxV, MotorConstants.maxA);
    private final ProfiledPIDController motorPID = 
        new ProfiledPIDController(MotorConstants.kp, MotorConstants.ki, MotorConstants.kd, constraints);

    // track the target position in rotations
    private double goalPosition = 0;

    public MotorSubsystem() {
        motorPID.setTolerance(MotorConstants.tolerance);
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            double currentPos = motor.getRotorPosition().getValueAsDouble();
            goalPosition = currentPos + 1.0; // 1 rotation forward
            motorPID.setGoal(goalPosition);
        });
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            double currentPos = motor.getRotorPosition().getValueAsDouble();
            goalPosition = currentPos - 1.0; // 1 rotation backward
            motorPID.setGoal(goalPosition);
        });
    }

    @Override
    public void periodic() {
        double position = motor.getRotorPosition().getValueAsDouble();
        double output = motorPID.calculate(position);

        // optional: clamp voltage to safe range
        double maxVoltage = 12.0;
        if (output > maxVoltage) output = maxVoltage;
        if (output < -maxVoltage) output = -maxVoltage;

        motor.setVoltage(output);
    }
}
