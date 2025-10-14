package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX thisMotor;
    private final ProfiledPIDController motorPID;
    private double goalRotations = 0;

    public MotorSubsystem() {
        thisMotor = new TalonFX(MotorConstants.motorCanId);
        thisMotor.setPosition(0);

        motorPID = new ProfiledPIDController(
            MotorConstants.kP, MotorConstants.kI, MotorConstants.kD,
            new TrapezoidProfile.Constraints(MotorConstants.maxV, MotorConstants.maxA)
        );

        motorPID.reset(0);
        goalRotations = 0;

        SmartDashboard.putData("motor PID", motorPID);
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            double current = thisMotor.getRotorPosition().getValueAsDouble();
            goalRotations = current + 1.0; // +1 rotation so clockwise
            motorPID.setGoal(goalRotations);
        }).andThen(run(this::updatePID)
        .until(() -> motorPID.atGoal())
        .andThen(stopClimb()));
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            double current = thisMotor.getRotorPosition().getValueAsDouble();
            goalRotations = current - 1.0; // -1 rotation (coutnerclockwise)
            motorPID.setGoal(goalRotations);
        }).andThen(run(this::updatePID)
        .until(() -> motorPID.atGoal())
        .andThen(stopClimb()));
    }

    public Command stopClimb() {
        return runOnce(thisMotor::stopMotor);
    }

    // update PID
    private void updatePID() {
        double currentPosition = thisMotor.getRotorPosition().getValueAsDouble();
        double output = motorPID.calculate(currentPosition);

        output = Math.max(0.0, output);
        thisMotor.setVoltage(output);

        // SmartDashboard.putNumber("output", output);
        // SmartDashboard.putNumber("currentPosition", currentPosition);
        // SmartDashboard.putNumber("goalRotations", goalRotations);
    }

    @Override
    public void periodic() {
        updatePID();
    }
}


