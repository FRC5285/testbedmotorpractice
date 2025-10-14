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

        SmartDashboard.putData("Motor PID", motorPID);
    }

    /** Turns the motor shaft one full rotation clockwise (positive). */
    public Command turnClockwise360() {
        return runOnce(() -> {
            double current = thisMotor.getRotorPosition().getValueAsDouble();
            goalRotations = current + 1.0; // +1 rotation
            motorPID.setGoal(goalRotations);
        }).andThen(run(this::updatePID)
        .until(() -> motorPID.atGoal())
        .andThen(stopClimb()));
    }

    /** Turns the motor shaft one full rotation counterclockwise (negative). */
    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            double current = thisMotor.getRotorPosition().getValueAsDouble();
            goalRotations = current - 1.0; // -1 rotation
            motorPID.setGoal(goalRotations);
        }).andThen(run(this::updatePID)
        .until(() -> motorPID.atGoal())
        .andThen(stopClimb()));
    }

    /** Stops the motor */
    public Command stopClimb() {
        return runOnce(thisMotor::stopMotor);
    }

    /** Called periodically to update the PID output */
    private void updatePID() {
        double currentPosition = thisMotor.getRotorPosition().getValueAsDouble();
        double output = motorPID.calculate(currentPosition);

        // Clamp output between -1 and 1
        output = Math.max(-1.0, Math.min(1.0, output));
        thisMotor.set(output);

        SmartDashboard.putNumber("Motor Output", output);
        SmartDashboard.putNumber("Motor Position", currentPosition);
        SmartDashboard.putNumber("Motor Goal", goalRotations);
    }

    @Override
    public void periodic() {
        // You could also call updatePID() here if you always want the PID running
    }
}


