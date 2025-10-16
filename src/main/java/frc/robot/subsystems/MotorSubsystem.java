package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import static edu.wpi.first.units.Units.Rotations;

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

        resetMotor();

        SmartDashboard.putData("motor PID", motorPID);
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            resetMotor();
            double current = thisMotor.getRotorPosition().getValueAsDouble();
            goalRotations = current + 1.0; // +1 rotation so clockwise
            motorPID.setGoal(goalRotations);
        }).andThen(run(this::updatePID)
        .until(() -> motorPID.atGoal())
        .andThen(stopClimb()));
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            resetMotor();
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

    public void resetMotor() {
        thisMotor.setPosition(0);
        motorPID.reset(0);
        //motorPID.setGoal(0);
        //goalRotations = 0;
    }

    // update PID
    private void updatePID() {
        double currentPosition = thisMotor.getRotorPosition().getValueAsDouble();
        double output = this.motorPID.calculate(currentPosition);

        output = Math.max(0.0 , Math.min(1.0, output));
        thisMotor.set(output);

        // SmartDashboard.putNumber("output", output);
        // SmartDashboard.putNumber("currentPosition", currentPosition);
        // SmartDashboard.putNumber("goalRotations", goalRotations);
    }

    @Override
    public void periodic() {
        //updatePID();
        System.out.println(thisMotor.getRotorPosition().getValueAsDouble());
        System.out.println(motorPID.atGoal());
        if (motorPID.atGoal()) {
            stopClimb();
            resetMotor();
        }
    }
}


