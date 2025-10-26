package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX thisMotor;
    private final ProfiledPIDController motorPID;
    private double goalRotations = 0;
    private boolean motorOverride = false;

    public MotorSubsystem() {
        thisMotor = new TalonFX(MotorConstants.motorCanId);
        //thisMotor.setPosition(0);

        motorPID = new ProfiledPIDController(
            MotorConstants.kP, MotorConstants.kI, MotorConstants.kD,
            new TrapezoidProfile.Constraints(MotorConstants.maxV, MotorConstants.maxA)
        );

        // bsic profiling and pid
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        thisMotor.getConfigurator().apply(slot0Configs);

        // Final target of 360 rot, 0 rps
        //TrapezoidProfile.State m_goal = new TrapezoidProfile.State(360, 0);
        //TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        //motorPID.setGoal(thisMotor.getRotorPosition().getValueAsDouble());
        //motorPID.enableContinuousInput(0.0, 1.0);
        //motorPID.setTolerance(0.01);

        resetMotor();

        SmartDashboard.putData("motor PID", motorPID);
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            // create a position closed-loop request, voltage output, slot 0 configs
            final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

            // set position to 10 rotations
            thisMotor.setControl(m_request.withPosition(-1));
        });
        //return runOnce(() -> thisMotor.set(0.3));
        /*.andThen(run(this::updatePID)
        .until(() -> motorPID.atGoal())
        .andThen(stopClimb()));*/
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            // create a position closed-loop request, voltage output, slot 0 configs
            final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

            // set position to 10 rotations
            thisMotor.setControl(m_request.withPosition(-1));
        });
        //return runOnce(() -> thisMotor.set(-0.3));
        /*.andThen(run(this::updatePID)
        .until(() -> motorPID.atGoal())
        .andThen(stopClimb()));*/
    }

    public Command stopClimb() {
        return runOnce(thisMotor::stopMotor);
    }

    public Command stopMotor() {
        return runOnce(() -> {
          this.motorOverride = true;
          thisMotor.stopMotor();
        });
      }

    public void resetMotor() {
        //thisMotor.setPosition(0);
        motorPID.reset(this.getCurrentPosition());
        //motorPID.setGoal(0);
        //goalRotations = 0;
    }

    public boolean isAtSetpoint() {
        return motorPID.atSetpoint();
      }

    // update PID
    private void updatePID() {
        double currentPosition = thisMotor.getPosition().getValueAsDouble();
        double output = this.motorPID.calculate(currentPosition, goalRotations);

        double calcAmt = motorPID.calculate(currentPosition);

        output = Math.max(0.0 , Math.min(1.0, output));
        thisMotor.set(calcAmt);

        // SmartDashboard.putNumber("output", output);
        // SmartDashboard.putNumber("currentPosition", currentPosition);
        // SmartDashboard.putNumber("goalRotations", goalRotations);
    }

    @Override
    public void periodic() {
        // double calcAmt = motorPID.calculate(this.getCurrentPosition(), motorPID.getGoal());
        // SmartDashboard.putNumber("calcAmt: ", calcAmt);
        // SmartDashboard.putNumber("Motor Position: ", this.getCurrentPosition());
        // SmartDashboard.putBoolean("atGoal", motorPID.atGoal());
        // if (motorOverride == false) this.thisMotor.set(calcAmt);
        // if (motorPID.atGoal()) {
        //     motorOverride = true;
        //     thisMotor.stopMotor();
        // }
    }

    public double getCurrentPosition(){
        double position = thisMotor.getPosition().getValueAsDouble();
        //return position < 0.0 ? position + 1.0 : position;
        return position;
      }
}


