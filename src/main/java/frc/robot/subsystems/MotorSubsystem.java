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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX thisMotor;
    private final ProfiledPIDController motorPID;
    private double goalRotations = 0;
    private boolean motorOverride = false;

    private double control;
    public MotorSubsystem() {
        thisMotor = new TalonFX(MotorConstants.motorCanId);
        //thisMotor.setPosition(0);

        motorPID = new ProfiledPIDController(
            MotorConstants.kP, MotorConstants.kI, MotorConstants.kD,
            new TrapezoidProfile.Constraints(MotorConstants.maxV, MotorConstants.maxA)
        );

        // // bsic profiling and pid
        // // in init function, set slot 0 gains
        // var slot0Configs = new Slot0Configs();
        // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        // slot0Configs.kI = 0; // no output for integrated error
        // slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        // thisMotor.getConfigurator().apply(slot0Configs);

        // Final target of 360 rot, 0 rps
        //TrapezoidProfile.State m_goal = new TrapezoidProfile.State(360, 0);
        //TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        //motorPID.setGoal(thisMotor.getRotorPosition().getValueAsDouble());
        //motorPID.enableContinuousInput(0.0, 1.0);
        //motorPID.setTolerance(0.01);

        //resetMotor();
        thisMotor.setPosition(0);

        motorPID.setTolerance(0.1);

        // Telemetry
        SendableRegistry.add(this, "Motor");
        SmartDashboard.putData(this);
        control = 0;
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            control += 1;

            //this.thisMotor.setPosition(control);
            this.motorPID.setGoal(control);
        });
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            control -= 1;

            //this.thisMotor.setPosition(control);
            this.motorPID.setGoal(control);
        });
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

    @Override
    public void periodic() {
        motorPID.setGoal(control);
        double calcAmt = motorPID.calculate(this.getCurrentPosition());
        // SmartDashboard.putNumber("calcAmt: ", calcAmt);
        // SmartDashboard.putNumber("Motor Position: ", this.getCurrentPosition());
        // SmartDashboard.putBoolean("atGoal", motorPID.atGoal());
        //this.thisMotor.setPosition(calcAmt);
        this.thisMotor.set(calcAmt);
        // if (motorPID.atGoal()) {
        //     motorOverride = true;
        //     thisMotor.stopMotor();
        // }
    }

    // Telemetry
    @Override
    public void initSendable(SendableBuilder builder){
        // goal rotations
        builder.addDoubleProperty("Goal Rotations", () -> this.control, null);

        // actual rotations
        builder.addDoubleProperty("Motor Rotations", () -> this.thisMotor.getPosition().getValueAsDouble(), null);

        // at goal
        builder.addBooleanProperty("At Goal", () -> this.motorPID.atGoal(), null);
    }

    public double getCurrentPosition(){
        double position = thisMotor.getPosition().getValueAsDouble();
        //return position < 0.0 ? position + 1.0 : position;
        return position;
    }

}


