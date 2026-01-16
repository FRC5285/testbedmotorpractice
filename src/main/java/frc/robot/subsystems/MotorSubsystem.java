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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorSubsystem extends SubsystemBase {
    private final TalonFX thisMotor;
    private final ProfiledPIDController motorPID;
    // Initializes a duty cycle encoder on DIO pins 0
    Encoder m_encoder = new Encoder(0, 1);
    private double goalRotations = 0;
    private boolean motorOverride = false;

    private double control;
    public MotorSubsystem() {
        thisMotor = new TalonFX(MotorConstants.motorCanId);

        motorPID = new ProfiledPIDController(
            MotorConstants.kP, MotorConstants.kI, MotorConstants.kD,
            new TrapezoidProfile.Constraints(MotorConstants.maxV, MotorConstants.maxA)
        );

        thisMotor.setPosition(0);

        motorPID.setTolerance(0.06);

        m_encoder.reset();
        m_encoder.setDistancePerPulse(1.0 / 248.0);

        // Telemetry
        SendableRegistry.add(this, "Motor");
        SmartDashboard.putData(this);
        control = 0;
    }

    public Command turnClockwise360() {
        return runOnce(() -> {
            this.control += 1;
            this.motorPID.setGoal(control);
        });
    }

    public Command turnCounterClockwise360() {
        return runOnce(() -> {
            this.control -= 1;
            this.motorPID.setGoal(control);
        });
    }

    public Command stopMotor() {
        return runOnce(() -> {
          this.motorOverride = true;
          thisMotor.stopMotor();
        });
      }

    @Override
    public void periodic() {
        control = m_encoder.getDistance();
        motorPID.setGoal(control);
        double calcAmt = motorPID.calculate(this.getCurrentPosition());
        this.thisMotor.set(calcAmt);
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

        builder.addDoubleProperty("control", () -> this.control, null);
    }

    public double getCurrentPosition(){
        double position = thisMotor.getPosition().getValueAsDouble();
        //return position < 0.0 ? position + 1.0 : position;
        return position;
    }
}


