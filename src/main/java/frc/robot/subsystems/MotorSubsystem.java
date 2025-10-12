package frc.robot.subsystems;

// Imported libraries and files
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableRegistry;

import static edu.wpi.first.units.Units.Rotations;

public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here
    private final TalonFX thisMotor;
    private final Encoder thisEncoder;
    //private final ProfiledPIDController thisPID;
    private boolean motorOverride = false;
    private final ProfiledPIDController motorPID;

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {
        thisMotor = new TalonFX(MotorConstants.motorCanId);
        thisMotor.setPosition(0);
        
        SendableRegistry.add(this, "Motor");
        SmartDashboard.putData(this);

        thisEncoder = new Encoder(MotorConstants.encoderA, MotorConstants.encoderB);
        thisEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);

        motorPID = new ProfiledPIDController(MotorConstants.kP, MotorConstants.kI, MotorConstants.kD,
            new TrapezoidProfile.Constraints(MotorConstants.maxV, MotorConstants.maxA)
        );
    }

    /**
     * Creates a command that turns the motor shaft 360 degrees clockwise.
     *
     * @return a command that turns the motor shaft 360 degrees clockwise.
     */
    public Command turnClockwise360() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        /*
        return runOnce(() -> {this.motorPID.reset(); this.motorPID.setSetpoint(MotorConstants.rotations);})
        .andThen(run(() -> {
            thisMotor.set(Math.max(this.motorPID.calculate(thisMotor.getPosition().getValue().in(Rotations)), 0.0));
        }))
        .until(() -> Math.abs(thisMotor.getPosition().getValue().in(Rotations)) >= MotorConstants.rotations)
        .andThen(this.stopClimb());
        */

        return runOnce(() -> {
            motorPID.setGoal(thisMotor.getRotorPosition().getValueAsDouble() + 360);
            //thisMotor.set(this.motorPID.calculate(thisEncoder.getDistance()));
        });

        // return run(() -> {
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }

    /**
     * Creates a command that turns the motor shaft 360 degrees counterclockwise.
     *
     * @return a command that turns the motor shaft 360 degrees counterclockwise.
     */
    public Command turnCounterClockwise360() {
        motorPID.setGoal(thisMotor.getRotorPosition().getValueAsDouble() - 360);
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        /*
        return runOnce(() -> {this.motorPID.reset(); this.motorPID.setSetpoint(MotorConstants.rotations2);})
        .andThen(run(() -> {
            thisMotor.set(Math.max(this.motorPID.calculate(thisMotor.getPosition().getValue().in(Rotations)), 0.0));
        }))
        .until(() -> Math.abs(thisMotor.getPosition().getValue().in(Rotations)) >= MotorConstants.rotations2)
        .andThen(this.stopClimb());
        */
        return runOnce(() -> {});
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }

    public Command stopClimb() {
        return runOnce(() -> thisMotor.stopMotor());
    }


    @Override // Rewrites (adds content to) a method from SubsystemBase
    public void periodic() {
        // This method will be called once per scheduler run (50 times per second)
        thisMotor.setPosition(this.motorPID.calculate(thisMotor.getRotorPosition().getValueAsDouble()));
        if (motorPID.atGoal()) thisMotor.stopMotor();
    }
}
