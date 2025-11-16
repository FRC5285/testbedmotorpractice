package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Imported libraries and files
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]

public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here
    private TalonFX motor;
    private double goalRotations;
    private ProfiledPIDController thePID;


    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {
        this.motor = new TalonFX(MotorConstants.motorCanId);
        this.goalRotations = 0.0;
        this.thePID = new ProfiledPIDController(MotorConstants.kP, MotorConstants.kI, MotorConstants.kD, new TrapezoidProfile.Constraints(MotorConstants.maxAccel, MotorConstants.maxVelocity));
        
        this.motor.setPosition(0,0);
        this.thePID.setGoal(this.goalRotations);

        SendableRegistry.add(this, "Motor");
        SmartDashboard.putData(this);
    }

    /**
     * Creates a command that turns the motor shaft 360 degrees clockwise.
     *
     * @return a command that turns the motor shaft 360 degrees clockwise.
     */
    public Command turnClockwise360() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            this.goalRotations -= 1.0;
            this.thePID.setGoal(this.goalRotations);
            /* one-time action goes here */
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
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            /* one-time action goes here */
            this.goalRotations += 1.0;
            this.thePID.setGoal(this.goalRotations);
        });
        // return run(() -> {
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }

    public void resetPID() {
        double motorPosition = this.motor.getPosition().getValueAsDouble();
        this.thePID.reset(motorPosition);
    }

    @Override // Rewrites (adds content to) a method from SubsystemBase
    public void periodic() {
        // This method will be called once per scheduler run (50 times per second)
        double motorPosition = this.motor.getPosition().getValueAsDouble();
        double newMotorSpeed = this.thePID.calculate(motorPosition);
        this.motor.set(newMotorSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Goal Rotations", () -> this.goalRotations, null);
        builder.addDoubleProperty("Motor Rotations", () -> this.motor.getPosition().getValueAsDouble(), null);
    }
}
