package frc.robot.subsystems;

// Imported libraries and files
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]


public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here
    private TalonFX motor; // motor object
    private double goalRotations; // number of rotations to go to
    private ProfiledPIDController thePID; // motor PID

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {
        // defines variables
        this.motor = new TalonFX(MotorConstants.motorCanId);
        this.goalRotations = 0.0;
        this.thePID = new ProfiledPIDController(MotorConstants.kP, MotorConstants.kI, MotorConstants.kD, new TrapezoidProfile.Constraints(MotorConstants.maxAccel, MotorConstants.maxVelocity));

        // Sets motor position and goal to 0
        this.motor.setPosition(0.0);
        this.thePID.setGoal(this.goalRotations);

        // Telemetry
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
            this.thePID.setGoal(this.goalRotations); // sets pid goal
        });
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
            this.goalRotations += 1.0;
            this.thePID.setGoal(this.goalRotations); // sets pid goal
        });
    }

    // Call this method in Robot.java when the robot re-enables, only needed when the "I" term in the PID is not 0
    public void resetPID() {
        double motorPosition = this.motor.getPosition().getValueAsDouble(); // gets motor position
        this.thePID.reset(motorPosition); // resets PID
    }

    @Override // Rewrites (adds content to) a method from SubsystemBase
    public void periodic() { // This method will be called once per scheduler run (50 times per second)
        double motorPosition = this.motor.getPosition().getValueAsDouble(); // gets motor position
        double newMotorSpeed = this.thePID.calculate(motorPosition); // motor speed calculation from PID
        this.motor.set(newMotorSpeed); // sets the motor speed

        // the above as one line:
        // this.motor.set(this.thePID.calculate(this.motor.getPosition().getValueAsDouble()));
    }

    // Telemetry
    @Override
    public void initSendable(SendableBuilder builder){
        // goal rotations
        builder.addDoubleProperty("Goal Rotations", () -> this.goalRotations, null);

        // actual rotations
        builder.addDoubleProperty("Motor Rotations", () -> this.motor.getPosition().getValueAsDouble(), null);
    }
}
