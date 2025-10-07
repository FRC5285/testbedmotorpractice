package frc.robot.subsystems;

// Imported libraries and files
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]

public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {

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
        });
        // return run(() -> {
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }

    @Override // Rewrites (adds content to) a method from SubsystemBase
    public void periodic() {
        // This method will be called once per scheduler run (50 times per second)
    }
}
