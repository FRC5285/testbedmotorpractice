package frc.robot.subsystems;

// Imported libraries and files
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;

public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here
    private final TalonFX thisMotor;
    private final Encoder thisEncoder;
    private final ProfiledPIDController thisPID;
    private boolean motorOverride = false;

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {
        thisMotor = new TalonFX(MotorConstants.motorCanId);
        thisEncoder = new Encoder(MotorConstants.encoderA, MotorConstants.encoderB);
        thisEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
        thisPID = new ProfiledPIDController(MotorConstants.kP, MotorConstants.kI, MotorConstants.kD,
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
        return runOnce(() -> {
            /* one-time action goes here */
            motorOverride = false;
            thisPID.setGoal(2 * Math.PI * 1.5);
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
            motorOverride = false;
            thisPID.setGoal(-2 * Math.PI * 1.5);
        });
        // return run(() -> {
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }

    @Override // Rewrites (adds content to) a method from SubsystemBase
    public void periodic() {
        // This method will be called once per scheduler run (50 times per second)
        double pidCalc = thisPID.calculate(thisEncoder.getDistance());

        if (motorOverride == false) this.thisMotor.setVoltage(pidCalc);

        if (thisPID.atGoal()) thisMotor.stopMotor();
    }
}
