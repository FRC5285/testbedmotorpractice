package frc.robot.subsystems;

// Imported libraries and files
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import com.ctre.phoenix6.hardware.TalonFX;


public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here
    private final TalonFX motor;
    private final Encoder motorEncoder;
    private final ProfiledPIDController motorPID;

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {
        motor = new TalonFX(MotorConstants.motorCanId);
        motorEncoder = new Encoder(0, 1); //placeholder
        motorPID = new ProfiledPIDController(MotorConstants.kp, MotorConstants.ki, MotorConstants.kd,
            new TrapezoidProfile.Constraints(MotorConstants.maxV, MotorConstants.maxA));
        motorPID.setTolerance(MotorConstants.tolerance);
        motorPID.setGoal(0); //reset goal
        }

    /**
     * Creates a command that turns the motor shaft 360 degrees clockwise.
     *
     * @return a command that turns the motor shaft 360 degrees clockwise.
     */
    public Command turnClockwise360() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> motorPID.setGoal(1)); //one rotation
    }
    /**
     * Creates a command that turns the motor shaft 360 degrees counterclockwise.
     *
     * @return a command that turns the motor shaft 360 degrees counterclockwise.
     */
    public Command turnCounterClockwise360() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> motorPID.setGoal(-1)); //negative one rotation
        // return run(() -> {
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }

    @Override // Rewrites (adds content to) a method from SubsystemBase
    public void periodic() {
        // This method will be called once per scheduler run (50 times per second)
        double pidCalc = motorPID.calculate(motorEncoder.getDistance());
        motor.set(pidCalc);
    }
}
