package frc.robot.subsystems;

// Imported libraries and files
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;

import org.wpilib.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.hardware.TalonFX;


import org.wpilib.telemetry.TelemetryTable;

import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]

public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here

    private final TalonFX motor = new TalonFX(MotorConstants.deviceid,MotorConstants.motorCanId);

    private final SimpleMotorFeedforward abc = new SimpleMotorFeedforward(MotorConstants.fF_k_S, MotorConstants.fF_k_V, 0);

    private final ProfiledPIDController pid = new ProfiledPIDController(
        MotorConstants.p,
        MotorConstants.i,
        MotorConstants.d,
        new TrapezoidProfile.Constraints(
            MotorConstants.m_V,
            MotorConstants.m_A));
    
    private double staticGoal = MotorConstants.motorGoal;
    private double dynamicGoal;
    private double encoderPos = 0.0;

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
            motor.setVoltage(5.0);
        });
        // return run(() -> {
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }


    public Command getEncoderPos(){ // OK so basically this is what the code predictor was saying to do and it looks pretty right so i just did it. Also, i used the encoderPos variable idk if thats the right one but i saw it and its not being used for anyting else so yeah.
        return runOnce(()->{
            encoderPos = MotorConstants.REVThroughBoreEncoder.getDistance();
        });
    }
    
    public Command outputEncoderPos(){
        return runOnce(()->{
            PublishTunable("Encoder Position: " + encoderPos); // HMMMm...
        });
    }

    public Command whatever(){
        return runOnce(()->{
            motor.setVoltage(0.0);
        });
    }

    @Override
    public void logTo(TelemetryTable table){
        table.log("velocity", this.motor.getVelocity());

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
