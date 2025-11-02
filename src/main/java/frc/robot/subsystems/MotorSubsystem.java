package frc.robot.subsystems; // Puts this file in the 'subsystems' package (part of the robot code structure)

// =========================== IMPORTS ===========================
// Import classes used for motor control, PID, telemetry, and commands
import edu.wpi.first.math.controller.ProfiledPIDController; // PID controller with velocity/acceleration limits
import edu.wpi.first.math.trajectory.TrapezoidProfile; // Defines motion profile constraints
import edu.wpi.first.util.sendable.SendableBuilder; // Used to display custom values on SmartDashboard
import edu.wpi.first.util.sendable.SendableRegistry; // Registers subsystems or sendables for telemetry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Displays live data on the SmartDashboard
import edu.wpi.first.wpilibj2.command.Command; // Base class for command objects
import edu.wpi.first.wpilibj2.command.SubsystemBase; // Base class for all subsystems in command-based programming

import com.ctre.phoenix6.hardware.TalonFX; // Class for controlling a Kraken X44 (TalonFX) motor via CAN bus

import frc.robot.Constants.MotorConstants; // Imports constants specific to the motor setup

// =========================== CLASS DEFINITION ===========================
public class MotorSubsystem extends SubsystemBase { // Defines the subsystem that controls the motor

    // Declare class variables (available to all methods in this class)
    private TalonFX motor;              // Motor controller object for the Kraken X44
    private double goalRotations;       // The target number of rotations the motor should move to
    private ProfiledPIDController thePID; // PID controller object for precise position control

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() { // Constructor runs once when the subsystem is created

        // =========================== MOTOR INITIALIZATION ===========================
        this.motor = new TalonFX(MotorConstants.motorCanId); // Creates motor object using its CAN ID from Constants.java

        // =========================== INITIAL GOAL SETUP ===========================
        this.goalRotations = 0.0; // Start with the motor goal position at 0 rotations

        // =========================== PID CONTROLLER SETUP ===========================
        this.thePID = new ProfiledPIDController(
            MotorConstants.kP, // Proportional term (how strongly the motor reacts to error)
            MotorConstants.kI, // Integral term (corrects accumulated error over time)
            MotorConstants.kD, // Derivative term (slows down overshoot by considering rate of change)
            new TrapezoidProfile.Constraints(
                MotorConstants.maxAccel, // Maximum acceleration allowed
                MotorConstants.maxVelocity // Maximum velocity allowed
            )
        );

        // =========================== INITIAL RESET ===========================
        this.motor.setPosition(0.0); // Reset the motor’s internal position sensor to 0 rotations
        this.thePID.setGoal(this.goalRotations); // Tell the PID that our first goal is 0 rotations (stay still)

        // =========================== TELEMETRY SETUP ===========================
        SendableRegistry.add(this, "Motor"); // Registers this subsystem with the dashboard system under the name "Motor"
        SmartDashboard.putData(this); // Adds this subsystem’s telemetry to the SmartDashboard
    }

    // =========================== COMMANDS ===========================

    /**
     * Creates a command that turns the motor shaft 360 degrees clockwise.
     *
     * @return a command that turns the motor shaft 360 degrees clockwise.
     */
    public Command turnClockwise360() {
        // Creates a one-time command that runs only once when the button is pressed
        return runOnce(() -> {
            this.goalRotations -= 1.0; // Decrease goal by 1 rotation (clockwise = negative direction)
            this.thePID.setGoal(this.goalRotations); // Update the PID controller’s goal to the new target
        });
    }

    /**
     * Creates a command that turns the motor shaft 360 degrees counterclockwise.
     *
     * @return a command that turns the motor shaft 360 degrees counterclockwise.
     */
    public Command turnCounterClockwise360() {
        // Creates a one-time command that runs only once when the button is pressed
        return runOnce(() -> {
            this.goalRotations += 1.0; // Increase goal by 1 rotation (counterclockwise = positive direction)
            this.thePID.setGoal(this.goalRotations); // Update the PID controller’s goal to the new target
        });
    }

    // =========================== PID RESET ===========================
    // Optional helper to reset the PID loop to current position when re-enabling robot
    public void resetPID() {
        double motorPosition = this.motor.getPosition().getValueAsDouble(); // Get the current motor position in rotations
        this.thePID.reset(motorPosition); // Reset the PID internal state to that position
    }

    // =========================== PERIODIC CONTROL LOOP ===========================
    @Override
    public void periodic() { // Called automatically ~50 times per second while robot code runs

        // Read the current position from the motor’s internal encoder
        double motorPosition = this.motor.getPosition().getValueAsDouble();

        // Use the PID controller to calculate how fast the motor should move to reach the goal
        double newMotorSpeed = this.thePID.calculate(motorPosition);

        // Send the calculated output (speed) to the motor controller
        this.motor.set(newMotorSpeed);

        // Equivalent single-line version of the above three lines:
        // this.motor.set(this.thePID.calculate(this.motor.getPosition().getValueAsDouble()));
    }

    // =========================== TELEMETRY / DASHBOARD DATA ===========================
    @Override
    public void initSendable(SendableBuilder builder){
        // Add a live display value for the goal rotations (the target position)
        builder.addDoubleProperty("Goal Rotations", () -> this.goalRotations, null);

        // Add a live display value for the actual motor position (from encoder)
        builder.addDoubleProperty("Motor Rotations", () -> this.motor.getPosition().getValueAsDouble(), null);
    }
}







