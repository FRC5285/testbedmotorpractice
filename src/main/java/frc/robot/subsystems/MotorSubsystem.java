package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import frc.robot.Constants.MotorConstants;


public class MotorSubsystem extends SubsystemBase {
    // Class variables (ints, doubles, motor objects) go here


    private Rev2mDistanceSensor lidarSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighSpeed);

    /** Creates a new MotorSubsystem. */
    public MotorSubsystem() {
        lidarSensor.setAutomaticMode(true);

    }

    public double getLidarMeters() {

        return lidarSensor.getRange() / 1000.0 - MotorConstants.lidarOffset;
    }

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

    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Lidar Distance", () -> getLidarMeters(), null);
    }
}
