package frc.robot.subsystems;

// Imported libraries and files
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Supplier;
import java.util.jar.JarFile;

import frc.robot.Constants.MotorConstants; // Constants for the motor, refer with MotorConstants.[variable name]

import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.tunable.Tunables;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class MotorSubsystem extends SubsystemBase implements ComplexTunable {
    // Class variables (ints, doubles, motor objects) go he

    private final TalonFX motor = new TalonFX(MotorConstants.deviceid, MotorConstants.motorCanId);

    private final SimpleMotorFeedforward f_F = new SimpleMotorFeedforward(
        MotorConstants.fF_k_S, MotorConstants.fF_k_V);

    private final ProfiledPIDController pid = new ProfiledPIDController(
        MotorConstants.p,
        MotorConstants.i,
        MotorConstants.d, 
        new TrapezoidProfile.Constraints(
            MotorConstants.m_V,
            MotorConstants.m_A));

    private double staticGoal = MotorConstants.motorGoal; //initializes the goal
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
    public Command turnClockwise() {360};
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            //this.motor.setVoltage(5);
            this.dynamicGoal = this.staticGoal;

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
    public Command turnCounterClockwise() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(() -> {
            //his.motor.setVoltage(-5);
            this.dynamicGoal = -this.staticGoal;
            /* one-time action goes here */
        });
        // return run(() -> {
        //
        // }); // run() returns a command that repeats 50x per second until canceled or interrupted
    }

    @Override // Rewrites (adds content to) a method from SubsystemBase
    public void periodic() {
        encoderPos = this.motor.getPosition().getValueAsDouble();

        double pidCalculation = this.pid.calculate(
            this.encoderPos,
            this.dynamicGoal
        );
        double fF_Calculation = this.f_F.calculate(
            this.pid.getSetpoint().velocity
        );

        this.motor.setVoltage(fF_Calculation + pidCalculation);

        Telemetry.log("MotorSubsystem", this);
        // This method will be called once per scheduler run (50 times per second)
    }
    public void resetPIDs() {
        this.pid.reset(this.encoderPos);
    }
    @Override
    public void logTo(TelemetryTable table){
        table.log("velocity",this.motor.getVelocity());
        table.log("pid",this.pid);
        table.log("encoder pos", this.encoderPos);
    }
    @Override
    public void publishTunable(TunableTable table) {
        table.publish("pid", this.pid);
        table.publishDouble(
            "Motor Feedforward kS",
            () -> this.f_F.getKs(),
            (newKS) -> this.f_F.setKs(newKS)
        );
        table.publishDouble(
            "Motor Feedforward kV",
            () -> this.f_F.getKv(),
            (newKv) -> this.f_F.setKv(newKv)
        );

        table.publishDouble(
            "Motor PID P",
            () -> this.pid.getP(),
            (newP) -> this.pid.setP(newP)
        );
        table.publishDouble(
            "Motor PID I",
            () -> this.pid.getI(),
            (newI) -> this.pid.setI(newI)
        );
        table.publishDouble(
            "Motor PID D",
            () -> this.pid.getD(),
            (newD) -> this.pid.setD(newD)
        );
    }
}
