package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MotorSubsystem;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.telemetry.TelemetryTable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final MotorSubsystem theMotor = new MotorSubsystem();

    // Xbox Controller Object
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * org.wpilib.command2.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
     * PS4} controllers or {@link org.wpilib.command2.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // When "b" is pressed on the controller, turn the motor clockwise 360 degrees.
        m_driverController.b().whileTrue(theMotor.turnClockwise());

        // When "x" is pressed on the controller, turn the motor counterclockwise 360 degrees.
        m_driverController.x().whileTrue(theMotor.turnCounterClockwise());

    }

    public void resetPIDs() {
        this.theMotor.resetPIDs();
    }
}
