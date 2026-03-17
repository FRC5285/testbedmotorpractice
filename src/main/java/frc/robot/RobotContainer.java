package frc.robot;

import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.ledSubSystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.ledConstants.Constants.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController m_joystick = new CommandXboxController(0);

    // The robot's subsystems and commands are defined here...
    private final MotorSubsystem Motor = new MotorSubsystem();
    private final ledSubSystem theLED = new ledSubSystem();

    // Xbox Controller Object
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        theLED.currentPattern = trans_flag; //first index is speed in Hz
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // When "b" is pressed on the controller, turn the motor clockwise 360 degrees.
        m_joystick.leftBumper().whileTrue(Motor.runmotor());
        m_joystick.leftBumper().whileFalse(Motor.stopMotor());
    }
}
