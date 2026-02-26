package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.TalonFX;

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

    //private TalonFX rotateMotor;
    //private double goalRotations;
    //private ProfiledPIDController rotatePID;

    private Rev2mDistanceSensor lidarSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighSpeed);

    public MotorSubsystem() {
        lidarSensor.setAutomaticMode(true);

        //goalRotations = 0.0;
        //rotateMotor = new TalonFX(MotorConstants.motorCanId);
        //rotatePID = new ProfiledPIDController(MotorConstants.kP, MotorConstants.kI, MotorConstants.kD, new TrapezoidProfile.Constraints(MotorConstants.maxA, MotorConstants.maxV));
        
        //rotateMotor.setPosition(0,0);
        //rotatePID.setGoal(goalRotations);
    }

    public double getLidarMeters() {

        return lidarSensor.getRange() / 1000.0 - MotorConstants.lidarOffset;
    }

    public Command turnClockwise360() {

        return runOnce(() -> {

            //goalRotations = MotorConstants.rotateGoalRotations;
            //rotatePID.setGoal(goalRotations);
        });
    }

    public Command turnCounterClockwise360() {

        return runOnce(() -> {

        });
    }

    public void resetPID() {

        //double motorPosition = rotateMotor.getPosition().getValueAsDouble();
        //rotatePID.reset(motorPosition);
    }

    public void periodic() {

        //double motorPosition = rotateMotor.getPosition().getValueAsDouble();
        //double rotateNewMotorSpeed = rotatePID.calculate(motorPosition);
        //rotateMotor.set(rotateNewMotorSpeed);
    }

    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Lidar Distance", () -> getLidarMeters(), null);
        //builder.addDoubleProperty("Rotate Motor Rotations", () -> rotateMotor.getPosition().getValueAsDouble(), null);
        //builder.addDoubleProperty("Rotate Goal Rotations", () -> this.goalRotations, null);
    }
}
