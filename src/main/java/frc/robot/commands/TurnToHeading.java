package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;


public class TurnToHeading extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final double relativeDegrees;

    private final SwerveRequest.RobotCentric turnRequest = new SwerveRequest.RobotCentric();

    private final ProfiledPIDController controller = new ProfiledPIDController(
            4.0, 0.0, 0.1,
            new TrapezoidProfile.Constraints(
                    Math.toRadians(270), // max angular velocity
                    Math.toRadians(540)  // max angular acceleration
            ));

    private Rotation2d goal;
    private double startTime;
    private static final double kTimeout = 5.0;

    public TurnToHeading(CommandSwerveDrivetrain drivetrain, double relativeDegrees) {
        this.drivetrain = drivetrain;
        this.relativeDegrees = relativeDegrees;

        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.toRadians(1.5), Math.toRadians(10));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Rotation2d current = drivetrain.getState().Pose.getRotation();
        goal = current.plus(Rotation2d.fromDegrees(relativeDegrees));
        controller.reset(current.getRadians());
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double current = drivetrain.getState().Pose.getRotation().getRadians();
        double output = controller.calculate(current, goal.getRadians());

        SmartDashboard.putNumber("Auto/RequestedOmega", output);
        SmartDashboard.putNumber("Auto/TurnErrorDeg", Math.toDegrees(goal.minus(drivetrain.getState().Pose.getRotation()).getRadians()));

        drivetrain.setControl(
                turnRequest
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withRotationalRate(output)
        );
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal() || (Timer.getFPGATimestamp() - startTime) > kTimeout;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
