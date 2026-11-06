package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

public class AimAtTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    private final int cameraIndex = 0;

    private final double maxAngularVelocity;
    private final PIDController controller = new PIDController(4.0, 0.0, 0.1
    );

    private final SwerveRequest.RobotCentric turnRequest =
        new SwerveRequest.RobotCentric();

    public AimAtTag(
        CommandSwerveDrivetrain drivetrain,
        Vision vision,
        double maxAngularVelocity
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.maxAngularVelocity = maxAngularVelocity;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {

        if (!vision.hasTarget(cameraIndex)) {
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());

            SmartDashboard.putBoolean("Vision/Aiming/HasTarget", false);
            SmartDashboard.putNumber("Vision/Aiming/TxDegrees", 0.0);
            SmartDashboard.putNumber("Vision/Aiming/RequestedOmega", 0.0);

            return;
        }

        double tx = vision.getTargetX(cameraIndex).getRadians();

        double output = controller.calculate(-tx, 0.0);

        output = MathUtil.clamp(
            output,
            -maxAngularVelocity,
            maxAngularVelocity
        );

        SmartDashboard.putBoolean("Vision/Aiming/HasTarget", true);
        SmartDashboard.putNumber(
            "Vision/Aiming/TxDegrees",
            Math.toDegrees(tx)
        );
        SmartDashboard.putNumber(
            "Vision/Aiming/RequestedOmega",
            output
        );

        drivetrain.setControl(
            turnRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(output)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}