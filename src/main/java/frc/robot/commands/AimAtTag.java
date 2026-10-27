package frc.robot.commands;

import static frc.robot.VisionConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final int tagId;

    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private boolean aligned;

    public AimAtTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, int tagId) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tagId = tagId;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        aligned = false;
        stopDrive();
    }

    @Override
    public void execute() {
        var observation = vision.getAimObservation(tagId);

        if (!ENABLE_AIM_TEST || observation.isEmpty()) {
            aligned = false;
            stopDrive();
            return;
        }

        double error = observation.get().bearingRadians();
        aligned = Math.abs(error) <= AIM_TOLERANCE_RAD;

        double omega = aligned ? 0.0 : MathUtil.clamp(
                AIM_KP * error, 
                -AIM_MAX_RAD_PER_S, 
                AIM_MAX_RAD_PER_S
        );

        drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(omega)
        );
    }

    @Override
    public void end(boolean interrupted) {
        aligned = false;
        stopDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isAligned() {
        return aligned;
    }

    private void stopDrive() {
        drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }
}
