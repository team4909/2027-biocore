package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.List;
import java.util.Optional;

public class AimAtTag extends Command {



    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final int targetTagId;
    private final double maxAngularVelocity;


    private final PIDController controller = new PIDController(4.0, 0.0, 0.2);

    private final SwerveRequest.RobotCentric turnRequest =
        new SwerveRequest.RobotCentric();

    public AimAtTag(
        CommandSwerveDrivetrain drivetrain,
        Vision vision,
        int targetTagId,
        double maxAngularVelocity
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetTagId = targetTagId;
        this.maxAngularVelocity = maxAngularVelocity;

        controller.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.reset();
    }

    @Override
    public void execute() {

        // get pose2d of the robot
        var robotPose = drivetrain.getState().Pose;

        
        // get the pose of the tag from the field layout
        var targetTagPose = VisionConstants.aprilTagLayout.getTagPose(targetTagId);
        //Think about: This will never be empty, i dont think
        if (targetTagPose.isEmpty()) {
            //requested omega is requested change, with a number
            //so think of it as "requested to turn at this speed/angle"
            SmartDashboard.putNumber("Vision/Aiming/RequestedOmega", 0.0);
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            return;
        }

        //Turn the target tag pose into a 2d pose
        Pose2d targetTagPose2d = targetTagPose.get().toPose2d();
        // calculate the angle to the target tag
        Translation2d robotToTag = targetTagPose2d.getTranslation().minus(robotPose.getTranslation());
        //where we want to be
        Rotation2d desiredAngle = new Rotation2d(robotToTag.getX(), robotToTag.getY());
        //get our current rotation in radians
        var currentHeading = robotPose.getRotation().getRadians();


        //calculate how much we need to turn to face the target tag
        var tx = desiredAngle.getRadians() - currentHeading;


        double output = controller.calculate(tx, 0.0);
        
        //this is so that we dont turn too fast.
        //maxAngularVelocity is the max speed we can turn at
        output = MathUtil.clamp(
            output,
            -maxAngularVelocity,
            maxAngularVelocity
        );

        SmartDashboard.putNumber(
        "Vision/Aiming/RequestedOmega",
            output
        );
        //request drive train to turn
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