package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

import java.util.List;

public class AimAtTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;




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


        List<Integer> CamerasWithTarget = vision.camerasWithTarget();

        int cameraIndex = vision.cameraOfBestChoice(CamerasWithTarget);
        //if no cameras have a target, set the
        //SmartDashboard values to false and 0, and set the drivetrain to brake
        if (cameraIndex == -1) {
            SmartDashboard.putNumber("Vision/Aiming/RequestedOmega", 0.0);
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            return;
        }
        
        double tx = vision.getTargetX(cameraIndex).getRadians();

        double output = controller.calculate(tx, 0.0);

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