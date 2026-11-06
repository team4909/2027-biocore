package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.MathUtil;

//Double = Decimal
//Boolean = T/F
//Int = Whole Number
//String = Text
//Rotation2d = Angle

public class AimAtTag extends Command {

    //I think this file does not aim a an april tag.
    //It just turns the robot a certain amount of degrees, 
    //based on the relativeDegrees variable.
    //When povDown is pressed the AimAtTag constructor is called with MaxAngularRate
    //Passed

    private final CommandSwerveDrivetrain drivetrain;
    private final double relativeDegrees;
    private final double maxAngularVelocity = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
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
    //--Walter--//
    //Called with Joystick pov.down
    //THIS EXPECTS RELATIVE DEGREES BUT YET WE PASS MAX ANGULAR RATE TO IT. 
    //WE NEED TO FIX THIS. - Walter
    //doesnt this need to have something to do with vision as well? 
    //--Walter--//
    public AimAtTag(CommandSwerveDrivetrain drivetrain, double relativeDegrees) {
        this.drivetrain = drivetrain;
        this.relativeDegrees = relativeDegrees;

        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.toRadians(1.5), Math.toRadians(10));

        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        //gets the current heading of the robot //--Walter--//
        Rotation2d current = drivetrain.getState().RawHeading;
        
        
        goal = current.plus(Rotation2d.fromDegrees(relativeDegrees));
        
        
        controller.reset(current.getRadians());

        //FPGA stands for Field Programmable Gate Array. //--Walter--//

        startTime = Timer.getFPGATimestamp();
    }
    //--Walter--//
    //Runs repeatedly while AimAtTag command is active.
    //About every 20 ms
    //--Walter--//
    @Override
    public void execute() {
        // get the current heading
        double current_heading = drivetrain.getState().RawHeading.getRadians();
        double output = controller.calculate(current_heading, goal.getRadians())
            + controller.getSetpoint().velocity;

        output = MathUtil.clamp(output, -maxAngularVelocity, maxAngularVelocity);

        //Omega == change //--Walter--//
        SmartDashboard.putNumber("Auto/RequestedOmega", output);
        SmartDashboard.putNumber("Auto/TurnErrorDeg", Math.toDegrees(goal.minus(drivetrain.getState().Pose.getRotation()).getRadians()));

        drivetrain.setControl(
                turnRequest
                        //--Walter--//
                        //Will not move forward or backwards at all
                        //Thats what the 0.0's are for
                        //--Walter--//
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        //--Walter--//
                        //Sets rotation speed round z Axis to the variables
                        //output, which is calculated by the PID controller
                        //--Walter--//
                        .withRotationalRate(output)
        );
    }

    @Override
    public boolean isFinished() {
        //Might be the reason why the robot is timing out when holding POV down, //--Walter--//
            //thanks to the or. //--Walter--//
            //From Srihaas, wdym or?
                //--Walter--//
                // From walter, the return has || which is or. so its
                //either controller is at goal
                // or the time is greater than 5 seconds,
                // so if the controller is not at goal,
                // it will wait for 5 seconds and then stop.
                
                //                HERE
        return controller.atGoal() || (Timer.getFPGATimestamp() - startTime) > kTimeout;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
    }
}
