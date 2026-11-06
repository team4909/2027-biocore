// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AimAtTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

// PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;





//---Walter--//
//ToDo: (for walter)
//fix aiming at april tag, currently it just turns the robot a certain amount of degrees (I think)
//To fix this we need to get a vision subsystem and actually use it
//We could pass the vision subsystem to AimAtTag's constructor 
//and have it use the vision subsystem to get the angle to the april tag
//or we could proccess the math here.
// I think the first option is better because it keeps the math in one place and makes it easier to test and debug
//---Walter--//
public class RobotContainer {
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    //The maximum speed at which the robot is allowed to
    //rotate around its Z/Vertical axis (In radians/second)
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    //How the drivetrain should move.
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            //.withDeadband sets a minimum transitional speed
            //withRotationalDeadband does the same thing
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
            
    private final Vision vision = new Vision(
        drivetrain::addVisionMeasurement,
        createCameraIO(VisionConstants.frontLeftCameraName, VisionConstants.robotToFrontLeftCamera),
        createCameraIO(VisionConstants.frontRightCameraName, VisionConstants.robotToFrontRightCamera)
    );

    private SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> mirrorChooser = new SendableChooser<>();

    public RobotContainer() {
        //registers vision as a subsystem
        //this allowed the scheduler to call the periodic method of vision
        CommandScheduler.getInstance().registerSubsystem(vision);

        //Currently does nothing since the commands are commented out in configureNamedCommands()
        configureNamedCommands();

        //Autonomous
        SmartDashboard.putNumber("Auto/HeadingDeg", 0.0);
        SmartDashboard.putNumber("Auto/RequestedOmega", 0.0);
        //--Walter--//
        //Depends on where you are on the field,
        //for autos.
        //--Walter--//
        mirrorChooser.setDefaultOption("Don't Mirror (Blue)", false);
        mirrorChooser.addOption("Mirror Path (Red)", true);
        SmartDashboard.putData("Manual Mirror Override", mirrorChooser);
        //--Walter--//
        //Configures pathPlanner with the drivetrains
        //pose, resetPose, and getRobotRelativeSpeeds methods
        //--Walter--//
        configureAutoBuilder();
        //--Walter--//
        //builds the autoChooser
        //--Walter--//
        autoChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auto Mode", autoChooser);
                SmartDashboard.putNumber("Auto/HeadingDeg",drivetrain.getState().Pose.getRotation().getDegrees());
        
        //Connects controller's buttons //--Walter--//
        configureBindings();
    }
    
    //Ran for simulations, creates fake cameras //--Walter--//
    private VisionIO createCameraIO(String name, Transform3d robotToCamera) {
        if (RobotBase.isSimulation()) {
            //creates a new VisionIOPhotonVisionSim object and returns it
            return new VisionIOPhotonVisionSim(name, robotToCamera, () -> drivetrain.getState().Pose);
        }
        return new VisionIOPhotonVision(name, robotToCamera);
    }


    private void configureNamedCommands() {
        // NamedCommands.registerCommand("Spin90", new TurnToHeading(drivetrain, 90.0));
        // NamedCommands.registerCommand("Spin180", new TurnToHeading(drivetrain, 180.0));
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                drivetrain::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), //Drive PID
                    new PIDConstants(6.0, 0.0, 0.4) //Rotation PID
                ),
                config,
                () -> mirrorChooser.getSelected() != null ? mirrorChooser.getSelected() : false,
                drivetrain
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner AutoBuilder", e.getStackTrace());
        }
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("Auto/RequestedOmega", speeds.omegaRadiansPerSecond);
        drivetrain.setControl(
            fieldCentricDrive
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
        );

    }
    //--Walter--//
    // configures the buttons for the controller
    //so the controller can use the buttons and do stuff.
    //--Walter--//
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // SysId (System Identification) routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset heading
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        //              Why are we passing MaxAngularRate to AimAtTag? //--Walter--//
        joystick.povDown().whileTrue(new AimAtTag(drivetrain, MaxAngularRate));
        
        // 90 degrees is LB, 180 is RB [UPDATE: Commented because it is unused]

        
        // joystick.leftBumper().onTrue(new TurnToHeading(drivetrain, 90.0));
        // joystick.rightBumper().onTrue(new TurnToHeading(drivetrain, 180.0));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}