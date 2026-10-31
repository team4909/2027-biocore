// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.TurnToHeading;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Music.MusicSubsystem;

// PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class RobotContainer {
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //Telementry logger for the swerve drive platform, to be used in the SmartDashboard
    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Joystick Declaration
    private final CommandXboxController joystick = new CommandXboxController(0);

    //Make the DT
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    //Choose auto and choose mirror.
    private SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> mirrorChooser = new SendableChooser<>();

    //Music subsystem
    private final MusicSubsystem musicSubsystem = new MusicSubsystem(
        List.of(
            drivetrain.getModule(0).getDriveMotor(), // Front Left
            drivetrain.getModule(1).getDriveMotor(), // Front Right
            drivetrain.getModule(2).getDriveMotor(), // Back Left
            drivetrain.getModule(3).getDriveMotor()  // Back Right
        )
    );

    public RobotContainer() {
        configureNamedCommands();
        SmartDashboard.putNumber("Auto/HeadingDeg", 0.0);
        SmartDashboard.putNumber("Auto/RequestedOmega", 0.0);
        mirrorChooser.setDefaultOption("Don't Mirror (Blue)", false);
        mirrorChooser.addOption("Mirror Path (Red)", true);
        SmartDashboard.putData("Manual Mirror Override", mirrorChooser);

        configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
                SmartDashboard.putNumber("Auto/HeadingDeg",drivetrain.getState().Pose.getRotation().getDegrees());
        configureBindings();
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Spin90", new TurnToHeading(drivetrain, 90.0));
        NamedCommands.registerCommand("Spin180", new TurnToHeading(drivetrain, 180.0));
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

        // SysId routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset heading
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        // 90 degrees is LB, 180 is RB
        joystick.leftBumper().onTrue(new TurnToHeading(drivetrain, 90.0));
        joystick.rightBumper().onTrue(new TurnToHeading(drivetrain, 180.0));

        // Music subsystem controls
        joystick.povUp().onTrue(Commands.runOnce(() -> 
            musicSubsystem.playMusic("mario.chrp"), musicSubsystem)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}