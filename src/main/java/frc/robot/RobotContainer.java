// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.PathingCommands;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.constants.PhysicalConstants.SterilizerConstants;
import frc.robot.constants.Positions.PathfindingPosition;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.lights.LEDSubsystem;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.shooter.ManuallyPivotShooterCommand;
import frc.robot.shooter.PivotShooterCommand;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.SterilizerSubsystem;
import frc.robot.swerve.SwerveDriveCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.SequencedCommands;
import frc.robot.utilities.Telemetry;

public class RobotContainer {
    // Singleton design pattern
    private static RobotContainer instance;

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final SendableChooser<Command> autoChooser;

    // Instance of the controllers used to drive the robot
    private CommandXboxController driveController;
    private CommandXboxController operatorController;

    /** Creates an instance of the robot controller */
    public RobotContainer() {
        this.driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);
        
        initializeSubsystems();
        // Register named commands for pathplanner (do this after subsystem initialization)
        registerNamedCommands();

        configureDriverBindings();
        configureOperatorBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
            .add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(11, 0)
            .withSize(4, 1);
    }

    /** Creates instances of each subsystem so periodic runs */
    private void initializeSubsystems() {
        LEDSubsystem.getInstance();
        LimelightSubsystem.getInstance();
        SwerveSubsystem.getInstance();
        IntakeSubsystem.getInstance();
        SterilizerSubsystem.getInstance();
        ShooterSubsystem.getInstance();
        Telemetry.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        // Pathing
        // NOTE use Pathplanner paths to return to speaker
        
        // NamedCommands.registerCommand("Pathfind AMP",
        //     Commands.runOnce(() -> PathingCommands.getPathfindCommand(PathfindingPosition.AMP).schedule()
        // ));
        // NamedCommands.registerCommand("Pathfind SPEAKER_TOP",
        //     Commands.runOnce(() -> PathingCommands.getPathfindCommand(PathfindingPosition.SPEAKER_TOP).schedule()
        // ));
        // NamedCommands.registerCommand("Pathfind SPEAKER_MIDDLE",
        //     Commands.runOnce(() -> PathingCommands.getPathfindCommand(PathfindingPosition.SPEAKER_MIDDLE).schedule()
        // ));
        // NamedCommands.registerCommand("Pathfind SPEAKER_BOTTOM",
        //     Commands.runOnce(() -> PathingCommands.getPathfindCommand(PathfindingPosition.SPEAKER_BOTTOM).schedule()
        // ));
        // NamedCommands.registerCommand("Bezier SPEAKER",
        //     PathingCommands.getBezierCommand(PathfindingPosition.SPEAKER));
        // NamedCommands.registerCommand("Bezier AMP",
        //     PathingCommands.getBezierCommand(PathfindingPosition.AMP));

        // Intake
        NamedCommands.registerCommand("FixNote", // TODO do this automatically
            Commands.run(() -> SterilizerSubsystem.getInstance().setSpeed(-SterilizerConstants.ADJUSTING_SPEED))
                .withTimeout(1));
        NamedCommands.registerCommand("Collect Note",
            SequencedCommands.getCollectNoteCommand());
        NamedCommands.registerCommand("Collect Note NOCENTER",
            SequencedCommands.getAutonCollectNoteCommand());
            
        // Shoot
        NamedCommands.registerCommand("Shoot SPEAKER",
            new ShootCommand(ShooterStates.SPEAKER));
        NamedCommands.registerCommand("Shoot AMP",
            new ShootCommand(ShooterStates.AMP));
        
        // Other
        NamedCommands.registerCommand("IntakeEject NOEND",
            SequencedCommands.getIntakeEjectCommand());
    }

    /** Configures the button bindings of the driver controller */
    private void configureDriverBindings() {
        // Sets the default command to driving swerve
        SwerveSubsystem.getInstance().setDefaultCommand(new SwerveDriveCommand(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> !(driveController.getHID().getLeftTriggerAxis() >= 0.5),
            () -> driveController.getHID().getRightTriggerAxis() >= 0.5,
            // D-Pad / POV movement
            ControllerConstants.DPAD_DRIVE_INPUT,
            (Integer angle) -> driveController.pov(angle).getAsBoolean()
        ));
        // Cancel all scheduled commands and turn off LEDs
        driveController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            ShooterSubsystem.getInstance().setShootingVelocities();
            LEDSubsystem.getInstance().setCommandStopState(false);
        }));
        // Zeroing functions
        // Double rectangle
        driveController.back().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().resetOdometryLimelight()));
        // Burger
        driveController.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().zeroHeading()));
        
        // driveController.leftBumper().onTrue(new CenterSpeakerCommand());
        driveController.leftBumper()
            .whileTrue(new PivotShooterCommand(ShooterStates.INTAKE));
        driveController.rightBumper()
            .whileTrue(new PivotShooterCommand(ShooterStates.SPEAKER));
        // driveController.rightBumper()
        //     .onTrue(SequencedCommands.getIntakeCommand())
        //     .onFalse(Commands.parallel(
        //         new PivotIntakeCommand(IntakeStates.IDLE),
        //         new PivotShooterCommand(ShooterStates.SPEAKER))
        // );
        driveController.y().onTrue(SequencedCommands.getCollectNoteCommand());
        
        // Line-up / Pathfinding commands
        driveController.x().onTrue(Commands.runOnce(
            () -> PathingCommands.getPathfindCommand(PathfindingPosition.SPEAKER_MIDDLE).schedule()
        ));
        driveController.a().whileTrue(Commands.runOnce(
            () -> PathingCommands.getPathfindCommand(PathfindingPosition.AMP).schedule()
        ));
        // driveController.a().whileTrue(PathingCommands.getPathfindCommand(PathfindingPosition.SAFETY_1));
    }

    /** Configures the button bindings of the driver controller */
    private void configureOperatorBindings() {
        ShooterSubsystem.getInstance().setDefaultCommand(new ManuallyPivotShooterCommand(
            () -> -operatorController.getLeftY(),
            () -> -operatorController.getRightY(),
            false
        ));

        // Cancel all scheduled commands and turn off LEDs
        operatorController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            ShooterSubsystem.getInstance().setShootingVelocities();
            LEDSubsystem.getInstance().setCommandStopState(false);
        }));

        // Shoot SPEAKER
        operatorController.rightBumper().whileTrue(Commands.sequence(
            new PivotShooterCommand(ShooterStates.SPEAKER),
            new ShootCommand(ShooterStates.SPEAKER)
        ));
        // Shoot AMP
        operatorController.leftBumper().whileTrue(Commands.sequence(
            new PivotShooterCommand(ShooterStates.AMP),
            new ShootCommand(ShooterStates.AMP)
        ));
        // TODO Run SHOOTER automatically
        operatorController.x().onTrue(SequencedCommands.getAutoSpeakerShootCommand());
        
        // Reverse sterilizer (0.2 speed)
        operatorController.y().whileTrue(Commands.runEnd(
            () -> SterilizerSubsystem.getInstance().setSpeed(-SterilizerConstants.ADJUSTING_SPEED),
            () -> SterilizerSubsystem.getInstance().setSpeed()
        ));
        // Reverse intake (0.5 speed)
        operatorController.a().whileTrue(Commands.runEnd(
            () -> IntakeSubsystem.getInstance().setIntakeSpeed(-IntakeConstants.INTAKE_SPEED / 2),
            () -> IntakeSubsystem.getInstance().setIntakeSpeed(0)
        ));
        // Front eject (double rectangle)
        operatorController.back().onTrue(Commands.parallel(
                new ShootCommand(ShooterStates.FRONT_EJECT).withTimeout(2),
                Commands.runOnce(() -> SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.FEEDING_SPEED))
            ))
            .onFalse(
                Commands.runOnce(() -> {
                    ShooterSubsystem.getInstance().setShootingVelocities();
                    SterilizerSubsystem.getInstance().setSpeed();;
                })
            );
        // Move sterilizer forward (burger)
        operatorController.start().whileTrue(Commands.runEnd(
            () -> SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.FEEDING_SPEED),
            () -> SterilizerSubsystem.getInstance().setSpeed()
        ));
        
        // Move the pivot manually (last resort, not recommended)
        operatorController.povUp().whileTrue(Commands.runEnd(
            () -> ShooterSubsystem.getInstance().setPivotSpeed(0.2, false),
            () -> ShooterSubsystem.getInstance().setPivotSpeed(0, true)
        ));
        operatorController.povDown().whileTrue(Commands.runEnd(
            () -> ShooterSubsystem.getInstance().setPivotSpeed(-0.2, false),
            () -> ShooterSubsystem.getInstance().setPivotSpeed(0, true)
        ));
        // Move the intake manually (last resort, not recommended)
        operatorController.povRight().whileTrue(Commands.runEnd(
            () -> IntakeSubsystem.getInstance().setPivotSpeed(0.2, true),
            () -> IntakeSubsystem.getInstance().setPivotSpeed(0)
        ));
        operatorController.povLeft().whileTrue(Commands.runEnd(
            () -> IntakeSubsystem.getInstance().setPivotSpeed(-0.1, true),
            () -> IntakeSubsystem.getInstance().setPivotSpeed(0)
        ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}