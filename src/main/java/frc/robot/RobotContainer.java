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
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShuffleboardTabConstants;
import frc.robot.Constants.AutonConstants.PathfindingPosition;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.auto.PathfindToGoalCommand;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.lights.LEDSubsystem;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.shooter.PivotShooterCommand;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.SterilizerSubsystem;
import frc.robot.swerve.SwerveDriveCommand;
import frc.robot.swerve.CenterSpeakerCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.JSONManager;
import frc.robot.utilities.SequencedCommands;

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

    /**
     * Creates an instance of the robot controller
     */
    public RobotContainer() {
        this.driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);
        
        initializeSubsystems();

        // Register named commands for pathplanner (do this after subsystem
        // initialization)
        NamedCommands.registerCommand("Pathfind AMP",
            new PathfindToGoalCommand(PathfindingPosition.AMP));
        NamedCommands.registerCommand("Pathfind SPEAKER",
            PathfindToGoalCommand.getPathfindCmd(PathfindingPosition.SPEAKER));
        NamedCommands.registerCommand("Collect Note NOCENTER",
            SequencedCommands.getCollectNoteCommandNoCenter());
        NamedCommands.registerCommand("Shoot SPEAKER",
            new ShootCommand(ShooterState.SPEAKER));
        NamedCommands.registerCommand("FixNote", Commands.run(
            () -> SterilizerSubsystem.getInstance().moveBackward(true))
            .withTimeout(0.5));
            
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

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
            .add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(11, 0)
            .withSize(4, 1);
    }

    /** Configures the button bindings of the controllers */
    private void configureBindings() {
        // Driver controller
        // Cancel all scheduled commands and turn off LEDs
        driveController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            LEDSubsystem.getInstance().setCommandStopState(false);
        }));
        // Zeroing functions
        // Double rectangle
        driveController.back().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().resetOdometryLimelight()));
        // Burger
        driveController.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().zeroHeading()));
        
        driveController.leftBumper().onTrue(new CenterSpeakerCommand());
        driveController.rightBumper()
            .onTrue(SequencedCommands.getIntakeCommand())
            .onFalse(Commands.parallel(
                new PivotIntakeCommand(IntakeState.IDLE),
                new PivotShooterCommand(ShooterState.SPEAKER))
        );
                
        driveController.y().onTrue(SequencedCommands.getCollectNoteCommand());
        driveController.x().whileTrue(new PathfindToGoalCommand(PathfindingPosition.SPEAKER));

        // Line up to SPEAKER
        // driveController.x().onTrue(new PathfindToGoalCommand(AutonConstants.SPEAKER));
        // Line up to AMP
        // driveController.y().onTrue(new PathfindLineUp(SwerveSubsystem.getInstance(),
        // AutonConstants.AMP));
        
        
        // Operator controller
        // Cancel all scheduled commands and turn off LEDs
        operatorController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            LEDSubsystem.getInstance().setCommandStopState(false);
        }));

        operatorController.rightBumper().whileTrue(Commands.sequence(
            new PivotShooterCommand(ShooterState.SPEAKER),
            new ShootCommand(ShooterState.SPEAKER)
        ));
        operatorController.leftBumper().whileTrue(Commands.sequence(
            new PivotShooterCommand(ShooterState.AMP),
            new ShootCommand(ShooterState.AMP)
        ));
        operatorController.y().whileTrue(new ShootCommand(ShooterState.MANUAL));
        operatorController.a().whileTrue(Commands.runEnd(
            () -> SterilizerSubsystem.getInstance().moveBackward(true),
            () -> SterilizerSubsystem.getInstance().moveStop()
        ));
        
        // Move the pivot manually (last resort, not recommended)
        operatorController.povUp().whileTrue(Commands.runEnd(
            () -> ShooterSubsystem.getInstance().setPivotSpeed(0.2),
            () -> ShooterSubsystem.getInstance().setPivotSpeed(0)
        ));
        operatorController.povDown().whileTrue(Commands.runEnd(
            () -> ShooterSubsystem.getInstance().setPivotSpeed(-0.2),
            () -> ShooterSubsystem.getInstance().setPivotSpeed(0)
        ));
        // Move the intake manually (last resort, not recommended)
        operatorController.povRight().whileTrue(Commands.runEnd(
            () -> IntakeSubsystem.getInstance().setPivotSpeedSafe(0.2),
            () -> IntakeSubsystem.getInstance().setPivotSpeed(0)
        ));
        operatorController.povLeft().whileTrue(Commands.runEnd(
            () -> IntakeSubsystem.getInstance().setPivotSpeedSafe(-0.1),
            () -> IntakeSubsystem.getInstance().setPivotSpeed(0)
        ));
    }

    /** Creates instances of each subsystem so periodic runs */
    private void initializeSubsystems() {
        JSONManager.getInstance();
        LEDSubsystem.getInstance();
        LimelightSubsystem.getInstance();
        SwerveSubsystem.getInstance();
        IntakeSubsystem.getInstance();
        SterilizerSubsystem.getInstance();
        ShooterSubsystem.getInstance();
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