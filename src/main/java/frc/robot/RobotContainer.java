// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.CenterSpeakerCommand;
import frc.robot.auto.PathingCommands;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.IntakeStates;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.Constants.ShooterStates;
import frc.robot.constants.PhysicalConstants.SterilizerConstants;
import frc.robot.constants.Positions.PathfindingPosition;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.PivotIntakeCommand;
import frc.robot.lights.LEDSubsystem;
import frc.robot.limelight.LimelightSubsystem;
import frc.robot.shooter.ManuallyPivotShooterCommand;
import frc.robot.shooter.PivotShooterMMCommand;
import frc.robot.shooter.RevUpCommand;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shooter.SterilizerSubsystem;
import frc.robot.swerve.SwerveDriveCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.utilities.SequencedCommands;
import frc.robot.utilities.Telemetry;

public class RobotContainer {
    // Thread-safe singleton design pattern
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
        Telemetry.getInstance();
        Timer.delay(2.5);
        
        LEDSubsystem.getInstance();
        LimelightSubsystem.getInstance();
        SwerveSubsystem.getInstance();
        IntakeSubsystem.getInstance();
        SterilizerSubsystem.getInstance();
        ShooterSubsystem.getInstance();
        ElevatorSubsystem.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        // Pathing
        // NOTE use Pathplanner paths to return to speaker
        
        // Intake
        NamedCommands.registerCommand("Collect Note CENTER",
            SequencedCommands.getCollectNoteCommand().withTimeout(7.5));
        NamedCommands.registerCommand("Collect Note NOCENTER",
            SequencedCommands.getCollectNoteCommandNoCenter().withTimeout(5));

        // Shoot
        NamedCommands.registerCommand("Shoot SPEAKER",
            new ShootCommand(ShooterStates.SPEAKER));
        NamedCommands.registerCommand("Shoot AMP",
            new ShootCommand(ShooterStates.AMP));
        NamedCommands.registerCommand("Shoot CALCULATE",
            SequencedCommands.getAutoSpeakerShootCommand());
        
        // Rev Up
        NamedCommands.registerCommand("Rev Up SPEAKER",
            new RevUpCommand(ShooterStates.SPEAKER));
            NamedCommands.registerCommand("Rev Up AMP",
            new RevUpCommand(ShooterStates.AMP));
        NamedCommands.registerCommand("Rev Up CALCULATE",
            new RevUpCommand(ShooterStates.SPEAKER_CALCULATE));

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
        
        driveController.leftBumper().onTrue(new CenterSpeakerCommand().withTimeout(1));
        driveController.rightBumper()
            .whileTrue(SequencedCommands.getIntakeCommand())
            .onFalse(Commands.parallel(
                new PivotIntakeCommand(IntakeStates.IDLE),
                new PivotShooterMMCommand(ShooterStates.SPEAKER)
            ));
        driveController.y().onTrue(SequencedCommands.getCollectNoteCommand());
        
        // Line-up / Pathfinding commands
        driveController.x().onTrue(Commands.runOnce(
            () -> PathingCommands.getPathfindCommand(PathfindingPosition.SPEAKER_MIDDLE).schedule()
        ));
        driveController.a().whileTrue(Commands.runOnce(
            () -> PathingCommands.getPathfindCommand(PathfindingPosition.AMP).schedule()
        ));
    }

    /** Configures the button bindings of the driver controller */
    private void configureOperatorBindings() {
        ShooterSubsystem.getInstance().getPivotRequirement().setDefaultCommand(new ManuallyPivotShooterCommand(
            () -> -operatorController.getLeftY(),
            () -> -operatorController.getRightY(),
            false
        ));
        operatorController.rightStick().onTrue(
            Commands.run(() -> ShooterSubsystem.getInstance().setRotorPositions())
        );

        // Cancel all scheduled commands and turn off LEDs
        operatorController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            // In case it interrupts the RevUpCommand
            ShooterSubsystem.getInstance().setShootingVelocities();
            LEDSubsystem.getInstance().setCommandStopState(false);
        }));

        // Shoot SPEAKER
        operatorController.rightBumper().whileTrue(Commands.parallel(
            new PivotShooterMMCommand(ShooterStates.SPEAKER),
            new ShootCommand(ShooterStates.SPEAKER)
        ));
        // Shoot AMP
        operatorController.leftBumper().whileTrue(Commands.parallel(
            new PivotShooterMMCommand(ShooterStates.AMP),
            new ShootCommand(ShooterStates.AMP)
        ));
        operatorController.x().onTrue(SequencedCommands.getAutoSpeakerShootCommand());
        
        // Reverse sterilizer (0.25 speed) and intake (0.25 speed)
        operatorController.y().whileTrue(Commands.runEnd(
            () -> {
                SterilizerSubsystem.getInstance().setSpeed(-0.25);
                IntakeSubsystem.getInstance().setIntakeSpeed(-0.25);
            },
            () -> {
                SterilizerSubsystem.getInstance().setSpeed();
                IntakeSubsystem.getInstance().setIntakeSpeed();
            }
        ));
        // Rev up both motors to 1000 RPM
        operatorController.a()
            .whileTrue(new RevUpCommand(1250))
            .onFalse(Commands.run(
                () -> ShooterSubsystem.getInstance().setShootingVelocities(),
                ShooterSubsystem.getInstance().getShootingRequirement()
            ));
        
        // Front eject (double rectangle)
        operatorController.back().whileTrue(new ShootCommand(ShooterStates.FRONT_EJECT));
        // Move sterilizer forward (burger)
        operatorController.start().whileTrue(Commands.runEnd(
            () -> SterilizerSubsystem.getInstance().setSpeed(SterilizerConstants.FEEDING_SPEED),
            () -> SterilizerSubsystem.getInstance().setSpeed()
        ));

        // ElevatorSubsystem.getInstance().setDefaultCommand(new MoveElevatorCommand(
        //     () -> operatorController.getRightTriggerAxis(),
        //     () -> operatorController.getLeftTriggerAxis()
        // ));
        
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