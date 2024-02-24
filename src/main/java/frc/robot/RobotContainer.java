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
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShuffleboardTabConstants;
import frc.robot.auto.PathfindToGoalCommand;
import frc.robot.lights.LEDSubsystem;
import frc.robot.lights.LEDSubsystem.LightState;
import frc.robot.swerve.SwerveDriveCommand;
import frc.robot.swerve.SwerveOrbitCommand;
import frc.robot.swerve.CenterSpeakerCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.intake.*;

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

    // Instance of the controller used to drive the robot
    private CommandXboxController driveController;
    private CommandXboxController operatorController;

    /**
     * Creates an instance of the robot controller
     */
    public RobotContainer() {
        this.driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        // Register named commands for pathplanner (do this after subsystem
        // initialization)
        NamedCommands.registerCommand("Pathfind AMP",
            new PathfindToGoalCommand(AutonConstants.AMP));
        NamedCommands.registerCommand("Pathfind SPEAKER",
            new PathfindToGoalCommand(AutonConstants.SPEAKER));

        // Sets the default command to driving swerve
        SwerveSubsystem.getInstance().setDefaultCommand(new SwerveDriveCommand(
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX(),
                () -> !(driveController.getHID().getLeftTriggerAxis() >= 0.5),
                () -> driveController.getHID().getRightTriggerAxis() >= 0.5,
                // D-Pad / POV movement
                ControllerConstants.DPAD_DRIVE_INPUT,
                (Integer angle) -> driveController.pov(angle).getAsBoolean()));
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabConstants.DEFAULT)
                .add("Auto Chooser", autoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 3)
                .withSize(3, 2);
    }

    // Configures the button bindings of the controllers
    private void configureBindings() {
        // Driver controller
        // Zeroing functions
        driveController.rightBumper().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().zeroHeading()));
        driveController.leftBumper()
                .onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().resetOdometryLimelight()));
        // Cancel all scheduled commands and turn off LEDs
        driveController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
            LEDSubsystem.getInstance().setLightState(LightState.OFF);
        }));
        // Orbit April-Tag
        driveController.a().toggleOnTrue(new SwerveOrbitCommand(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> !(driveController.getHID().getLeftTriggerAxis() >= 0.5),
            () -> driveController.getHID().getRightTriggerAxis() >= 0.5,
            // D-Pad / POV Movement
            ControllerConstants.DPAD_DRIVE_INPUT,
            (Integer angle) -> driveController.pov(angle).getAsBoolean()
            ));
        
        // driveController.y().onTrue(Commands.sequence(new CenterNoteCommand(), new DriveToNoteCommand()));
        driveController.y().onTrue(new CenterSpeakerCommand());
        // Operator controller
        // Line up to SPEAKER
        // driveController.x().onTrue(new
        // PathfindToGoalCommand(AutonConstants.SPEAKER));
        // Line up to AMP
        // driveController.y().onTrue(new PathfindLineUp(SwerveSubsystem.getInstance(),
        // AutonConstants.AMP));

        operatorController.a()
                .onTrue(new IntakePivot(IntakeConstants.PIVOT_DOWN_DEGREE))
                .onFalse(new IntakePivot(IntakeConstants.PIVOT_UP_DEGREE));

        operatorController.b().onTrue(new Intake());
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
